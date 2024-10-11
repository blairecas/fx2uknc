#define UNICODE

#include <windows.h>
#include <process.h>
#include <stdio.h>
#include "lib/libusb.h"

#pragma comment(lib, "lib/libusb-1.0.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "winmm.lib")


#define UKNC_SCR_WIDTH  0x00320     // 800 pix clk in line
#define UKNC_SCR_FULL   0x3CF00     // 249600 pix clk in full screen
#define TR_CHUNK_SIZE   0x20000     // chunk size for async data transferring from fx2
// sigrok sources say it should hold 10ms of data (and be aligned with 0x200 bytes)

#define VID 0x04B4                  // (0xFFFF:0x2048) for y-salnikov's)
#define PID 0x8613                  // 
#define ENDPOINT 0x82

// protocol commands
#define CMD_START                       0xB1
#define CMD_START_FLAGS_INV_CLK         0x01


////////////////////////////////////////////////////////////////////////////////
// Data
////////////////////////////////////////////////////////////////////////////////

    libusb_device_handle* device_h = NULL;
    const char* fw_filename = "fx2lafw-cypress-fx2.fw";

    volatile uint32_t* scr_buffers[8];
    volatile uint32_t  scr_n_cur     = 0;
    volatile uint32_t  scr_cur_addr  = 0;
    volatile uint32_t  scr_lsync_cnt = 0;
    volatile uint8_t   scr_show_sync = 0;

    int stop = 0;                   // encountered an error somewhere
    int handled_count = 0;          // count of processed usb bulk transfers
    int errors_count = 0;           // count of not processed

    uint32_t fx2data2grb[32] = {
        // 16 main colors
        0x000000, 0x800000, 0x008000, 0x808000, 0x000080, 0x800080, 0x008080, 0x808080,
        0x000000, 0xFF0000, 0x00FF00, 0xFFFF00, 0x0000FF, 0xFF00FF, 0x00FFFF, 0xFFFFFF,
        // to show sync signal
        0x005234, 0x800000, 0x008000, 0x808000, 0x000080, 0x800080, 0x008080, 0x808080,
        0x005234, 0xFF0000, 0x00FF00, 0xFFFF00, 0x0000FF, 0xFF00FF, 0x00FFFF, 0xFFFFFF
    };

    wchar_t error[1024];


////////////////////////////////////////////////////////////////////////////////
// FX2 code
////////////////////////////////////////////////////////////////////////////////

// read/write FX2 RAM (0x40 - write, 0xC0 - read)
int fx2_ram_readwrite ( uint16_t addr, uint8_t* buf, uint16_t length, uint8_t command )
{
    int chunk_size = 0x1000;
    while (length > 0)
    {
        if (length < chunk_size) chunk_size = length;
        int res = libusb_control_transfer(device_h, command, 0xA0, addr, 0, buf, chunk_size, 1000);
        if (res < 0) {
            wsprintfW(error, L"0x%X (%s) unable to perform operation 0x%X with FX2 (0x%X, 0x%X)", 
                res, libusb_error_name(res), 
                command, addr, chunk_size);
            return res;            
        }
        addr += chunk_size;
        buf += chunk_size;
        length -= chunk_size;
    }
    return 0;
}

// write data to FX2 RAM 
int fx2_ram_write ( uint16_t addr, uint8_t* buf, uint16_t length )
{
    return fx2_ram_readwrite(addr, buf, length, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT);
}

// read data from FX2 RAM
int fx2_ram_read ( uint16_t addr, uint8_t* buf, uint16_t length )
{
    return fx2_ram_readwrite(addr, buf, length, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN );
}

// put fx2 to stop or run (write 1 or 0 to address 0xE600)
int fx2_reset(struct libusb_device_handle *hdl, int is_stop)
{
    uint8_t buf[1];
    buf[0] = is_stop ? 1 : 0;
    return fx2_ram_write(0xE600, buf, 1);
}

// send start acquisition control
int fx2_send_start ()
{
    uint8_t cbuf[3] = {CMD_START_FLAGS_INV_CLK, 0, 0};
    int res = libusb_control_transfer(device_h, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, CMD_START, 0, 0, cbuf, 3, 1000);
    if (res < 0) {
        wsprintfW(error, L"0x%X (%s) unable to start fx2 data polling", res, libusb_error_name(res));
        return res;
    }
    return 0;
}

// read acquisition data (not async)
int fx2_data_read ( uint8_t* buf, uint32_t length)
{
    int res, readed;
    res = libusb_bulk_transfer(device_h, ENDPOINT, buf, length, &readed, 1000);
    if (res < 0) {
        wsprintfW(error, L"0x%X (%s) unable to read fx2 data", res, libusb_error_name(res));
        return res;
    }
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
// USB code
////////////////////////////////////////////////////////////////////////////////

// close all without even checking for errors
void usb_close ()
{
    if (device_h != 0) { 
        libusb_release_interface(device_h, 0); 
        libusb_close(device_h);
        libusb_exit(NULL);
        device_h = 0; 
    }
}

// init libusb, find FX2 device etc.
int usb_init (uint16_t vid, uint16_t pid)
{
    usb_close();
    libusb_init(NULL);
    // libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);   
    device_h = libusb_open_device_with_vid_pid(NULL, vid, pid);
    if (device_h == NULL) {
        wsprintfW(error, L"can't find usb device with VID:PID %04x:%04x (or 0xFFFF:0x2048)", VID, PID);
        return 1;
    }
	int res = libusb_set_configuration(device_h, 1);
    if (res != 0) {
		wsprintfW(error, L"0x%X (%s) can't set device configuration 1", res, libusb_error_name(res));
        return 2;
    }
    res = libusb_claim_interface(device_h, 0);
    if (res != 0) {
		wsprintfW(error, L"0x%X (%s) can't claim interface 0", res, libusb_error_name(res));
        return 3;
    }
    return 0;
}

// write firmware to fx2 device (restart it)
int usb_write_firmware ()
{
    // init usb with standard VID:PID for fx2
    int res = usb_init(VID, PID);
    if (res != 0) {
        // not found, lets try y-salnikov's modified (and we don't need to change firmware then)
        if (res == 1) res = usb_init(0xFFFF, 0x2048);
        if (res == 0) error[0] = 0x0000;
        return res;
    }
    // stop device
    res = fx2_reset(device_h, 1);
    if (res !=0) return res;
    // read firmware file and try to write to device RAM
    FILE* f = fopen(fw_filename, "rb");
    if (f == NULL) {
        wsprintfW(error, L"unable to open firmware file %s", fw_filename);
        return 1;
    }
    uint8_t* buf = (uint8_t*) malloc(0x2000);
    fread(buf, 1, 0x2000, f);
    res = fx2_ram_write(0, buf, 0x2000);
    fclose(f);
    free(buf);
    if (res != 0) return res;
    // start device then wait some to reinit usb
    res = fx2_reset(device_h, 0);
    if (res != 0) return res;
    Sleep(3000);
    return usb_init(VID, PID);
}

void LIBUSB_CALL cb_transfer_complete (libusb_transfer *t);

// init bulk transfer struct and send it
int add_transfer()
{
    uint8_t *buf = (uint8_t *) malloc(TR_CHUNK_SIZE);
    struct libusb_transfer *t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, device_h, ENDPOINT, buf, TR_CHUNK_SIZE, cb_transfer_complete, (void*)1, 100);
    int res = libusb_submit_transfer(t);
    if (res < 0) {
        wsprintfW(error, L"0x%X (%s) unable to submit usb data transfer", res, libusb_error_name(res));
        return res;
    }
    return 0;
}

// process usb events thread
DWORD WINAPI thread_usb_events (LPVOID lpParam)
{
    struct timeval zero_tv = {0, 0};
    while (stop == 0) {
        libusb_handle_events_timeout(NULL, &zero_tv);
    }
    return 0;
}

////////////////////////////////////////
// callback function for bulk transfer
//////////////////////////////////////

void LIBUSB_CALL cb_transfer_complete (libusb_transfer *t)
{
    if (t == NULL) return;
    if (stop != 0) return;
    if (t->actual_length == 0) {
        // it can be timeout or whatever, signal to halt application
        if (++errors_count > 10) {
            wsprintfW(error, L"too many empty transfers (successed=%i)", handled_count);
            stop = 1;
            return;
        }
    } else {
        ++handled_count;
    }
    // process pixel data
    volatile uint32_t* screen_buf = scr_buffers[scr_n_cur];
    uint8_t bmask = scr_show_sync ? 0x1F : 0x0F;
    //
    for (int i=0; i<t->actual_length; i++)
    {
        uint8_t b = t->buffer[i] ^ 0xFF; // signals are inverted
        uint32_t dw = fx2data2grb[b & bmask];
        if (b == 0) {
            scr_lsync_cnt++;
        } else {
            // sort of hsync, exact 0x40 low sync signals
            if (scr_lsync_cnt == 0x40) {
                scr_cur_addr = 0x320 * (scr_cur_addr / 0x320);
            } else
            // sort of vsync, exact 0x20 low sync signals
            // to be 100% sure - change to >=0xC0 and adjust current addr with another value
            if (scr_lsync_cnt == 0x20) {
                scr_cur_addr = UKNC_SCR_FULL-0x960-0x40-0x320*6; // for centering
            }
            scr_lsync_cnt = 0;
        }
        screen_buf[scr_cur_addr++] = dw;
        if (scr_cur_addr >= UKNC_SCR_FULL) {
            scr_cur_addr = 0;
            scr_n_cur = ++scr_n_cur & 0x07;
            screen_buf = scr_buffers[scr_n_cur];
        }
    }    
    // resubmit transfer
    int res = libusb_submit_transfer(t);
    if (res < 0) {
        wsprintfW(error, L"0x%X (%s) unable to submit usb data transfer", res, libusb_error_name(res));
        stop = 1;
    }
}


////////////////////////////////////////////////////////////////////////////////
// Windows 3.11 for workgroups ^_^ stuff
////////////////////////////////////////////////////////////////////////////////

    LPCWSTR sMainClass    = L"DT_UKNC_FX2";
    LPCWSTR sMainCaption  = L"UKNC FX2";
    LPCWSTR sErrorCaption = L"UKNC FX2 ERROR";

    HINSTANCE       hMainInstance;
    HWND            hMain;
    HFONT           hFont;
    HMENU           hMenuOptions;
    HANDLE          hUsbThread, hRenderThread;

    int W_X  = 300;
    int W_Y  = 200;
    int W_DX = 800;
    int W_DY = 624;

    const int IDM_SHOW_SYNC = 1;

    uint32_t    ntimes[1024];
    uint32_t    ttimes[1024];
    uint32_t    idx_times;


// (helper) creates child window with some style
HWND helpCreateChild (LPCWSTR sclass, LPCWSTR caption, DWORD style, int x, int y, int dx, int dy, DWORD ext)
{
    HWND h = CreateWindowExW(ext, sclass, caption, WS_CHILD | WS_VISIBLE | style, x, y, dx, dy, hMain, NULL, hMainInstance, NULL);
    if (!h) return NULL;
    SendMessageW(h, WM_SETFONT, (WPARAM)hFont, 0);
    return h;
}


// paint UKNC picture on main window
void PaintUkncScreen (int nbuf)
{
    if (stop == 1) return;
    BITMAPINFO info;
    memset(&info, 0, sizeof(BITMAPINFO));
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biWidth = 800;
    info.bmiHeader.biHeight = -312;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biSizeImage = 0;
    info.bmiHeader.biCompression = BI_RGB;
    HDC dc = GetDC(hMain);    
    StretchDIBits(dc, 0, 0, 800, 624, 0, 0, 800, 312, (void *)(scr_buffers[nbuf]), &info, DIB_RGB_COLORS, SRCCOPY);
    ReleaseDC( hMain, dc );
}


// render pic in separate thread
DWORD WINAPI RenderThreadProc (LPVOID lpParam)
{
    LONGLONG rtStart;
    while (stop == 0) {
        uint32_t n = scr_n_cur; // (scr_n_cur-1)&0x07;
        while (n == scr_n_cur) {}
        PaintUkncScreen(n);
        //
        SYSTEMTIME st; GetSystemTime(&st);
        ntimes[idx_times] = n;
        ttimes[idx_times] = st.wMilliseconds;
        idx_times = ++idx_times & 1023;
    }
    return 0;
}


// processing messages for all windows in class
//
LONG MainWndProc (HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
    if (hwnd != hMain) return(DefWindowProc( hwnd, msg, wparam, lparam ));
    switch( msg )
    {
        case WM_COMMAND:
            switch (LOWORD(wparam)) {
                case IDM_SHOW_SYNC:
                    scr_show_sync = 1 - scr_show_sync;
                    CheckMenuItem(hMenuOptions, IDM_SHOW_SYNC, scr_show_sync ? MF_CHECKED : MF_UNCHECKED);
                    break;
            }
            break;
        case WM_SIZE: 
            if (wparam==SIZE_MINIMIZED) {}
            break;
        case WM_MOVE: 
            W_X = (int) LOWORD(lparam);
            W_Y = (int) HIWORD(lparam);
            return(0L);
        // timer ticks
        // TODO: check device health and try to restart it if something happened
        // case WM_TIMER:
        //     return( 0L );
        case WM_DESTROY: 
            PostQuitMessage(0);
            return( 0L );
        default: return( DefWindowProc( hwnd, msg, wparam, lparam ) );
    }
    // if not processed - process by Windows
    return( DefWindowProcW( hwnd, msg, wparam, lparam ) );
}


// start usb acquisition
//
int StartUsbProcess ()
{
    // allocate 8 buffers for receiving screens
    for (int i=0; i<8; i++) 
        scr_buffers[i] = (uint32_t*) malloc(UKNC_SCR_FULL*sizeof(uint32_t));
    // start usb 
    int res = usb_write_firmware();
    if (res != 0) return res;
    hUsbThread = CreateThread(NULL, 0, thread_usb_events, 0, 0, NULL);
    SetThreadPriority(hUsbThread, THREAD_PRIORITY_ABOVE_NORMAL);
    for (int i=0; i<4; i++) {
        res = add_transfer();
        if (res != 0) return res;
    }
    return fx2_send_start();
}


// Initialize windows
//
void InitWindows (void)
{
    HWND hwnd;
    char s[40];
    RECT rect = {W_X, W_Y, W_X+W_DX, W_Y+W_DY};
    DWORD style = WS_CAPTION | WS_MINIMIZEBOX | WS_SYSMENU | WS_VISIBLE;
    AdjustWindowRectEx(&rect, style, /*menu presence*/true, NULL);
    hFont = CreateFontW(18,0, 0,0, FW_NORMAL, 0,0,0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, VARIABLE_PITCH, L"Tahoma" );
    hMain = CreateWindowExW(0, sMainClass, sMainCaption, style,
                            rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top,
                            NULL, 
                            NULL,
                            hMainInstance,
                            NULL);
    if (hMain == NULL) {
        MessageBoxW(NULL, L"Unable to create main window (how it can be?)", sErrorCaption, MB_OK);
        ExitProcess(1);
    }
    // menus
    hMenuOptions = CreateMenu();
    AppendMenuW(hMenuOptions, MF_STRING, IDM_SHOW_SYNC, L"Show sync signal");
    HMENU hMenubar = CreateMenu();
    AppendMenuW(hMenubar, MF_POPUP, (UINT_PTR)hMenuOptions, L"Options");
    SetMenu(hMain, hMenubar);
    // rendering
    hRenderThread = CreateThread(NULL, 0, RenderThreadProc, 0, 0, NULL);
    // TODO: use timer for check FX2 is alive and try to (re)start it if not
    // SetTimer(hMain, 1/*Timer ID*/, 1000, NULL);
}


// main entry point
//
int WinMain (HINSTANCE this_inst, HINSTANCE prev_inst, LPSTR cmdline, int cmdshow)
{
    MSG          msg;
    WNDCLASSEXW  wcx;

    // Hey! That's system wide function on old windows!
    timeBeginPeriod(1);

    // store application handle
    hMainInstance = this_inst;

    // register application local class
    wcx.cbSize = sizeof(wcx);
    wcx.style  = 0;
    wcx.lpfnWndProc = (WNDPROC)MainWndProc;
    wcx.cbClsExtra  = 0;
    wcx.cbWndExtra  = 0;
    wcx.hInstance   = this_inst;
    wcx.hIcon   = LoadIconW(NULL, IDI_WINLOGO);
    wcx.hCursor = LoadCursorW(NULL, IDC_ARROW);
    wcx.hbrBackground = NULL;
    wcx.lpszMenuName  = NULL;
    wcx.lpszClassName = sMainClass;
    wcx.hIconSm = NULL;
    if (!RegisterClassExW(&wcx)) {
        MessageBoxW(NULL, L"Unable to register class for this application", sErrorCaption, MB_OK);
        ExitProcess(1);
    }

    // initialize window
    InitWindows();
    // start fx2 acquisition
    if (StartUsbProcess() != 0) stop = 1;

    // message loop
    while (GetMessageW(&msg, (HWND)NULL, 0, 0))
    {
        // if (msg.message == WM_KEYDOWN) ...
        // if (msg.message == WM_KEYUP) ...
        DispatchMessageW(&msg);
        if (error[0]) {
            HWND h = helpCreateChild(L"STATIC", error, SS_CENTER|SS_CENTERIMAGE, 0, 0, W_DX, W_DY, 0);
            error[0] = 0;
        }
    }

    // cleanup ... well - let's windows do it 
    stop = 1;
    timeEndPeriod(1);
    Sleep(100);
    usb_close();

    // TODO: save config
    return 0;
}


/*
    Direct2D version:

    #include <d2d1.h>
    #include <d2d1helper.h>
    #pragma comment(lib, "d2d1.lib")
    #pragma comment(lib, "dxguid.lib")

    Declarations
    ============    
        ID2D1Factory* factory;
        ID2D1DeviceContext* deviceContext;
        ID2D1HwndRenderTarget* renderTarget;
        ID2D1BitmapRenderTarget* bitmapTarget;
        ID2D1Bitmap* bitmap;

    Init phase
    ==========
        // Direct2D factory
        HRESULT hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &factory);
        D2D1_PIXEL_FORMAT pf = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        D2D1_RENDER_TARGET_PROPERTIES props = D2D1::RenderTargetProperties();
        props.pixelFormat = pf;
        RECT rc;
        GetClientRect(hMain, &rc);
        D2D1_SIZE_U size = D2D1::SizeU(800, 620);
        factory->CreateHwndRenderTarget(props, D2D1::HwndRenderTargetProperties(hMain, size), &renderTarget);
        renderTarget->QueryInterface(&deviceContext);
        renderTarget->CreateCompatibleRenderTarget(&bitmapTarget);

    Rendering
    =========
        bitmapTarget->BeginDraw();
        bitmapTarget->GetBitmap(&bitmap);
        D2D1_RECT_U dstRect = D2D1::RectU(0,0,800,310);
        HRESULT hr = bitmap->CopyFromMemory(&dstRect, screen_buf, 800*4);
        bitmapTarget->EndDraw();
        deviceContext->BeginDraw();
        deviceContext->DrawImage(bitmap, D2D1_INTERPOLATION_MODE_LINEAR, D2D1_COMPOSITE_MODE_SOURCE_COPY);
        hr  = deviceContext->EndDraw();
*/

/* 
    Save video
    ==========

#include <mfapi.h>
#include <mfidl.h>
#include <Mfreadwrite.h>
#include <mferror.h>
#pragma comment(lib, "mfreadwrite")
#pragma comment(lib, "mfplat")
#pragma comment(lib, "mfuuid")

template <class T> void SafeRelease(T **ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}


// Format constants
const UINT32 VIDEO_WIDTH = 800;
const UINT32 VIDEO_HEIGHT = 312;
const UINT32 VIDEO_FPS = 50;
const UINT64 VIDEO_FRAME_DURATION = 10 * 1000 * 1000 / VIDEO_FPS;
const UINT32 VIDEO_BIT_RATE = 800000;
const GUID   VIDEO_ENCODING_FORMAT = MFVideoFormat_WMV3;
const GUID   VIDEO_INPUT_FORMAT = MFVideoFormat_RGB32;
const UINT32 VIDEO_PELS = VIDEO_WIDTH * VIDEO_HEIGHT;

    IMFSinkWriter* pSinkWriter = NULL;
    DWORD stream;

HRESULT InitializeSinkWriter(IMFSinkWriter **ppWriter, DWORD *pStreamIndex)
{
    *ppWriter = NULL;
    *pStreamIndex = NULL;
    IMFSinkWriter   *pSinkWriter = NULL;
    IMFMediaType    *pMediaTypeOut = NULL;   
    IMFMediaType    *pMediaTypeIn = NULL;   
    DWORD           streamIndex;
    HRESULT hr = MFCreateSinkWriterFromURL(L"output.wmv", NULL, NULL, &pSinkWriter);
    if (SUCCEEDED(hr)) hr = MFCreateMediaType(&pMediaTypeOut);   
    if (SUCCEEDED(hr)) hr = pMediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);     
    if (SUCCEEDED(hr)) hr = pMediaTypeOut->SetGUID(MF_MT_SUBTYPE, VIDEO_ENCODING_FORMAT);   
    if (SUCCEEDED(hr)) hr = pMediaTypeOut->SetUINT32(MF_MT_AVG_BITRATE, VIDEO_BIT_RATE);   
    if (SUCCEEDED(hr)) hr = pMediaTypeOut->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);   
    if (SUCCEEDED(hr)) hr = MFSetAttributeSize(pMediaTypeOut, MF_MT_FRAME_SIZE, VIDEO_WIDTH, VIDEO_HEIGHT);   
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_FRAME_RATE, VIDEO_FPS, 1);   
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);   
    if (SUCCEEDED(hr)) hr = pSinkWriter->AddStream(pMediaTypeOut, &streamIndex);   
    if (SUCCEEDED(hr)) hr = MFCreateMediaType(&pMediaTypeIn);   
    if (SUCCEEDED(hr)) hr = pMediaTypeIn->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);   
    if (SUCCEEDED(hr)) hr = pMediaTypeIn->SetGUID(MF_MT_SUBTYPE, VIDEO_INPUT_FORMAT);     
    if (SUCCEEDED(hr)) hr = pMediaTypeIn->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);   
    if (SUCCEEDED(hr)) hr = MFSetAttributeSize(pMediaTypeIn, MF_MT_FRAME_SIZE, VIDEO_WIDTH, VIDEO_HEIGHT);   
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(pMediaTypeIn, MF_MT_FRAME_RATE, VIDEO_FPS, 1);   
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(pMediaTypeIn, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);   
    if (SUCCEEDED(hr)) hr = pSinkWriter->SetInputMediaType(streamIndex, pMediaTypeIn, NULL);   
    if (SUCCEEDED(hr)) hr = pSinkWriter->BeginWriting();
    if (SUCCEEDED(hr))
    {
        *ppWriter = pSinkWriter;
        (*ppWriter)->AddRef();
        *pStreamIndex = streamIndex;
    }
    SafeRelease(&pSinkWriter);
    SafeRelease(&pMediaTypeOut);
    SafeRelease(&pMediaTypeIn);
    return hr;
}

HRESULT WriteFrame(IMFSinkWriter *pWriter, DWORD streamIndex, const LONGLONG& rtStart, int n)
{
    IMFSample *pSample = NULL;
    IMFMediaBuffer *pBuffer = NULL;
    const LONG cbWidth = 4 * VIDEO_WIDTH;
    const DWORD cbBuffer = cbWidth * VIDEO_HEIGHT;
    BYTE *pData = NULL;
    // Create a new memory buffer.
    HRESULT hr = MFCreateMemoryBuffer(cbBuffer, &pBuffer);
    // Lock the buffer and copy the video frame to the buffer.
    if (SUCCEEDED(hr)) hr = pBuffer->Lock(&pData, NULL, NULL);
    if (SUCCEEDED(hr))
        hr = MFCopyImage(
            pData,                      // Destination buffer.
            cbWidth,                    // Destination stride.
            (BYTE*)scr_buffers[n],      // First row in source image.
            cbWidth,                    // Source stride.
            cbWidth,                    // Image width in bytes.
            VIDEO_HEIGHT                // Image height in pixels.
            );
    if (pBuffer) pBuffer->Unlock();
    // Set the data length of the buffer.
    if (SUCCEEDED(hr)) hr = pBuffer->SetCurrentLength(cbBuffer);
    // Create a media sample and add the buffer to the sample.
    if (SUCCEEDED(hr)) hr = MFCreateSample(&pSample);
    if (SUCCEEDED(hr)) hr = pSample->AddBuffer(pBuffer);
    if (SUCCEEDED(hr)) hr = pSample->SetSampleTime(rtStart);
    if (SUCCEEDED(hr)) hr = pSample->SetSampleDuration(VIDEO_FRAME_DURATION);
    if (SUCCEEDED(hr)) hr = pWriter->WriteSample(streamIndex, pSample);
    SafeRelease(&pSample);
    SafeRelease(&pBuffer);
    return hr;
}

    HRESULT hr = MFStartup(MF_VERSION);
    hr = InitializeSinkWriter(&pSinkWriter, &stream);
    if (FAILED(hr)) {
        exit(1);
    }

    ... WriteFrame ...

    hr = pSinkWriter->Finalize();
    SafeRelease(&pSinkWriter);
    MFShutdown();

*/

