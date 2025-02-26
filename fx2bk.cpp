// compile: 
// 1) setup visual studio dev console
// 2) cl /O2 fx2bk.cpp 

#define UNICODE

#include <windows.h>
#include <process.h>
#include <stdio.h>
#include "lib/libusb.h"

#pragma comment(lib, "lib/libusb-1.0.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "winmm.lib")

#define MODE_BK         0
#define MODE_UKNC       1

#define B_SCR_WIDTH     0x00300     // (BK0011M) 768 pix clk in line
#define B_SCR_HEIGHT    0x00140     // (BK0011M) 320 lines
#define B_SCR_FULL      0x3C000     // (BK0011M) 245760 pix clk in full screen

#define U_SCR_WIDTH     0x00320     // (UKNC) 800 pix clk in line
#define U_SCR_HEIGHT    0x00138     // (UKNC) 312 lines
#define U_SCR_FULL      0x3CF00     // (UKNC) 249600 pix clk in full screen

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
    uint32_t  scr_cur_addr  = 0;
    uint32_t  scr_lsync_cnt = 0;
    uint8_t   scr_show_sync = 0;

    int scr_mode   = MODE_UKNC;     // default mode to BK
    int scr_width  = U_SCR_WIDTH;
    int scr_height = U_SCR_HEIGHT;
    int scr_full   = U_SCR_FULL;

    int stop = 0;                   // encountered an error somewhere
    int nactive = 0;                // active transfers count
    int handled_count = 0;          // count of processed usb bulk transfers
    int errors_count = 0;           // count of not processed

    // BK palettes
    uint8_t palette = 1;
    uint32_t palette_data[] = {
        0x000000, 0x0000FF, 0x00FF00, 0xFF0000, // 0 - (special) black/white palette
        0x000000, 0x0000FF, 0x00FF00, 0xFF0000, // 1 - std palette 0
        0x000000, 0xFFFF00, 0xFF00FF, 0xFF0000, // .. etc
        0x000000, 0x00FFFF, 0x0000FF, 0xFF00FF,
        0x000000, 0x00FF00, 0x00FFFF, 0xFFFF00,
        0x000000, 0xFF00FF, 0x00FFFF, 0xFFFFFF,
        0x000000, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF,
        0x000000, 0xC00000, 0x900000, 0xFF0000,
        0x000000, 0xC0FF00, 0x90FF00, 0xFFFF00,
        0x000000, 0xC000FF, 0x9000FF, 0xFF00FF,
        0x000000, 0x90FF00, 0x9000FF, 0x900000,
        0x000000, 0xC0FF00, 0xC000FF, 0xC00000,
        0x000000, 0x00FFFF, 0xFFFF00, 0xFF0000,
        0x000000, 0xFF0000, 0x00FF00, 0x00FFFF,
        0x000000, 0x00FFFF, 0xFFFF00, 0xFFFFFF,
        0x000000, 0xFFFF00, 0x00FF00, 0xFFFFFF,
        0x000000, 0x00FFFF, 0x00FF00, 0xFFFFFF
    };

    // UKNC palette
    uint32_t palette_uknc[16] = {
        0x000000, 0x800000, 0x008000, 0x808000, 0x000080, 0x800080, 0x008080, 0x808080,
        0x000000, 0xFF0000, 0x00FF00, 0xFFFF00, 0x0000FF, 0xFF00FF, 0x00FFFF, 0xFFFFFF
    };

    char error[1024];


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
            sprintf(error, "0x%X (%s) unable to perform operation 0x%X with FX2 (0x%X, 0x%X)", 
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
    int res = libusb_control_transfer(device_h, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, CMD_START, 0, 0, cbuf, 3, 100);
    if (res < 0) {
        sprintf(error, "0x%X (%s) unable to start fx2 data polling", res, libusb_error_name(res));
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
        sprintf(error, "0x%X (%s) unable to read fx2 data", res, libusb_error_name(res));
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
    if (device_h) { 
        libusb_release_interface(device_h, 0); 
        libusb_close(device_h);
        // libusb_exit(NULL);
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
        sprintf(error, "can't find usb device with VID:PID %04x:%04x (or 0xFFFF:0x2048)", VID, PID);
        return 1;
    }
	int res = libusb_set_configuration(device_h, 1);
    if (res != 0) {
		sprintf(error, "0x%X (%s) can't set device configuration 1", res, libusb_error_name(res));
        return 2;
    }
    res = libusb_claim_interface(device_h, 0);
    if (res != 0) {
		sprintf(error, "0x%X (%s) can't claim interface 0", res, libusb_error_name(res));
        return 3;
    }
    return 0;
}

// write firmware to fx2 device (restart it)
int usb_write_firmware ()
{
    // init usb with standard VID:PID for fx2
    int res = usb_init(VID, PID); // 04B4:8613
    if (res != 0) {
        // not found, lets try y-salnikov's modified (and we don't need to change firmware then)
        if (res == 1) res = usb_init(0xFFFF, 0x2048);
        if (res == 0) error[0] = 0x00;
        return res;
    }
    // stop device
    res = fx2_reset(device_h, 1);
    if (res !=0) return res;
    // read firmware file and try to write to device RAM
    FILE* f = fopen(fw_filename, "rb");
    if (f == NULL) {
        sprintf(error, "unable to open firmware file %s", fw_filename);
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
    // init with y-salnikov's now
    return usb_init(0xFFFF, 0x2048);
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
        sprintf(error, "0x%X (%s) unable to submit usb data transfer", res, libusb_error_name(res));
        return res;
    }
    nactive++;
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
    nactive--;
    if (t == NULL) return;
    if (stop) return;
    if (t->actual_length == 0) {
        // it can be timeout or whatever
        // if (++errors_count > 10) {
        //     sprintf(error, "too many empty transfers (successed=%i)", handled_count);
        //     stop = 1;
        //     return;
        // }
        return;
    } else {
        ++handled_count;
    }
    // process pixel data
    volatile uint32_t* screen_buf = scr_buffers[scr_n_cur];
    for (int i=0; i<t->actual_length; i++)
    {
        // byte of data
        uint8_t b = t->buffer[i] ^ 0xFF;
        // filter it just in case
        b = (scr_mode==MODE_BK ? (b & 0x13) : (b & 0x1F));
        // color dword
        uint32_t dw = (scr_mode==MODE_BK ? (palette_data[(palette<<2) | (b&3)]) : palette_uknc[b&0xF]);
        // sync presence (taken inverted in UKNC)
        bool have_sync = (scr_mode==MODE_BK ? (b == 0x10) : (b == 0x00));        
        if (have_sync) {
            if (scr_show_sync) dw = dw | 0x808080;
            scr_lsync_cnt++;
        } else {
            // BK mode
            if (scr_mode == MODE_BK)
            {
                // sort of hsync, exact 0x38 low sync signals
                // seems BK is stable without using hsync (UKNC is not!)
                //if (scr_lsync_cnt == 0x38) {
                //    scr_cur_addr = 0x300 * (scr_cur_addr / 0x300);
                //} else
                // sort of vsync, exact 0x50 low sync signals
                if (scr_lsync_cnt == 0x50) {
                    scr_cur_addr = B_SCR_FULL - 0x38 - B_SCR_WIDTH*10; // for centering
                }
            // UKNC mode
            } else {
                // sort of hsync, exact 0x40 low sync signals
                if (scr_lsync_cnt == 0x40) {
                    scr_cur_addr = U_SCR_WIDTH * (scr_cur_addr / U_SCR_WIDTH);
                } else
                // sort of vsync, exact 0x20 low sync signals
                // to be 100% sure - change to >=0xC0 and adjust current addr with another value
                if (scr_lsync_cnt == 0x20) {
                    scr_cur_addr = U_SCR_FULL - 0x40 - U_SCR_WIDTH*9; // for centering
                }
            }
            scr_lsync_cnt = 0;
        }
        screen_buf[scr_cur_addr++] = dw;
        if (scr_cur_addr >= scr_full) {
            scr_cur_addr = 0;
            scr_n_cur = ++scr_n_cur & 0x07;
            screen_buf = scr_buffers[scr_n_cur];
        }
    }
    // resubmit transfer
    int res = libusb_submit_transfer(t);
    if (res < 0) {
        sprintf(error, "0x%X (%s) unable to submit usb data transfer", res, libusb_error_name(res));
        stop = 1;
    } else {
        nactive++;
    }
}


////////////////////////////////////////////////////////////////////////////////
// Windows 3.11 for workgroups ^_^ stuff
////////////////////////////////////////////////////////////////////////////////

    LPCWSTR sMainClass    = L"DT_BK_FX2";
    LPCWSTR sMainCaption  = L"BK FX2";
    LPCWSTR sErrorCaption = L"BK FX2 ERROR";

    HINSTANCE       hMainInstance;
    HWND            hMain, hError;
    HFONT           hFont;
    HMENU           hMenuOptions/*, hMenuSavebin*/;
    HANDLE          hUsbThread, hRenderThread;

    int W_X  = 300;
    int W_Y  = 200;
    int W_DX = scr_width;
    int W_DY = scr_height*2;

    const int IDM_SHOW_SYNC = 1;
    const int IDM_SAVE_SIG  = 2;
    const int IDM_SAVESCR   = 4;

    const int IDM_PALETTEBW  = 0x0F;
    const int IDM_PALETTE00  = 0x10;
    const int IDM_PALETTE01  = 0x11;
    const int IDM_PALETTE02  = 0x12;
    const int IDM_PALETTE03  = 0x13;
    const int IDM_PALETTE04  = 0x14;
    const int IDM_PALETTE05  = 0x15;
    const int IDM_PALETTE06  = 0x16;
    const int IDM_PALETTE07  = 0x17;
    const int IDM_PALETTE08  = 0x18;
    const int IDM_PALETTE09  = 0x19;
    const int IDM_PALETTE10  = 0x1A;
    const int IDM_PALETTE11  = 0x1B;
    const int IDM_PALETTE12  = 0x1C;
    const int IDM_PALETTE13  = 0x1D;
    const int IDM_PALETTE14  = 0x1E;
    const int IDM_PALETTE15  = 0x1F;

    uint32_t    nLastBuf = 0;
    
    wchar_t     wError[1024];
    wchar_t     wcsTemp[256];

    uint32_t    ntimes[1024];
    uint32_t    ttimes[1024];
    uint32_t    idx_times;


// obviously writes .bmp
int WriteBmp()
{
    tagBITMAPFILEHEADER header;
    tagBITMAPINFOHEADER info;
    header.bfType = 0x4d42; // magic sequence 'BM'
    header.bfSize = sizeof(tagBITMAPFILEHEADER);
    header.bfOffBits = sizeof(tagBITMAPINFOHEADER) + sizeof(tagBITMAPFILEHEADER);
    info.biSize = sizeof(tagBITMAPINFOHEADER);
    info.biWidth = scr_width;
    info.biHeight = scr_height*2;
    info.biPlanes = 1;
    info.biBitCount = 24;
    info.biSizeImage = info.biWidth*info.biHeight;
    info.biCompression = 0;
    FILE* f = fopen("screenshot.bmp", "wb");
    if (f == NULL) return 1;
    fwrite(&header, 1, sizeof(header), f);
    fwrite(&info, 1, sizeof(info), f);
    for (int u=scr_full-scr_width; u>=0; u-=scr_width) 
    {
        uint32_t* data = (uint32_t*) scr_buffers[nLastBuf];
        for (int v=0; v<scr_width; v++) fwrite(&data[u+v], 1, 3, f);
        for (int v=0; v<scr_width; v++) fwrite(&data[u+v], 1, 3, f);
    }
    fclose(f);
    return 0;
}


// (helper) creates child window with some style
HWND helpCreateChild (LPCWSTR sclass, LPCWSTR caption, DWORD style, int x, int y, int dx, int dy, DWORD ext)
{
    HWND h = CreateWindowExW(ext, sclass, caption, WS_CHILD | WS_VISIBLE | style, x, y, dx, dy, hMain, NULL, hMainInstance, NULL);
    if (!h) return NULL;
    SendMessageW(h, WM_SETFONT, (WPARAM)hFont, 0);
    return h;
}


// paint picture on main window
void PaintScreen (int nbuf)
{
    if (stop == 1) return;
    BITMAPINFO info;
    memset(&info, 0, sizeof(BITMAPINFO));
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biWidth = scr_width;
    info.bmiHeader.biHeight = 0-scr_height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biSizeImage = 0;
    info.bmiHeader.biCompression = BI_RGB;
    HDC dc = GetDC(hMain);    
    StretchDIBits(dc, 0, 0, scr_width, scr_height*2, 0, 0, scr_width, scr_height, (void *)(scr_buffers[nbuf]), &info, DIB_RGB_COLORS, SRCCOPY);
    ReleaseDC( hMain, dc );
}


// render pic in separate thread
DWORD WINAPI RenderThreadProc (LPVOID lpParam)
{
    while (stop == 0) {
        uint32_t n = scr_n_cur;
        while (n == scr_n_cur) {}
        nLastBuf = n;
        volatile uint32_t *buf = scr_buffers[n];
        // black & white mode?
        if (palette == 0) {
            for (uint32_t u=0; u<scr_full; u+=2) {
                uint32_t b1 = (buf[u] & 0x00000F) ? 0xFFFFFF : 0x000000;
                uint32_t b2 = (buf[u] & 0x000F00) ? 0xFFFFFF : 0x000000;
                if (buf[u] & 0x0F0000) { b1=0xFFFFFF; b2=0xFFFFFF; }
                buf[u] = b1;
                buf[u+1] = b2;
            }
        }
        //
        PaintScreen(n);
        //
        SYSTEMTIME st; GetSystemTime(&st);
        ntimes[idx_times] = n;
        ttimes[idx_times] = st.wMilliseconds;
        idx_times = ++idx_times & 1023;
    }
    return 0;
}


// start usb acquisition
//
int StartUsbProcess ()
{
    // allocate 8 buffers for receiving screens
    for (int i=0; i<8; i++) 
        scr_buffers[i] = (uint32_t*) malloc(scr_full*sizeof(uint32_t));
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


// processing messages for all windows in class
//
LONG MainWndProc (HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
    if (hwnd == hMain) switch( msg )
    {
        // usually menu
        case WM_COMMAND:
            switch (LOWORD(wparam)) {
                case IDM_SHOW_SYNC:
                    scr_show_sync = 1 - scr_show_sync;
                    CheckMenuItem(hMenuOptions, IDM_SHOW_SYNC, scr_show_sync ? MF_CHECKED : MF_UNCHECKED);
                    break;
                //case IDM_SAVEBIN:
                //    MessageBoxW(hMain, L"Signal data saved to signal.bin", L"Info", MB_OK);
                //    break;
                // save screen from current-1 buffer
                case IDM_SAVESCR:
                    WriteBmp();
                    MessageBoxW(hMain, L"Screenshot written to file screenshot.bmp", L"Info", MB_OK);
                    break;
            }
            // palettes menu
            if ((LOWORD(wparam) >= IDM_PALETTEBW) && (LOWORD(wparam) <= IDM_PALETTE15)) 
            {
                palette = LOWORD(wparam) - IDM_PALETTEBW;
                for (int i=IDM_PALETTEBW; i<=IDM_PALETTE15; i++) CheckMenuItem(hMenuOptions, i, MF_UNCHECKED);
                CheckMenuItem(hMenuOptions, LOWORD(wparam), MF_CHECKED);
            }
            break;
        // minimize/maximize/resize
        case WM_SIZE: 
            if (wparam==SIZE_MINIMIZED) {}
            break;
        // move
        case WM_MOVE: 
            W_X = (int) LOWORD(lparam);
            W_Y = (int) HIWORD(lparam);
            return 0L;
        // timer ticks - check device health and try to restart it if something happened
        case WM_TIMER:
            if (stop==0 && nactive<=0) {
                nactive = 0;
                int res = usb_write_firmware();
                if (res) return 0L;
                for (int i=0; i<4; i++) add_transfer();
                fx2_send_start();
            }
            return 0L;
        // the end
        case WM_DESTROY: 
            PostQuitMessage(0);
            return 0L;
    }
    // default process by Windows
    return DefWindowProcW( hwnd, msg, wparam, lparam );
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
    AppendMenuW(hMenuOptions, MF_SEPARATOR, 0, 0);
    AppendMenuW(hMenuOptions, MF_STRING, IDM_PALETTEBW, L"Black & white");
    for (int i=IDM_PALETTE00; i<=IDM_PALETTE15; i++) {
        wsprintf(wcsTemp, L"Palette %i", i-IDM_PALETTE00);
        AppendMenuW(hMenuOptions, MF_STRING, i, wcsTemp);
    }
    CheckMenuItem(hMenuOptions, IDM_PALETTEBW+palette, MF_CHECKED);
    AppendMenuW(hMenuOptions, MF_SEPARATOR, 0, 0);
    AppendMenuW(hMenuOptions, MF_STRING, IDM_SAVESCR, L"Save screenshot");
    // AppendMenuW(hMenuOptions, MF_STRING, IDM_SAVE_SIG, L"Save signal binary");
    HMENU hMenubar = CreateMenu();
    AppendMenuW(hMenubar, MF_POPUP, (UINT_PTR)hMenuOptions, L"Options");
    SetMenu(hMain, hMenubar);
    // rendering
    hRenderThread = CreateThread(NULL, 0, RenderThreadProc, 0, 0, NULL);
    // TODO: use timer for check FX2 is alive and try to (re)start it if not
    SetTimer(hMain, 1/*Timer ID*/, 5000, NULL);
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
    stop = StartUsbProcess();

    // message loop
    while (GetMessageW(&msg, (HWND)NULL, 0, 0))
    {
        // if (msg.message == WM_KEYDOWN) ...
        // if (msg.message == WM_KEYUP) ...
        DispatchMessageW(&msg);
        if (stop!=0 && error[0]) {
            // show error
            if (hError == 0) hError = helpCreateChild(L"STATIC", L"", SS_CENTER|SS_CENTERIMAGE, 0, 0, W_DX, W_DY, 0);
            mbstowcs(wError, error, 1024);
            SendMessageW(hError, WM_SETTEXT, 0, (LPARAM)wError);
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

