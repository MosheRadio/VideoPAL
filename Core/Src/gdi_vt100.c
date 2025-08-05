#include "vt100.h"
#include "gdi.h"
#include <stdint.h>
#include <string.h>

// VT100 cursor position
static int cursor_x = 0;
static int cursor_y = 0;

// Set cursor position
void gdiSetCursor(int x, int y) {
    cursor_x = x;
    cursor_y = y;
}

// Get cursor position
void gdiGetCursor(int *x, int *y) {
    if (x) *x = cursor_x;
    if (y) *y = cursor_y;
}

// Clear screen
void gdiClearScreen(void) {
    // Fill the screen with spaces or clear buffer
    // Implement according to your PAL
    // Example: for (int y = 0; y < VID_PIXELS_Y; y++)
    //              for (int x = 0; x < VID_PIXELS_X; x++)
    //                  gdiDrawTextEx(x, y, " ");
}

// Write a character at the current cursor position
void gdiPutChar(uint8_t ch) {
    char str[2] = { (char)ch, 0 };
    gdiDrawTextEx(cursor_x, cursor_y, str);
    cursor_x += GDI_SYSFONT_WIDTH; // Advance cursor
    // Optionally handle line wrap and scrolling
}

// Handle VT100 CSI commands
void gdiHandleCSI(const char *buf, uint8_t len) {
    // Example: ESC [ <row> ; <col> H (move cursor)
    if (buf[len-1] == 'H' || buf[len-1] == 'f') {
        int row = 1, col = 1;
        sscanf(buf, "%d;%d", &row, &col);
        gdiSetCursor(col * GDI_SYSFONT_WIDTH, row * GDI_SYSFONT_HEIGHT);
    } else if (buf[len-1] == 'J') {
        // ESC [ J (clear screen)
        gdiClearScreen();
        gdiSetCursor(0, 0);
    }
    // Add more VT100 CSI command handling as needed
}

// Feed a character to VT100 parser and handle with PAL
void gdiVT100Feed(uint8_t ch) {
    static enum { IDLE, ESC, CSI } state = IDLE;
    static char csi_buf[16];
    static uint8_t csi_len = 0;

    switch (state) {
        case IDLE:
            if (ch == 0x1B) {
                state = ESC;
            } else {
                gdiPutChar(ch);
            }
            break;
        case ESC:
            if (ch == '[') {
                state = CSI;
                csi_len = 0;
            } else {
                state = IDLE;
            }
            break;
        case CSI:
            if ((ch >= 0x40 && ch <= 0x7E) || csi_len >= sizeof(csi_buf)-1) {
                csi_buf[csi_len++] = ch;
                csi_buf[csi_len] = 0;
                gdiHandleCSI(csi_buf, csi_len);
                state = IDLE;
            } else {
                csi_buf[csi_len++] = ch;
            }
            break;
    }
}

// Process a buffer of VT100 data
void gdiVT100ProcessBuffer(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        gdiVT100Feed(buf[i]);
    }
}
