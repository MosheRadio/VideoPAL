#include "vt100.h"
#include "gdi.h" // For PAL drawing functions
#include <string.h>

// Internal state for VT100 parser
static enum {
    VT100_STATE_IDLE,
    VT100_STATE_ESC,
    VT100_STATE_CSI
} vt100_state = VT100_STATE_IDLE;

static char csi_buf[16];
static uint8_t csi_len = 0;

void vt100_init(void) {
    vt100_state = VT100_STATE_IDLE;
    csi_len = 0;
}

static void vt100_handle_csi(const char *buf, uint8_t len) {
    // Example: handle cursor movement (ESC [ <row> ; <col> H)
    if (buf[len-1] == 'H' || buf[len-1] == 'f') {
        int row = 1, col = 1;
        sscanf(buf, "%d;%d", &row, &col);
        gdiSetCursor(col, row); // You must implement this in gdi.c
    }
    // Add more VT100 CSI command handling here
}

void vt100_feed(uint8_t ch) {
    switch (vt100_state) {
        case VT100_STATE_IDLE:
            if (ch == 0x1B) {
                vt100_state = VT100_STATE_ESC;
            } else {
                gdiPutChar(ch); // Print regular character
            }
            break;
        case VT100_STATE_ESC:
            if (ch == '[') {
                vt100_state = VT100_STATE_CSI;
                csi_len = 0;
            } else {
                vt100_state = VT100_STATE_IDLE;
            }
            break;
        case VT100_STATE_CSI:
            if ((ch >= 0x40 && ch <= 0x7E) || csi_len >= sizeof(csi_buf)-1) {
                csi_buf[csi_len++] = ch;
                csi_buf[csi_len] = 0;
                vt100_handle_csi(csi_buf, csi_len);
                vt100_state = VT100_STATE_IDLE;
            } else {
                csi_buf[csi_len++] = ch;
            }
            break;
    }
}

void vt100_process_buffer(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        vt100_feed(buf[i]);
    }
}
