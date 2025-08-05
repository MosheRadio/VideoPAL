/*
 * gdi_vt100.c
 *
 * This module provides a simple VT100 terminal abstraction on top of the
 * existing GDI drawing primitives in the VideoPAL project.  It implements
 * cursor positioning, screen clearing and character output.  It also
 * interprets a subset of VT100 escape sequences (CSI commands) such as
 * cursor positioning (H/f), cursor movement (A/B/C/D) and clear screen (J).
 */

#include "vt100.h"
#include "video.h"    /* for vidClearScreen() */
#include "gdi.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* Pixel cursor position.  These variables hold the current X/Y coordinates
 * in pixel space.  They are advanced by gdiPutChar() and updated by
 * escape sequences. */
static int cursor_x = 0;
static int cursor_y = 0;

/* Set the pixel cursor position. */
void gdiSetCursor(int x, int y)
{
    /* Clamp to the visible area */
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= VID_PIXELS_X) x = VID_PIXELS_X - GDI_SYSFONT_WIDTH;
    if (y >= VID_PIXELS_Y) y = VID_PIXELS_Y - GDI_SYSFONT_HEIGHT;
    cursor_x = x;
    cursor_y = y;
}

/* Get the current pixel cursor position. */
void gdiGetCursor(int *x, int *y)
{
    if (x) *x = cursor_x;
    if (y) *y = cursor_y;
}

/* Clear the entire screen and reset the cursor to the top left. */
void gdiClearScreen(void)
{
    /* Use the provided video function to clear the frame buffer. */
    vidClearScreen();
    cursor_x = 0;
    cursor_y = 0;
}

/* Output a single character at the current cursor position.  Handles
 * carriage return and newline automatically. */
void gdiPutChar(uint8_t ch)
{
    switch (ch) {
    case '\r':
        /* Carriage return: move to column 0. */
        cursor_x = 0;
        return;
    case '\n':
        /* Newline: move to next line and column 0. */
        cursor_x = 0;
        cursor_y += GDI_SYSFONT_HEIGHT;
        if (cursor_y >= VID_PIXELS_Y) {
            /* Simple scroll-wrap: start at top when bottom is exceeded. */
            cursor_y = 0;
        }
        return;
    default:
        break;
    }
    /* Draw the character using the existing GDI function. */
    char buf[2] = { (char)ch, '\0' };
    gdiDrawTextEx(cursor_x, cursor_y, buf);
    /* Advance the cursor.  Wrap to the next line if we exceed the width. */
    cursor_x += GDI_SYSFONT_WIDTH;
    if (cursor_x >= VID_PIXELS_X) {
        cursor_x = 0;
        cursor_y += GDI_SYSFONT_HEIGHT;
        if (cursor_y >= VID_PIXELS_Y) {
            cursor_y = 0;
        }
    }
}

/* Helper: parse numeric prefix of a CSI sequence.  Returns the parsed
 * integer or a default value of 1 if no digits are present. */
static int parse_csi_number(const char *buf, int len)
{
    /* Copy into a temporary buffer so we can null‑terminate. */
    char tmp[8] = {0};
    int n = (len < (int)sizeof(tmp) - 1) ? len : (int)sizeof(tmp) - 1;
    memcpy(tmp, buf, n);
    tmp[n] = '\0';
    int value = 0;
    if (tmp[0] == '\0') {
        return 1;
    }
    /* Convert to integer; atoi returns 0 if no digits, default to 1. */
    value = atoi(tmp);
    return value > 0 ? value : 1;
}

/* Handle a complete CSI sequence.  The buffer contains everything
 * between the '[' and the final command character. */
void gdiHandleCSI(const char *buf, uint8_t len)
{
    if (len == 0) {
        return;
    }
    char cmd = buf[len - 1];
    switch (cmd) {
    case 'H': /* CUP: ESC [ <row> ; <col> H */
    case 'f': /* HVP: ESC [ <row> ; <col> f */
    {
        int row = 1, col = 1;
        /* sscanf will parse two integers separated by a semicolon.  If
         * either integer is missing, it will leave the value unchanged. */
        sscanf(buf, "%d;%d", &row, &col);
        /* Convert 1‑based row/col into pixel coordinates. */
        gdiSetCursor((col - 1) * GDI_SYSFONT_WIDTH, (row - 1) * GDI_SYSFONT_HEIGHT);
        break;
    }
    case 'A': /* CUU: cursor up n lines */
    {
        int n = parse_csi_number(buf, len - 1);
        cursor_y -= n * GDI_SYSFONT_HEIGHT;
        if (cursor_y < 0) cursor_y = 0;
        break;
    }
    case 'B': /* CUD: cursor down n lines */
    {
        int n = parse_csi_number(buf, len - 1);
        cursor_y += n * GDI_SYSFONT_HEIGHT;
        if (cursor_y >= VID_PIXELS_Y) cursor_y = VID_PIXELS_Y - GDI_SYSFONT_HEIGHT;
        break;
    }
    case 'C': /* CUF: cursor forward n columns */
    {
        int n = parse_csi_number(buf, len - 1);
        cursor_x += n * GDI_SYSFONT_WIDTH;
        if (cursor_x >= VID_PIXELS_X) cursor_x = VID_PIXELS_X - GDI_SYSFONT_WIDTH;
        break;
    }
    case 'D': /* CUB: cursor backward n columns */
    {
        int n = parse_csi_number(buf, len - 1);
        cursor_x -= n * GDI_SYSFONT_WIDTH;
        if (cursor_x < 0) cursor_x = 0;
        break;
    }
    case 'J': /* ED: erase display */
    {
        /* Only support ESC [ 2 J (clear entire screen) for now. */
        gdiClearScreen();
        break;
    }
    default:
        /* Unsupported CSI command: ignore */
        break;
    }
}

/* Feed one character into the VT100 parser.  Maintains a tiny state
 * machine to detect ESC and CSI sequences and calls the appropriate
 * handlers. */
void gdiVT100Feed(uint8_t ch)
{
    enum ParserState { STATE_IDLE, STATE_ESC, STATE_CSI };
    static enum ParserState state = STATE_IDLE;
    static char csi_buf[16];
    static uint8_t csi_len = 0;

    switch (state) {
    case STATE_IDLE:
        if (ch == 0x1B) {
            state = STATE_ESC;
        } else {
            gdiPutChar(ch);
        }
        break;
    case STATE_ESC:
        if (ch == '[') {
            state = STATE_CSI;
            csi_len = 0;
        } else {
            /* Unknown escape: just print it and return to idle. */
            gdiPutChar(0x1B);
            gdiPutChar(ch);
            state = STATE_IDLE;
        }
        break;
    case STATE_CSI:
        if ((ch >= 0x40 && ch <= 0x7E) || csi_len >= (sizeof(csi_buf) - 1)) {
            /* End of CSI sequence */
            csi_buf[csi_len++] = ch;
            csi_buf[csi_len] = '\0';
            gdiHandleCSI(csi_buf, csi_len);
            state = STATE_IDLE;
        } else {
            /* Accumulate parameters */
            csi_buf[csi_len++] = ch;
        }
        break;
    }
}

/* Process a buffer of VT100 data.  This simply feeds each byte into
 * gdiVT100Feed(). */
void gdiVT100ProcessBuffer(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; ++i) {
        gdiVT100Feed(buf[i]);
    }
}
