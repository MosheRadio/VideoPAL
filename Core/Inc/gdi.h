/*
 * Modified GDI header with cursor and VT100 helper prototypes.
 *
 * This header is based off of the original gdi.h from the VideoPAL
 * project.  It retains the original definitions for font sizes,
 * raster operations, data structures and drawing APIs, and adds
 * prototypes for cursor management and VT100 helper functions.  These
 * additional functions are implemented in gdi_vt100.c.
 */

#ifndef __GDI_H
#define __GDI_H

#include <stdint.h>

/* System font metrics */
#define GDI_SYSFONT_WIDTH            6   /* Width in pixels */
#define GDI_SYSFONT_HEIGHT           10  /* Height in pixels */
#define GDI_SYSFONT_WIDTH_BIG        7   /* Width in pixels */
#define GDI_SYSFONT_HEIGHT_BIG       14  /* Height in pixels */
#define GDI_SYSFONT_WIDTH_SMALL      5   /* Width in pixels */
#define GDI_SYSFONT_HEIGHT_SMALL     7   /* Height in pixels */
#define GDI_SYSFONT_OFFSET           0x20

/* Raster operation modes */
typedef enum {
    GDI_ROP_COPY = 0,
    GDI_ROP_BONW = 1,
    GDI_ROP_NAND = 2,
    GDI_ROP_XOR  = 3,
    GDI_ROP_OR   = 4
} RasterOP;

/* Example default text strings used elsewhere in the project.  Kept here
 * for completeness.  You can change these as needed. */
#define KOPTEKST  "MOSHE IS THE KING"
#define SUBTITEL  "IDAN WHEN WE EAT TODAY?"

/* Rectangle type used for clipping and windowing. */
typedef struct {
    int16_t x;  /* X position */
    int16_t y;  /* Y position */
    int16_t w;  /* Width */
    int16_t h;  /* Height */
} GDI_RECT, *PGDI_RECT;

/* Bitmap pointer type for BitBlt; allows passing either a 16‑bit or 8‑bit
 * bitmap in ROM or RAM. */
typedef union {
    uint16_t       *halfwords;
    const uint16_t *halfwordsROM;
    uint8_t        *bytesin;
    const uint8_t  *bytesinROM;
} pBMP;

/* Window attributes and alignment flags */
#define GDI_WINCAPTION      0x0001
#define GDI_WINBORDER       0x0002
#define GDI_WINCLOSEICON    0x0003

#define GDI_WINCAPTION_LEFT    0x0000
#define GDI_WINCAPTION_CENTER  0x0010
#define GDI_WINCAPTION_RIGHT   0x0020
#define GDI_WINCAPTION_MASK    0x0030

/* Window descriptor */
typedef struct {
    uint16_t  style;   /* Mode; see GDI_WINxxx defines */
    GDI_RECT  rc;      /* Absolute rectangle */
    uint8_t  *caption; /* Caption text */
} GDI_WINDOW, *PGDI_WINDOW;

/* Generic bitmap descriptor */
typedef struct {
    int16_t   w;  /* Width in bits */
    int16_t   h;  /* Height in bits */
    int16_t   wb; /* Width in bytes */
    int16_t   wh; /* Height in bytes */
    uint8_t  *bm; /* Pointer to bitmap bits */
} GDI_BITMAP, *PGDI_BITMAP;

/* Drawing function declarations (original API) */
void gdiGetClientRect(PGDI_WINDOW pwin, PGDI_RECT prc);
void gdiBitBlt(PGDI_RECT prc, int16_t x, int16_t y, int16_t w, int16_t h, pBMP bm);
void gdiPoint(PGDI_RECT prc, uint16_t x, uint16_t y);
void gdiLine(PGDI_RECT prc, int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void gdiRectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void gdiDrawTextEx(int16_t x, int16_t y, char *ptext);

/*
 * Cursor and VT100 helper functions.
 *
 * These functions provide cursor positioning, screen clearing and
 * character output for a basic VT100 implementation.  They are
 * implemented in gdi_vt100.c.  See vt100.c for a simple parser
 * that uses these helpers.
 */
void gdiSetCursor(int x, int y);
void gdiGetCursor(int *x, int *y);
void gdiClearScreen(void);
void gdiPutChar(uint8_t ch);
void gdiHandleCSI(const char *buf, uint8_t len);
void gdiVT100Feed(uint8_t ch);
void gdiVT100ProcessBuffer(const uint8_t *buf, uint16_t len);

#endif /* __GDI_H */
