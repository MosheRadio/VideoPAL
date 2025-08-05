/*
 * vt100.c
 *
 * This module provides a thin wrapper around the VT100 parser
 * implemented in gdi_vt100.c.  It exposes the same API as the
 * original vt100.c from the feature‑vt100 branch (vt100_init(),
 * vt100_feed() and vt100_process_buffer()) but delegates all
 * parsing work to the gdiVT100Feed() and gdiVT100ProcessBuffer()
 * functions.  This approach avoids duplication between two
 * different VT100 parsers and ensures that only one implementation
 * is maintained.
 */

#include <stddef.h>
#include <stdint.h>
#include "gdi.h"
#include "vt100.h"

/*
 * Initialise the VT100 parser.  For now this simply clears the
 * screen and resets the cursor.  Call this after initialising
 * your peripherals (e.g. UART, LCD) and before feeding any data
 * into the parser.
 */
void vt100_init(void)
{
    /* Clear the screen and reset the cursor.  The gdiClearScreen()
     * function defined in gdi_vt100.c will clear the framebuffer
     * and position the cursor at (0,0).  If you wish to perform
     * additional initialisation (e.g. setting text colours), do it
     * here.
     */
    gdiClearScreen();
}

/*
 * Feed a single byte into the VT100 parser.  This function is
 * intended to be called from your UART interrupt or polling
 * routine whenever a new character arrives from the host.  It
 * delegates all parsing to gdiVT100Feed().
 */
void vt100_feed(uint8_t ch)
{
    gdiVT100Feed(ch);
}

/*
 * Process an entire buffer of bytes through the VT100 parser.
 * This convenience function allows you to parse a contiguous
 * sequence of bytes (for example when reading from a file or
 * buffered input).  It forwards the call to
 * gdiVT100ProcessBuffer().
 */
void vt100_process_buffer(const uint8_t *buf, uint16_t len)
{
    gdiVT100ProcessBuffer(buf, len);
}
