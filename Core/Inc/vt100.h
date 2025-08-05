/*
 * vt100.h
 *
 * Public interface for the VT100 parser.  This header declares
 * three functions that initialise the parser and feed data into
 * it.  The implementation in vt100.c delegates parsing to
 * functions defined in gdi_vt100.c.  See gdi.h for additional
 * helper functions (cursor positioning, screen clearing, etc.).
 */

#ifndef VT100_H
#define VT100_H

#include <stdint.h>

/* Initialise the VT100 parser.  Clears the screen and resets the
 * cursor.  Must be called once before feeding any data. */
void vt100_init(void);

/* Feed a single byte into the VT100 parser.  Call this for each
 * character received from the UART or other input source. */
void vt100_feed(uint8_t ch);

/* Process a contiguous buffer of bytes.  This is a convenience
 * wrapper around vt100_feed() that handles multiple bytes in one
 * call. */
void vt100_process_buffer(const uint8_t *buf, uint16_t len);

#endif /* VT100_H */
