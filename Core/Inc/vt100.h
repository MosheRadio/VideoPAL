#ifndef VT100_H
#define VT100_H

#include <stdint.h>

// VT100 command handler initialization
void vt100_init(void);

// Feed a received character to the VT100 parser
void vt100_feed(uint8_t ch);

// Optionally, expose a function to process a buffer
void vt100_process_buffer(const uint8_t *buf, uint16_t len);

#endif // VT100_H
