#ifndef SCREEN_UTILS_H
#define SCREEN_UTILS_H

#include <stdint.h>
#include "video.h"
#include "gdi.h"

// Get screen center X coordinate
static inline uint16_t get_screen_center_x(void) {
    return VID_PIXELS_X / 2;
}

// Get screen center Y coordinate
static inline uint16_t get_screen_center_y(void) {
    return VID_PIXELS_Y / 2;
}

// Calculate X position to center text
static inline uint16_t get_centered_text_x(const char* text) {
    return (VID_PIXELS_X - (strlen(text) * GDI_SYSFONT_WIDTH)) / 2;
}

// Calculate Y position to center text vertically
static inline uint16_t get_centered_text_y(void) {
    return (VID_PIXELS_Y - GDI_SYSFONT_HEIGHT) / 2;
}

#endif /* SCREEN_UTILS_H */