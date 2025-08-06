# VT100 Implementation Status

This document describes the VT100 commands that are currently implemented in the VideoPAL project.

## Implemented Commands

### Cursor Functions
- ✓ `ESC [ pn A` - Move cursor up n lines (CUU)
- ✓ `ESC [ pn B` - Move cursor down n lines (CUD)
- ✓ `ESC [ pn C` - Move cursor right n columns (CUF)
- ✓ `ESC [ pn D` - Move cursor left n columns (CUB)
- ✓ `ESC [ pl ; pc H` - Set cursor position to line pl, column pc (CUP)
- ✓ `ESC [ pl ; pc f` - Set cursor position to line pl, column pc (HVP)

### Erasing
- ✓ `ESC [ 2 J` - Clear entire screen (cursor doesn't move)

### Control Characters
- ✓ `\r` (CR) - Carriage Return: Move to column 0
- ✓ `\n` (LF) - Line Feed: Move to next line and column 0

## Special Features
- Screen wrapping is implemented - when cursor reaches the right edge, it automatically moves to the beginning of the next line
- Screen scrolling is implemented as a wrap-around - when cursor reaches bottom, it moves back to top

## Not Yet Implemented
Many VT100 features from the specification are not yet implemented, including:
- Character attributes (bold, underline, blink, inverse)
- Line attributes
- Scrolling regions
- Tab functions
- Character sets
- LED functions
- Most setup functions
- Print functions
- Requests/Reports

## Notes
The implementation is focused on basic cursor movement and screen clearing functionality, which covers the most common use cases for terminal emulation. The parser is extensible and new commands can be added by extending the CSI command handler in `gdi_vt100.c`.