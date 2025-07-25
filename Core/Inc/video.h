/***************************************************************************
 * STM32 video demo
### video.h
***************************************************************************/

#ifndef	__VIDEO_H
#define	__VIDEO_H
#include <stdint.h>
#define NO_TOG		0xFFFF

/* PAL synchronization signal properties
	Pixel freq.			8 MHz
	PAL resolution		336 x 240
	Horizontal freq		15625 Hz
	Hsync pulse width	= 4,7 us
	Line width			= 512 dots
*/
/* here are all the numbers for each macro:
hese values match PAL video timing specifications:
15.625 kHz line frequency
50 Hz field rate
64µs line period
4.7µs sync pulse
336x240 visible resolution
625 total lines per field
*/

#define PAL_Hsyncinterval			64			// microseconds per line
#define PAL_HsyncPulsewidth			4.7			// microseconds per pulse
#define PAL_Horizontalfrequency		(1000000 / PAL_Hsyncinterval) // 1000000/64 = 15625 Hz
#define TIMERCOUNTS					(HSI_VALUE / PAL_Horizontalfrequency) // // 16000000/15625 = 1024 counts
#define HSYNCCOUNTS					(TIMERCOUNTS * PAL_HsyncPulsewidth / PAL_Hsyncinterval) // 1024 * 4.7 / 64 = 76 counts

#define HDELAY			6		// HalfWords DMA length sync offset (I2S)
#define HPORCH			11		// 176 dots	DMA length (in HalfWords)
#define	XFERS_PERLINE	21		// 336 dots	DMA length (in HalfWords)

#define	VID_HSIZE		(HPORCH+XFERS_PERLINE)	// Horizontal line duration 11 + 21 = 32 halfwords per line
#define	VLINES			240		// Vertical resolution (number of visible lines)
#define BLINES			73		// invisible lines (including VSYNC)
#define	VID_VSIZE		(2 * (VLINES+BLINES) -1)	// number of lines per field 2*(240+73)-1 = 625 lines per field

#define	VID_PIXELS_X	(XFERS_PERLINE * sizeof(uint16_t) * 8) // 21 * 2 * 8 = 336 pixels per line
#define	VID_PIXELS_Y	VLINES // 240 lines per field

extern uint16_t Vblack[XFERS_PERLINE + HPORCH]; // 21 + 11 = 32 halfwords per line
extern uint16_t Vwhite[XFERS_PERLINE+HPORCH]; // 21 + 11 = 32 halfwords per line
extern uint16_t *borders[VID_VSIZE];
/*
 * The CMAR addresses have an offset to the actual framebuffer address because
 * the DMA adds a variable offset and counts from HDELAY ... (HDELAY+19).
 * Therefore the HDELAY should be subtracted from the framebuffer addresses.
 */
#define lad(l)			(screen[l]-HDELAY)
//#define mad(m)		(m-HDELAY)

extern uint16_t *lineptrs[VID_VSIZE];
extern uint16_t screen[VLINES][XFERS_PERLINE];
extern uint32_t screenBB[VID_PIXELS_Y][VID_PIXELS_X];	/* bit banding alias of screen */


//	Function definitions

void	vidClearScreen(void);

#endif	// __VIDEO_H
