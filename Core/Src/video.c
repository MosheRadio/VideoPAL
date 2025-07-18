/* @file    PALneat/video.c
 * STM32 PAL video demo
 * Written by  E.J.W. Vrieze
  * @version V1.5.1
  * @date    1-July-2016
  * @brief   Basic PAL video output using DMA and Timers.
 */
#include <string.h>
#include "stm32g4xx_hal.h"
#include "video.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint16_t SyncTable[] = {
		/* line   8 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  18 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  28 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  38 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  48 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  58 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  68 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  78 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  88 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line  98 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 108 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 118 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 128 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 138 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 148 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 158 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 168 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 178 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 188 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 198 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 208 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 218 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 228 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 238 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 248 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 258 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 268 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 278 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 288 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 298 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 308 */		NO_TOG, NO_TOG, NO_TOG,
		/* line 311 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2,			/* line 311 */
		/* line 312 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2,			/* line 312 */
		/* line 313 */		TIMERCOUNTS/2,	TIMERCOUNTS   - HSYNCCOUNTS,			/* line 313 */
		/* line 314 */		RESET,			TIMERCOUNTS/2 - HSYNCCOUNTS, 			/* line 314 */
							TIMERCOUNTS/2,	TIMERCOUNTS   - HSYNCCOUNTS,
		/* line 315 */		RESET,			TIMERCOUNTS/2 - HSYNCCOUNTS, 			/* line 315 */
							TIMERCOUNTS/2,	TIMERCOUNTS   - HSYNCCOUNTS,
		/* line 316 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2, 			/* line 316 */
		/* line 317 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2, NO_TOG,	/* line 317 */
		/* line 318 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 323 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 333 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 343 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 353 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 363 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 373 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 383 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 393 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 403 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 413 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 423 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 433 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 443 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 453 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 463 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 473 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 483 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 493 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 503 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 513 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 523 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 533 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 543 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 553 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 563 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 573 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 583 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 593 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 603 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 613 */		NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG, NO_TOG,
		/* line 623 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2,			/* line 623 */
		/* line 624 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2,			/* line 624 */
		/* line 625 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2,			/* line 625 */
		/* line 1 */		RESET,			TIMERCOUNTS/2 - HSYNCCOUNTS, 			/* line 1 */
							TIMERCOUNTS/2,	TIMERCOUNTS - HSYNCCOUNTS,
		/* line 2 */		RESET,			TIMERCOUNTS/2 - HSYNCCOUNTS, 			/* line 2 */
							TIMERCOUNTS/2,	TIMERCOUNTS - HSYNCCOUNTS,
		/* line 3 */		RESET,			TIMERCOUNTS/2 - HSYNCCOUNTS, 			/* line 3 */
							TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2,
		/* line 4 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2, 			/* line 4 */
		/* line 5 */		TIMERCOUNTS/2,	TIMERCOUNTS/2 + HSYNCCOUNTS/2, NO_TOG,	/* line 5 */
		/* line 6 */		NO_TOG, NO_TOG
};
#if 0
uint16_t RHline [XFERS_PERLINE] =		/* RAM array for first line video data */
{ [XFERS_PERLINE/2 ... XFERS_PERLINE-1] = 0xFFFF };
uint16_t LHline [XFERS_PERLINE] =		/* RAM array for first line video data */
{ [0 ... XFERS_PERLINE/2-1] = 0xFFFF };
uint16_t wholeline [XFERS_PERLINE] =		/* RAM array for first line video data */
{ [0 ... XFERS_PERLINE-1] = 0xFFFF };
#endif
uint16_t Vblack[XFERS_PERLINE+HPORCH];
uint16_t Vwhite[XFERS_PERLINE+HPORCH] =
{ [0 ... HPORCH-7] = 0,			[HPORCH-6 ... XFERS_PERLINE+HPORCH-5] = 0xFFFF, 0,0,0,0 };
uint16_t Kader1[XFERS_PERLINE+HPORCH] =
{ [0 ... HPORCH-7] = 0, 0xFFF8, [HPORCH-5 ... XFERS_PERLINE+HPORCH-6] = 0xFFFF, 0xFFFF, 0,0,0,0 };
uint16_t Kader2[XFERS_PERLINE+HPORCH] =
{ [0 ... HPORCH-7] = 0, 0xFFFB, [HPORCH-5 ... XFERS_PERLINE+HPORCH-6] = 0, 0x1FFF, 0,0,0,0 };
uint16_t Zijden[XFERS_PERLINE+HPORCH] =
{ [0 ... HPORCH-7] = 0, 0xFFFB, [HPORCH-5 ... XFERS_PERLINE+HPORCH-6] = 0xFFFF, 0xDFFF, 0,0,0,0 };
uint16_t Kader3[XFERS_PERLINE+HPORCH] =
{ [0 ... HPORCH-7] = 0, 0xFFF8, [HPORCH-5 ... XFERS_PERLINE+HPORCH-6] = 0xFFFF, 0xDFFF, 0,0,0,0 };
uint16_t Kader4[XFERS_PERLINE+HPORCH] =
{ [0 ... HPORCH-7] = 0, 0xFFFF, [HPORCH-5 ... XFERS_PERLINE+HPORCH-6] = 0, 0x1FFF, 0,0,0,0 };

uint16_t *lineptrs[VID_VSIZE] = {
/* line 8 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack,
/* line 18 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 28 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 38 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Kader1,	Kader2,	Zijden, Zijden, Zijden,
/* line 49 */		lad(0), lad(1), lad(2), lad(3), lad(4), lad(5), lad(6), lad(7), lad(8), lad(9),
/* line 59 */		lad(10), lad(11), lad(12), lad(13), lad(14), lad(15), lad(16), lad(17), lad(18), lad(19),
/* line 69 */		lad(20), lad(21), lad(22), lad(23), lad(24), lad(25), lad(26), lad(27), lad(28), lad(29),
/* line 79 */		lad(30), lad(31), lad(32), lad(33), lad(34), lad(35), lad(36), lad(37), lad(38), lad(39),
/* line 89 */		lad(40), lad(41), lad(42), lad(43), lad(44), lad(45), lad(46), lad(47), lad(48), lad(49),
/* line 99 */		lad(50), lad(51), lad(52), lad(53), lad(54), lad(55), lad(56), lad(57), lad(58), lad(59),
/* line 109 */		lad(60), lad(61), lad(62), lad(63), lad(64), lad(65), lad(66), lad(67), lad(68), lad(69),
/* line 119 */		lad(70), lad(71), lad(72), lad(73), lad(74), lad(75), lad(76), lad(77), lad(78), lad(79),
/* line 129 */		lad(80), lad(81), lad(82), lad(83), lad(84), lad(85), lad(86), lad(87), lad(88), lad(89),
/* line 139 */		lad(90), lad(91), lad(92), lad(93), lad(94), lad(95), lad(96), lad(97), lad(98), lad(99),
/* line 149 */		lad(100), lad(101), lad(102), lad(103), lad(104), lad(105), lad(106), lad(107), lad(108), lad(109),
/* line 159 */		lad(110), lad(111), lad(112), lad(113), lad(114), lad(115), lad(116), lad(117), lad(118), lad(119),
/* line 169 */		lad(120), lad(121), lad(122), lad(123), lad(124), lad(125), lad(126), lad(127), lad(128), lad(129),
/* line 179 */		lad(130), lad(131), lad(132), lad(133), lad(134), lad(135), lad(136), lad(137), lad(138), lad(139),
/* line 189 */		lad(140), lad(141), lad(142), lad(143), lad(144), lad(145), lad(146), lad(147), lad(148), lad(149),
/* line 199 */		lad(150), lad(151), lad(152), lad(153), lad(154), lad(155), lad(156), lad(157), lad(158), lad(159),
/* line 209 */		lad(160), lad(161), lad(162), lad(163), lad(164), lad(165), lad(166), lad(167), lad(168), lad(169),
/* line 219 */		lad(170), lad(171), lad(172), lad(173), lad(174), lad(175), lad(176), lad(177), lad(178), lad(179),
/* line 229 */		lad(180), lad(181), lad(182), lad(183), lad(184), lad(185), lad(186), lad(187), lad(188), lad(189),
/* line 239 */		lad(190), lad(191), lad(192), lad(193), lad(194), lad(195), lad(196), lad(197), lad(198), lad(199),
/* line 249 */		lad(200), lad(201), lad(202), lad(203), lad(204), lad(205), lad(206), lad(207), lad(208), lad(209),
/* line 259 */		lad(210), lad(211), lad(212), lad(213), lad(214), lad(215), lad(216), lad(217), lad(218), lad(219),
/* line 269 */		lad(220), lad(221), lad(222), lad(223), lad(224), lad(225), lad(226), lad(227), lad(228), lad(229),
/* line 279 */		lad(230), lad(231), lad(232), lad(233), lad(234), lad(235), lad(236), lad(237), lad(238), lad(239),
/* line 289 */		Zijden,	Zijden,	Kader3,	Kader4,	Zijden,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 299 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 309 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack,
/* line 319 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack,
/* line 329 */		Vblack,	Vblack, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 339 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 349 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Kader1, Kader2,	Zijden,
/* line 359 */		Zijden, Zijden,
/* line 361 */		lad(0), lad(1), lad(2), lad(3), lad(4), lad(5), lad(6), lad(7), lad(8), lad(9),
/* line 371 */		lad(10), lad(11), lad(12), lad(13), lad(14), lad(15), lad(16), lad(17), lad(18), lad(19),
/* line 381 */		lad(20), lad(21), lad(22), lad(23), lad(24), lad(25), lad(26), lad(27), lad(28), lad(29),
/* line 391 */		lad(30), lad(31), lad(32), lad(33), lad(34), lad(35), lad(36), lad(37), lad(38), lad(39),
/* line 401 */		lad(40), lad(41), lad(42), lad(43), lad(44), lad(45), lad(46), lad(47), lad(48), lad(49),
/* line 411 */		lad(50), lad(51), lad(52), lad(53), lad(54), lad(55), lad(56), lad(57), lad(58), lad(59),
/* line 421 */		lad(60), lad(61), lad(62), lad(63), lad(64), lad(65), lad(66), lad(67), lad(68), lad(69),
/* line 431 */		lad(70), lad(71), lad(72), lad(73), lad(74), lad(75), lad(76), lad(77), lad(78), lad(79),
/* line 441 */		lad(80), lad(81), lad(82), lad(83), lad(84), lad(85), lad(86), lad(87), lad(88), lad(89),
/* line 451 */		lad(90), lad(91), lad(92), lad(93), lad(94), lad(95), lad(96), lad(97), lad(98), lad(99),
/* line 461 */		lad(100), lad(101), lad(102), lad(103), lad(104), lad(105), lad(106), lad(107), lad(108), lad(109),
/* line 471 */		lad(110), lad(111), lad(112), lad(113), lad(114), lad(115), lad(116), lad(117), lad(118), lad(119),
/* line 481 */		lad(120), lad(121), lad(122), lad(123), lad(124), lad(125), lad(126), lad(127), lad(128), lad(129),
/* line 491 */		lad(130), lad(131), lad(132), lad(133), lad(134), lad(135), lad(136), lad(137), lad(138), lad(139),
/* line 501 */		lad(140), lad(141), lad(142), lad(143), lad(144), lad(145), lad(146), lad(147), lad(148), lad(149),
/* line 511 */		lad(150), lad(151), lad(152), lad(153), lad(154), lad(155), lad(156), lad(157), lad(158), lad(159),
/* line 521 */		lad(160), lad(161), lad(162), lad(163), lad(164), lad(165), lad(166), lad(167), lad(168), lad(169),
/* line 531 */		lad(170), lad(171), lad(172), lad(173), lad(174), lad(175), lad(176), lad(177), lad(178), lad(179),
/* line 541 */		lad(180), lad(181), lad(182), lad(183), lad(184), lad(185), lad(186), lad(187), lad(188), lad(189),
/* line 551 */		lad(190), lad(191), lad(192), lad(193), lad(194), lad(195), lad(196), lad(197), lad(198), lad(199),
/* line 561 */		lad(200), lad(201), lad(202), lad(203), lad(204), lad(205), lad(206), lad(207), lad(208), lad(209),
/* line 571 */		lad(210), lad(211), lad(212), lad(213), lad(214), lad(215), lad(216), lad(217), lad(218), lad(219),
/* line 581 */		lad(220), lad(221), lad(222), lad(223), lad(224), lad(225), lad(226), lad(227), lad(228), lad(229),
/* line 591 */		lad(230), lad(231), lad(232), lad(233), lad(234), lad(235), lad(236), lad(237), lad(238), lad(239),
/* line 601 */		Zijden,	Zijden,	Kader3,	Kader4,	Zijden,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 611 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vblack,
/* line 621 */		Vblack, Vblack,	Vblack, Vblack, Vblack,
/* line 1 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack };

uint16_t *borders[VID_VSIZE] = {
/* line 8 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack,
/* line 18 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 28 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 38 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Kader1,	Kader2,	Zijden, Zijden,
/* line 48 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 58 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 68 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 78 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 88 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 98 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 108 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 118 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 128 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 138 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 148 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 158 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 168 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 178 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 188 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 198 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 208 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 218 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 228 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 238 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 248 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 258 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 268 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 278 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 288 */		Zijden,	Zijden,	Zijden,	Kader3,	Kader4,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 298 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 308 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack,
/* line 318 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack,
/* line 328 */		Vblack,	Vblack, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 338 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 348 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Kader1,	Kader2,
/* line 358 */		Zijden, Zijden,
/* line 360 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 370 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 380 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 390 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 400 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 410 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 420 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 430 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 440 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 450 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 460 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 470 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 480 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 490 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 500 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 510 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 520 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 530 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 540 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 550 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 560 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 570 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 580 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 590 */		Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden, Zijden,
/* line 600 */		Zijden,	Zijden,	Zijden,	Kader3,	Kader4,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 610 */		Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite, Vwhite,	Vwhite,
/* line 620 */		Vblack,	Vblack, Vblack,	Vblack, Vblack, Vblack,
/* line 1 */		Vblack,	Vblack, Vblack,	Vblack, Vblack,	Vblack, Vblack };

/* screen frame buffer, declared in the linker script to enable bit banding */
uint16_t screen[VLINES][XFERS_PERLINE] __attribute__((section (".screen")));
/* bit banding alias of screen buffer : */
uint32_t screenBB[VID_PIXELS_Y][VID_PIXELS_X] __attribute__((section (".screenBB")));

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void vidClearScreen(void)
{
//	uint8_t line;
#if 0
	for ( int nr = 0; nr < VLINES; nr++) {
		fb[nr][0]				= 1<<15;	// 16 bits MSB first
		fb[nr][XFERS_PERLINE-1]	= 1;		// 16 bits LSB  last
	}
	for ( int nr = 1; nr<239; nr++) {
		fb[nr][nr>>4] = 1<<(15-(nr&0xF));	// 16 bits MSB first
	}
	for (line = 0; line < VLINES; line++)
	{
		fb[line]				[0] = 0b1111111111111110;
		fb[line]  [XFERS_PERLINE-1] = 0b1111111111111110;
	}
#else
	memset(screen,0xFF,sizeof(screen));
#endif

}
