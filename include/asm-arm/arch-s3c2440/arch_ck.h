/*
 * Enumerate the clocks for the S3C2410
 *
 * Initial OMAP1510 code by Gordon McNutt <gmcnutt@ridgerun.com> 
 * Copyright (c) 2001 2001 RidgeRun, Inc. <http://www.ridgerun.com>
 *
 * Addapted for S3C2410 by Seongil Na.
 * Copyright (c) 2003 Samsung Electronics, Inc. <http://www.samsungsemi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * History:
 * 2003-10-29:	Seongil Na <seongil@samsung.com>
 *	- Initialize
 *	
 * 2003-12-16:	Seongil Na <seongil@samsung.com>
 * 	- Add for loops_per_jiffy
 *
 */

#ifndef arch_ck_h
#define arch_ck_h

#include <linux/config.h>

/*
 * This enum is the bare minimum needed to support the kmod/ck.h interface.
 * 
 */
typedef enum
{ 
	/* Fixed system clocks(12MHz) */
	clkin = 0,

	/* Main clocks */
	ck_fclk,	/* clock for ARM920T */
	ck_hclk,	/* clock for AHB bus and ARM920T */
	ck_pclk,	/* clock for APB bus */
	ck_uclk,	/* fixed USB clock(48MHz) */
	
	/* HCLK */
	ck_lcdc,	/* LCD controller clock */
	ck_nandc,	/* NAND controller clock */

	/* PCLK */
	ck_spi,		/* SPI */
	ck_iis,		/* I2S */
	ck_iic,		/* I2C */
	ck_adc,		/* ADC */
	ck_rtc,		/* RTC */
	ck_gpio,	/* GPIO */
	ck_uart0,	/* UART0 */
	ck_uart1,	/* UART1 */
	ck_uart2,	/* UART2 */
	ck_sdi,		/* SDI */
	ck_pwm,		/* PWM */

	/* UCLK */
	ck_usbh,	/* clock for usb host */
	ck_usbd,	/* clock for usb device */
} ck_t;

typedef enum
{
	arm_rst = 0,
} reset_t;

#define CK_RATEF	1
#define CK_IDLEF	2
#define CK_ENABLEF	4
#define CK_SELECTF	8

enum
{
	MPLL_SUSPEND = 0,	/* Suspend */
	MPLL_48MHZ,
	MPLL_96MHZ,
	MPLL_101MHZ,
	MPLL_112MHZ,
	
	MPLL_118MHZ,
	MPLL_124MHZ,
	MPLL_135MHZ,
	MPLL_146MHZ,
	MPLL_152MHZ,
	
	MPLL_158MHZ,
	MPLL_169MHZ,
	MPLL_180MHZ,
	MPLL_186MHZ,
	MPLL_192MHZ,
	
	MPLL_203MHZ,
	MPLL_214MHZ,
	MPLL_220MHZ,
	MPLL_225MHZ,
	MPLL_237MHZ,
	
	MPLL_248MHZ,
	MPLL_254MHZ,
	MPLL_259MHZ,
	MPLL_270MHZ,
	MPLL_282MHZ,
	
	MPLL_288MHZ,
	MPLL_293MHZ,
	MPLL_304MHZ,
	MPLL_316MHZ,
	MPLL_321MHZ,
	
	MPLL_327MHZ,
	MPLL_338MHZ,
	MPLL_350MHZ,
	MPLL_355MHZ,
	MPLL_361MHZ,
	
	MPLL_372MHZ,
	MPLL_384MHZ,
	MPLL_389MHZ,
	MPLL_400MHZ,

	MPLL_SIZE,
};

static unsigned int mpll_value[MPLL_SIZE] =
{
	0x00000000,		/* 0.00MHz: Power off */
	0x00038022,
	0x00038021,
	0x000F60D1,
	0x000C7091,
	
	0x00096061,
	0x000C7081,
	0x00096051,
	0x000D4071,
	0x00077031,
	
	0x00047011,
	0x000F6071,
	0x000E9061,
	0x00055011,
	0x00058011,
	
	0x000F60D0,
	0x00087021,
	0x000F90C0,
	0x000C7090,
	0x00096060,
	
	0x000C7080,
	0x00077040,
	0x000E6090,
	0x00096050,
	0x00056020,
	
	0x00040010,
	0x000D4070,
	0x00077030,
	0x00047010,
	0x0007E030,
	
	0x000B7050,
	0x000F6070,
	0x000A7040,
	0x000E5060,
	0x000E9060,
	
	0x00055010,
	0x00058010,
	0x000DB050,
	0x0005C010,
};

static unsigned long lpj_value[MPLL_SIZE] = 
{
	0,			/* 0.00MHz: Suspend */
	0,			/* 48MHz */
	0,			/* 96MHz */
	0,			/* 101MHz */
	0,			/* 112MHz */
	
	0,			/* 118MHz */
	0,			/* 124MHz */
	337920,			/* 135MHz */
	0,			/* 146MHz */
	0,			/* 152MHz */
	
	0,			/* 158MHz */
	0,			/* 169MHz */
	0,			/* 180MHz */
	0,			/* 186MHz */
	0,			/* 192MHz */

	0,			/* 203MHz */
	0,			/* 214MHz */
	0,			/* 220MHz */
	0,			/* 225MHz */
	0,			/* 237MHz */

	0,			/* 248MHz */
	0,			/* 254MHz */
	0,			/* 259Hz */
	0,			/* 270MHz */
	0,			/* 282MHz */

	0,			/* 288MHz */
	0,			/* 293MHz */
	0,			/* 304MHz */
	0,			/* 316MHz */
	0,			/* 321MHz */

	0,			/* 327MHz */
	0,			/* 338MHz */
	0,			/* 350MHz */
	0,			/* 355MHz */
	0,			/* 361MHz */

	0,			/* 372MHz */
	0,			/* 384MHz */
	0,			/* 389MHz */
	999424,			/* 400MHz */
};
#endif /* arch_ck_h */
