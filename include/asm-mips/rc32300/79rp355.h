/*
 *
 * BRIEF MODULE DESCRIPTION
 *	Definitions for IDT 79RP355 evaluation board.
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2002 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _79RP355_H_
#define _79RP355_H_

#define IDT_BUS_FREQ   66 // MHz
#define IDT_CLOCK_MULT 2

/* Memory map of 79RP355 board */

/* DRAM */
#define RAM_BASE        0x00000000
#define RAM_SIZE	(16*1024*1024)

/* SRAM (device 1) */
#define SRAM_BASE       0x02000000
#define SRAM_SIZE       0x00100000

/* FLASH (device 2) */
#define FLASH_BASE      0x0C000000
#define FLASH_SIZE      0x00C00000

/* ATM PHY (device 4) */
#define ATM_PHY_BASE    0x14000000

/* TDM switch (device 3) */
#define TDM_BASE        0x1A000000

/* LCD panel (device 3) */
#define LCD_BASE        0x1A002000

/* RTC (DS1511W) (device 3) */
#define RTC_BASE        0x1A004000

/* NVRAM (256 bytes internal to the DS1511 RTC) */
#define NVRAM_ADDR      RTC_BASE + 0x10
#define NVRAM_DATA      RTC_BASE + 0x13
#define NVRAM_ENVSIZE_OFF  4
#define NVRAM_ENVSTART_OFF 32

#include <asm/rc32300/rc32355.h>
#include <asm/rc32300/ds1501rtc.h>

#endif /* _79RP355_H_ */
