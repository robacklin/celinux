/*
 * linux/include/asm-arm/arch-s3c2440/irqs.h
 *
 * Copyright (C) 2003 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/config.h>

/* Interrupt Controller */
#define IRQ_EINT0		0	/* External interrupt 0 */
#define IRQ_EINT1		1	/* External interrupt 1 */
#define IRQ_EINT2		2	/* External interrupt 2 */
#define IRQ_EINT3		3	/* External interrupt 3 */
#define IRQ_EINT4_7		4	/* External interrupt 4 ~ 7 */
#define IRQ_EINT8_23		5	/* External interrupt 8 ~ 23 */
#define IRQ_RESERVED6		6	/* Reserved for future use */
#define IRQ_BAT_FLT		7	/* Battery Fault interrupt */
#define IRQ_TICK		8	/* RTC time tick interrupt  */
#define IRQ_WDT			9	/* Watch-Dog timer interrupt */
#define IRQ_TIMER0		10	/* Timer 0 interrupt */
#define IRQ_TIMER1		11	/* Timer 1 interrupt */
#define IRQ_TIMER2		12	/* Timer 2 interrupt */
#define IRQ_TIMER3		13	/* Timer 3 interrupt */
#define IRQ_TIMER4		14	/* Timer 4 interrupt */
#define IRQ_UART2		15	/* UART 2 interrupt  */
#define IRQ_LCD			16	/* reserved for future use */
#define IRQ_DMA0		17	/* DMA channel 0 interrupt */
#define IRQ_DMA1		18	/* DMA channel 1 interrupt */
#define IRQ_DMA2		19	/* DMA channel 2 interrupt */
#define IRQ_DMA3		20	/* DMA channel 3 interrupt */
#define IRQ_SDI			21	/* SD Interface interrupt */
#define IRQ_SPI0		22	/* SPI0 interrupt */
#define IRQ_UART1		23	/* UART1 receive interrupt */
#define IRQ_RESERVED24		24
#define IRQ_USBD		25	/* USB device interrupt */
#define IRQ_USBH		26	/* USB host interrupt */
#define IRQ_IIC			27	/* IIC interrupt */
#define IRQ_UART0		28	/* UART0 transmit interrupt */
#define IRQ_SPI1		29	/* SPI1 interrupt */
#define IRQ_RTC			30	/* RTC alarm interrupt */
#define IRQ_ADCTC		31	/* ADC EOC interrupt */

/* External Interrupt */
#define IRQ_EINT4		32
#define IRQ_EINT5		33
#define IRQ_EINT6		34
#define IRQ_EINT7		35
#define IRQ_EINT8		36
#define IRQ_EINT9		37
#define IRQ_EINT10		38
#define IRQ_EINT11		39
#define IRQ_EINT12		40
#define IRQ_EINT13		41
#define IRQ_EINT14		42
#define IRQ_EINT15		43
#define IRQ_EINT16		44
#define IRQ_EINT17		45
#define IRQ_EINT18		46
#define IRQ_EINT19		47
#define IRQ_EINT20		48
#define IRQ_EINT21		49
#define IRQ_EINT22		50
#define IRQ_EINT23		51

/* sub Interrupt */
#define IRQ_RXD0		52
#define IRQ_TXD0		53
#define IRQ_ERR0		54
#define IRQ_RXD1		55
#define IRQ_TXD1		56 
#define IRQ_ERR1		57
#define IRQ_RXD2		58
#define IRQ_TXD2		59
#define IRQ_ERR2		60
#define IRQ_TC			61  
#define IRQ_ADC_DONE		62

#define NR_IRQS			63
