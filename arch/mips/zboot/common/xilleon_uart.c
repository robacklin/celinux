/*
 * BRIEF MODULE DESCRIPTION
 *	Simple Au1000 uart routines.
 *
 * Author: Pete Popov <ppopov@mvista.com, or source@mvista.com>
 *
 * 2001 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <asm/io.h>
#include <asm/au1000.h>
#include "ns16550.h"

typedef         unsigned char uint8;
typedef         unsigned int  uint32;

#define         UART16550_BAUD_2400             2400
#define         UART16550_BAUD_4800             4800
#define         UART16550_BAUD_9600             9600
#define         UART16550_BAUD_19200            19200
#define         UART16550_BAUD_38400            38400
#define         UART16550_BAUD_57600            57600
#define         UART16550_BAUD_115200           115200

#define         UART16550_PARITY_NONE           0
#define         UART16550_PARITY_ODD            0x08
#define         UART16550_PARITY_EVEN           0x18
#define         UART16550_PARITY_MARK           0x28
#define         UART16550_PARITY_SPACE          0x38

#define         UART16550_DATA_5BIT             0x0
#define         UART16550_DATA_6BIT             0x1
#define         UART16550_DATA_7BIT             0x2
#define         UART16550_DATA_8BIT             0x3

#define         UART16550_STOP_1BIT             0x0
#define         UART16550_STOP_2BIT             0x4


/* memory-mapped read/write of the port */
#define UART16550_READ(y)    (readl(UART_BASE + y) & 0xff)
#define UART16550_WRITE(y,z) (writel(z&0xff, UART_BASE + y))

/*
 * We use uart 0, which is already initialized by
 * yamon. 
 */
volatile struct NS16550 *
serial_init(int chan)
{
	volatile struct NS16550 *com_port;
	com_port = (struct NS16550 *) UART_BASE;
	return (com_port);
}

void
serial_putc(volatile struct NS16550 *com_port, unsigned char c)
{
	while ((UART16550_READ(UART_LSR)&0x40) == 0);
	UART16550_WRITE(UART_TX, c);
	return 1;
}

unsigned char
serial_getc(volatile struct NS16550 *com_port)
{
	while((UART16550_READ(UART_LSR) & 0x1) == 0);
	return UART16550_READ(UART_RX);
}

int
serial_tstc(volatile struct NS16550 *com_port)
{
	return((UART16550_READ(UART_LSR) & LSR_DR) != 0);
}
