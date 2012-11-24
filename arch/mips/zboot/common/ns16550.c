/*
 * BK Id: SCCS/s.ns16550.c 1.21 11/26/01 14:32:25 paulus
 */
/*
 * COM1 NS16550 support
 */

#include <linux/config.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>
#include <asm/serial.h>

#define SERIAL_BAUD	115200

/* memory-mapped read/write of the port */
static unsigned char ns16550_read(volatile unsigned long addr)
{
	return (*(volatile unsigned char *)addr & 0xff); 
}

static void ns16550_write(volatile unsigned long addr, unsigned char data)
{
	*(volatile unsigned char *)addr = data;
}

static struct serial_state rs_table[RS_TABLE_SIZE] = {
	SERIAL_PORT_DFNS	/* Defined in <asm/serial.h> */
};

static int shift = 0;

unsigned long serial_init(int chan, void *ignored)
{
	unsigned long com_port;
	unsigned char lcr, dlm;

	/* We need to find out which type io we're expecting.  If it's
	 * 'SERIAL_IO_PORT', we get an offset from the isa_io_base.
	 * If it's 'SERIAL_IO_MEM', we can the exact location.  -- Tom */
	switch (rs_table[chan].io_type) {
		case SERIAL_IO_PORT:
			com_port = rs_table[chan].port;
			break;
		case SERIAL_IO_MEM:
			com_port = (unsigned long)rs_table[chan].iomem_base;
			break;
		default:
			/* We can't deal with it. */
			return -1;
	}

	if (com_port < 0xA0000000) 
		(unsigned)com_port += 0xA0000000;

	/* How far apart the registers are. */
	shift = rs_table[chan].iomem_reg_shift;
	
	/* save the LCR */
	lcr = ns16550_read(com_port + (UART_LCR << shift));
	/* Access baud rate */
	ns16550_write(com_port + (UART_LCR << shift), 0x80);
	dlm = ns16550_read(com_port + (UART_DLM << shift));
	/*
	 * Test if serial port is unconfigured.
	 * We assume that no-one uses less than 110 baud or
	 * less than 7 bits per character these days.
	 *  -- paulus.
	 */

	if ((dlm <= 4) && (lcr & 2))
		/* port is configured, put the old LCR back */
		ns16550_write(com_port + (UART_LCR << shift), lcr);
	else {
		/* Input clock. */
		ns16550_write(com_port + (UART_DLL << shift),
		     (BASE_BAUD / SERIAL_BAUD));
		ns16550_write(com_port + (UART_DLM << shift),
		     (BASE_BAUD / SERIAL_BAUD) >> 8);
		/* 8 data, 1 stop, no parity */
		ns16550_write(com_port + (UART_LCR << shift), 0x03);
		/* RTS/DTR */
		ns16550_write(com_port + (UART_MCR << shift), 0x03);
	}
	/* Clear & enable FIFOs */
	ns16550_write(com_port + (UART_FCR << shift), 0x07);

	return (com_port);
}

void
serial_putc(unsigned long com_port, unsigned char c)
{
	while ((ns16550_read(com_port + (UART_LSR << shift)) & UART_LSR_THRE) == 0)
		;
	ns16550_write(com_port, c);
}

unsigned char
serial_getc(unsigned long com_port)
{
	while ((ns16550_read(com_port + (UART_LSR << shift)) & UART_LSR_DR) == 0)
		;
	return ns16550_read(com_port);
}

int
serial_tstc(unsigned long com_port)
{
	return ((ns16550_read(com_port + (UART_LSR << shift)) & UART_LSR_DR) != 0);
}
