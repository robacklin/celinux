/*
 *  linux/drivers/char/serial_sa1100.c
 *
 *  Driver for s3c2440 serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2003 Samsung Electronics
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
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/circ_buf.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/hardware.h>
#include <asm/arch/cpu_s3c2440.h>
#include <asm/kgdb.h>

#if defined(CONFIG_SERIAL_S3C2440_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/serial_core.h>

/* We've been assigned a range on the "Low-density serial ports" major */
#define SERIAL_S3C2440_MAJOR	204
#define CALLOUT_S3C2440_MAJOR	205
#define MINOR_START		5

#ifdef CONFIG_S3C2440_SMDK2440
#define NR_PORTS		3
#endif


#define UART_GET_ULCON(port)	__raw_readl((port)->membase + oULCON)
#define UART_GET_UCON(port)	__raw_readl((port)->membase + oUCON)
#define UART_GET_UFCON(port)	__raw_readl((port)->membase + oUFCON)
#define UART_GET_UMCON(port)	__raw_readl((port)->membase + oUMCON)
#define UART_GET_UTRSTAT(port)	__raw_readl((port)->membase + oUTRSTAT)
#define UART_GET_UERSTAT(port)	__raw_readl((port)->membase + oUERSTAT)
#define UART_GET_UFSTAT(port)	__raw_readl((port)->membase + oUFSTAT)
#define UART_GET_UMSTAT(port)	__raw_readl((port)->membase + oUMSTAT)
#define UART_GET_CHAR(port)	__raw_readl((port)->membase + oURXH)
#define UART_GET_UBRDIV(port)	__raw_readl((port)->membase + oUBRDIV)

#define UART_PUT_ULCON(port,v)   __raw_writel((v),(port)->membase + oULCON)
#define UART_PUT_UCON(port,v)    __raw_writel((v),(port)->membase + oUCON)
#define UART_PUT_UFCON(port,v)   __raw_writel((v),(port)->membase + oUFCON)
#define UART_PUT_UMCON(port,v)   __raw_writel((v),(port)->membase + oUMCON)
#define UART_PUT_UTRSTAT(port,v) __raw_writel((v),(port)->membase + oUTRSTAT)
#define UART_PUT_UERSTAT(port,v) __raw_writel((v),(port)->membase + oUERSTAT)
#define UART_PUT_UFSTAT(port,v)  __raw_writel((v),(port)->membase + oUFSTAT)
#define UART_PUT_UMSTAT(port,v)  __raw_writel((v),(port)->membase + oUMSTAT)
#define UART_PUT_CHAR(port,v)    __raw_writel((v),(port)->membase + oUTXH)
#define UART_PUT_UBRDIV(port,v)  __raw_writel((v),(port)->membase + oUBRDIV)

#define RX_IRQ(port)  	((port)->irq)
#define TX_IRQ(port)  	((port)->irq+1)
#define ERR_IRQ(port) 	((port)->irq+2)

/* This is inline code from 2.4.21 linux/serial_core.h */
#define uart_circ_empty(circ)           ((circ)->head == (circ)->tail)
#define uart_circ_clear(circ)           ((circ)->head = (circ)->tail = 0)
 
#define uart_circ_chars_pending(circ)   \
        (CIRC_CNT((circ)->head, (circ)->tail, UART_XMIT_SIZE))

#define uart_circ_chars_free(circ)      \
	        (CIRC_SPACE((circ)->head, (circ)->tail, UART_XMIT_SIZE))
	                
#define uart_tx_stopped(info)           \
	        (info->tty->stopped || info->tty->hw_stopped)
	             

/*
 * This is the size of our serial port register set.
 */
#define UART_PORT_SIZE	0x30

static struct tty_driver normal, callout;
static struct tty_struct *s3c2440_table[NR_PORTS];
static struct termios *s3c2440_termios[NR_PORTS], *s3c2440_termios_locked[NR_PORTS];

#ifdef SUPPORT_SYSRQ
static struct console s3c2440_console;
#endif

static struct uart_port s3c2440_ports[NR_PORTS];

static int tx_irq_enabled=0;

/*
 * interrupts diabled on entry
 */

static void s3c2440_stop_tx(struct uart_port *port, u_int from_tty)
{
	disable_irq (TX_IRQ(port));
	tx_irq_enabled = 0;
}

/*
 * interrupts may not be disabled on entry
 */
static void s3c2440_start_tx(struct uart_port *port, u_int nonempty, u_int from_tty)
{
	if (!tx_irq_enabled)
		enable_irq (TX_IRQ(port));
	tx_irq_enabled = 1;
}

/*
 * interrupts enabled
 */
static void s3c2440_stop_rx(struct uart_port *port)
{
	disable_irq (RX_IRQ(port));
}

/*
 * No modem control lines
 */
static void s3c2440_enable_ms(struct uart_port *port)
{
}

static void s3c2440_rx_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_info *info = dev_id;
	struct tty_struct *tty = info->tty;
	unsigned int status, ch, max_count = 256;
	struct uart_port *port = info->port;

	status = UART_GET_UTRSTAT(port);
	while ((status & UTRSTAT_RX_RDY) && max_count--) {
		if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
			tty->flip.tqueue.routine((void *) tty);
			if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
				printk(KERN_WARNING "TTY_DONT_FLIP set\n");
				return;
			}
		}

		ch = UART_GET_CHAR(port);

		*tty->flip.char_buf_ptr = ch;
		*tty->flip.flag_buf_ptr = TTY_NORMAL;
		port->icount.rx++;
		tty->flip.flag_buf_ptr++;
		tty->flip.char_buf_ptr++;
		tty->flip.count++;
		status = UART_GET_UTRSTAT(port);
	}
	tty_flip_buffer_push(tty);
	return;
}

static void s3c2440_tx_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_info *info = (struct uart_info*)dev_id;
	struct uart_port *port = info->port;
	struct circ_buf *xmit = &info->xmit;
	int count;

	if (port->x_char) {
		UART_PUT_CHAR(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	/*
	 * Check the modem control lines before
	 * transmitting anything.
	 */

	if (uart_circ_empty(xmit) || uart_tx_stopped(info)) {
		s3c2440_stop_tx(port, 0);
		return;
	}

	count = port->fifosize >> 1;
	do {
		UART_PUT_CHAR (port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (xmit->head == xmit->tail)
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		tasklet_schedule(&info->tlet);

	if (uart_circ_empty(xmit))
		s3c2440_stop_tx(port, 0);
}

static void s3c2440_err_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_info *info = dev_id;
	struct uart_port *port = info->port;
	struct tty_struct *tty = info->tty;
	unsigned int ch, flg;
	unsigned char err;

	spin_lock(&info->lock);
	
	ch = UART_GET_CHAR (port);
	err = UART_GET_UERSTAT(port) & (UERSTAT_BRK | UERSTAT_FRAME |
			UERSTAT_PARITY | UERSTAT_OVERRUN);
	if (!err) return ;

	if (err & UERSTAT_BRK)
		port->icount.brk++;
	if (err & UERSTAT_FRAME)
		port->icount.frame++;
	if (err & UERSTAT_PARITY)
		port->icount.parity++;
	if (err & UERSTAT_OVERRUN)
		port->icount.overrun++;

	err &= port->read_status_mask;

	if (err & UERSTAT_PARITY)
		flg = TTY_PARITY;
	else if (err & UERSTAT_FRAME)
		flg = TTY_FRAME;
	else
		flg = TTY_NORMAL;

	if (err & UERSTAT_OVERRUN) {
		*tty->flip.char_buf_ptr = ch;
		*tty->flip.flag_buf_ptr = flg;
		tty->flip.flag_buf_ptr++;
		tty->flip.char_buf_ptr++;
		tty->flip.count++;
		if (tty->flip.count < TTY_FLIPBUF_SIZE) {
			ch = 0;
			flg = TTY_OVERRUN;
		}
	}

	*tty->flip.flag_buf_ptr++ = flg;
	*tty->flip.char_buf_ptr++ = ch;
	tty->flip.count++;
	
	spin_unlock(&info->lock);
}

static unsigned int s3c2440_tx_empty(struct uart_port *port)
{
	return UART_GET_UTRSTAT(port) & UTRSTAT_TR_EMP ? 0 : TIOCSER_TEMT;
}

static unsigned int s3c2440_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void s3c2440_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

/*
 * Interrupts always disabled.
 */
static void s3c2440_break_ctl(struct uart_port *port, int break_state)
{
	unsigned int ucon;

	ucon = UART_GET_UCON(port);
	if (break_state == -1)
		ucon |= UCON_BRK_SIG;
	else
		ucon &= ~UCON_BRK_SIG;
	UART_PUT_UCON(port, ucon);
}

static int s3c2440_startup(struct uart_port *port, struct uart_info *info)
{
	int retval, ucon, irq_name_num;
	char irq_name[3][3][20] = {{"s3c2440_uart0_rx", "s3c2440_uart0_tx", "s3c2440_uart0_err"},
				{"s3c2440_uart1_rx", "s3c2440_uart1_tx", "s3c2440_uart1_err"},
				{"s3c2440_uart2_rx", "s3c2440_uart2_tx", "s3c2440_uart2_err"}};
	
	if (RX_IRQ(port) == IRQ_RXD0)
		irq_name_num = 0;
	else if (RX_IRQ(port) == IRQ_RXD1)
		irq_name_num = 1;
	else if (RX_IRQ(port) == IRQ_RXD2)
		irq_name_num = 2;
	else
		return (1);

	retval = request_irq(RX_IRQ(port), s3c2440_rx_int, SA_INTERRUPT,
			     irq_name[irq_name_num][0], info);
	if (retval){
		return retval;
	}

	retval = request_irq(TX_IRQ(port), s3c2440_tx_int, SA_INTERRUPT,
			     irq_name[irq_name_num][1], info);
	if (retval) {
		free_irq (RX_IRQ(port), info);
		return retval;
	}

	retval = request_irq(ERR_IRQ(port), s3c2440_err_int, SA_INTERRUPT,
			     irq_name[irq_name_num][2], info);
	if (retval) {
		free_irq (RX_IRQ(port), info);
		free_irq (TX_IRQ(port), info);
		return retval;
	}

	ucon = (UCON_TX_INT_LVL | UCON_RX_INT_LVL |
	                UCON_TX_INT | UCON_RX_INT | UCON_RX_TIMEOUT);
	UART_PUT_UCON (port, ucon);

	return 0;
}

static void s3c2440_shutdown(struct uart_port *port, struct uart_info *info)
{
	free_irq (RX_IRQ(port), info);
	free_irq (TX_IRQ(port), info);
	free_irq (ERR_IRQ(port), info);
}

static void s3c2440_change_speed(struct uart_port *port, unsigned int cflag, unsigned int iflag, unsigned int quot)
{
	unsigned long flags;
	unsigned int ulcon, ufcon;

	ufcon = UART_GET_UFCON (port);
	
	switch (cflag & CSIZE) {
            case CS5:       ulcon = ULCON_WL5; break;
            case CS6:       ulcon = ULCON_WL6; break;
            case CS7:       ulcon = ULCON_WL7; break;
            default:        ulcon = ULCON_WL8; break;
	}

	if (cflag & CSTOPB)
		ulcon |= ULCON_STOP;
	if (cflag & PARENB) {
	        if (!(cflag & PARODD))
	            ulcon |= ULCON_PAR_EVEN;
	}

	if (port->fifosize > 1)
		ufcon |= UFCON_FIFO_EN;
	port->read_status_mask =  UERSTAT_OVERRUN;
	if (iflag & INPCK)
		port->read_status_mask |= UERSTAT_PARITY | UERSTAT_FRAME;
	port->ignore_status_mask = 0;
	if (iflag & IGNPAR)
		port->ignore_status_mask |= UERSTAT_FRAME | UERSTAT_PARITY;
	if (iflag & IGNBRK) {
		if (iflag & IGNPAR)
			port->ignore_status_mask |= UERSTAT_OVERRUN;
	}

	quot -= 1;

	save_flags(flags);
	cli();

	UART_PUT_UFCON (port, ufcon);
	UART_PUT_ULCON (port, (UART_GET_ULCON(port) & ~(ULCON_PAR | ULCON_WL)) | ulcon);
	UART_PUT_UBRDIV (port, quot);

	sti();
}

static const char *s3c2440_type(struct uart_port *port)
{
	return port->type == PORT_S3C2440 ? "S3C2440" : NULL;
}

static void s3c2440_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, UART_PORT_SIZE);
}

static int s3c2440_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, UART_PORT_SIZE,
			"s3c2440-uart") != NULL ? 0 : -EBUSY;
}

static void s3c2440_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE &&
	    s3c2440_request_port(port) == 0)
		port->type = PORT_S3C2440;
}

static int s3c2440_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_S3C2440)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if ((void *)port->mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops s3c2440_pops = {
	tx_empty :      s3c2440_tx_empty,
	set_mctrl :     s3c2440_set_mctrl,
	get_mctrl :     s3c2440_get_mctrl,
	stop_tx :       s3c2440_stop_tx,
	start_tx :      s3c2440_start_tx,
        stop_rx :       s3c2440_stop_rx,
	enable_ms :     s3c2440_enable_ms,
        break_ctl :     s3c2440_break_ctl,
        startup   :     s3c2440_startup,
        shutdown  :     s3c2440_shutdown,
        change_speed :  s3c2440_change_speed,
        type      :     s3c2440_type,
        release_port :  s3c2440_release_port,
        request_port :  s3c2440_request_port,
        config_port  :  s3c2440_config_port,
        verify_port  :  s3c2440_verify_port,
};


static void __init s3c2440_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < NR_PORTS; i++) {
		s3c2440_ports[i].uartclk   = (u_int) s3c2440_get_pclk();
		s3c2440_ports[i].ops       = &s3c2440_pops;
//		s3c2440_ports[i].fifosize  = 8;
//		minsung chane 2003.12.29 fifosize modification
		s3c2440_ports[i].fifosize  = 64;
		s3c2440_ports[i].line      = i;
		s3c2440_ports[i].iotype    = SERIAL_IO_MEM;
	}
}

void __init s3c2440_register_uart(int idx, int port)
{
	switch (idx)
	{
		case 0:
			s3c2440_ports[idx].membase	= (void *)io_p2v(UART0_CTL_BASE);
			s3c2440_ports[idx].mapbase	= UART0_CTL_BASE;
			s3c2440_ports[idx].irq	= IRQ_RXD0;
			break;
		case 1:
			s3c2440_ports[idx].membase	= (void *)io_p2v(UART1_CTL_BASE);
			s3c2440_ports[idx].mapbase	= UART1_CTL_BASE;
			s3c2440_ports[idx].irq	= IRQ_RXD1;
			break;
		case 2:
			s3c2440_ports[idx].membase	= (void *)io_p2v(UART2_CTL_BASE);
			s3c2440_ports[idx].mapbase	= UART2_CTL_BASE;
			s3c2440_ports[idx].irq	= IRQ_RXD2;
			break;
		default:
			printk(KERN_ERR "%s: bad index number %d\n", __FUNCTION__, idx);
			return;
	}

	s3c2440_ports[idx].flags	= ASYNC_BOOT_AUTOCONF;
}


#ifdef CONFIG_SERIAL_S3C2440_CONSOLE

/*
 * Interrupts are disabled on entering
 */
static void
s3c2440_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &s3c2440_ports[co->index];
	unsigned int status, i;

	/*
	 *	First, save UTCR3 and then disable interrupts
	 */
	for (i = 0; i < count; i++) {
		do {
			status = UART_GET_UTRSTAT(port);
		} while (!(status & UTRSTAT_TX_EMP));
		UART_PUT_CHAR(port, s[i]);
		if (s[i] == '\n') {
			do {
				status = UART_GET_UTRSTAT(port);
			} while (!(status & UTRSTAT_TX_EMP));
			UART_PUT_CHAR(port, '\r');
		}
	}
}

static kdev_t s3c2440_console_device(struct console *co)
{
	return MKDEV(SERIAL_S3C2440_MAJOR, MINOR_START + co->index);
}

static int __init
s3c2440_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200; //		CONFIG_S3C2440_DEFAULT_BAUDRATE;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */

	port = uart_get_console(s3c2440_ports, NR_PORTS, co);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console s3c2440_console = {
	name		: "ttyS",
	write		: s3c2440_console_write,
	device		: s3c2440_console_device,
	setup		: s3c2440_console_setup,
	flags		: CON_PRINTBUFFER,
	index		: -1,
};

void __init s3c2440_console_init(void)
{
	s3c2440_init_ports();
	register_console(&s3c2440_console);
}

#define S3C2440_CONSOLE	&s3c2440_console
#else
#define S3C2440_CONSOLE	NULL
#endif

static struct uart_driver s3c2440_reg = {
	owner			: THIS_MODULE,
	normal_major		: SERIAL_S3C2440_MAJOR,
#ifdef CONFIG_DEVFS_FS
	normal_name		: "ttyS%d",
	callout_name		: "cua%d",
#else
	normal_name		: "ttyS",
	callout_name		: "cua",
#endif
	normal_driver		: &normal,
	callout_major		: CALLOUT_S3C2440_MAJOR,
	callout_driver		: &callout,
	table			: s3c2440_table,
	termios			: s3c2440_termios,
	termios_locked		: s3c2440_termios_locked,
	minor			: MINOR_START,
	nr			: NR_PORTS,
	port			: s3c2440_ports,	/////
	cons			: S3C2440_CONSOLE,
};

static int __init s3c2440_serial_init(void)
{
	int i, ret;

	s3c2440_init_ports();

	ret = uart_register_driver(&s3c2440_reg);
	if (ret)
		return ret;

	return 0;
}

static void __exit s3c2440_serial_exit(void)
{
	uart_unregister_driver(&s3c2440_reg);
}

module_init(s3c2440_serial_init);
module_exit(s3c2440_serial_exit);

EXPORT_NO_SYMBOLS;

MODULE_AUTHOR("Samsung Electronics Ltd");
MODULE_DESCRIPTION("S3C2440 generic serial port driver");
MODULE_LICENSE("GPL");
