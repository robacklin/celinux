/*
 * drivers/pcmcia/tx4925_pcmcia.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Toshiba Corporation
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _TX4925_PCMCIA
#define _TX4925_PCMCIA

#define TX4925_PCMCIA_MAX_SOCK   (2)

struct pcmcia_init {
	void (*handler)( int irq, void *dev, struct pt_regs *regs );
};

struct pcmcia_state{
	unsigned	detect	: 1,
			ready	: 1,
			bvd1	: 1,
			bvd2	: 1,
			wrprot	: 1,
			vs_3v	: 1,
			vs_Xv	: 1;
};

struct pcmcia_configure {
	unsigned	sock	: 8,
			vcc	: 8,
			vpp	: 8,
			output	: 1,
			speaker	: 1,
			reset	: 1,
			iocard	: 1;
};

struct pcmcia_irq_info {
	unsigned int	sock;
	unsigned int	irq;
};

struct pcmcia_low_level {
	int	(*init)( struct pcmcia_init * );
	int	(*shutdown)( void );
	int	(*socket_state)( unsigned sock, struct pcmcia_state * );
	int	(*get_irq_info)( struct pcmcia_irq_info * );
	int	(*configure_socket)( const struct pcmcia_configure * );
};

/*
 * Declaration for all implementation specific low_level operations.
 */
extern struct pcmcia_low_level rbtx4925_pcmcia_ops;

#endif /* _TX4925_PCMCIA */
