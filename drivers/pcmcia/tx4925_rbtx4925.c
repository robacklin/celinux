/* $Id: tx4925_rbtx4925.c,v 1.1 2001/12/19 10:48:56 nemoto Exp $
 * tx4925_rbtx4925.c: RBTX4925 board specific pcmcia routines.
 *
 * RBTX4925 has 1 PCMCIA slot.  The slot is connected to TX4925 via
 * LinkUp L1121 chip.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Toshiba Corporation
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/sched.h>

#include <asm/tx4925/toshiba_rbtx4925.h>
#include "tx4925_pcmcia.h"

#define IRQ_NO( x )	( RBTX4925_PCMCIA_IRQ + (x) )

struct _irq_param{
	int	offs;
	u8	name[12];
};

static struct _irq_param irq_parm[] = {
	{ L1121_CD1, "PCMCIA_CD1" },
	{ L1121_CD2, "PCMCIA_CD2" },
	{ L1121_VS1, "PCMCIA_VS1" },
	{ L1121_VS2, "PCMCIA_VS2" }
};


static int rbtx4925_pcmcia_init( struct pcmcia_init *init )
{
	int	rc, irq, irq_parm_count;
	u8	id[4], tmp;

	/* EBUSC are configured in arch-specific setup routine */

	/* L1121 are configured in arch-specific setup routine */

	if( RBTX4925_IOC_REG( PIOSEL ) & RBTX4925_PIOSEL_NOPCMCIA ){
		printk( KERN_ERR "%s: disabled by PIOSEL\n", __FUNCTION__ );
		return -1;
	}

	id[0] = L1121_inb( L1121_IDR1 );
	id[1] = L1121_inb( L1121_IDR2 );
	id[2] = L1121_inb( L1121_IDR3 );
	id[3] = L1121_inb( L1121_IDR4 );

	if( ( id[0] != L1121_IDR1_VAL ) || ( id[1] != L1121_IDR2_VAL ) ||
	    ( id[2] != L1121_IDR3_VAL ) || ( id[3] != L1121_IDR4_VAL ) ){
		printk( KERN_ERR "%s: L1121 ID mismatch[%2.2x:%2.2x:%2.2x:%2.2x]\n",
			__FUNCTION__, id[0], id[1], id[2], id[3] );
		return -1;
	}

	/* set endian mode */
	tmp = L1121_inb( L1121_CR1 );
#ifdef __MIPSEB__
	tmp |= L1121_CR1_BIG;
#else
	tmp &= ~L1121_CR1_BIG;
#endif
	L1121_outb( tmp, L1121_CR1 );

	/* set interrupt type and clear edge interrupt */
	tmp = ( L1121_INT_CD1  | L1121_INT_CD2  |
		L1121_INT_VS1  | L1121_INT_VS2  |
		L1121_INT_BVD1 | L1121_INT_BVD2 ); 
	L1121_outb( tmp, L1121_ITSR );
	L1121_outb( tmp, L1121_ECLR );
	
	/* set IO-C PCMCIA interrupt pole */
	RBTX4925_IOC_REG( PCMCIA_INT_POLE ) = 1;

	/* Register interrupts */
	irq_parm_count = ( sizeof( irq_parm ) / sizeof( struct _irq_param ) );
	for( irq = 0; irq < irq_parm_count; irq++ ){
		rc = request_irq( IRQ_NO( irq_parm[irq].offs ),
				  init->handler,
				  SA_INTERRUPT,
				  irq_parm[irq].name,
				  NULL );
		if( rc < 0 ){
			printk( KERN_ERR "%s: Request for IRQ %u failed\n",
				__FUNCTION__,
				RBTX4925_PCMCIA_IRQ + irq_parm[irq].offs );
			while( --irq >= 0 ){
				free_irq( IRQ_NO( irq_parm[irq].offs ), NULL );
			}
			return -1;
		}
	}

	/* enable on-chip power switch */
	L1121_outb( L1121_CR2_PDCS, L1121_CR2 );

	/* enable IO-C PCMCIA interrupt */
	RBTX4925_IOC_REG( PCMCIA_INT_MASK ) = 1;

	/* There's two sockets, but only the first one, 0, is used */
	return 1;
}

static int rbtx4925_pcmcia_shutdown( void )
{
	/* disable on-chip power switch */
	L1121_outb( 0, L1121_CR2 );
	return 0;
}

static int rbtx4925_pcmcia_socket_state( unsigned sock, struct pcmcia_state *state )
{
	u8	sr;

	if( sock != 0 ){
		return -1;
	}

	state->ready  = 0;
	state->vs_Xv  = 0;
	state->vs_3v  = 0;
	state->detect = 0;

	/* 
	 * This is tricky. The READY pin is also the #IRQ pin.  We'll treat
	 * READY as #IRQ and set state->ready to 1 whenever state->detect 
	 * is true.
	 */
	sr = L1121_inb( L1121_SR );
	if( !( sr & ( L1121_SR_CD1 | L1121_SR_CD2 ) ) ){
		state->detect = 1;
		if( sr & L1121_SR_VS1 ){
			state->vs_3v = 1;
		}
	}

	if( state->detect ){
		state->ready = 1;
	}

	state->bvd1   = ( sr & L1121_SR_BVD1  ) ? 0 : 1;
	state->bvd2   = ( sr & L1121_SR_BVD2  ) ? 0 : 1;
	state->wrprot = ( sr & L1121_SR_IOIS16) ? 0 : 1;
	return 1;
}


static int rbtx4925_pcmcia_get_irq_info( struct pcmcia_irq_info *info )
{
	if( info->sock != 0 ){
		return -1;
	}

	info->irq = IRQ_NO( L1121_RDY );
	return 0;
}


static int rbtx4925_pcmcia_configure_socket(
		const struct pcmcia_configure *configure )
{
	u8	cr;

	if( configure->sock != 0 ){
		return -1;
	}

	cr = L1121_inb( L1121_CR1 );
	cr &= ~( L1121_CR1_RESET | L1121_CR1_SOE );
	if( configure->reset ){
		cr |= L1121_CR1_RESET;
	}
	if( configure->output ){
		cr |= L1121_CR1_SOE;
	}
	L1121_outb( cr, L1121_CR1 );

	cr  = L1121_inb( L1121_CR2 );
	cr &= ~( L1121_CR2_S3 | L1121_CR2_S4 );
	switch( configure->vcc ){
	case 0:  /* Vcc 0 */
		break;
	case 50: /* Vcc 5V */
		cr |= L1121_CR2_S3;
		break;
	case 33: /* Vcc 3.3V */
		cr |= L1121_CR2_S4;
		break;
	default: /* what's this ? */
		printk( KERN_ERR "%s: bad Vcc %d\n", 
			__FUNCTION__, configure->vcc );
		break;
	}
	L1121_outb( cr, L1121_CR2 );

	cr  = L1121_inb( L1121_CR3 );
	cr &= ~L1121_CR3_MIO;
	if( !configure->iocard ){
		cr |= L1121_CR3_MIO;
	}
	L1121_outb( cr, L1121_CR3 );
	return 0;
}

struct pcmcia_low_level rbtx4925_pcmcia_ops = { 
	rbtx4925_pcmcia_init,
	rbtx4925_pcmcia_shutdown,
	rbtx4925_pcmcia_socket_state,
	rbtx4925_pcmcia_get_irq_info,
	rbtx4925_pcmcia_configure_socket
};

