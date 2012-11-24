/*
 * Based on:
 *
 *   Copyright 2001 MontaVista Software Inc.
 *   Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 *    arch/mips/yboard/irq_vr5701
 *       This file defines the irq handler for Vr5701
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

/*
 * Vr5701 defines 32 IRQs.
 *
 */
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/ptrace.h>

#include <asm/debug.h>
#include <asm/tcube.h>

static int vr5701_irq_base = -1;
#define NUM_5701_IRQ  32
#define NUM_EPCI_IRQ  4
#define NUM_IPCI_IRQ  4

void ll_vr5701_irq_disable(int vr5701_irq, int ack);

static void 
vr5701_irq_enable(unsigned int irq)
{
	db_assert(vr5701_irq_base != -1);
	db_assert(irq >= vr5701_irq_base);
	db_assert(irq < vr5701_irq_base + NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ);

	ll_vr5701_irq_enable(irq - vr5701_irq_base);
}

static void 
vr5701_irq_disable(unsigned int irq)
{
	db_assert(vr5701_irq_base != -1);
	db_assert(irq >= vr5701_irq_base);
	db_assert(irq < vr5701_irq_base + NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ);

	ll_vr5701_irq_disable(irq - vr5701_irq_base, 0 );
}

static unsigned int vr5701_irq_startup(unsigned int irq)
{
	vr5701_irq_enable(irq);
	return 0;
}

#define	vr5701_irq_shutdown	vr5701_irq_disable

static void
vr5701_irq_ack(unsigned int irq)
{
	db_assert(vr5701_irq_base != -1);
	db_assert(irq >= vr5701_irq_base);
	db_assert(irq < vr5701_irq_base + NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ);

	/* clear the interrupt bit for edge trigger */
	/* some irqs require the driver to clear the sources */
	if( irq < vr5701_irq_base + NUM_5701_IRQ ){
		ddb_out32(INT_CLR, 1 << (irq - vr5701_irq_base));
	}
	/* don't need for PCIs, for they are level triger */

	/* disable interrupt - some handler will re-enable the irq
	 * and if the interrupt is leveled, we will have infinite loop
	 */
	ll_vr5701_irq_disable(irq - vr5701_irq_base, 1 );
}

static void
vr5701_irq_end(unsigned int irq)
{
	db_assert(vr5701_irq_base != -1);
	db_assert(irq >= vr5701_irq_base);
	db_assert(irq < vr5701_irq_base + NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ);

	if(!(irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS))){
		ll_vr5701_irq_enable( irq - vr5701_irq_base);
	}
}

hw_irq_controller vr5701_irq_controller = {
	"vr5701_irq",
	vr5701_irq_startup,
	vr5701_irq_shutdown,
	vr5701_irq_enable,
	vr5701_irq_disable,
	vr5701_irq_ack,
	vr5701_irq_end,
	NULL			/* no affinity stuff for UP */
};

void 
vr5701_irq_init(u32 irq_base)
{
	extern irq_desc_t irq_desc[];
	u32 i;

#if 0
	// --------------------------
	// KOSE PCIX:=LEVEL triger
	// --------------------------
	ddb_out32(VR5701_LEVEL_EDGE, 0xffff);   /* edge trigger */
#endif
	/* printk("vr5701_irq_init(%d)\n",irq_base); */
	for (i= irq_base; i< irq_base + NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = NULL;
		irq_desc[i].depth = 1;
		irq_desc[i].handler = &vr5701_irq_controller;
	}
	vr5701_irq_base = irq_base;
}


int vr5701_irq_to_irq(int irq)
{
	db_assert(irq >= 0);
	db_assert(irq < NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ);

	return irq + vr5701_irq_base;
}

void ll_vr5701_irq_route(int vr5701_irq, int ip)
{
	u32 reg_value;
	u32 reg_bitmask;
	u32 reg_index;

 	if( vr5701_irq >= NUM_5701_IRQ ) {	// PCI
 		if (vr5701_irq >= NUM_5701_IRQ + NUM_EPCI_IRQ) {
 			vr5701_irq = 7 ;
 		}else{
			vr5701_irq = 6;
		}
	}

	db_assert(vr5701_irq >= 0);
	db_assert(vr5701_irq < NUM_5701_IRQ);
	db_assert(ip >= 0);
	db_assert((ip < 5) || (ip == 6));

	reg_index = INT_ROUTE0 + vr5701_irq/8*4;
	reg_value = ddb_in32(reg_index);
	reg_bitmask = 7 << (vr5701_irq % 8 * 4);
	reg_value &= ~reg_bitmask;
	reg_value |= ip << (vr5701_irq % 8 * 4);
	ddb_out32(reg_index, reg_value);
}

void ll_vr5701_irq_enable(int vr5701_irq)
{
	u16 reg_value;
	u32 reg_bitmask;

	db_assert(vr5701_irq < NUM_5701_IRQ + NUM_EPCI_IRQ + NUM_IPCI_IRQ);

	irq_desc[vr5701_irq_base + vr5701_irq].depth ++;

	if( vr5701_irq >= NUM_5701_IRQ ) {	// PCI
		if (vr5701_irq >= NUM_5701_IRQ + NUM_EPCI_IRQ) {
			reg_value = ddb_in32(IPCI_INTM);
			reg_bitmask = 1 << (vr5701_irq - NUM_5701_IRQ - NUM_EPCI_IRQ);
			ddb_out32(IPCI_INTM, reg_value | reg_bitmask);
			vr5701_irq=7;
		}else{
			reg_value = ddb_in32(EPCI_INTM);
			reg_bitmask = 1 << (vr5701_irq - NUM_5701_IRQ );
			ddb_out32(EPCI_INTM, reg_value | reg_bitmask);
			vr5701_irq = 6 ;
		}
	}
	db_assert(vr5701_irq >= 0);
	db_assert(vr5701_irq < NUM_5701_IRQ);
	reg_value = ddb_in32(INT_MASK);
	ddb_out32(INT_MASK, reg_value | (1 << vr5701_irq));

}

void ll_vr5701_irq_disable(int vr5701_irq, int ack)
{
	u16 reg_value;
	u32 udummy;
	u32 reg_bitmask;

	if( !ack ) {
		irq_desc[vr5701_irq_base + vr5701_irq].depth --;
		if (irq_desc[vr5701_irq_base + vr5701_irq].depth)
			return ;
#ifdef DEBUG
		clear_c0_status(0x0400);
		printk("disable irq %d %x\n",
		       vr5701_irq_base + vr5701_irq,
		       irq_desc[vr5701_irq_base + vr5701_irq].depth);
		set_c0_status(0x0400); // enable cpu_int[1]
#endif
	}

	if( vr5701_irq >= NUM_5701_IRQ ) {	// PCI
		if (vr5701_irq >= NUM_5701_IRQ + NUM_EPCI_IRQ) {
			goto DISABLE_IRQ_IPCI;
		}else{
			goto DISABLE_IRQ_EPCI;
		}
	} 

	db_assert(vr5701_irq >= 0);
	db_assert(vr5701_irq < NUM_5701_IRQ);
	reg_value = ddb_in32(INT_MASK);
	ddb_out32(INT_MASK, reg_value & ~(1 << vr5701_irq));
	udummy    = ddb_in32(INT_MASK);
        return ;

  DISABLE_IRQ_IPCI:
	reg_value = ddb_in32(IPCI_INTM);
	reg_bitmask = 1 << (vr5701_irq - NUM_5701_IRQ - NUM_EPCI_IRQ);
	db_assert((reg_value & reg_bitmask) != 0);
	ddb_out32(IPCI_INTM, reg_value & ~reg_bitmask);
	return;

  DISABLE_IRQ_EPCI:
	reg_value = ddb_in32(EPCI_INTM);
	reg_bitmask = 1 << (vr5701_irq - NUM_5701_IRQ );
	db_assert((reg_value & reg_bitmask) != 0);
	ddb_out32(EPCI_INTM, reg_value & ~reg_bitmask);
}

/*
 * Local Variables:
 *   c-basic-offset: 8
 * End:
 */
