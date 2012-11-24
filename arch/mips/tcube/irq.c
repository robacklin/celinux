#define PCIX_BRIDGE

/*
 * Based on:
 *
 *   Copyright 2001 MontaVista Software Inc.
 *   Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 *    arch/mips/ddb5xxx/yboard/irq.c
 *       The irq setup and misc routines for DDB5476.
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/ptrace.h>

#include <asm/system.h>
#include <asm/mipsregs.h>
#include <asm/debug.h>
#include <asm/tcube.h>

/*
 * IRQ mapping
 *
 *  0-7: 8 CPU interrupts
 *	0 -	software interrupt 0
 *	1 -	software interrupt 1
 *	2 -	most Vrc5477 interrupts are routed to this pin
 *	3 -	(optional) some other interrupts routed to this pin for debugg
 *	4 -	not used
 *	5 -	not used
 *	6 -	not used
 *	7 -	cpu timer (used by default)
 *
 */

#define ACTIVE_LOW		1
#define ACTIVE_HIGH		0

#define LEVEL_SENSE		2
#define EDGE_TRIGGER		0

#define INTA			0
#define INTB			1
#define INTC			2
#define INTD			3
#define INTE			4

extern void tcube_irq_init(u32 base);
extern void mips_cpu_irq_init(u32 base);
extern asmlinkage void tcube_handle_int(void);
extern void vr5701_irq_init(u32 irq_base);

void
tcube_irq_setup(void)
{
	db_run(printk("T-Cube irq setup invoked.\n"));

	/* by default, we disable all interrupts and route all vr5701 
	 * interrupts to pin 0 (irq 2) */
	ddb_out32(INT_ROUTE0, 0);
	ddb_out32(INT_ROUTE1, 0);
	ddb_out32(INT_ROUTE2, 0);
	ddb_out32(INT_ROUTE3, 0);
	ddb_out32(INT_MASK, 0);
	ddb_out32(INT_CLR, ~0x0);

	clear_c0_status(0xff00);
	set_c0_status(0x0400);

	ll_vr5701_irq_route(24, 1); ll_vr5701_irq_enable(24);	// tbus err
	ll_vr5701_irq_route(25, 1); ll_vr5701_irq_enable(25);   // local err
	ll_vr5701_irq_route(28, 1); ll_vr5701_irq_enable(28);   // epci ierr
	ll_vr5701_irq_route(29, 1); ll_vr5701_irq_enable(29);   // epci serr
	ll_vr5701_irq_route(30, 1); ll_vr5701_irq_enable(30);   // ipci ierr
	ll_vr5701_irq_route(31, 1); ll_vr5701_irq_enable(31);   // ipci serr
	set_c0_status(0x0800); // enable cpu_int[1]

	/* init all controllers */
	mips_cpu_irq_init(0);
	vr5701_irq_init(8);

	/* hook up the first-level interrupt handler */
	set_except_vector(0, tcube_handle_int);
}

#define NUM_5701_IRQS 	32
#define NUM_5701_EPCI_IRQS 	4
#define NUM_5701_IPCI_IRQS 	4

/*
 * the first level int-handler will jump here if it is a vr7701 irq
 */

asmlinkage void
tcube_irq_dispatch(struct pt_regs *regs)
{
	extern unsigned int do_IRQ(int irq, struct pt_regs *regs);

	u32 intStatus;
	u32 bitmask;
	u32 i;
	u32 intPCIStatus;

	db_assert(ddb_in32(INT2_STAT) == 0);
	db_assert(ddb_in32(INT3_STAT) == 0);
	db_assert(ddb_in32(INT4_STAT) == 0);
	db_assert(ddb_in32(NMI_STAT) == 0);

	if (ddb_in32(INT1_STAT) != 0) {
		printk("NMI  = %x\n",ddb_in32(NMI_STAT));
		printk("INT0 = %x\n",ddb_in32(INT0_STAT));
		printk("INT1 = %x\n",ddb_in32(INT1_STAT));
		printk("INT2 = %x\n",ddb_in32(INT2_STAT));
		printk("INT3 = %x\n",ddb_in32(INT3_STAT));
		printk("INT4 = %x\n",ddb_in32(INT4_STAT));
		printk("EPCI_ERR = %x\n",ddb_in32(EPCI_ERR));
		printk("IPCI_ERR = %x\n",ddb_in32(IPCI_ERR));

		panic("error interrupt has happened.");
	}

	intStatus = ddb_in32(INT0_STAT);

	if (intStatus & 1<<6)  // EPCI
		goto IRQ_EPCI;

	if (intStatus & 1<<7)  // IPCI
		goto IRQ_IPCI;

 IRQ_OTHER:
	for (i=0, bitmask=1; i<= NUM_5701_IRQS; bitmask <<=1, i++) {
		/* do we need to "and" with the int mask? */
		if (intStatus & bitmask) {
				do_IRQ(8 + i, regs);
		}
	}
	return;


 IRQ_EPCI:
	intStatus &= ~(1<<6);			/* unset Status flag */
	intPCIStatus = ddb_in32(EPCI_INTS);
	for (i=0, bitmask=1; i< 4 ; bitmask<<=1, i++) {
		if (intPCIStatus & bitmask) {
			do_IRQ(8 + NUM_5701_IRQS + i, regs);
		}
	}
	if ( !intStatus ) return;

 IRQ_IPCI:
	intStatus &= ~(1<<7);
	intPCIStatus = ddb_in32(IPCI_INTS);
	if (!intPCIStatus)
		goto IRQ_OTHER;

	for (i=0, bitmask=1; i< 4 ; bitmask<<=1, i++) {
		if (intPCIStatus & bitmask) {
			do_IRQ(8 + NUM_5701_IRQS + NUM_5701_EPCI_IRQS + i, regs);
		}
	}

	if ( !intStatus ) return;

	goto IRQ_OTHER;
}
