/*
 * linux/arch/arm/mach-s3c2440/irq.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>


#define EINT_MASK	0x7

int set_external_irq(int irq, int edge, int pullup)
{
	unsigned long flags;
	int real_irq, shift;

	if (((irq < IRQ_EINT0) && (irq > IRQ_EINT23)) ||
	    ((irq > IRQ_EINT3) && (irq < IRQ_EINT4)))
		return -EINVAL;

	real_irq = (irq < IRQ_EINT4) ? irq : (irq + 4 - 32);

	if (real_irq < 8) {
		GPFCON &= ~(0x3 << (real_irq*2));
		GPFCON |=  (0x2 << (real_irq*2));
		GPFUP  &= ~(0x1 << real_irq);
		GPFUP  |=  (pullup << real_irq);
	} else {
		GPGCON &= ~(0x3 << ((real_irq-8)*2));
		GPGCON |=  (0x2 << ((real_irq-8)*2));
		GPGUP  &= ~(0x1 << (real_irq - 8));
		GPGUP  |=  (pullup << (real_irq - 8));
	} 
	
	local_irq_save(flags);

	shift = (real_irq & 0x7) << 2 ; 
	if (real_irq < 8) {
		EXTINT0 = ~(EINT_MASK << shift);
		EXTINT0 = edge << shift;
	} else if (real_irq < 16) {
		EXTINT1 = ~(EINT_MASK << shift);
		EXTINT1 = edge << shift;
	}else {
		EXTINT2 = ~(EINT_MASK << shift);
		EXTINT2 = edge << shift;
	}

	if (irq < 4) {
		SRCPND |= (1 << real_irq);
		INTPND |= (1 << real_irq);
	} else {
		EINTPEND |= (1 << real_irq);
	}

	irq_desc[irq].valid = 1;

	restore_flags(flags);

	return 0;
}
EXPORT_SYMBOL(set_external_irq);


/*
 * Defined irq handlers
 */
static void s3c2440_mask_ack_irq(unsigned int irq)
{
	INTMSK |= (1 << irq);
	SRCPND = (1 << irq);
	INTPND = (1 << irq);
}

static void s3c2440_mask_irq(unsigned int irq)
{
	INTMSK |= (1 << irq);
}

static void s3c2440_unmask_irq(unsigned int irq)
{
	INTMSK &= ~(1 << irq);
}

#define EINTIRQ_OFFSET(x)		((x) - 32 + 4)

static void EINT4_23mask_ack_irq(unsigned int irq)
{
	EINTMASK |= (1 << EINTIRQ_OFFSET(irq));
	EINTPEND = (1 << EINTIRQ_OFFSET(irq));

	if (irq <= EINTIRQ_OFFSET(IRQ_EINT7)) {
		SRCPND = (1 << IRQ_EINT4_7);
		INTPND = (1 << IRQ_EINT4_7);
	} else {
		SRCPND = (1 << IRQ_EINT8_23);
		INTPND = (1 << IRQ_EINT8_23);
	}
}

static void EINT4_23mask_irq(unsigned int irq)
{
	irq = EINTIRQ_OFFSET(irq);
	EINTMASK |= (1 << irq);
}

static void EINT4_23unmask_irq(unsigned int irq)
{
	EINTMASK &= ~(1 << EINTIRQ_OFFSET(irq));

	if (irq <= IRQ_EINT7) {
	  INTMSK &= ~(1 << IRQ_EINT4_7);
	} else {
	  INTMSK &= ~(1 << IRQ_EINT8_23);
	}
}

#define SUBIRQ_OFFSET(x)	((x) - 52)

static void SUB_mask_ack_irq(unsigned int irq)
{
	INTSUBMSK |= (1 << SUBIRQ_OFFSET(irq));
	SUBSRCPND = (1 << SUBIRQ_OFFSET(irq));

	if (irq <= IRQ_ERR0) {
		SRCPND = (1 << IRQ_UART0);
		INTPND = (1 << IRQ_UART0);
        } else if (irq <= IRQ_ERR1) {
		SRCPND = (1 << IRQ_UART1);
		INTPND = (1 << IRQ_UART1);
	} else if (irq <= IRQ_ERR2) {
		SRCPND = (1 << IRQ_UART2);
		INTPND = (1 << IRQ_UART2);
        } else {
		SRCPND = (1 << IRQ_ADCTC);
		INTPND = (1 << IRQ_ADCTC);
	}
}

static void SUB_mask_irq(unsigned int irq)
{
	INTSUBMSK |= (1 << SUBIRQ_OFFSET(irq));
}

static void SUB_unmask_irq(unsigned int irq)
{
	INTSUBMSK &= ~(1 << SUBIRQ_OFFSET(irq));

	if (irq <= IRQ_ERR0) 
		INTMSK &= ~(1 << IRQ_UART0); 
        else if (irq <= IRQ_ERR1) 
		INTMSK &= ~(1 << IRQ_UART1);
	else if (irq <= IRQ_ERR2)
	    	INTMSK &= ~(1 << IRQ_UART2);
        else 
		INTMSK &= ~(1 << IRQ_ADCTC);
}

/*
 *  fixup_irq() for do_IRQ() in kernel/irq.c
 */
inline unsigned int fixup_extirq(int begin, int end, int fail) 
{
	int i;
	unsigned long pend;

	pend = EINTPEND & ~EINTMASK;
		  
	for(i=begin; i <= end; i++) 
		if (pend & (1<<i))
			return (32 - 4 + i);
	return fail;
}

inline unsigned int fixup_subirq(int begin, int end, int fail) 
{
	int i;
	unsigned long pend;

	pend = SUBSRCPND & ~INTSUBMSK;

	for(i=begin; i <= end; i++) 
		if (pend & (1<<i))
			return (52 + i);
	return fail;
}


unsigned int fixup_irq(int irq)
{
	switch (irq) {
	case IRQ_EINT4_7: 	return fixup_extirq(4,7, irq);
	case IRQ_EINT8_23:	return fixup_extirq(8,23, irq);
	case IRQ_UART0:		return fixup_subirq(0,2, irq);
	case IRQ_UART1:		return fixup_subirq(3,5, irq);
	case IRQ_UART2:		return fixup_subirq(6,8, irq);
    	case IRQ_ADCTC:		return fixup_subirq(9,10, irq);
	default :		return irq;
	}
}

static struct resource irq_resource = {
	name:	"irqs",
	start:	0x4a000000,
	end:	0x4a00001f,
};

static struct resource eint_resource = {
	name:	"external irqs",
	start:	0x56000088,
	end:	0x560000ab,
};

void __init s3c2440_init_irq(void)
{
	int irq;

	request_resource(&iomem_resource, &irq_resource);
	request_resource(&iomem_resource, &eint_resource);

	/* disable all IRQs */
	INTMSK = 0xffffffff;

	/* all IRQs are IRQ, not FIQ */
	INTMOD = 0x00000000;

	/* clear Source/Interrupt Pending Register */
	SRCPND = 0xffffffff;
	INTPND = 0xffffffff;
	SUBSRCPND = 0x7ff;
	EINTPEND = 0x00fffff0;

	/* Define irq handler */
	for (irq=0; irq < 32; irq++) {
		irq_desc[irq].valid	= 1;
		irq_desc[irq].probe_ok	= 1;
		irq_desc[irq].mask_ack	= s3c2440_mask_ack_irq;
		irq_desc[irq].mask	= s3c2440_mask_irq;
		irq_desc[irq].unmask	= s3c2440_unmask_irq;
	}

	for (irq=32; irq < 52; irq++) {
		irq_desc[irq].valid	= 0;
		irq_desc[irq].probe_ok	= 1;
		irq_desc[irq].mask_ack	= EINT4_23mask_ack_irq;
		irq_desc[irq].mask	= EINT4_23mask_irq;
		irq_desc[irq].unmask	= EINT4_23unmask_irq;
	}

	for (irq=52; irq < 63; irq++) {
		irq_desc[irq].valid	= 1;
		irq_desc[irq].probe_ok	= 1;
		irq_desc[irq].mask_ack	= SUB_mask_ack_irq;
		irq_desc[irq].mask	= SUB_mask_irq;
		irq_desc[irq].unmask	= SUB_unmask_irq;
	}  
}
