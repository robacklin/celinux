/*
 * linux/drivers/pcmcia/pxa/trizeps2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) 2002 Accelent Systems, Inc. All Rights Reserved
 * 
 * Platform specific routines for the Keith-n-Koep Trizeps-II, based on IDP
 *
 * Copyright (c) 2003 Teradyne DS, Ltd.
 * Port to Trizeps-2 MT6N board by Luc De Cock
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>

#include <pcmcia/ss.h>

#include <asm/delay.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/arch/pcmcia.h>

static int trizeps2_pcmcia_init(struct pcmcia_init *init)
{
	int return_val = 0;
	unsigned short *bcr = (unsigned short *) TRIZEPS2_BCR_BASE;
	unsigned short val;

	/* reset the PCMCIA controller */
	val = trizeps2_bcr_shadow | BCR_PCMCIA_RESET;
	*bcr = val;
	udelay(500);
	/* un-reset it again */
	trizeps2_bcr_shadow &= ~BCR_PCMCIA_RESET;
	/* enable the PCMCIA buffer */
	trizeps2_bcr_shadow &= ~(1 << 5);
	*bcr = trizeps2_bcr_shadow;

	GPDR(IRQ_TO_GPIO_2_80(PCMCIA_S_CD_VALID)) &=
	       	~GPIO_bit(IRQ_TO_GPIO_2_80(PCMCIA_S_CD_VALID));
	set_GPIO_IRQ_edge(IRQ_TO_GPIO_2_80(PCMCIA_S_CD_VALID),
				PCMCIA_S_CD_VALID_EDGE);
	GPDR(IRQ_TO_GPIO(PCMCIA_S_RDYINT)) &=
	       	~GPIO_bit(IRQ_TO_GPIO(PCMCIA_S_RDYINT));
	set_GPIO_IRQ_edge(IRQ_TO_GPIO(PCMCIA_S_RDYINT),
				PCMCIA_S_RDYINT_EDGE);

	return_val = request_irq(PCMCIA_S_CD_VALID, init->handler, SA_INTERRUPT,
			"PXA PCMCIA CD", NULL);
	if (return_val < 0) {
		return -1;
	}
	/* only 1 slot */
	return 1;
}

static int trizeps2_pcmcia_shutdown(void)
{
	free_irq(PCMCIA_S_CD_VALID, NULL);

	unsigned short *bcr = (unsigned short *) TRIZEPS2_BCR_BASE;
	trizeps2_bcr_shadow |= (1 << 5); /* pcmcia buffer off */
	*bcr = trizeps2_bcr_shadow;
	trizeps2_bcr_shadow &= 0xFFF0; /* pcmcia control logic grounded */
	*bcr = trizeps2_bcr_shadow;

	return 0;
}

static int trizeps2_pcmcia_socket_state(struct pcmcia_state_array *state_array)
{
	unsigned long status;
	int return_val = 1;
	volatile unsigned short *stat_regs[1] = {
	       	&TRIZEPS2_PCCARD_STATUS
	};

	if (state_array->size < 1)
		return -1;

	memset(state_array->state, 0,
	       (state_array->size) * sizeof (struct pcmcia_state));
	
	status = *stat_regs[0];

      	/* this one is a gpio */
	state_array->state[0].detect = (PCC_DETECT) ? 0 : 1;
	state_array->state[0].ready =  (PCC_READY) ? 1 : 0;
	state_array->state[0].bvd1   = (status & PCC_BVD1) ? 1 : 0;
	state_array->state[0].bvd2   = (status & PCC_BVD2) ? 1 : 0;
	state_array->state[0].wrprot = 0; /* r/w all the time */
	state_array->state[0].vs_3v  = (status & PCC_VS1) ? 0 : 1;
	state_array->state[0].vs_Xv  = (status & PCC_VS2) ? 0 : 1;

	return return_val;
}

static int trizeps2_pcmcia_get_irq_info(struct pcmcia_irq_info *info)
{
	switch (info->sock) {
	    case 0:
		info->irq = PCMCIA_S_RDYINT;
		break;

	    default:
		return -1;
	}

	return 0;
}

static int trizeps2_pcmcia_configure_socket(unsigned int sock, socket_state_t *state)
{
	unsigned short cntr_logic = trizeps2_bcr_shadow & 0xF;
	unsigned short *bcr = (unsigned short *) TRIZEPS2_BCR_BASE;

	/* configure Vcc and Vpp */
	switch (sock) {
	    case 0:
		switch (state->Vcc) {
		    case 0:
			cntr_logic &= ~(PCC_3V | PCC_5V);
			break;

		    case 33:
			cntr_logic &= ~(PCC_3V | PCC_5V);
			cntr_logic |= PCC_3V;
			break;

		    case 50:
			cntr_logic &= ~(PCC_3V | PCC_5V);
			cntr_logic |= PCC_5V;
			break;

		    default:
			printk(KERN_ERR "%s(): unrecognized Vcc %u\n",
			       __FUNCTION__, state->Vcc);
			return -1;
		}

		switch (state->Vpp) {
		    case 0:
			cntr_logic &= ~(PCC_EN0 | PCC_EN1);
			break;

		    case 120:
			cntr_logic &= ~(PCC_EN0 | PCC_EN1);
			cntr_logic |= PCC_EN1;
			break;

		    default:
			if (state->Vpp == state->Vcc) {
				cntr_logic &= ~(PCC_EN0 | PCC_EN1);
				cntr_logic |= PCC_EN0;
			}
			else {
				printk(KERN_ERR "%s(): unrecognized Vpp %u\n",
				       __FUNCTION__, state->Vpp);
				return -1;
			}
	    }
	    trizeps2_bcr_shadow &= ~(PCC_EN0 | PCC_EN1 | PCC_3V | PCC_5V |
			    		BCR_PCMCIA_RESET);
	    trizeps2_bcr_shadow |= cntr_logic;
	    *bcr = trizeps2_bcr_shadow;
	    /* reset PCMCIA controller if requested */
	    trizeps2_bcr_shadow |=
	    	    (state->flags & SS_RESET) ? BCR_PCMCIA_RESET : 0;
	    *bcr = trizeps2_bcr_shadow;
	    udelay(500);
	    break;
	}
	return 0;
}

struct pcmcia_low_level trizeps2_pcmcia_ops = { 
  trizeps2_pcmcia_init,
  trizeps2_pcmcia_shutdown,
  trizeps2_pcmcia_socket_state,
  trizeps2_pcmcia_get_irq_info,
  trizeps2_pcmcia_configure_socket
};

