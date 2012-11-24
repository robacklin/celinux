/*
 * arch/mips/ps2/reset.c
 *
 * PlayStation 2 Reset Routines
 *
 * Copyright (C) 2001 MontaVista Software, Inc.
 * 
 * Written by: Paul Mundt <pmundt@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>

void ps2_machine_restart(void)
{
	/* FIXME: Unsupported at the moment.. */
}

void ps2_machine_halt(void)
{
	printk(KERN_NOTICE "\n** You can safely turn off the power\n");

	/* 
	 * Should halt properly, but this will do for now..
	 */
	for (;;) {
		__asm__ __volatile__ (
			".set mips3\n\t"
			".set noreorder\n\t"
			"wait\n\t"
			".set reorder\n\t"
			".set mips0\n\t"
			: /* no outputs */
			: /* no inputs */
		);
	}
}

void ps2_machine_power_off(void)
{
	/*
	 * Again, we should make an attempt to actually power
	 * off the machine at this point.. but we'll just
	 * wait indefinately instead (until a proper implementation
	 * happens).
	 */
	ps2_machine_halt();
}

