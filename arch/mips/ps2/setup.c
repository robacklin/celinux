/*
 * arch/mips/ps2/setup.c
 *
 * PlayStation 2 Setup Routines
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
#include <linux/config.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/sched.h>

#include <asm/addrspace.h>

extern void ps2_machine_restart(void);
extern void ps2_machine_halt(void);
extern void ps2_machine_power_off(void);

struct {
	struct resource mem;
	struct resource ee_reg;
	struct resource gs_reg;
	struct resource boot;
	struct resource ext_mem;
} ps2_resources = {
	{ "Main Memory",        0x00000000, 0x0fffffff, IORESOURCE_MEM },
	{ "EE Registers",       0x10000000, 0x11ffffff                 },
	{ "GS Registers",       0x12000000, 0x13ffffff                 },
	{ "Boot ROM",           0x1fc00000, 0x1fffffff                 },
	{ "Extend Main Memory", 0x40000000, 0x7fffffff, IORESOURCE_MEM },
};

void __init ps2_setup(void)
{
	_machine_restart   = ps2_machine_restart;
	_machine_halt      = ps2_machine_halt;
	_machine_power_off = ps2_machine_power_off;

	set_io_port_base(KSEG0);
	
	ioport_resource.start = ps2_resources.ee_reg.start;
	ioport_resource.end   = ps2_resources.gs_reg.end;

	iomem_resource.start  = ps2_resources.mem.start;
	iomem_resource.end    = ps2_resources.ext_mem.end;
}

