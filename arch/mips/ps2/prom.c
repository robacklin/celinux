/*
 * arch/mips/ps2/prom.c
 *
 * PlayStation 2 PROM Library Initialization Routines
 *
 * Copyright (C) 2001 Paul Mundt <lethal@chaoticdreams.org>
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/init.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <asm/bootinfo.h>
#include <asm/addrspace.h>

char arcs_cmdline[COMMAND_LINE_SIZE];

void __init prom_init(int argc, char **argv, char **envp)
{
	int i;

	for (i = 1; i < argc; i++) {
		strcat(arcs_cmdline, argv[i]);

		if (i < (argc - 1))
			strcat(arcs_cmdline, " ");
	}

	/*
	 * Only deals with the PS2 right now.. worry about the DTL-10000
	 * later..
	 */
	mips_machgroup = MACH_GROUP_EE;
	mips_machtype  = MACH_PS2;

	add_memory_region(0, PAGE_ALIGN((32 << 20) - PAGE_SIZE), BOOT_MEM_RAM);
}

void __init prom_free_prom_memory(void)
{
}

void __init prom_fixup_mem_map(unsigned long start, unsigned long end)
{
}

