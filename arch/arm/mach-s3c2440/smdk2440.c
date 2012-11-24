/*
 * linux/arch/arm/mach-s3c2440/smdk2440.c
 *
 * Copyright (C) 2003 Samsung Electronics
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/serial_core.h>

#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/page.h>
#include <asm/pgtable.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "generic.h"

#ifdef CONFIG_BOOT_FREQ
#include <asm/arch/cpu_s3c2440.h>
#endif /* CONFIG_BOOT_FREQ */

static void __init
fixup_smdk2440(struct machine_desc *desc, struct param_struct *params,
		char **cmdline, struct meminfo *mi)
{
        mi->bank[0].start = 0x30000000;
        mi->bank[0].size =  64*1024*1024;
        mi->bank[0].node =  0;

        mi->nr_banks = 1;

        ROOT_DEV = MKDEV(RAMDISK_MAJOR,0);
        setup_ramdisk( 1, 0, 0, 8192 );
        setup_initrd( 0xc0800000, 3*1024*1024 );
}

static struct map_desc smdk_io_desc[] __initdata = {
    /* virtual    physical    length      domain     r  w  c  b */
    { vCS8900_BASE, pCS8900_BASE, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 },	
   LAST_DESC
};

static void __init smdk_map_io(void)
{
	s3c2440_map_io();
	iotable_init(smdk_io_desc);

#ifdef CONFIG_BOOT_FREQ
        s3c2440_ck_init_boot();
#endif /* CONFIG_BOOT_FREQ */

	s3c2440_register_uart(0, 0);
	s3c2440_register_uart(1, 1);
	s3c2440_register_uart(2, 2);
}

MACHINE_START(SMDK2440, "Samsung-SMDK2440")
	BOOT_MEM(0x30000000, 0x48000000, 0xe8000000)
	BOOT_PARAMS(0x30000100)
	FIXUP(fixup_smdk2440)
	MAPIO(smdk_map_io)
	INITIRQ(s3c2440_init_irq)
MACHINE_END
