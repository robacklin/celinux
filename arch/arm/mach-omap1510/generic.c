/*
 * linux/arch/arm/mach-omap/generic.c
 *
 * Code common to all OMAP machines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>

#include <asm/pm.h>
#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/arch/ck.h>
#include <asm/io.h>

#include "generic.h"

int
cpu_type(void)
{
	int processor_type;
	processor_type = inl(OMAP1510_ID_CODE_REG);
	if (processor_type == 0) {
		return OMAP1509;
	} else {
		return OMAP1510;
	}
}

/*
 * Common OMAP I/O mapping
 *
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */

#define MAP_DESC(base,start,size,domain,r,w,c,b) { base,start,size,domain,r,w,c,b }
static struct map_desc standard_io_desc[] __initdata = {
	MAP_DESC(IO_BASE,
		 IO_START,
		 IO_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
	MAP_DESC(OMAP1510_DSP_BASE,
		 OMAP1510_DSP_START,
		 OMAP1510_DSP_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
	MAP_DESC(OMAP1510_DSPREG_BASE,
		 OMAP1510_DSPREG_START,
		 OMAP1510_DSPREG_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
#if defined(CONFIG_FB_OMAP) && !defined(CONFIG_FB_SDRAM)
	MAP_DESC(OMAP1510_SRAM_BASE,
		 OMAP1510_SRAM_START,
		 OMAP1510_SRAM_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
#elif defined(CONFIG_OMAP1510_PM)
	MAP_DESC(OMAP1510_SRAM_IDLE_SUSPEND,
		 __pa(OMAP1510_SRAM_IDLE_SUSPEND),
		 OMAP1510_SRAM_BASE + OMAP1510_SRAM_SIZE -
		 OMAP1510_SRAM_IDLE_SUSPEND,
		 DOMAIN_IO,
		 0,1,0,0),
#endif
	LAST_DESC
};

void __init omap1510_map_io(void)
{
	iotable_init(standard_io_desc);

	/* REVISIT: Refer to OMAP5910 Errata, Advisory SYS_1: "Timeout Abort
	 * on a Posted Write in the TIPB Bridge".
	 */
	outw(0x0, MPU_PUBLIC_TIPB_CNTL_REG);
	outw(0x0, MPU_PRIVATE_TIPB_CNTL_REG);

	/* Must init clocks early to assure that timer interrupt works
	 */
	init_ck();

	return;
}

EXPORT_SYMBOL(cpu_type);
