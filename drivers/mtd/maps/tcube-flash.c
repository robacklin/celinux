/*
 * Flash memory access on T-Cube
 * 
 * Copyright (C) 2004 Lineo uSolutions, Inc.
 * 
 * Based on pb1xxx-flash.c by Pete Popov
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#ifdef 	DEBUG_RW
#define	DBG(x...)	printk(x)
#else
#define	DBG(x...)	
#endif

#define WINDOW_ADDR 0x1F000000
#define WINDOW_SIZE 0x1000000


static struct map_info tcube_map = {
	.name =	"t-cube flash",
};

static unsigned long flash_size = 0x01000000;
static unsigned char flash_buswidth = 2;
static struct mtd_partition tcube_partitions[] = {
#if 1
	{
		.name = "rootfs",
		.size = 0x00c00000,	/* 12MBytes */
		.offset = 0,
		.mask_flags = 0
	},{
		.name = "PMON",
		.size = 0x000e0000,	/* 896KBytes */
		.offset = MTDPART_OFS_APPEND,
		.mask_flags = MTD_WRITEABLE,
	},{
		.name = "params",
		.size = 0x00020000,	/* 128KBytes */
		.offset = MTDPART_OFS_APPEND,
		.mask_flags = MTD_WRITEABLE,
	},{
		.name = "kernel",
		.offset = MTDPART_OFS_APPEND,	/* 3MBytes */
		.size = MTDPART_SIZ_FULL,
		.mask_flags = 0,
        }
#else
	{
		.name = "kernel",
		.size = 0x00300000,	/* 3MBytes */
		.offset = 0,
		.mask_flags = 0
	},{
		.name = "rootfs",
		.size = 0x00900000,	/* 9MBytes */
		.offset = MTDPART_OFS_APPEND,
		.mask_flags = 0,
	},{
		.name = "PMON",
		.size = 0x00100000,	/* 1MBytes */
		.offset = MTDPART_OFS_APPEND,
		.mask_flags = MTD_WRITEABLE,
	},{
		.name = "test",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
		.mask_flags = 0,
        }
#endif
};

static struct mtd_partition *parsed_parts;
static struct mtd_info *mymtd;

int __init tcube_mtd_init(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	char *part_type;
#ifdef CONFIG_XIP_ROM
	const char *driver_name = "map_rom";
#else
	const char *driver_name = "cfi_probe";
#endif
	
	/* Default flash buswidth */
	tcube_map.buswidth = flash_buswidth;

	/*
	 * Static partition definition selection
	 */
	part_type = "static";
	parts = tcube_partitions;
	nb_parts = ARRAY_SIZE(tcube_partitions);
	tcube_map.size = flash_size;

	/*
	 * Now let's probe for the actual flash.  Do it here since
	 * specific machine settings might have been set above.
	 */
	printk(KERN_NOTICE "T-Cube flash: probing %d-bit flash bus\n", 
			tcube_map.buswidth*8);
	tcube_map.phys = WINDOW_ADDR;
	tcube_map.virt = (unsigned long)ioremap(WINDOW_ADDR, WINDOW_SIZE);

	simple_map_init(&tcube_map);

	mymtd = do_map_probe(driver_name, &tcube_map);
	if (!mymtd) {
		iounmap((void*)tcube_map.virt);
		return -ENXIO;
	}
	mymtd->owner = THIS_MODULE;

	add_mtd_partitions(mymtd, parts, nb_parts);
	return 0;
}

static void __exit tcube_mtd_cleanup(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
		if (parsed_parts)
			kfree(parsed_parts);
	}
	if (tcube_map.virt)
		iounmap((void*)tcube_map.virt);
}

module_init(tcube_mtd_init);
module_exit(tcube_mtd_cleanup);

MODULE_AUTHOR("Lineo Solutions, Inc.");
MODULE_DESCRIPTION("T-Cube CFI map driver");
MODULE_LICENSE("GPL");
