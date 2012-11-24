/*
 * $Id: iq80321.c,v 1.1.8.1 2003/09/05 17:39:46 mlocke Exp $
 *
 * Mapping for the Intel XScale IQ80321 evaluation board
 *
 * Author:	Rory Bolt <rorybolt@pacbell.net>
 * Copyright:	(C) 2002 Rory Bolt
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>


#define WINDOW_ADDR 	0xf0000000
#define WINDOW_SIZE 	8*1024*1024
#define BUSWIDTH 	1

static struct mtd_info *mymtd;

static struct map_info iq80321_map = {
	name: "IQ80321 flash",
	size: WINDOW_SIZE,
	buswidth: BUSWIDTH,
};

static struct mtd_partition iq80321_partitions[4] = {
	{
		name:		"Firmware",
		size:		0x00080000,
		offset:		0,
		mask_flags:	MTD_WRITEABLE  /* force read-only */
	},{
		name:		"Kernel",
		size:		0x000a0000,
		offset:		0x00080000,
	},{
		name:		"Filesystem",
		size:		0x00600000,
		offset:		0x00120000
	},{
		name:		"RedBoot",
		size:		0x000e0000,
		offset:		0x00720000,
		mask_flags:	MTD_WRITEABLE
	}
};

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

static struct mtd_info *mymtd;
static struct mtd_partition *parsed_parts;
static const char *probes[] = { "RedBoot", "cmdlinepart", NULL };

static int __init init_iq80321(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	int parsed_nr_parts = 0;
	char *part_type = "Static";

	iq80321_map.virt = (unsigned long)__ioremap(WINDOW_ADDR, WINDOW_SIZE, 0);
	if (!iq80321_map.virt) {
		printk("Failed to ioremap\n");
		return -EIO;
	}
	simple_map_init(&iq80321_map);

	mymtd = do_map_probe("cfi_probe", &iq80321_map);
	if (!mymtd) {
		iounmap((void *)iq80321_map.virt);
		return -ENXIO;
	}
	mymtd->owner = THIS_MODULE;

	parsed_nr_parts = parse_mtd_partitions(mymtd, probes, &parsed_parts, 0);

	if (parsed_nr_parts > 0) {
		parts = parsed_parts;
		nb_parts = parsed_nr_parts;
	} else {
		parts = iq80321_partitions;
		nb_parts = NB_OF(iq80321_partitions);
	}
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
	add_mtd_partitions(mymtd, parts, nb_parts);
	return 0;
}

static void __exit cleanup_iq80321(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
		if (parsed_parts)
			kfree(parsed_parts);
	}
	if (iq80321_map.virt)
		iounmap((void *)iq80321_map.virt);
}

module_init(init_iq80321);
module_exit(cleanup_iq80321);
MODULE_LICENSE("GPL");

