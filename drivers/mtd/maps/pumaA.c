/*
 * Handle mapping of the flash on PUMA-A (PPC 6750FX) boards
 *
 * Author:	Gary Thomas <gary@mlbassoc.com>
 * Copyright:	(C) 2003 MLB Associates
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <platforms/pumaA.h>
#include <asm/gt64260_defs.h>   // REMOVE ME!


#define WINDOW_ADDR PUMA_EXT_FLASH_BASE
#define WINDOW_SIZE PUMA_EXT_FLASH_SIZE

/* Flash / Partition sizing */
#define MAX_SIZE_KiB              8192
#define BOOT_PARTITION_SIZE_KiB    512
#define KERNEL_PARTITION_SIZE_KiB 5632
#define APP_PARTITION_SIZE_KiB    2048

#define NUM_PARTITIONS 3

/* partition_info gives details on the logical partitions that the split the
 * single flash device into. If the size is zero we use up to the end of the
 * device. */
static struct mtd_partition partition_info[]={
	{ name: "PUMA-A flash BOOT partition",
	offset: 0,
	size:   BOOT_PARTITION_SIZE_KiB*1024 },
	{ name: "PUMA-A flash DATA partition",
	offset: BOOT_PARTITION_SIZE_KiB*1024,
	size: (KERNEL_PARTITION_SIZE_KiB)*1024 },
	{ name: "PUMA-A flash APPLICATION partition",
	offset: (BOOT_PARTITION_SIZE_KiB+KERNEL_PARTITION_SIZE_KiB)*1024 }
};
				   
static struct mtd_partition *parsed_parts;
static struct mtd_info *mymtd;

__u8 puma_read8(struct map_info *map, unsigned long ofs)
{
    __u8 val;
    val = readb(map->map_priv_1 + ofs);
    return val;
}

__u16 puma_read16(struct map_info *map, unsigned long ofs)
{
    __u16 val;
    val = readw(map->map_priv_1 + ofs);
    return val;
}

__u32 puma_read32(struct map_info *map, unsigned long ofs)
{
    __u32 val;
    val = readl(map->map_priv_1 + ofs);
    return val;
}

void puma_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
    memcpy_fromio(to, (void *)(map->map_priv_1 + from), len);
}

void puma_write8(struct map_info *map, __u8 d, unsigned long adr)
{
    writeb(d, map->map_priv_1 + adr);
}

void puma_write16(struct map_info *map, __u16 d, unsigned long adr)
{
    writew(d, map->map_priv_1 + adr);
}

void puma_write32(struct map_info *map, __u32 d, unsigned long adr)
{
    writel(d, map->map_priv_1 + adr);
}

void puma_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
    memcpy_toio((void *)(map->map_priv_1 + to), from, len);
}

struct map_info puma_map = {
	name: "PUMA-A flash",
	size: WINDOW_SIZE,
	buswidth: 1,
	read8: puma_read8,
	read16: puma_read16,
	read32: puma_read32,
	copy_from: puma_copy_from,
	write8: puma_write8,
	write16: puma_write16,
	write32: puma_write32,
	copy_to: puma_copy_to
};

int __init init_puma(void)
{
    struct mtd_partition *parts;
    int nb_parts = 0;
    int parsed_nr_parts = 0;
    char *part_type;
    struct puma_cpld *cpld;

    printk(KERN_NOTICE "Momentum PUMA-A (PPC 750FX) flash device: %x at %x\n", WINDOW_SIZE, WINDOW_ADDR);
    puma_map.map_priv_1 = (unsigned long)ioremap(WINDOW_ADDR, WINDOW_SIZE);
    cpld = (struct puma_cpld *)ioremap(PUMA_CPLD_BASE, PUMA_CPLD_SIZE);

    if (!puma_map.map_priv_1) {
        printk("Failed to ioremap FLASH\n");
        return -EIO;
    }
    if (!cpld) {
        printk("Failed to ioremap CPLD\n");
        return -EIO;
    }
    // Enable write access (so probe can work)
    cpld->flash_ctl |= 0x20;
    mymtd = do_map_probe("cfi_probe", &puma_map);
    if (mymtd) {
        mymtd->module = THIS_MODULE;
#ifdef CONFIG_MTD_REDBOOT_PARTS
	if (parsed_nr_parts == 0) {
            int ret = parse_redboot_partitions(mymtd, &parsed_parts, (void *)WINDOW_ADDR);
            if (ret > 0) {
                part_type = "RedBoot";
                parsed_nr_parts = ret;
            }
	}
#endif
	if (parsed_nr_parts > 0) {
            parts = parsed_parts;
            nb_parts = parsed_nr_parts;
	}
	if (nb_parts == 0) {
            printk(KERN_NOTICE "Momentum PUMA-A flash: no partition info available, registering whole flash at once\n");
            add_mtd_device(mymtd);
	} else {
            printk(KERN_NOTICE "Using %s partition definition\n", part_type);
            add_mtd_partitions(mymtd, parts, nb_parts);
	}
        return 0;
    }

    iounmap((void *)puma_map.map_priv_1);
    return -ENXIO;
}

static void __exit cleanup_puma(void)
{
    if (mymtd) {
        del_mtd_device(mymtd);
        map_destroy(mymtd);
    }
    if (puma_map.map_priv_1) {
        iounmap((void *)puma_map.map_priv_1);
        puma_map.map_priv_1 = 0;
    }
}

module_init(init_puma);
module_exit(cleanup_puma);

MODULE_AUTHOR("Gary Thomas <gary@mlbassoc.com>");
MODULE_DESCRIPTION("MTD map driver for Momentum PUMA-A (PPC 750FX) board");
MODULE_LICENSE("GPL");
