/*
 * drivers/mtd/maps/arctic-mtd.c MTD mappings and partition tables for 
 *                              IBM 405LP Arctic boards.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002, International Business Machines Corporation
 * All Rights Reserved.
 *
 * Bishop Brock
 * IBM Research, Austin Center for Low-Power Computing
 * bcbrock@us.ibm.com
 * March 2002
 *
 * modified for Arctic by,
 * David Gibson
 * IBM OzLabs, Canberra, Australia
 * <arctic@gibson.dropbear.id.au>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/ibm4xx.h>

/*
 * 0 : 0xFE00 0000 - 0xFEFF FFFF : Filesystem 1 (16MB)
 * 1 : 0xFF00 0000 - 0xFF4F FFFF : kernel (5.12MB)
 * 2 : 0xFF50 0000 - 0xFFF5 FFFF : Filesystem 2 (10.624MB) (if non-XIP)
 * 3 : 0xFFF6 0000 - 0xFFFF FFFF : PIBS Firmware (640KB)
 */

#define FFS1_SIZE	0x01000000 /* 16MB */
#define KERNEL_SIZE	0x00500000 /* 5.12MB */
#define FFS2_SIZE	0x00a60000 /* 10.624MB */
#define FIRMWARE_SIZE	0x000a0000 /* 640K */

#ifdef CONFIG_XIP_ROM
#define PARTITIONS	1
#define SIZE		FFS1_SIZE
#else
#define PARTITIONS	4
#define SIZE		SUBZERO_BOOTFLASH_SIZE
#endif

#define NAME     "Arctic Linux Flash"
#define PADDR    SUBZERO_BOOTFLASH_PADDR
#define BUSWIDTH 2

/* Flash memories on these boards are memory resources, accessed big-endian. */

static u8
arctic_mtd_read8(struct map_info *map, unsigned long offset)
{
	return __raw_readb(map->map_priv_1 + offset);
}

static u16
arctic_mtd_read16(struct map_info *map, unsigned long offset)
{
	return __raw_readw(map->map_priv_1 + offset);
}

static u32
arctic_mtd_read32(struct map_info *map, unsigned long offset)
{
	return __raw_readl(map->map_priv_1 + offset);
}

static void
arctic_mtd_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	memcpy_fromio(to, (void *) (map->map_priv_1 + from), len);
}

static void
arctic_mtd_write8(struct map_info *map, u8 data, unsigned long address)
{
	__raw_writeb(data, map->map_priv_1 + address);
	mb();
}

static void
arctic_mtd_write16(struct map_info *map, u16 data, unsigned long address)
{
	__raw_writew(data, map->map_priv_1 + address);
	mb();
}

static void
arctic_mtd_write32(struct map_info *map, u32 data, unsigned long address)
{
	__raw_writel(data, map->map_priv_1 + address);
	mb();
}

static void
arctic_mtd_copy_to(struct map_info *map,
		  unsigned long to, const void *from, ssize_t len)
{
	memcpy_toio((void *) (map->map_priv_1 + to), from, len);
}

static struct map_info arctic_mtd_map = {
	.name		= NAME,
	.size		= SIZE,
	.buswidth	= BUSWIDTH,
	.read8		= arctic_mtd_read8,
	.read16		= arctic_mtd_read16,
	.read32		= arctic_mtd_read32,
	.copy_from	= arctic_mtd_copy_from,
	.write8		= arctic_mtd_write8,
	.write16	= arctic_mtd_write16,
	.write32	= arctic_mtd_write32,
	.copy_to	= arctic_mtd_copy_to,
};

static struct mtd_info *arctic_mtd;

static struct mtd_partition arctic_partitions[PARTITIONS] = {
	{ .name		= "Filesystem",
	  .size		= FFS1_SIZE,
	  .offset	= 0,},
#ifndef CONFIG_XIP_ROM
        { .name		= "Kernel",
	  .size		= KERNEL_SIZE,
	  .offset	= FFS1_SIZE,},
	{ .name		= "Filesystem",
	  .size		= FFS2_SIZE,
	  .offset	= FFS1_SIZE + KERNEL_SIZE,},
	{ .name		= "Firmware",
	  .size		= FIRMWARE_SIZE,
	  .offset	= SUBZERO_BOOTFLASH_SIZE - FIRMWARE_SIZE,},
#endif /* CONFIG_XIP_ROM */
};

static int __init
init_arctic_mtd(void)
{
	printk("%s: 0x%08x at 0x%08x\n", NAME, SIZE, PADDR);

	arctic_mtd_map.virt = (unsigned long) ioremap(PADDR, SIZE);

	if (!arctic_mtd_map.virt) {
		printk("%s: failed to ioremap 0x%x\n", NAME, PADDR);
		return -EIO;
	}
	simple_map_init(&arctic_mtd_map);

	printk("%s: probing %d-bit flash bus\n", NAME, BUSWIDTH * 8);
	arctic_mtd = do_map_probe("cfi_probe", &arctic_mtd_map);

	if (!arctic_mtd)
		return -ENXIO;

	arctic_mtd->module = THIS_MODULE;

	return add_mtd_partitions(arctic_mtd, arctic_partitions, PARTITIONS);
}

static void __exit
cleanup_arctic_mtd(void)
{
	if (arctic_mtd) {
		del_mtd_partitions(arctic_mtd);
		map_destroy(arctic_mtd);
		iounmap((void *) arctic_mtd_map.virt);
	}
}

module_init(init_arctic_mtd);
module_exit(cleanup_arctic_mtd);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Gibson <arctic@gibson.dropbear.id.au>");
MODULE_DESCRIPTION("MTD map and partitions for IBM 405LP Arctic boards");
