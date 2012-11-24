/*
 * Mapping for the Intel XScale IXP2000 based systems
 * Copyright:	(C) 2002 Intel Corp.
 * Author: 	Naeem M Afzal <naeem.m.afzal@intel.com>
 *
 * Maintainer: Deepak Saxena <dsaxena@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>


#define WINDOW_ADDR 	0xc4000000
#define WINDOW_SIZE 	16*1024*1024
#define BUSWIDTH 	1

#ifdef __ARMEB__
/*
 * Rev A0 and A1 of IXP2400 and all revs of IXP2800 silicon have a 
 * broken addressing unit which causes the lower address bits to be 
 * XORed with 0x11 on 8 bit accesses  and XORed with 0x10 on 16 bit 
 * accesses. See the spec update for IXP2400, erratta 44.
 */
static int errata44_workaround = 0;

static inline unsigned long address_fix8_read(unsigned long addr)
{
	return (addr ^ 2);
}

static inline unsigned long address_fix16_read(unsigned long addr)
{
	return (addr ^ 3);
}

static inline unsigned long address_fix8_write(unsigned long addr)
{
	if (errata44_workaround) {
		return (addr ^ 3);
	}
	return addr;
}

static inline unsigned long address_fix16_write(unsigned long addr)
{
	if (errata44_workaround) {
		return (addr ^ 2);
	}
	return addr;
}
#else

#define address_fix8_read(x)	(x)
#define address_fix16_read(x)	(x)
#define address_fix8_write(x)	(x)
#define address_fix16_write(x)	(x)

#endif

static __u8 ixp2000_read8(struct map_info *map, unsigned long ofs)
{
	return *(__u8 *) (address_fix8_read(map->virt + ofs));
}

static __u16 ixp2000_read16(struct map_info *map, unsigned long ofs)
{
	return *(__u16 *) (address_fix16_read(map->virt + ofs));
}

static __u32 ixp2000_read32(struct map_info *map, unsigned long ofs)
{
	return *(__u32 *) (map->virt + ofs);
}

static void ixp2000_copy_from(struct map_info *map, void *to,
			      unsigned long from, ssize_t len)
{
#ifdef __ARMEB__
	while(len--) 
		*(__u8 *) to++ = *(__u8 *) (address_fix8_read(map->virt + from++));
#else
	memcpy(to, (void *)(map->virt + from), len);
#endif
}

static void ixp2000_write8(struct map_info *map, __u8 d, unsigned long ofs)
{
	*(__u8 *) (address_fix8_write(map->virt + ofs)) = d;
}

static void ixp2000_write16(struct map_info *map, __u16 d,
			    unsigned long ofs)
{
	*(__u16 *) (address_fix16_write(map->virt + ofs)) = d;
}

static void ixp2000_write32(struct map_info *map, __u32 d,
			    unsigned long ofs)
{
	*(__u32 *) (map->virt + ofs) = d;
}

static void ixp2000_copy_to(struct map_info *map, unsigned long to,
			    const void *from, ssize_t len)
{
#ifdef __ARMEB__
	while(len--) {
		unsigned long tmp = address_fix8_write(map->virt + to++);
		*(__u8 *)(tmp) = *(__u8 *)(from++);
	}
#else
	memcpy((void *)(map->virt + to), from, len);
#endif
}

static struct map_info ixp2000_map = {
	name:		"IXP2000 flash",
	size:		WINDOW_SIZE,
	buswidth:	BUSWIDTH,
	read8:		ixp2000_read8,
//	read16:		ixp2000_read16,
//	read32:		ixp2000_read32,				
	copy_from:	ixp2000_copy_from,
	write8:		ixp2000_write8,
//	write16:	ixp2000_write16,
//	write32:	ixp2000_write32,
	copy_to:	ixp2000_copy_to
};

#ifdef CONFIG_ARCH_IXDP2400
static struct mtd_partition ixp2000_partitions[4] = {
	{
		name:           "RedBoot",
		size:           0x00040000,
		offset:         0,
		mask_flags:     MTD_WRITEABLE  /* force read-only */
	},{
		name:           "System Log",
		size:           0x00020000,
		offset:         0x00fa0000,
	},{
		name:           "linux",
		size:           0x100000,
		offset:         0x00100000,
	},{
		name:           "ramdisk",
		size:           0x400000,
		offset:         0x00200000,
	}
};
#elif defined(CONFIG_ARCH_IXDP2800)
static struct mtd_partition ixp2000_partitions[] = {
	{
		name:           "vBOOT",
		size:           0x00100000,
		offset:         0,
		mask_flags:     MTD_WRITEABLE  /* force read-only */
	},{
		name:           "vWARE FFS",
		size:           0x00700000,
		offset:         0x00100000,
		mask_flags:	MTD_WRITEABLE  /* force read-only */
	},{
		name:           "vWARE free",
		size:           0x00400000,
		offset:         0x00800000,
		mask_flags:	MTD_WRITEABLE  /* force read-only */
	},{
		name:		"free",
		size:		0x00400000,
		offset:		0x00c00000,
	}
};

#else 
#error No Architecture defined for MTD partition
#endif

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

static struct mtd_info *mymtd;
static struct mtd_partition *parsed_parts;
static const char *probes[] = { "RedBoot", "cmdlinepart", NULL };

extern int parse_redboot_partitions(struct mtd_info *master, struct mtd_partition **pparts);

static int __init init_ixp2000(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	int parsed_nr_parts = 0;

#ifdef __ARMEB__
	/*
	 * Enable errata 44 workaround for NPUs with broken slowport
	 */

	errata44_workaround = npu_has_broken_slowport();
	printk(KERN_NOTICE "IXP2000 Flash: Errata 44 workaround %s\n",
	       errata44_workaround ? "enabled" : "disabled");
#endif

	ixp2000_map.virt = (unsigned long)ioremap(WINDOW_ADDR, WINDOW_SIZE);
	if (!ixp2000_map.virt) {
		printk("Failed to ioremap\n");
		return -EIO;
	}

	mymtd = do_map_probe("cfi_probe", &ixp2000_map);
	if (!mymtd) {
		iounmap((void *)ixp2000_map.virt);
		return -ENXIO;
	}

	mymtd->owner = THIS_MODULE;
	mymtd->priv = &ixp2000_map;

	parsed_nr_parts = parse_mtd_partitions(mymtd, probes, &parsed_parts, 0);

	if (parsed_nr_parts > 0) {
		parts = parsed_parts;
		nb_parts = parsed_nr_parts;
	} else {
		parts = ixp2000_partitions;
		nb_parts = NB_OF(ixp2000_partitions);
		printk(KERN_NOTICE "Using static MTD partition definition\n");
	}

	add_mtd_partitions(mymtd, parts, nb_parts);
	return 0;
}

static void __exit cleanup_ixp2000(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
		if (parsed_parts)
			kfree(parsed_parts);
	}

	if (ixp2000_map.virt)
		iounmap((void *)ixp2000_map.virt);
}

module_init(init_ixp2000);
module_exit(cleanup_ixp2000);
MODULE_LICENSE("GPL");

