/* -------------------------------------------------------------------- */
/* rts7751r2d-flash.c:                                                     */
/* -------------------------------------------------------------------- */
/*  This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    Copyright 2003 (c) Lineo uSolutions,Inc.
*/
/* -------------------------------------------------------------------- */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#define BUSWIDTH   2 

#undef FLASH4M_16BIT
#define FLASH16M_16BIT

#if defined(FLASH4M_16BIT)
#define RTS7751R2D_FLASH_SIZE 0x00100000
#else
#define RTS7751R2D_FLASH_SIZE 0x01000000
#endif

static __u8 rts7751r2d_read8(struct map_info *map, unsigned long ofs)
{
//	printk("rts7751r2d_read8 %x\n",readb(map->map_priv_1 + ofs));
	return readb(map->map_priv_1 + ofs);
}

static __u16 rts7751r2d_read16(struct map_info *map, unsigned long ofs)
{
//	printk("rts7751r2d_read16 address %x data %x\n", map->map_priv_1+ofs, readw(map->map_priv_1 + ofs));
	return readw(map->map_priv_1 + ofs);
}

static __u32 rts7751r2d_read32(struct map_info *map, unsigned long ofs)
{
//	printk("rts7751r2d_read32 %x\n",readl(map->map_priv_1 + ofs));
	return readl(map->map_priv_1 + ofs);
}

static void rts7751r2d_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	memcpy_fromio(to, (map->map_priv_1 + from), len);
}

static void rts7751r2d_write8(struct map_info *map, __u8 d, unsigned long adr)
{
//	printk("w8:%x->%x\n",adr,d);
	writeb(d , map->map_priv_1 + adr);
	
}

static void rts7751r2d_write16(struct map_info *map, __u16 d, unsigned long adr)
{
//	printk("w16:address %x-> data %x\n",adr,d);
	writew(d , map->map_priv_1 + adr);
}

static void rts7751r2d_write32(struct map_info *map, __u32 d, unsigned long adr)
{
//	printk("w32:%x->%x\n",adr,d);
	writel(d , map->map_priv_1 + adr);
}

static void rts7751r2d_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	printk("copy_to %x -> %x\n",from,map->map_priv_1 + to);
	memcpy((void *)(map->map_priv_1 + to), from, len);
}

static struct map_info rts7751r2d_map = {
	name:		"SH-Graphic flash",
	buswidth:   BUSWIDTH,
	size:       RTS7751R2D_FLASH_SIZE,
	read8:		rts7751r2d_read8,
	read16:		rts7751r2d_read16,
	read32:		rts7751r2d_read32,
	copy_from:	rts7751r2d_copy_from,
	write8:		rts7751r2d_write8,
	write16:	rts7751r2d_write16,
	write32:	rts7751r2d_write32,
	copy_to:	rts7751r2d_copy_to,

};


/*
 * Here are partition information for all known SH-Graphic based devices.
 * See include/linux/mtd/partitions.h for definition of the mtd_partition
 * structure.
 * 
 * The *_max_flash_size is the maximum possible mapped flash size which
 * is not necessarily the actual flash size.  It must correspond to the 
 * value specified in the mapping definition defined by the
 * "struct map_desc *_io_desc" for the corresponding machine.
 */

#if defined(FLASH4M_16BIT)
static struct mtd_partition rts7751r2d_partitions[] = {
	{
		name: "bootloader",
		size:   0x00080000,
		offset: 0xa0000000,
		mask_flags: MTD_WRITEABLE  /* force read-only */
	},{
		name: "SH-Graphic jffs",
		size:   0x00080000,
		offset: 0xa0080000
	}
};
#else
static struct mtd_partition rts7751r2d_partitions[] = {
        {
                name: "bootloader",
                size:   0x00020000,
                offset: 0x00000000,
                mask_flags: MTD_WRITEABLE  /* force read-only */
        },{
                name: "mtdblock1",
                size:   0x00300000,
                offset: 0x00020000,
        },{
                name: "mtdblock2",
                size:   0x004e0000,
                offset: 0x00320000,
        },{
                name: "mtdblock3",
                size:   0x00800000,
                offset: 0x00800000
        }
};
#endif

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

static struct mtd_partition *parsed_parts;
static struct mtd_info *mymtd;

int __init rts7751r2d_mtd_init(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	int parsed_nr_parts = 0;
	char *part_type;

	/* Default flash buswidth */

	/*
	 * Static partition definition selection
	 */
	part_type = "static";
	parts = rts7751r2d_partitions;
	nb_parts = NB_OF(rts7751r2d_partitions);
	rts7751r2d_map.map_priv_1 = P2SEGADDR(0);

	/*
	 * Now let's probe for the actual flash.  Do it here since
	 * specific machine settings might have been set above.
	 */
	printk(KERN_NOTICE "RTS7751R2D flash: probing %d-bit flash bus\n", rts7751r2d_map.buswidth*8);

#if defined(CONFIG_XIP_KERNEL)	/* CRAMFS */
	mymtd = do_map_probe("map_rom", &rts7751r2d_map);
#else	/* JFFS2 */
	mymtd = do_map_probe("cfi_probe", &rts7751r2d_map);
#endif
//	mymtd = do_map_probe("jedec_probe", &rts7751r2d_map);
	if (!mymtd)
		return -ENXIO;
	mymtd->owner = THIS_MODULE;
//	mymtd->erasesize = 0x2000;
	mymtd->erasesize = 0x10000;

	/*
	 * Dynamic partition selection stuff (might override the static ones)
	 */

	if (parsed_nr_parts > 0) {
		parts = parsed_parts;
		nb_parts = parsed_nr_parts;
	}

	if (nb_parts == 0) {
		printk(KERN_NOTICE "RTS7751R2D partition info available, registering whole flash at once\n");
		add_mtd_device(mymtd);
	} else {
		printk(KERN_NOTICE "Using %s partition definition\n", part_type);
		add_mtd_partitions(mymtd, parts, nb_parts);
	}
	return 0;
}

static void __exit rts7751r2d_mtd_cleanup(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
		if (parsed_parts)
			kfree(parsed_parts);
	}
}

module_init(rts7751r2d_mtd_init);
module_exit(rts7751r2d_mtd_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lineo uSolutions,Inc.");
MODULE_DESCRIPTION("MTD map driver for RTS7751R2D base board");
