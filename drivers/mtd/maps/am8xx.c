/*
 * $Id: am8xx.c,v 1.1.2.2 2003/05/03 14:04:33 gthomas Exp $
 *
 * Handle mapping of the flash on the A&M 8xx platforms
 * Patterned after "rpxlite.c"
 *
 * Copyright (c) 2003 Gary Thomas <gary@mlbassoc.com>
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>


#define WINDOW_ADDR 0xfe000000
#define WINDOW_SIZE 0x00400000

static struct mtd_partition *parsed_parts;
static struct mtd_info *mymtd;

__u8 am8xx_read8(struct map_info *map, unsigned long ofs)
{
    __u8 res;
    res = __raw_readb(map->map_priv_1 + ofs);
    return res;
}

__u16 am8xx_read16(struct map_info *map, unsigned long ofs)
{
    __u16 res;
    res = __raw_readw(map->map_priv_1 + ofs);
    return res;
}

__u32 am8xx_read32(struct map_info *map, unsigned long ofs)
{
    __u32 res;
    res = __raw_readl(map->map_priv_1 + ofs);
    return res;
}

void am8xx_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
    memcpy_fromio(to, (void *)(map->map_priv_1 + from), len);
}

void am8xx_write8(struct map_info *map, __u8 d, unsigned long adr)
{
    __raw_writeb(d, map->map_priv_1 + adr);
    mb();
}

void am8xx_write16(struct map_info *map, __u16 d, unsigned long adr)
{
    __raw_writew(d, map->map_priv_1 + adr);
    mb();
}

void am8xx_write32(struct map_info *map, __u32 d, unsigned long adr)
{
    __raw_writel(d, map->map_priv_1 + adr);
    mb();
}

void am8xx_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
    memcpy_toio((void *)(map->map_priv_1 + to), from, len);
}

struct map_info am8xx_map = {
    name: "A&M 8xx boards",
    size: WINDOW_SIZE,
#if defined(CONFIG_ADDER_II)
    buswidth: 2,
#elif defined(CONFIG_VIPER860)
    buswidth: 1,
#else
#error Unknown board
#endif
    read8: am8xx_read8,
    read16: am8xx_read16,
    read32: am8xx_read32,
    copy_from: am8xx_copy_from,
    write8: am8xx_write8,
    write16: am8xx_write16,
    write32: am8xx_write32,
    copy_to: am8xx_copy_to
};

int __init init_am8xx(void)
{
    struct mtd_partition *parts;
    int nb_parts = 0;
    int parsed_nr_parts = 0;
    char *part_type;
    printk(KERN_NOTICE "A&M 8xx flash device: %x at %x\n", WINDOW_SIZE, WINDOW_ADDR);
    am8xx_map.map_priv_1 = (unsigned long)ioremap(WINDOW_ADDR, WINDOW_SIZE);

    if (!am8xx_map.map_priv_1) {
        printk("Failed to ioremap\n");
        return -EIO;
    }
    mymtd = do_map_probe("cfi_probe", &am8xx_map);
    if (mymtd) {
        mymtd->module = THIS_MODULE;
#ifdef CONFIG_MTD_REDBOOT_PARTS
	if (parsed_nr_parts == 0) {
            int ret = parse_redboot_partitions(mymtd, &parsed_parts, 0);
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
            printk(KERN_NOTICE "A&M 8xx flash: no partition info available, registering whole flash at once\n");
            add_mtd_device(mymtd);
	} else {
            printk(KERN_NOTICE "Using %s partition definition\n", part_type);
            add_mtd_partitions(mymtd, parts, nb_parts);
	}
        return 0;
    }

    iounmap((void *)am8xx_map.map_priv_1);
    return -ENXIO;
}

static void __exit cleanup_am8xx(void)
{
    if (mymtd) {
        del_mtd_device(mymtd);
        map_destroy(mymtd);
    }
    if (am8xx_map.map_priv_1) {
        iounmap((void *)am8xx_map.map_priv_1);
        am8xx_map.map_priv_1 = 0;
    }
}

module_init(init_am8xx);
module_exit(cleanup_am8xx);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gary Thomas <gary@mlbassoc.com>");
MODULE_DESCRIPTION("MTD map driver for A&M 8xx boards");
