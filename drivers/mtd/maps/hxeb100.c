/*
 *
 *  drivers/mtd/maps/hxeb100.c
 *
 *  Copyright 2003, MontaVista Software, Inc.
 *
 *  This software may be used and distributed according to the terms of
 *  the GNU Public License, Version 2, incorporated herein by reference.
 *
 *  Contact:  <source@mvista.com>
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/config.h>

#include <asm/gt64260.h>
#include <platforms/hxeb100.h>

#define BANK_A_FLASH_ADDR	HXEB100_BANK_A_FLASH_BASE
#define BANK_A_FLASH_SIZE	HXEB100_BANK_A_FLASH_SIZE
#define BANK_A_FLASH_WIDTH	HXEB100_BANK_A_FLASH_BUSWIDTH

#ifdef CONFIG_MTD_HXEB100_BANK_B_FLASH
#define BANK_B_FLASH_ADDR	HXEB100_BANK_B_FLASH_BASE
#define BANK_B_FLASH_SIZE	HXEB100_BANK_B_FLASH_SIZE
#define BANK_B_FLASH_WIDTH	HXEB100_BANK_B_FLASH_BUSWIDTH
static struct mtd_info *bank_b_flash;
#endif

static struct mtd_info *bank_a_flash;

static struct map_info hxeb100_bank_a_flash_map = {
	name: "HXEB100 Flash Bank A",
	size: BANK_A_FLASH_SIZE,
	buswidth: BANK_A_FLASH_WIDTH,
};

#ifdef CONFIG_MTD_HXEB100_BANK_B_FLASH
static struct map_info hxeb100_bank_b_flash_map = {
	name: "HXEB100 Flash Bank B",
	size: BANK_B_FLASH_SIZE,
	buswidth: BANK_B_FLASH_WIDTH,
};
#endif /* CONFIG_MTD_HXEB100_BANK_B_FLASH */

int __init init_hxeb100(void)
{
	printk(KERN_NOTICE "Flash Bank A: 0x%x at 0x%x\n", BANK_A_FLASH_SIZE, BANK_A_FLASH_ADDR);
	hxeb100_bank_a_flash_map.virt = (unsigned long)ioremap(BANK_A_FLASH_ADDR, BANK_A_FLASH_SIZE);
	if (!hxeb100_bank_a_flash_map.virt) {
		printk("Failed to ioremap bank_a_flash\n");
		return -EIO;
	}

	simple_map_init(&hxeb100_bank_a_flash_map);
	/* blah. Not much error checking XXX */
	bank_a_flash = do_map_probe("cfi_probe", &hxeb100_bank_a_flash_map);

	if (bank_a_flash) {
		bank_a_flash->owner = THIS_MODULE;
		add_mtd_device(bank_a_flash);
	} else {
		printk("map probe failed for bank_a_flash\n");
	}

#ifdef CONFIG_MTD_HXEB100_BANK_B_FLASH
	printk(KERN_NOTICE "Flash Bank B: 0x%x at 0x%x\n", BANK_B_FLASH_SIZE, BANK_B_FLASH_ADDR);
	hxeb100_bank_b_flash_map.virt = (unsigned long)ioremap(BANK_B_FLASH_ADDR, BANK_B_FLASH_SIZE);
	if (!hxeb100_bank_b_flash_map.virt) {
		printk("Failed to ioremap bank_b_flash\n");
		return -EIO;
	}

	simple_map_init(&hxeb100_bank_b_flash_map);
	bank_b_flash = do_map_probe("jedec_probe", &hxeb100_bank_b_flash_map);
	if (!bank_b_flash)
		bank_b_flash = do_map_probe("cfi_probe", &hxeb100_bank_b_flash_map);
	if (bank_b_flash) {
		bank_b_flash->owner = THIS_MODULE;
		add_mtd_device(bank_b_flash);
	} else {
		printk("map probe failed for bank_b_flash\n");
		return -ENXIO;
	}
#endif /* CONFIG_MTD_HXEB100_BANK_B_FLASH */

	return 0;
}

static void __exit cleanup_hxeb100(void)
{
	if (bank_a_flash) {
		del_mtd_device(bank_a_flash);
		map_destroy(bank_a_flash);
	}
	if (hxeb100_bank_a_flash_map.virt) {
		iounmap((void *)hxeb100_bank_a_flash_map.virt);
		hxeb100_bank_a_flash_map.virt = 0;
	}
#ifdef CONFIG_MTD_HXEB100_BANK_B_FLASH
	if (bank_b_flash) {
		del_mtd_device(bank_b_flash);
		map_destroy(bank_b_flash);
	}
	if (hxeb100_bank_b_flash_map.virt) {
		iounmap((void *)hxeb100_bank_b_flash_map.virt);
		hxeb100_bank_b_flash_map.virt = 0;
	}
#endif
}

module_init(init_hxeb100);
module_exit(cleanup_hxeb100);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HXEB100 flash map");
