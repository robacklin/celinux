/*
 * Support for flash modules on ATI Xilleon 220 platforms
 *
 * Copyright (C) Robert Lembree <lembree@metrolink.com>
 *
 * ########################################################################
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * ########################################################################
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/io.h>
// #include <asm/arch/hardware.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/ati/xilleon.h>

static struct mtd_info *core_flash_mtd;

struct map_info xilleon_core_flash_map;

static int __init init_xilleon_core_flash (void)
{
	xilleon_core_flash_map.phys = (unsigned long)XILLEON_CORE_FLASH_BASE; 
	xilleon_core_flash_map.virt = 
		(unsigned long)ioremap(XILLEON_CORE_FLASH_BASE, 
				       XILLEON_CORE_FLASH_SIZE);
	simple_map_init(&xilleon_core_flash_map);
	core_flash_mtd = do_map_probe("jedec_probe", &xilleon_core_flash_map);
	if (!core_flash_mtd) {
		printk("Boot FLASH probe failed\n");
		return -ENXIO;
	}

	core_flash_mtd->owner = THIS_MODULE;
	
	if (add_mtd_device(core_flash_mtd)) {
		printk("Boot FLASH device addition failed\n");
		map_destroy(core_flash_mtd);
		core_flash_mtd = 0;
		return -ENOMEM;
	}
		
	return 0;
}


static struct mtd_info *user_flash_mtd;

struct map_info xilleon_user_flash_map;

static int __init init_xilleon_user_flash (void)
{
	xilleon_user_flash_map.phys = (unsigned long)XILLEON_USER_FLASH_BASE;
	xilleon_user_flash_map.virt = 
		(unsigned long)ioremap(XILLEON_USER_FLASH_BASE, 
				       XILLEON_USER_FLASH_SIZE);
	simple_map_init(&xilleon_user_flash_map);
	user_flash_mtd = do_map_probe("jedec_probe", &xilleon_user_flash_map);

	if (!user_flash_mtd) {
		printk("User FLASH probe failed\n");
		return -ENXIO;
	}

	user_flash_mtd->owner = THIS_MODULE;
	
	if (add_mtd_device(user_flash_mtd)) {
		printk("User FLASH device addition failed\n");
		map_destroy(user_flash_mtd);
		user_flash_mtd = 0;
		return -ENOMEM;
	}
		
	return 0;
}



static int __init init_xilleon_maps(void)
{

       	printk(KERN_INFO "XILLEON Flash mappings: \n");

	init_xilleon_core_flash();
	init_xilleon_user_flash();
	
	return 0;
}
	

static void __exit cleanup_xilleon_maps(void)
{
	if (core_flash_mtd) {
		del_mtd_device(core_flash_mtd);
		map_destroy(core_flash_mtd);
	}


	if (user_flash_mtd) {
		del_mtd_device(user_flash_mtd);
		map_destroy(user_flash_mtd);
	}

}

module_init(init_xilleon_maps);
module_exit(cleanup_xilleon_maps);

