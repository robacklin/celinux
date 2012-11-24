/*
 * linux/drivers/mtd/maps/omap-flash.c
 *
 * Flash memory access on OMAP based devices
 * 
 * Author: MontaVista Software, Inc.
 *         <gdavis@mvista.com> or <source@mvista.com>
 *
 * 2002, 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * History:
 *
 * 20020910: George G. Davis <gdavis@mvista.com>
 *	Initially based on drivers/mtd/map/sa1100-flash.c
 *	(C) 2000 Nicolas Pitre <nico@cam.org>
 * 20030731: George G. Davis <gdavis@mvista.com>
 *	Sync up with latest MTD CVS changes. Borrowed sa1100-flash.c
 *	as the baseline for omap-flash.c again.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/concat.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/sizes.h>

#ifndef CONFIG_ARCH_OMAP
#error This is for OMAP architecture only
#endif

/*
 * Here are partition information for all known OMAP based devices.
 * See include/linux/mtd/partitions.h for definition of the mtd_partition
 * structure.
 *
 * Please note:
 *  1. We no longer support static flash mappings via the machine io_desc
 *     structure.
 *  2. The flash size given should be the largest flash size that can
 *     be accommodated.
 *
 * The MTD layer will detect flash chip aliasing and reduce the size of
 * the map accordingly.
 *
 * Please keep these in alphabetical order, and formatted as per existing
 * entries.  Thanks.
 */

#ifdef CONFIG_OMAP_INNOVATOR
static struct mtd_partition innovator_partitions[] = {
	{
		.name		= "BootLoader",
		.size		= 0x00020000,
		.offset		= 0,
		.mask_flags	= MTD_WRITEABLE,  /* force read-only */
	}, {
		.name		= "Params",
		.size		= 0x00040000,
		.offset		= MTDPART_OFS_APPEND,
		.mask_flags	= MTD_WRITEABLE,  /* force read-only */
	}, {
		.name		= "Kernel",
		.size		= 0x00200000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "Flash0 FileSys",
		.size		= 0x00da0000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "Flash1 FileSys",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	}
};
#endif

static int __init
omap_static_partitions(struct mtd_partition **parts)
{
	int nb_parts = 0;

#ifdef CONFIG_OMAP_INNOVATOR
	if (machine_is_innovator()) {
		*parts       = innovator_partitions;
		nb_parts     = ARRAY_SIZE(innovator_partitions);
	}
#endif

	return nb_parts;
}

static void
omap_set_vpp(struct map_info *map, int vpp)
{
	uint v = *((volatile uint *) (OMAP_EMIFS_CONFIG_REG));

	if (vpp)
		v |= OMAP_EMIFS_CONFIG_WP;
	else
		v &= ~OMAP_EMIFS_CONFIG_WP;

	*((volatile uint *) (OMAP_EMIFS_CONFIG_REG)) = v;
}

struct omap_info {
	unsigned long base;
	unsigned long size;
	int width;
	void *vbase;
	void (*set_vpp) (struct map_info *, int);
	struct map_info *map;
	struct mtd_info *mtd;
	struct resource *res;
};

#define NR_SUBMTD 4

static struct omap_info info[NR_SUBMTD];

static int __init
omap_setup_mtd(struct omap_info *omap, int nr, struct mtd_info **rmtd)
{
	struct mtd_info *subdev[nr];
	struct map_info *maps;
	int i, found = 0, ret = 0;

	/*
	 * Allocate the map_info structs in one go.
	 */
	maps = kmalloc(sizeof (struct map_info) * nr, GFP_KERNEL);
	if (!maps)
		return -ENOMEM;

	memset(maps, 0, sizeof (struct map_info) * nr);

	/*
	 * Claim and then map the memory regions.
	 */
	for (i = 0; i < nr; i++) {
		if (omap[i].base == (unsigned long) -1)
			break;

		omap[i].res =
		    request_mem_region(omap[i].base, omap[i].size,
				       "omap-flash");
		if (!omap[i].res) {
			ret = -EBUSY;
			break;
		}

		omap[i].map = maps + i;

		omap[i].vbase = ioremap(omap[i].base, omap[i].size);
		if (!omap[i].vbase) {
			ret = -ENOMEM;
			break;
		}

		omap[i].map->virt = (unsigned long) omap[i].vbase;
		omap[i].map->phys = omap[i].base;
		omap[i].map->set_vpp = omap[i].set_vpp;
		omap[i].map->buswidth = omap[i].width;
		omap[i].map->size = omap[i].size;

		simple_map_init(omap[i].map);

		/*
		 * Now let's probe for the actual flash.  Do it here since
		 * specific machine settings might have been set above.
		 */
		omap[i].mtd = do_map_probe("cfi_probe", omap[i].map);
		if (omap[i].mtd == NULL) {
			ret = -ENXIO;
			break;
		}
		omap[i].mtd->owner = THIS_MODULE;
		subdev[i] = omap[i].mtd;

		printk(KERN_INFO "OMAP flash: CFI device at 0x%08lx, %dMiB, "
		       "%d-bit\n", omap[i].base, omap[i].mtd->size >> 20,
		       omap[i].width * 8);
		found += 1;
	}

	/*
	 * ENXIO is special.  It means we didn't find a chip when
	 * we probed.  We need to tear down the mapping, free the
	 * resource and mark it as such.
	 */
	if (ret == -ENXIO) {
		iounmap(omap[i].vbase);
		omap[i].vbase = NULL;
		release_resource(omap[i].res);
		omap[i].res = NULL;
	}

	/*
	 * If we found one device, don't bother with concat support.
	 * If we found multiple devices, use concat if we have it
	 * available, otherwise fail.
	 */
	if (ret == 0 || ret == -ENXIO) {
		if (found == 1) {
			*rmtd = subdev[0];
			ret = 0;
		} else if (found > 1) {
			/*
			 * We detected multiple devices.  Concatenate
			 * them together.
			 */
#ifdef CONFIG_MTD_CONCAT
			*rmtd = mtd_concat_create(subdev, found, "omap-flash");
			if (*rmtd == NULL)
				ret = -ENXIO;
#else
			printk(KERN_ERR "OMAP flash: multiple devices found "
			       "but MTD concat support disabled.\n");
			ret = -ENXIO;
#endif
		}
	}

	/*
	 * If we failed, clean up.
	 */
	if (ret) {
		do {
			if (omap[i].mtd)
				map_destroy(omap[i].mtd);
			if (omap[i].vbase)
				iounmap(omap[i].vbase);
			if (omap[i].res)
				release_resource(omap[i].res);
		} while (i--);

		kfree(maps);
	}

	return ret;
}

static void __exit
omap_destroy_mtd(struct omap_info *omap, struct mtd_info *mtd)
{
	int i;

	del_mtd_partitions(mtd);

#ifdef CONFIG_MTD_CONCAT
	if (mtd != omap[0].mtd)
		mtd_concat_destroy(mtd);
#endif
	for (i = NR_SUBMTD; i >= 0; i--) {
		if (omap[i].mtd)
			map_destroy(omap[i].mtd);
		if (omap[i].vbase)
			iounmap(omap[i].vbase);
		if (omap[i].res)
			release_resource(omap[i].res);
	}
	kfree(omap[0].map);
}

static int __init
omap_locate_flash(void)
{
	int nr = -ENODEV;

	if (machine_is_innovator()) {
		info[0].set_vpp = omap_set_vpp;
		info[0].base = OMAP_FLASH_0_START;
		info[0].size = OMAP_FLASH_0_SIZE;
		info[0].width = 2;
		info[1].set_vpp = omap_set_vpp;
		info[1].base = OMAP_FLASH_1_START;
		info[1].size = OMAP_FLASH_1_SIZE;
		info[1].width = 2;
		nr = 2;
	}

	if (nr < 0)
		return nr;

	return nr;
}

static struct mtd_partition *parsed_parts;
const char *part_probes[] = { "cmdlinepart", "RedBoot", NULL };

static void __init
omap_locate_partitions(struct mtd_info *mtd)
{
	const char *part_type = NULL;
	int nr_parts = 0;

        do {
                /*
                 * Partition selection stuff.
                 */
#ifdef CONFIG_MTD_PARTITIONS
                nr_parts = parse_mtd_partitions(mtd, part_probes,
						&parsed_parts, 0);
                if (nr_parts > 0) {
                        part_type = "dynamic";
                        break; 
                }
#endif
                nr_parts = omap_static_partitions(&parsed_parts);
                if (nr_parts > 0) {
                        part_type = "static";
                        break;
                }
        } while (0);

	if (nr_parts == 0) {
		printk(KERN_NOTICE "OMAP flash: no partition info available, "
		       "registering whole flash\n");
		add_mtd_device(mtd);
	} else {
		printk(KERN_NOTICE "OMAP flash: using %s partition "
		       "definition\n", part_type);
		add_mtd_partitions(mtd, parsed_parts, nr_parts);
	}

	/* Always succeeds. */
}

static void __exit
omap_destroy_partitions(void)
{
	if (parsed_parts)
		kfree(parsed_parts);
}

static struct mtd_info *mymtd;

static int __init
omap_mtd_init(void)
{
	int ret;
	int nr;

	nr = omap_locate_flash();
	if (nr < 0)
		return nr;

	ret = omap_setup_mtd(info, nr, &mymtd);
	if (ret == 0)
		omap_locate_partitions(mymtd);

	return ret;
}

static void __exit
omap_mtd_cleanup(void)
{
	omap_destroy_mtd(info, mymtd);
	omap_destroy_partitions();
}

module_init(omap_mtd_init);
module_exit(omap_mtd_cleanup);

MODULE_AUTHOR("George G. Davis");
MODULE_DESCRIPTION("OMAP CFI map driver");
MODULE_LICENSE("GPL");
