/*
 * drivers/mtd/nand/tx4938ndfmc.c
 *
 * Based on spia.c by Steven J. Hill
 *
 *  Overview:
 *   This is a device driver for the NAND flash device connected to
 *   TX4938 internal NAND Memory Controller.
 *   TX4938 NDFMC is almost same as TX4925 NDFMC, but register size are 64 bit.
 *
 * Author: source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 * Copyright (C) 2000-2001 Toshiba Corporation 
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/tx4938/rbtx4938.h>

/*
 * MTD structure for TX4938 NDFMC
 */
static struct mtd_info *tx4938ndfmc_mtd;

/*
 * Define partitions for flash device
 */
#define flush_wb()	(void)tx4938_ndfmcptr->mcr;

#define NUM_PARTITIONS  3
#ifndef CONFIG_MTD_CMDLINE_PARTS
static struct mtd_partition partition_info[NUM_PARTITIONS] = {
	{
		name: "RBTX4938 CIS Area",
 		offset:  0,
 		size:    24 * 0x00004000,
 		mask_flags : MTD_WRITEABLE	/* This partition is not writable */
 	},
 	{
 		name: "RBTX4938 kernel image",
 		offset:  MTDPART_OFS_APPEND,
 		size:    8 * 0x00100000,	/* 8MB */
 		mask_flags : MTD_WRITEABLE	/* This partition is not writable */
 	},
 	{
 		name: "Root FS (JFFS2)",
 		offset:  MTDPART_OFS_APPEND,
 		size:    MTDPART_SIZ_FULL
 	},
};
#endif

static void tx4938ndfmc_hwcontrol(int cmd)
{
	switch (cmd) {
		case NAND_CTL_SETCLE:
			tx4938_ndfmcptr->mcr |= TX4938_NDFMCR_CLE;
			break;
		case NAND_CTL_CLRCLE:
			tx4938_ndfmcptr->mcr &= ~TX4938_NDFMCR_CLE;
			break;
		case NAND_CTL_SETALE:
			tx4938_ndfmcptr->mcr |= TX4938_NDFMCR_ALE;
			break;
		case NAND_CTL_CLRALE:
			tx4938_ndfmcptr->mcr &= ~TX4938_NDFMCR_ALE;
			break;
		/* TX4938_NDFMCR_CE bit is 0:high 1:low */
		case NAND_CTL_SETNCE:
			tx4938_ndfmcptr->mcr |= TX4938_NDFMCR_CE;
			break;
		case NAND_CTL_CLRNCE:
			tx4938_ndfmcptr->mcr &= ~TX4938_NDFMCR_CE;
			break;
		case NAND_CTL_SETWP:
			tx4938_ndfmcptr->mcr |= TX4938_NDFMCR_WE;
			break;
		case NAND_CTL_CLRWP:
			tx4938_ndfmcptr->mcr &= ~TX4938_NDFMCR_WE;
			break;
	}
}
static int tx4938ndfmc_dev_ready(void)
{
	flush_wb();
	return !(tx4938_ndfmcptr->sr & TX4938_NDFSR_BUSY);
}
static void tx4938ndfmc_calculate_ecc(const u_char *dat, u_char *ecc_code)
{
	u32 mcr = tx4938_ndfmcptr->mcr;
	mcr &= ~TX4938_NDFMCR_ECC_ALL;
	tx4938_ndfmcptr->mcr = mcr | TX4938_NDFMCR_ECC_OFF;
	tx4938_ndfmcptr->mcr = mcr | TX4938_NDFMCR_ECC_READ;
	ecc_code[1] = tx4938_ndfmcptr->dtr;
	ecc_code[0] = tx4938_ndfmcptr->dtr;
	ecc_code[2] = tx4938_ndfmcptr->dtr;
	//printk("tx4938ndfmc_calculate_ecc: \n");
	//printk("ecc_code[0] is %x\n",ecc_code[0]);
	//printk("ecc_code[1] is %x\n",ecc_code[1]);
	//printk("ecc_code[2] is %x\n",ecc_code[2]);
	tx4938_ndfmcptr->mcr = mcr | TX4938_NDFMCR_ECC_OFF;
}
static void tx4938ndfmc_enable_hwecc(int mode)
{
	u32 mcr = tx4938_ndfmcptr->mcr;
	mcr &= ~TX4938_NDFMCR_ECC_ALL;
	tx4938_ndfmcptr->mcr = mcr | TX4938_NDFMCR_ECC_RESET;
	tx4938_ndfmcptr->mcr = mcr | TX4938_NDFMCR_ECC_OFF;
	tx4938_ndfmcptr->mcr = mcr | TX4938_NDFMCR_ECC_ON;
}

static u_char tx4938ndfmc_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	return tx4938_read_nfmc(this->IO_ADDR_R);
}

static void tx4938ndfmc_nand_write_byte(struct mtd_info *mtd, u_char byte)
{
	struct nand_chip *this = mtd->priv;
	tx4938_write_nfmc(byte, this->IO_ADDR_W);
}

#ifdef CONFIG_MTD_CMDLINE_PARTS
extern int parse_cmdline_partitions(struct mtd_info *master, struct mtd_partition **pparts, char *);
#endif
/*
 * Main initialization routine
 */
int __init tx4938ndfmc_init (void)
{
	struct nand_chip *this;
	int bsprt = 0, hold = 0xf, spw = 0xf;
	int protected = 0;

	if ((*rbtx4938_piosel_ptr & 0x0c) != 0x08) {
		//printk(KERN_DEBUG "TX4938 NDFMC: disabled by IOC PIOSEL\n");
		printk("TX4938 NDFMC: disabled by IOC PIOSEL\n");
		return -ENODEV;
	}
	bsprt = 1;
	hold = 2;
	spw = 9 - 1;	/* 8 GBUSCLK = 80ns (@ GBUSCLK 100MHz) */

	if ((tx4938_ccfgptr->pcfg &
	     (TX4938_PCFG_ATA_SEL|TX4938_PCFG_ISA_SEL|TX4938_PCFG_NDF_SEL))
	    != TX4938_PCFG_NDF_SEL) {
		//printk(KERN_DEBUG "TX4938 NDFMC: disabled by PCFG.\n");
		printk("TX4938 NDFMC: disabled by PCFG.\n");
		return -ENODEV;
	}

	/* reset NDFMC */
	tx4938_ndfmcptr->rstr |= TX4938_NDFRSTR_RST;
	while (tx4938_ndfmcptr->rstr & TX4938_NDFRSTR_RST)
		;
	/* setup BusSeparete, Hold Time, Strobe Pulse Width */
	tx4938_ndfmcptr->mcr = bsprt ? TX4938_NDFMCR_BSPRT : 0;
	tx4938_ndfmcptr->spr = hold << 4 | spw;

	/* Allocate memory for MTD device structure and private data */
	tx4938ndfmc_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				      GFP_KERNEL);
	if (!tx4938ndfmc_mtd) {
		printk ("Unable to allocate TX4938 NDFMC MTD device structure.\n");
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&tx4938ndfmc_mtd[1]);

	/* Initialize structures */
	memset((char *) tx4938ndfmc_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	tx4938ndfmc_mtd->priv = this;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = (unsigned long)&tx4938_ndfmcptr->dtr;
	this->IO_ADDR_W = (unsigned long)&tx4938_ndfmcptr->dtr;
	this->hwcontrol = tx4938ndfmc_hwcontrol;
	this->dev_ready = tx4938ndfmc_dev_ready;
	this->calculate_ecc = tx4938ndfmc_calculate_ecc;
	this->correct_data = nand_correct_data;
	this->enable_hwecc = tx4938ndfmc_enable_hwecc;
	this->eccmode = NAND_ECC_HW3_256;
	this->chip_delay = 100;
	this->read_byte = tx4938ndfmc_nand_read_byte;
	this->write_byte = tx4938ndfmc_nand_write_byte;

	/* Scan to find existance of the device */
	if (nand_scan (tx4938ndfmc_mtd)) {
		kfree (tx4938ndfmc_mtd);
		return -ENXIO;
	}

	/* Allocate memory for internal data buffer */
	this->data_buf = kmalloc (sizeof(u_char) * (tx4938ndfmc_mtd->oobblock + tx4938ndfmc_mtd->oobsize), GFP_KERNEL);
	if (!this->data_buf) {
		printk ("Unable to allocate NAND data buffer for TX4938.\n");
		kfree (tx4938ndfmc_mtd);
		return -ENOMEM;
	}

	/* Allocate memory for internal data buffer */
	this->data_cache = kmalloc (sizeof(u_char) * (tx4938ndfmc_mtd->oobblock + tx4938ndfmc_mtd->oobsize), GFP_KERNEL);
	if (!this->data_cache) {
		printk("Unable to allocate NAND data cache for TX4938.\n");
		kfree (this->data_buf);
		kfree (tx4938ndfmc_mtd);
		return -ENOMEM;
	}
	this->cache_page = -1;

	if (protected) {
		printk(KERN_INFO "TX4938 NDFMC: write protected.\n");
		tx4938ndfmc_mtd->flags &= ~(MTD_WRITEABLE | MTD_ERASEABLE);
	}

#ifdef CONFIG_MTD_CMDLINE_PARTS
	{
		int mtd_parts_nb = 0;
		struct mtd_partition *mtd_parts = 0;
		mtd_parts_nb = parse_cmdline_partitions(tx4938ndfmc_mtd, &mtd_parts, "tx4938ndfmc");
		if (mtd_parts_nb > 0)
			add_mtd_partitions(tx4938ndfmc_mtd, mtd_parts, mtd_parts_nb);
		else
			add_mtd_device(tx4938ndfmc_mtd);
	}
#else
	add_mtd_partitions(tx4938ndfmc_mtd, partition_info, NUM_PARTITIONS );
#endif

	return 0;
}
module_init(tx4938ndfmc_init);

/*
 * Clean up routine
 */
static void __exit tx4938ndfmc_cleanup (void)
{
	struct nand_chip *this = (struct nand_chip *) tx4938ndfmc_mtd->priv;

	/* Unregister the device */
#ifdef CONFIG_MTD_CMDLINE_PARTS
	del_mtd_partitions(tx4938ndfmc_mtd);
#endif
	del_mtd_device (tx4938ndfmc_mtd);

	/* Free the MTD device structure */
	kfree (tx4938ndfmc_mtd);

	/* Free internal data buffer */
	kfree (this->data_buf);
	kfree (this->data_cache);
}
module_exit(tx4938ndfmc_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Board-specific glue layer for NAND flash on TX4938 NDFMC");
