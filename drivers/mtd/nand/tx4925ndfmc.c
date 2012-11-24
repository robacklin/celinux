/*
 *  drivers/mtd/tx4925ndfmc.c
 *
 * Author: MontaVista Software, Inc.  source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 * Derived from drivers/mtd/autcpu12.c
 *       Copyright (c) 2001 Thomas Gleixner (gleixner@autronix.de)
 *
 * Copyright 2002 MontaVista Software Inc.
 * Author: Alice Hennessy, ahennessy@mvista.com
 *
 * Copyright (C) 2001 Toshiba Corporation 
 * 
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   Toshiba RBTX4925 reference board, which is a SmartMediaCard. It supports 
 *   16MB, 32MB and 64MB cards.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/tx4925/tx4925_nand.h>

/*
 * MTD structure for RBTX4925 board
 */
static struct mtd_info *tx4925ndfmc_mtd = NULL;

/*
 * Module stuff
 */
#if LINUX_VERSION_CODE < 0x20212 && defined(MODULE)
#define tx4925ndfmc_init init_module
#define tx4925ndfmc_cleanup cleanup_module
#endif

/*
 * Define partitions for flash devices
 */

static struct mtd_partition partition_info16k[] = {
	{ name: "RBTX4925 flash partition 1",
	  offset:  0,
	  size:    8 * 0x00100000 },
	{ name: "RBTX4925 flash partition 2",
	  offset:  8 * 0x00100000,
	  size:    8 * 0x00100000 },
};

static struct mtd_partition partition_info32k[] = {
	{ name: "RBTX4925 flash partition 1",
	  offset:  0,
	  size:    8 * 0x00100000 },
	{ name: "RBTX4925 flash partition 2",
	  offset:  8 * 0x00100000,
	  size:   24 * 0x00100000 },
};

static struct mtd_partition partition_info64k[] = {
	{ name: "User FS",
	  offset:  0,
	  size:   16 * 0x00100000 },
	{ name: "RBTX4925 flash partition 2",
	  offset: 16 * 0x00100000,
	  size:   48 * 0x00100000},
};

static struct mtd_partition partition_info128k[] = {
	{ name: "Skip bad section",
	  offset:  0,
	  size:   16 * 0x00100000 },
	{ name: "User FS",
	  offset: 16 * 0x00100000,
	  size:   112 * 0x00100000 },
};
#define NUM_PARTITIONS16K  2
#define NUM_PARTITIONS32K  2
#define NUM_PARTITIONS64K  2
#define NUM_PARTITIONS128K 2

/* 
 *	hardware specific access to control-lines
*/
void tx4925ndfmc_hwcontrol(int cmd)
{

	switch(cmd){

		case NAND_CTL_SETCLE: 
			tx4925_ndfmcptr->mcr |= TX4925_NDFMCR_CLE;
			break;
		case NAND_CTL_CLRCLE:
			tx4925_ndfmcptr->mcr &= ~TX4925_NDFMCR_CLE;
			break;
		case NAND_CTL_SETALE:
			tx4925_ndfmcptr->mcr |= TX4925_NDFMCR_ALE;
			break;
		case NAND_CTL_CLRALE: 
			tx4925_ndfmcptr->mcr &= ~TX4925_NDFMCR_ALE;
			break;
		case NAND_CTL_SETNCE:
			tx4925_ndfmcptr->mcr |= TX4925_NDFMCR_CE;
			break;
		case NAND_CTL_CLRNCE:
			tx4925_ndfmcptr->mcr &= ~TX4925_NDFMCR_CE;
			break;
		case NAND_CTL_SETWE:
			tx4925_ndfmcptr->mcr |= TX4925_NDFMCR_WE;
			break;
		case NAND_CTL_CLRWE:
			tx4925_ndfmcptr->mcr &= ~TX4925_NDFMCR_WE;
			break;
	}
}

/*
*	read device ready pin
*/
int tx4925ndfmc_device_ready(void)
{
	int ready;
	ready = (tx4925_ndfmcptr->sr & TX4925_NDSFR_BUSY) ? 0 : 1;
	return ready;
}
void tx4925ndfmc_enable_hwecc(int mode)
{
	/* reset first */
	tx4925_ndfmcptr->mcr |= TX4925_NDFMCR_ECC_CNTL_MASK;
	tx4925_ndfmcptr->mcr &= ~TX4925_NDFMCR_ECC_CNTL_MASK;
	tx4925_ndfmcptr->mcr |= TX4925_NDFMCR_ECC_CNTL_ENAB;
}
void tx4925ndfmc_disable_ecc(void)
{
	tx4925_ndfmcptr->mcr &= ~TX4925_NDFMCR_ECC_CNTL_MASK;
}
void tx4925ndfmc_enable_read_ecc(void)
{
	tx4925_ndfmcptr->mcr &= ~TX4925_NDFMCR_ECC_CNTL_MASK;
	tx4925_ndfmcptr->mcr |= TX4925_NDFMCR_ECC_CNTL_READ;
}
void tx4925ndfmc_readecc(const u_char *dat, u_char *ecc_code){
	int i;
	u_char *ecc = ecc_code;
        tx4925ndfmc_enable_read_ecc();
	for (i = 0;i < 6;i++,ecc++)
		*ecc = tx4925_read_nfmc(&(tx4925_ndfmcptr->dtr));
        tx4925ndfmc_disable_ecc();
}
void tx4925ndfmc_device_setup(void)
{

	*(unsigned char *)0xbb005000 &= ~0x08;

        /* reset NDFMC */
        tx4925_ndfmcptr->rstr |= TX4925_NDFRSTR_RST;
	while (tx4925_ndfmcptr->rstr & TX4925_NDFRSTR_RST);       

	/* setup BusSeparete, Hold Time, Strobe Pulse Width */
	tx4925_ndfmcptr->mcr = TX4925_BSPRT ? TX4925_NDFMCR_BSPRT : 0;
	tx4925_ndfmcptr->spr = TX4925_HOLD << 4 | TX4925_SPW;             
}

/*
 * Main initialization routine
 */
extern int nand_6_512_correct_data (u_char *dat, u_char *read_ecc, u_char *calc_ecc);
int __init tx4925ndfmc_init (void)
{
	struct nand_chip *this;
	int err = 0;

	/* Allocate memory for MTD device structure and private data */
	tx4925ndfmc_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!tx4925ndfmc_mtd) {
		printk ("Unable to allocate RBTX4925 NAND MTD device structure.\n");
		err = -ENOMEM;
		goto out;
	}

        tx4925ndfmc_device_setup();

	/* io is indirect via a register so don't need to ioremap address */

	/* Get pointer to private data */
	this = (struct nand_chip *) (&tx4925ndfmc_mtd[1]);

	/* Initialize structures */
	memset((char *) tx4925ndfmc_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	tx4925ndfmc_mtd->priv = this;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = (unsigned long)&(tx4925_ndfmcptr->dtr);
	this->IO_ADDR_W = (unsigned long)&(tx4925_ndfmcptr->dtr);
	this->hwcontrol = tx4925ndfmc_hwcontrol;
#define USE_HW_ECC 
#ifdef USE_HW_ECC
	this->enable_hwecc = tx4925ndfmc_enable_hwecc;
	this->calculate_ecc = tx4925ndfmc_readecc;
	this->correct_data = nand_6_512_correct_data;
	this->eccmode = NAND_ECC_HW6_512;	
#else
	this->eccmode = NAND_ECC_SOFT;	
#endif
	this->dev_ready = tx4925ndfmc_device_ready;
	/* 20 us command delay time */
	this->chip_delay = 20;		

	/* Scan to find existance of the device */
	if (nand_scan (tx4925ndfmc_mtd)) {
		err = -ENXIO;
		goto out_ior;
	}

	/* Allocate memory for internal data buffer */
	this->data_buf = kmalloc (sizeof(u_char) * (tx4925ndfmc_mtd->oobblock + tx4925ndfmc_mtd->oobsize), GFP_KERNEL);
	if (!this->data_buf) {
		printk ("Unable to allocate NAND data buffer for RBTX4925.\n");
		err = -ENOMEM;
		goto out_ior;
	}

	/* Allocate memory for internal data buffer */
	this->data_cache = kmalloc (sizeof(u_char) * (tx4925ndfmc_mtd->oobblock + tx4925ndfmc_mtd->oobsize), GFP_KERNEL);
	if (!this->data_cache) {
		printk ("Unable to allocate NAND data cache for RBTX4925.\n");
		err = -ENOMEM;
		goto out_buf;
	}
	this->cache_page = -1;

	/* Register the partitions */
	switch(tx4925ndfmc_mtd->size){
		case 0x01000000: add_mtd_partitions(tx4925ndfmc_mtd, partition_info16k, NUM_PARTITIONS16K); break;
		case 0x02000000: add_mtd_partitions(tx4925ndfmc_mtd, partition_info32k, NUM_PARTITIONS32K); break;
		case 0x04000000: add_mtd_partitions(tx4925ndfmc_mtd, partition_info64k, NUM_PARTITIONS64K); break; 
		case 0x08000000: add_mtd_partitions(tx4925ndfmc_mtd, partition_info128k, NUM_PARTITIONS128K); break; 
		default: {
			printk ("Unsupported SmartMedia device\n"); 
			err = -ENXIO;
			goto out_cac;
		}
	}
	goto out;

out_cac:
	kfree (this->data_cache);    
out_buf:
	kfree (this->data_buf);    
out_ior:
out:
	return err;
}

module_init(tx4925ndfmc_init);

/*
 * Clean up routine
 */
#ifdef MODULE
static void __exit tx4925ndfmc_cleanup (void)
{
	struct nand_chip *this = (struct nand_chip *) &tx4925ndfmc_mtd[1];

	/* Unregister partitions */
	del_mtd_partitions(tx4925ndfmc_mtd);
	
	/* Unregister the device */
	del_mtd_device (tx4925ndfmc_mtd);

	/* Free internal data buffers */
	kfree (this->data_buf);
	kfree (this->data_cache);

	/* Free the MTD device structure */
	kfree (tx4925ndfmc_mtd);
}
module_exit(tx4925ndfmc_cleanup);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alice Hennessy <ahennessy@mvista.com>");
MODULE_DESCRIPTION("Glue layer for SmartMediaCard on Toshiba RBTX4925");
