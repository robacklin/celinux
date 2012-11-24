/*
 *  drivers/mtd/smdk2440.c
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/sizes.h>

/*
 * MTD structure for SMDK2440 board
 */
static struct mtd_info *smdk2440_mtd = NULL;

/*
 * Module stuff
 */
#if LINUX_VERSION_CODE < 0x20212 && defined(MODULE)
#define smdk2440_smc_init init_module
#define smdk2440_smc_cleanup cleanup_module
#endif

/*
 * Define partitions for flash devices
 */
static struct mtd_partition partition_info16k[] = {
	{ name: "SMDK2440 SMC partition 1",
	  offset:  0,
	  size:    8 * SZ_1M },
	{ name: "SMDK2440 SMC partition 2",
	  offset:  8 * SZ_1M,
	  size:    8 * SZ_1M },
};

static struct mtd_partition partition_info32k[] = {
	{ name: "SMDK2440 SMC partition 1",
	  offset:  0,
	  size:    8 * SZ_1M },
	{ name: "SMDK2440 SMC partition 2",
	  offset:  8 * SZ_1M,
	  size:   24 * SZ_1M },
};

#if 0
static struct mtd_partition partition_info64k[] =
{
	{
		name:		"Partition 0 : kernel",
		offset:		0,
		size:		2 * SZ_1M ,
		mask_flags:	MTD_WRITEABLE
	},
	{
		name:		"Partition 1 : root",
		offset:		2 * SZ_1M,
		size:		38 * SZ_1M ,

	},
	{
		name:		"Partition 2 : jffs2",
		offset:		40 * SZ_1M,
		size:		12 * SZ_1M
	},
	{
		name:		"Partition 3 : yaffs",
		offset:		52 * SZ_1M,
		size:		12 * SZ_1M
	}
};
#endif


/* 
0x00000000-0x00020000 : "Bootloader" - 128K
0x00020000-0x00200000 : "Kernel"
0x00200000-0x00500000 : "Filesystem"
0x00500000-0x01000000 : "JFFS2"
*/

/* wogns test */
#if 1 
static struct mtd_partition partition_info64k[] =
{
	{
		name: "Bootloader",
		size: 0x00100000,    /* 1M */
		offset: 0,
		mask_flags: MTD_WRITEABLE /* force read-only */
	},{
		name: "Kernel",
		size: 0x00A00000,    /* 10M */
		offset: 0x100000
	},{
		name: "Filesystem",
		size: 0x02A00000,    /* 42M */
		offset: 0x00B00000
	},{
		name: "JFFS2",
		size: 0x00A00000,    /* 10M */
		offset: 0x03500000
	}
};
#endif

#if 0 
static struct mtd_partition partition_info64k[] =
{
	{
		name: "Bootloader",
		size: 0x00020000,
		offset: 0,
		mask_flags: MTD_WRITEABLE /* force read-only */
	},{
		name: "Kernel",
		size: 0x001e0000,
		offset: 0x20000
	},{
		name: "Filesystem",
		size: 0x03000000,    /* 48M */
		offset: 0x00200000
	},{
		name: "JFFS2",
		size: MTDPART_SIZ_FULL,
		offset: 0x03200000
	}
};
#endif


#if 0
static struct mtd_partition partition_info64k[] =
{
        {
                name:           "Partition 0 : root",
                offset:         2 * SZ_1M,
                size:           50 * SZ_1M ,

        }
        ,
        {
                name:           "Partition 1 : jffs2",
                offset:         52 * SZ_1M,
                size:           10 * SZ_1M
        },
        {
                name:           "Partition 2 : yaffs",
                offset:         62 * SZ_1M,
                size:           2 * SZ_1M
        }
};
#endif


static struct mtd_partition partition_info128k[] = {
	{ name: "SMDK2440 SMC partition 1",
	  offset:  0,
	  size:   16 * SZ_1M },
	{ name: "SMDK2440 SMC partition 2",
	  offset: 16 * SZ_1M,
	  size:   112 * SZ_1M },
};

#define NUM_PARTITIONS16K 2
#define NUM_PARTITIONS32K 2
#define NUM_PARTITIONS64K 4
#define NUM_PARTITIONS128K 2


///////////////////////////////////////////////////////////
#define NF_CMD(cmd)     {NFCMD=cmd;}
#define NF_ADDR(addr)   {NFADDR=addr;} 
/* wjluv change this for SMDK2440 */
#if 0
#define NF_nFCE_L()     {rNFCONF&=~(1<<11);}
#define NF_nFCE_H()     {rNFCONF|=(1<<11);}
#define NF_RSTECC()     {rNFCONF|=(1<<12);}
#else
#define NF_nFCE_L()     {NFCONT&=~(1<<1);}
#define NF_nFCE_H()     {NFCONT|=(1<<1);}
#define NF_RSTECC()     {NFCONT|=(1<<4);}
#endif

#define NF_RDDATA()     (NFDATA)
#define NF_WRDATA(data) {NFDATA=data;}

#define NF_CLEARRB()    {NFSTAT=0x6;}
#define NF_WAITRB()    {while(!(NFSTAT&(1<<0)));} 
////////////////////////////////////////////////////////////
static inline void write_NFCMD(u_char cmd)
{
        //*(volatile unsigned char*)NFCMD = cmd;
        *(volatile unsigned char*)(0xee000008) = cmd;
}

static inline void write_NFADDR(u_char addr)
{
         //*(volatile unsigned char*)NFADDR = addr;
        *(volatile unsigned char*)(0xee00000c) = addr;
}

static inline u_char read_NFDATA()
{
        return (u_char) NFDATA;
}

#if 0
static inline void write_NFDATA(u_char data)
{
         *(volatile unsigned char*)NFDATA = data;
}
#endif



static void s3c2440_nand_command (struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
        register struct nand_chip *this = mtd->priv;

        /*
         * Write out the command to the device.
         */

        if (command != NAND_CMD_SEQIN)
                write_NFCMD (command);
        else {
                if (mtd->oobblock == 256 && column >= 256) {
                        column -= 256;
                        write_NFCMD (NAND_CMD_READOOB);
                        write_NFCMD (NAND_CMD_SEQIN);
                } else if (mtd->oobblock == 512 && column >= 256) {
                        if (column < 512) {
                                column -= 256;
                                write_NFCMD (NAND_CMD_READ1);
                                write_NFCMD (NAND_CMD_SEQIN);
                        } else {
                                column -= 512;
                                write_NFCMD (NAND_CMD_READOOB);
                                write_NFCMD (NAND_CMD_SEQIN);
                        }
                } else {
                        write_NFCMD (NAND_CMD_READ0);
                        write_NFCMD (NAND_CMD_SEQIN);
                }
        }

        if (column != -1 || page_addr != -1) {
                /* Serially input address */
                if (column != -1)
                        write_NFADDR (column);
                if (page_addr != -1) {
                        write_NFADDR ((u_char) (page_addr & 0xff));
                        write_NFADDR ((u_char) ((page_addr >> 8) & 0xff));
                        /* One more address cycle for higher density devices */
                        write_NFADDR ((u_char) ((page_addr >> 16) & 0xff)); //kwg
			/* kwg 
                        if (mtd->size & 0x0c000000)
                                write_NFADDR ((u_char) ((page_addr >> 16) & 0x0f)); */
                }
        }

        /* 
         * program and erase have their own busy handlers 
         * status and sequential in needs no delay
        */
        switch (command) {

        case NAND_CMD_PAGEPROG:
        case NAND_CMD_ERASE1:
        case NAND_CMD_ERASE2:
        case NAND_CMD_SEQIN:
        case NAND_CMD_STATUS:
                return;
#if 0
        case NAND_CMD_RESET:
                if (this->dev_ready())
                        break;
                write_NFCMD (NAND_CMD_STATUS);
                while ( !(read_NFDATA() & 0x40));
                return;
#endif

        /* This applies to read commands */
        default:
                /* 
                 * If we don't have access to the busy pin, we apply the given
                 * command delay
                */
                if (!this->dev_ready) {
                        udelay (this->chip_delay);
                        return;
                }
        }

        /* wait until command is processed */
        while (!this->dev_ready());
}





/*
 *      hardware specific access to control-lines
*/
void s3c2440_hwcontrol(int cmd)
{
        switch(cmd)
        {
                //case NAND_CTL_SETNCE: NFCONF &= ~NFCONF_nFCE_HIGH; break;
                case NAND_CTL_SETNCE:
			NFCONT&=~(1<<1);
			break;

                //case NAND_CTL_CLRNCE: NFCONF |= NFCONF_nFCE_HIGH;  break;
                case NAND_CTL_CLRNCE:
			NFCONT|=(1<<1);
			break;

                case NAND_CTL_SETCLE: break;
                case NAND_CTL_CLRCLE: break;


                case NAND_CTL_SETALE: break;
                case NAND_CTL_CLRALE: break;
        }
}


/*
 *       read device ready pin
 */
int s3c2440_device_ready(void)
{
    //return (NFSTAT & NFSTAT_RnB) ? 1 : 0;
    return (NFSTAT & 0x1) ? 1 : 0;
}


/*
 * Main initialization routine
 */
//static int __init smdk2440_smc_init(void)
int __init smdk2440_smc_init(void)
{
	struct nand_chip *this;
	u_int16_t nfconf;

	int err = 0;


	/* Allocate memory for MTD device structure and private data */
	smdk2440_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!smdk2440_mtd) {
		printk ("Unable to allocate SMDK2440 SMC MTD device structure.\n");
		err = -ENOMEM;
		goto out;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&smdk2440_mtd[1]);

	/* Initialize structures */
	memset((char *) smdk2440_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	smdk2440_mtd->priv = this;

	/* set & initialize NAND Flash controller */
	{
#if 0		
		nfconf = NFCONF;

		/* NAND Flash controller enable */
		nfconf |= NFCONF_FCTRL_EN;

		/* Set flash memory timing */
		nfconf &= ~NFCONF_TWRPH1;   /* 0x0 */
		nfconf |= NFCONF_TWRPH0_3;  /* 0x3 */
		nfconf &= ~NFCONF_TACLS;    /* 0x0 */

		NFCONF = nfconf;

		/* Chip Enable -> RESET -> Wait for Ready -> Chip Disable */
		NFCONF &= ~NFCONF_nFCE_HIGH;
		NFCMD = (u_char) (NAND_CMD_RESET);
		while (!(NFSTAT & NFSTAT_RnB)) udelay(10);
		NFCONF |= NFCONF_nFCE_HIGH;
#endif
		NFCONF = 0xf776; 	//kwg TODO
		
		NF_nFCE_L();
		 /* wjluv add this for SMDK2440 */
		NF_CLEARRB();
		NF_CMD(0xFF);       //reset command
		{ int i; for(i=0;i<10;i++); } //tWB = 100ns. //??????
		NF_WAITRB();      //wait 200~500us;
		NF_nFCE_H();
		NFCONT |=0x1;
	}

        /* Set address of NAND IO lines */
        this->IO_ADDR_R = 0xee000000 + 0x10;
        this->IO_ADDR_W = 0xee000000 + 0x10;
        this->hwcontrol = s3c2440_hwcontrol;
	this->cmdfunc = s3c2440_nand_command;

        this->dev_ready = s3c2440_device_ready;
        /* 20 us command delay time */
        this->chip_delay = 20;
        //this->eccmode = NAND_ECC_SOFT;
        this->eccmode = NAND_ECC_NONE;


	/* Scan to find existance of the device */
	if (nand_scan(smdk2440_mtd)) {
		err = -ENXIO;
		goto out_ior;
	}

	/* Allocate memory for internal data buffer */
	this->data_buf = kmalloc (sizeof(u_char) * (smdk2440_mtd->oobblock + smdk2440_mtd->oobsize), GFP_KERNEL);
	if (!this->data_buf) {
		printk ("Unable to allocate NAND data buffer for SMDK2440 SMC.\n");
		err = -ENOMEM;
		goto out_ior;
	}
	/* Register the partitions */
	printk("smdk2440_mtd->size=0x%08x.\n", smdk2440_mtd->size);

	switch(smdk2440_mtd->size){
		case SZ_16M: add_mtd_partitions(smdk2440_mtd, partition_info16k, NUM_PARTITIONS16K); break;
		case SZ_32M: add_mtd_partitions(smdk2440_mtd, partition_info32k, NUM_PARTITIONS32K); break;
		case SZ_64M: add_mtd_partitions(smdk2440_mtd, partition_info64k, NUM_PARTITIONS64K); break; 
		case SZ_128M: add_mtd_partitions(smdk2440_mtd, partition_info128k, NUM_PARTITIONS128K); break; 
		default: {
			printk ("Unsupported SmartMedia device\n"); 
			err = -ENXIO;
			goto out_buf;
		}
	}
	goto out;

out_buf:
	kfree (this->data_buf);    
out_ior:
	kfree (smdk2440_mtd);
out:
	return err;
}

module_init(smdk2440_smc_init);

/*
 * Clean up routine
 */
#ifdef MODULE
static void __exit smdk2440_smc_cleanup(void)
{
	struct nand_chip *this = (struct nand_chip *) &smdk2440_mtd[1];

	/* Unregister partitions */
	del_mtd_partitions(smdk2440_mtd);
	
	/* Unregister the device */
	del_mtd_device (smdk2440_mtd);

	/* Free internal data buffers */
	kfree (this->data_buf);

	/* Free the MTD device structure */
	kfree (smdk2440_mtd);
}
module_exit(smdk2440_smc_cleanup);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("S.W. LEE");
MODULE_DESCRIPTION("SMC on SMDK2440 board");
