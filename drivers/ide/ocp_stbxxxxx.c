/*
 *    Copyright 2002 MontaVista Software Inc.
 *      Completed implementation.
 *      Author: Armin Kuster <akuster@mvista.com>
 *      MontaVista Software, Inc.  <source@mvista.com>
 *
 *    Module name: ocp_stbxxxx.c
 *
 *    Description:
 *
 *    Based on stb03xxx.c
 *
 *    Version 07/23/02 - Armin
 *    removed many mtdcr/mfdcr dma calls to standard 4xx dma calls
 */

#include <linux/types.h>
#include <linux/hdreg.h>
#include <linux/delay.h>
#include <linux/ide.h>

#include <asm/io.h>
#include <asm/ppc4xx_dma.h>

#include "ide_modes.h"

#define IDEVR			"1.2"
ppc_dma_ch_t dma_ch;

/* use DMA channel 2 for IDE DMA operations */
#define IDE_DMACH	2	/* 2nd DMA channel */
#define IDE_DMA_INT	6	/* IDE dma channel 2 interrupt */

extern char *ide_dmafunc_verbose(ide_dma_action_t dmafunc);

#define WMODE	0		/* default to DMA line mode */
#define PIOMODE	0

#ifdef CONFIG_REDWOOD_4

/* psc=00, pwc=000101 phc=010, tce=1, resvd-must-be-one=1 */

unsigned long dmacr_def_line = 0x0000AB02;

#else
/* psc=00, pwc=000001 phc=010, resvd-must-be-one=1 */

unsigned long dmacr_def_line = 0x00002A02;
#endif

/* psc=00, pwc=000110 phc=010, resvd-must-be-one=1 */

unsigned long dmacr_def_word = 0x0000CA02;

#ifdef CONFIG_REDWOOD_4
#define DCRXBCR_MDMA2		0xC0000000
#else /* CONFIG_REDWOOD_6 */
#define DCRXBCR_MDMA2		0x80000000
#endif

#define DCRXBCR_WRITE		0x20000000
#define DCRXBCR_ACTIVATE	0x10000000

#ifdef CONFIG_REDWOOD_4
#define IDE_CMD_OFF		0x00100000
#define IDE_CTL_OFF		0x00100000
#define UIC_D2			0x02000000	/* 6 DMA Channel 2 */
#endif

#define IDE_DMASR_TC	0x20000000
#define IDE_DMASR_EOT	0x02000000
#define IDE_DMASR_ERR	0x00200000
#define IDE_DMASR_CB	0x00000100
#define IDE_DMASR_CT	0x00000020

/* Function Prototypes */
static void redwood_ide_tune_drive(ide_drive_t *, byte);
static byte redwood_ide_dma_2_pio(byte);
static int redwood_ide_tune_chipset(ide_drive_t *, byte);
static int redwood_ide_dmaproc(ide_dma_action_t, ide_drive_t *);

#ifdef DEBUG
static void dump_dcrs(void)
{
	printk("DMASR=%x\n", mfdcr(DCRN_DMASR));
	printk("DMACR2=%x\n", mfdcr(DCRN_DMACR2));
	printk("DMACT2=%d\n", mfdcr(DCRN_DMACT2));
	printk("DMAS2=%x\n", mfdcr(DCRN_DMAS2));
	printk("DMASA2=%x\n", mfdcr(DCRN_DMASA2));
	printk("DMADA2=%x\n", mfdcr(DCRN_DMADA2));

	if (mfdcr(DCRN_DMASR) & 0x00200000) {
		printk("BESR=%x\n", mfdcr(DCRN_BESR));
		printk("BEAR=%x\n", mfdcr(DCRN_BEAR));
		printk("PLB0_BESR=%x\n", mfdcr(DCRN_PLB0_BESR));
		printk("PLB0_BEAR=%x\n", mfdcr(DCRN_PLB0_BEAR));
		printk("PLB1_BESR=%x\n", mfdcr(DCRN_PLB1_BESR));
		printk("PLB1_BEAR=%x\n", mfdcr(DCRN_PLB1_BEAR));
		printk("OPB0_BESR0=%x\n", mfdcr(DCRN_POB0_BESR0));
		printk("OPB0_BEAR=%x\n", mfdcr(DCRN_POB0_BEAR));
		printk("SDRAM0_BESR=%x\n", mfdcr(0x1E1));
		printk("SDRAM0_BEAR=%x\n", mfdcr(0x1E2));
		printk("SDRAM1_BESR=%x\n", mfdcr(0x1C1));
		printk("SDRAM1_BEAR=%x\n", mfdcr(0x1C2));
	}

	return;
}
#endif /* DEBUG */

static void
redwood_ide_tune_drive(ide_drive_t * drive, byte pio)
{
	pio = ide_get_best_pio_mode(drive, pio, 5, NULL);
}

static byte
redwood_ide_dma_2_pio(byte xfer_rate)
{
	switch (xfer_rate) {
	case XFER_UDMA_5:
	case XFER_UDMA_4:
	case XFER_UDMA_3:
	case XFER_UDMA_2:
	case XFER_UDMA_1:
	case XFER_UDMA_0:
	case XFER_MW_DMA_2:
	case XFER_PIO_4:
		return 4;
	case XFER_MW_DMA_1:
	case XFER_PIO_3:
		return 3;
	case XFER_SW_DMA_2:
	case XFER_PIO_2:
		return 2;
	case XFER_MW_DMA_0:
	case XFER_SW_DMA_1:
	case XFER_SW_DMA_0:
	case XFER_PIO_1:
	case XFER_PIO_0:
	case XFER_PIO_SLOW:
	default:
		return 0;
	}
}

static int
redwood_ide_tune_chipset(ide_drive_t * drive, byte speed)
{
	int err = 0;

	redwood_ide_tune_drive(drive, redwood_ide_dma_2_pio(speed));

	if (!drive->init_speed)
		drive->init_speed = speed;
	err = ide_config_drive_speed(drive, speed);
	drive->current_speed = speed;
	return err;
}

#ifdef CONFIG_BLK_DEV_IDEDMA

static int
redwood_config_drive_for_dma(ide_drive_t * drive)
{
	struct hd_driveid *id = drive->id;
	byte speed;
	int func = ide_dma_off;

	/*
	 * Enable DMA on any drive that has multiword DMA
	 */
	if (id->field_valid & 2) {
		if (id->dma_mword & 0x0004) {
			speed = XFER_MW_DMA_2;
			func = ide_dma_on;
		} else if (id->dma_mword & 0x0002) {
			speed = XFER_MW_DMA_1;
			func = ide_dma_on;
		} else if (id->dma_mword & 1) {
			speed = XFER_MW_DMA_0;
			func = ide_dma_on;
		} else if (id->dma_1word & 0x0004) {
			speed = XFER_SW_DMA_2;
			func = ide_dma_on;
		} else {
			speed = XFER_PIO_0 +
			    ide_get_best_pio_mode(drive, 255, 5, NULL);
		}
	}

	redwood_ide_tune_drive(drive, redwood_ide_dma_2_pio(speed));
	return redwood_ide_dmaproc(func, drive);
}

ide_startstop_t
redwood_ide_intr(ide_drive_t * drive)
{
	int i;
	byte dma_stat;
	unsigned int nsect;
	ide_hwgroup_t *hwgroup = HWGROUP(drive);
	struct request *rq = hwgroup->rq;
	unsigned long block, b1, b2, b3, b4;

	nsect = rq->current_nr_sectors;

	dma_stat = HWIF(drive)->dmaproc(ide_dma_end, drive);

	rq->sector += nsect;
	rq->buffer += nsect << 9;
	rq->errors = 0;
	i = (rq->nr_sectors -= nsect);
	ide_end_request(1, HWGROUP(drive));
	if (i > 0) {
		b1 = IN_BYTE(IDE_SECTOR_REG);
		b2 = IN_BYTE(IDE_LCYL_REG);
		b3 = IN_BYTE(IDE_HCYL_REG);
		b4 = IN_BYTE(IDE_SELECT_REG);
		block = ((b4 & 0x0f) << 24) + (b3 << 16) + (b2 << 8) + (b1);
		block++;
		if (drive->select.b.lba) {
			OUT_BYTE(block, IDE_SECTOR_REG);
			OUT_BYTE(block >>= 8, IDE_LCYL_REG);
			OUT_BYTE(block >>= 8, IDE_HCYL_REG);
			OUT_BYTE(((block >> 8) & 0x0f) | drive->select.all,
				 IDE_SELECT_REG);
		} else {
			unsigned int sect, head, cyl, track;
			track = block / drive->sect;
			sect = block % drive->sect + 1;
			OUT_BYTE(sect, IDE_SECTOR_REG);
			head = track % drive->head;
			cyl = track / drive->head;
			OUT_BYTE(cyl, IDE_LCYL_REG);
			OUT_BYTE(cyl >> 8, IDE_HCYL_REG);
			OUT_BYTE(head | drive->select.all, IDE_SELECT_REG);
		}

		if (rq->cmd == READ)
			dma_stat = HWIF(drive)->dmaproc(ide_dma_read, drive);
		else
			dma_stat = HWIF(drive)->dmaproc(ide_dma_write, drive);
		return ide_started;
	}
	return ide_stopped;
}

void
redwood_ide_dma_intr (int irq, void *dev_id, struct pt_regs *regs)
{
	ppc4xx_clr_dma_status(IDE_DMACH);
}


static int redwood_ide_dma_verbose(ide_drive_t * drive)
{
        struct hd_driveid *id = drive->id;

        if (id->field_valid & 2) {
                if (id->dma_mword & 0x0004) {
                        printk(", mdma2");
                } else if (id->dma_mword & 0x0002) {
                        printk(", mdma1");
                } else if (id->dma_mword & 1) {
                        printk(", mdma0");
                } else if (id->dma_1word & 0x0004) {
                        printk(", sdma2");
                } else {
                        printk(", pio%d",
                               ide_get_best_pio_mode(drive, 255, 5, NULL));
                }
        }

        return 0;
}

static int
redwood_ide_dmaproc(ide_dma_action_t func, ide_drive_t * drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	int reading = 0;
	struct request *rq = HWGROUP(drive)->rq;
	unsigned long flags;
	unsigned long length;

	switch (func) {
	case ide_dma_off:
	case ide_dma_off_quietly:
		/*disable_dma */
		return 0;

	case ide_dma_on:
#if PIOMODE
		return 1;
#endif

		mtdcr(DCRN_DMACR2, 0);
		ppc4xx_clr_dma_status(IDE_DMACH);

#ifdef CONFIG_REDWOOD_4
		/* level-sensitive (UICTR bit = 0) and positive-active (UICPR bit = 1) */

		mtdcr(DCRN_UIC_PR(UIC0), (mfdcr(DCRN_UIC_PR(UIC0)) | UIC_D2));
#endif

		save_flags(flags);
		cli();
		if (ide_request_irq
		    (IDE_DMA_INT, &redwood_ide_dma_intr, SA_INTERRUPT,
		     hwif->name, hwif->hwgroup)) {
			printk("ide_redwood: ide_request_irq failed int=%d\n",
			       IDE_DMA_INT);
			restore_flags(flags);
			return 1;
		}
		restore_flags(flags);

		drive->using_dma = (func == ide_dma_on);
#if WMODE
		mtdcr(DCRN_DCRXBCR, 0);
		mtdcr(DCRN_CICCR, mfdcr(DCRN_CICCR) | 0x00000400);
#else
		/* Configure CIC reg for line mode dma */
		mtdcr(DCRN_CICCR, mfdcr(DCRN_CICCR) & ~0x00000400);
#endif
		return 0;

	case ide_dma_check:
		return redwood_config_drive_for_dma(drive);
	case ide_dma_read:
		reading = 1;
	case ide_dma_write:
		if (drive->media != ide_disk)
			return -1;

		if (ppc4xx_get_channel_config(IDE_DMACH, &dma_ch) & DMA_CHANNEL_BUSY )	/* DMA is busy? */
			return -1;


		if (reading) {
			dma_cache_inv((unsigned long) rq->buffer,
				      rq->current_nr_sectors * 512);
#if WMODE
			ppc4xx_set_src_addr(IDE_DMACH, 0);
#else
			ppc4xx_set_src_addr(IDE_DMACH, 0xfce00000);
#endif
			ppc4xx_set_dst_addr(IDE_DMACH, virt_to_bus(rq->buffer));
		} else {
			dma_cache_wback_inv((unsigned long) rq->buffer,
					    rq->current_nr_sectors * 512);
			ppc4xx_set_src_addr(IDE_DMACH, virt_to_bus(rq->buffer));
#if WMODE
			ppc4xx_set_dst_addr(2, 0);
#else
			ppc4xx_set_dst_addr(IDE_DMACH, 0xfce00000);
#endif
		}

		OUT_BYTE(rq->current_nr_sectors, IDE_NSECTOR_REG);
		length = rq->current_nr_sectors * 512;

		/* set_dma_count doesn't do M2M line xfer sizes right. */

#if WMODE
		mtdcr(DCRN_DMACT2, length >> 2);
#else
		mtdcr(DCRN_DMACT2, length >> 4);
#endif

		/* CE=0 disable DMA */
		/* Set up the Line Buffer Control Register
		 * 11d1xxxx0.. - 11=Mode 2 (120 ns cycle), d=1/0(read/write)
		 * 1=active, 0=1X clock mode.
		 */

		if (reading) {
#if WMODE
			ppc4xx_set_dma_mode(IDE_DMACH,DMA_TD | TM_S_MM);
#else
			mtdcr(DCRN_DCRXBCR, DCRXBCR_MDMA2 | DCRXBCR_ACTIVATE);

			mtdcr(DCRN_DMACR2, SET_DMA_DAI(1) | SET_DMA_SAI(0) |
			      DMA_MODE_MM_DEVATSRC | SET_DMA_PW(PW_64) |
			      dmacr_def_line);
			ppc4xx_set_dma_mode(IDE_DMACH, DMA_MODE_MM_DEVATSRC);
#endif
		} else {
#if WMODE
			ppc4xx_set_dma_mode(IDE_DMACH,DMA_TD | TM_S_MM);
#else
			mtdcr(DCRN_DCRXBCR, DCRXBCR_WRITE | DCRXBCR_MDMA2 | 
			      DCRXBCR_ACTIVATE);

			mtdcr(DCRN_DMACR2,SET_DMA_DAI(0) | SET_DMA_SAI(1) | 
			      DMA_MODE_MM_DEVATDST| SET_DMA_PW(PW_64) | 
			      dmacr_def_line);
			ppc4xx_set_dma_mode(IDE_DMACH, DMA_MODE_MM_DEVATDST);
#endif
		}

		drive->waiting_for_dma = 1;
		ide_set_handler(drive, &redwood_ide_intr, WAIT_CMD, NULL);
		OUT_BYTE(reading ? WIN_READDMA : WIN_WRITEDMA, IDE_COMMAND_REG);

	case ide_dma_begin:
		/* enable DMA */
		ppc4xx_enable_dma_interrupt(IDE_DMACH);
		ppc4xx_enable_dma(IDE_DMACH);
		return 0;

	case ide_dma_end:
		drive->waiting_for_dma = 0;

		/* disable DMA */
		ppc4xx_disable_dma_interrupt(IDE_DMACH);
		ppc4xx_disable_dma(IDE_DMACH);
		return 0;

	case ide_dma_test_irq:
		return 1;	/* returns 1 if dma irq issued, 0 otherwise */

	case ide_dma_verbose:
		return redwood_ide_dma_verbose(drive);

	case ide_dma_bad_drive:
	case ide_dma_good_drive:
	case ide_dma_timeout:
	case ide_dma_retune:
	case ide_dma_lostirq:
		printk("ide_dmaproc: chipset supported %s func only: %d\n",
		       ide_dmafunc_verbose(func), func);
		return 1;
	default:
		printk("ide_dmaproc: unsupported %s func: %d\n",
		       ide_dmafunc_verbose(func), func);
		return 1;

	}

}
#endif /* CONFIG_BLK_DEV_IDEDMA */

void
ibm4xx_ide_spinup(int index)
{
	int i;
	ide_ioreg_t *io_ports;

	printk("ide_redwood: waiting for drive ready..");
	io_ports = ide_hwifs[index].io_ports;

	/* wait until drive is not busy (it may be spinning up) */
	for (i = 0; i < 30; i++) {
		unsigned char stat;
		stat = inb_p(io_ports[7]);
		/* wait for !busy & ready */
		if ((stat & 0x80) == 0) {
			break;
		}

		udelay(1000 * 1000);	/* 1 second */
	}

	printk("..");

	/* select slave */
	outb_p(0xa0 | 0x10, io_ports[6]);

	for (i = 0; i < 30; i++) {
		unsigned char stat;
		stat = inb_p(io_ports[7]);
		/* wait for !busy & ready */
		if ((stat & 0x80) == 0) {
			break;
		}

		udelay(1000 * 1000);	/* 1 second */
	}

	printk("..");

	outb_p(0xa0, io_ports[6]);
	printk("Drive spun up \n");
}

int
nonpci_ide_default_irq(ide_ioreg_t base)
{
	return IDE0_IRQ;
}

void
nonpci_ide_init_hwif_ports(hw_regs_t * hw, ide_ioreg_t data_port,
			   ide_ioreg_t ctrl_port, int *irq)
{
	unsigned long ioaddr;
#ifdef CONFIG_REDWOOD_4
	unsigned long reg = data_port;
	unsigned long xilinx;
#endif
	int i, index;

	printk("IBM Redwood 4/6 IDE driver version %s\n", IDEVR);

	if (!request_region(REDWOOD_IDE_CMD, 0x10, "IDE"))
		return;

	if (!request_region(REDWOOD_IDE_CTRL, 2, "IDE")) {
		release_region(REDWOOD_IDE_CMD, 0x10);
		return;
	}

#ifdef CONFIG_REDWOOD_4
	mtdcr(DCRN_DCRXICR, 0x40000000);	/* set dcrx internal arbiter */

	/* add RE & OEN to value set by boot rom */
	mtdcr(DCRN_BRCR3, 0x407cfffe);

	/* reconstruct phys addrs from EBIU config regs for CS2# */
	reg = ((mfdcr(DCRN_BRCR2) & 0xff000000) >> 4) | 0xf0000000;
	xilinx = reg | 0x00040000;
	reg = reg | IDE_CMD_OFF;

	ioaddr = (unsigned long)ioremap(reg, 0x10);
	xilinx = (unsigned long)ioremap(xilinx, 0x10);

	i=readw(xilinx);
	if(i & 0x0001) {
		writew( i & ~0x8001, xilinx);
		writew( 0, xilinx+7*2);
		udelay(10*1000);	/* 10 ms */
	}

	/* init xilinx control registers - enable ide mux, clear reset bit */
	writew( i | 0x8001, xilinx);
	writew( 0, xilinx+7*2);

#else /* CONFIG_REDWOOD_6 */
	ioaddr = (unsigned long) ioremap(REDWOOD_IDE_CMD, 0x10);
#endif

	hw->irq = IDE0_IRQ;

	for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++) {
		hw->io_ports[i] = ioaddr;
		ioaddr += 2;
	}
	hw->io_ports[IDE_CONTROL_OFFSET] =
	    (unsigned long) ioremap(REDWOOD_IDE_CTRL, 2);

	/* use DMA channel 2 for IDE DMA operations */
	hw->dma = IDE_DMACH;
#ifdef WMODE
   /*Word Mode psc(11-12)=00,pwc(13-18)=000110, phc(19-21)=010, 22=1, 30=1  ----  0xCB02*/

    dma_ch.mode	=TM_S_MM;	  /* xfer from peripheral to mem */
    dma_ch.td	= 1;
    dma_ch.buffer_enable = 0;
    dma_ch.tce_enable = 0;
    dma_ch.etd_output = 0;
    dma_ch.pce = 0;
    dma_ch.pl = EXTERNAL_PERIPHERAL;    /* no op */
    dma_ch.pwidth = PW_16;
    dma_ch.dai = 1;
    dma_ch.sai = 0;
    dma_ch.psc = 0;                      /* set the max setup cycles */
    dma_ch.pwc = 6;                     /* set the max wait cycles  */
    dma_ch.phc = 2;                      /* set the max hold cycles  */
    dma_ch.cp = PRIORITY_LOW;
    dma_ch.int_enable = 0;
    dma_ch.ch_enable = 0;		/* No chaining */
    dma_ch.tcd_disable = 1;		/* No chaining */
#else
/*Line Mode psc(11-12)=00,pwc(13-18)=000001, phc(19-21)=010, 22=1, 30=1  ----  0x2B02*/

   dma_ch.mode	=DMA_MODE_MM_DEVATSRC;	  /* xfer from peripheral to mem */
   dma_ch.td	= 1;
   dma_ch.buffer_enable = 0;
    dma_ch.tce_enable = 0;
    dma_ch.etd_output = 0;
    dma_ch.pce = 0;
    dma_ch.pl = EXTERNAL_PERIPHERAL;    /* no op */
    dma_ch.pwidth = PW_64;		/* Line mode on stbs */
    dma_ch.dai = 1;
    dma_ch.sai = 0;
    dma_ch.psc = 0;                      /* set the max setup cycles */
    dma_ch.pwc = 1;                     /* set the max wait cycles  */
    dma_ch.phc = 2;                      /* set the max hold cycles  */
    dma_ch.cp = PRIORITY_LOW;
    dma_ch.int_enable = 0;
    dma_ch.ch_enable = 0;		/* No chaining */
    dma_ch.tcd_disable = 1;		/* No chaining */

#endif
    if (ppc4xx_init_dma_channel(IDE_DMACH, &dma_ch) != DMA_STATUS_GOOD)
        return;

	ppc4xx_disable_dma_interrupt(IDE_DMACH);

	/* init CIC control reg to enable IDE interface PIO mode */
	mtdcr(DCRN_CICCR, (mfdcr(DCRN_CICCR) & 0xffff7bff) | 0x0003);

	/* init CIC select2 reg to connect external DMA port 3 to internal
	 * DMA channel 2
	 */

	mtdcr(DCRN_DMAS2, (mfdcr(DCRN_DMAS2) & 0xfffffff0) | 0x00000002);
	index = 0;

	ide_hwifs[index].tuneproc = &redwood_ide_tune_drive;
	ide_hwifs[index].drives[0].autotune = 1;
#ifdef CONFIG_BLK_DEV_IDEDMA
	ide_hwifs[index].autodma = 1;
	ide_hwifs[index].dmaproc = &redwood_ide_dmaproc;
#endif
	ide_hwifs[index].speedproc = &redwood_ide_tune_chipset;
	ide_hwifs[index].noprobe = 0;

	memcpy(ide_hwifs[index].io_ports, hw->io_ports, sizeof (hw->io_ports));
	ide_hwifs[index].irq = hw->irq;
	ibm4xx_ide_spinup(index);
}
