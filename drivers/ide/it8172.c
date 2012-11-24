/*
 *
 * BRIEF MODULE DESCRIPTION
 *      IT8172 IDE controller support
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2000 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/hdreg.h>
#include <linux/ide.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/device.h>

#include <asm/io.h>
#include <asm/it8172/it8172.h>
#include <asm/it8172/it8172_int.h>

#include "ide_modes.h"

/*
 * LDM
 */
static int ide_suspend(struct device *dev, u32 state, u32 level);
static int ide_resume(struct device *dev, u32 level);

static struct device_driver ide_driver_ldm = {
       name:      "ite8172-ide-cont",
       devclass:  NULL,
       probe:     NULL,
       suspend:   ide_suspend,
       resume:    ide_resume,
       remove:    NULL,
       constraints: NULL,
};

static struct device ide_device_ldm = {
       name: "ITE IDE controller",
       bus_id: "ide",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void ide_ldm_driver_register(void)
{
   extern void pci_driver_ldm_register(struct device_driver *driver);

   pci_driver_ldm_register(&ide_driver_ldm);
}

static void ide_ldm_device_register(void)
{
   extern void pci_device_ldm_register(struct device *device);

   pci_device_ldm_register(&ide_device_ldm);
}

static int ide_suspend(struct device *dev, u32 state, u32 level)
{
  u16 pmdsr;
  int unit, err=0;
  ide_hwif_t *hwif = (ide_hwif_t *)dev->driver_data;

  switch(level)
  { 

     case SUSPEND_POWER_DOWN:
     
       /* Put all the present drives in standby mode */
       for (unit = 0; unit < MAX_DRIVES; ++unit) {
	       ide_drive_t *drive = &hwif->drives[unit];

	       if (!drive->present)
		       continue;

	       /* set the drive to standby */
	       if (drive->driver != NULL)
		       err = DRIVER(drive)->standby(drive);
	       if (err)
		       break;
       }

       IT_IO_READ16(IT_PM_DSR, pmdsr);
       pmdsr |= IT_PM_DSR_IDESB;
       IT_IO_WRITE16(IT_PM_DSR, pmdsr);
       break;
  }
  
  return err;
}

static int ide_resume(struct device *dev, u32 level)
{
  u16 pmdsr;

  switch(level)
  {
     case RESUME_POWER_ON:

       IT_IO_READ16(IT_PM_DSR, pmdsr);
       pmdsr &= ~IT_PM_DSR_IDESB;
       IT_IO_WRITE16(IT_PM_DSR, pmdsr);       
       break;
  }
  return 0;
}

/*
 * Prototypes
 */
static void it8172_tune_drive (ide_drive_t *drive, byte pio);
#if defined(CONFIG_BLK_DEV_IDEDMA) && defined(CONFIG_IT8172_TUNING)
static byte it8172_dma_2_pio (byte xfer_rate);
static int it8172_tune_chipset (ide_drive_t *drive, byte speed);
static int it8172_config_chipset_for_dma (ide_drive_t *drive);
static int it8172_dmaproc(ide_dma_action_t func, ide_drive_t *drive);
#endif
unsigned int __init pci_init_it8172 (struct pci_dev *dev, const char *name);
void __init ide_init_it8172 (ide_hwif_t *hwif);


static void it8172_tune_drive (ide_drive_t *drive, byte pio)
{
    unsigned long flags;
    u16 drive_enables;
    u32 drive_timing;
    int is_slave	= (&HWIF(drive)->drives[1] == drive);
    
    pio = ide_get_best_pio_mode(drive, pio, 4, NULL);
    pci_read_config_word(HWIF(drive)->pci_dev, 0x40, &drive_enables);
    pci_read_config_dword(HWIF(drive)->pci_dev, 0x44, &drive_timing);

    /*
     * FIX! The DIOR/DIOW pulse width and recovery times in port 0x44
     * are being left at the default values of 8 PCI clocks (242 nsec
     * for a 33 MHz clock). These can be safely shortened at higher
     * PIO modes. The DIOR/DIOW pulse width and recovery times only
     * apply to PIO modes, not to the DMA modes.
     */
    
    /*
     * Enable port 0x44. The IT8172G spec is confused; it calls
     * this register the "Slave IDE Timing Register", but in fact,
     * it controls timing for both master and slave drives.
     */
    drive_enables |= 0x4000;

    if (is_slave) {
	drive_enables &= 0xc006;
	if (pio > 1)
	    /* enable prefetch and IORDY sample-point */
	    drive_enables |= 0x0060;
    } else {
	drive_enables &= 0xc060;
	if (pio > 1)
	    /* enable prefetch and IORDY sample-point */
	    drive_enables |= 0x0006;
    }

    save_flags(flags);
    cli();
    pci_write_config_word(HWIF(drive)->pci_dev, 0x40, drive_enables);
    restore_flags(flags);
}

#if defined(CONFIG_BLK_DEV_IDEDMA) && defined(CONFIG_IT8172_TUNING)
/*
 *
 */
static byte it8172_dma_2_pio (byte xfer_rate)
{
    switch(xfer_rate) {
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

static int it8172_tune_chipset (ide_drive_t *drive, byte speed)
{
    ide_hwif_t *hwif	= HWIF(drive);
    struct pci_dev *dev	= hwif->pci_dev;
    int a_speed		= 3 << (drive->dn * 4);
    int u_flag		= 1 << drive->dn;
    int u_speed		= 0;
    int err		= 0;
    byte reg48, reg4a;

    pci_read_config_byte(dev, 0x48, &reg48);
    pci_read_config_byte(dev, 0x4a, &reg4a);

    /*
     * Setting the DMA cycle time to 2 or 3 PCI clocks (60 and 91 nsec
     * at 33 MHz PCI clock) seems to cause BadCRC errors during DMA
     * transfers on some drives, even though both numbers meet the minimum
     * ATAPI-4 spec of 73 and 54 nsec for UDMA 1 and 2 respectively.
     * So the faster times are just commented out here. The good news is
     * that the slower cycle time has very little affect on transfer
     * performance.
     */
    
    switch(speed) {
    case XFER_UDMA_4:
    case XFER_UDMA_2:	//u_speed = 2 << (drive->dn * 4); break;
    case XFER_UDMA_5:
    case XFER_UDMA_3:
    case XFER_UDMA_1:	//u_speed = 1 << (drive->dn * 4); break;
    case XFER_UDMA_0:	u_speed = 0 << (drive->dn * 4); break;
    case XFER_MW_DMA_2:
    case XFER_MW_DMA_1:
    case XFER_MW_DMA_0:
    case XFER_SW_DMA_2:	break;
    default:		return -1;
    }

    if (speed >= XFER_UDMA_0) {
	pci_write_config_byte(dev, 0x48, reg48 | u_flag);
	reg4a &= ~a_speed;
	pci_write_config_byte(dev, 0x4a, reg4a | u_speed);
    } else {
	pci_write_config_byte(dev, 0x48, reg48 & ~u_flag);
	pci_write_config_byte(dev, 0x4a, reg4a & ~a_speed);
    }

    it8172_tune_drive(drive, it8172_dma_2_pio(speed));

    if (!drive->init_speed)
	drive->init_speed = speed;
    err = ide_config_drive_speed(drive, speed);
    drive->current_speed = speed;
    return err;
}

static int it8172_config_chipset_for_dma (ide_drive_t *drive)
{
    struct hd_driveid *id = drive->id;
    byte speed;

    if (id->dma_ultra & 0x0010) {
	speed = XFER_UDMA_2;
    } else if (id->dma_ultra & 0x0008) {
	speed = XFER_UDMA_1;
    } else if (id->dma_ultra & 0x0004) {
	speed = XFER_UDMA_2;
    } else if (id->dma_ultra & 0x0002) {
	speed = XFER_UDMA_1;
    } else if (id->dma_ultra & 0x0001) {
	speed = XFER_UDMA_0;
    } else if (id->dma_mword & 0x0004) {
	speed = XFER_MW_DMA_2;
    } else if (id->dma_mword & 0x0002) {
	speed = XFER_MW_DMA_1;
    } else if (id->dma_mword & 0x0001) {
	speed = XFER_MW_DMA_0;
    } else if (id->dma_1word & 0x0004) {
	speed = XFER_SW_DMA_2;
    } else {
	speed = XFER_PIO_0 + ide_get_best_pio_mode(drive, 255, 4, NULL);
    }

    (void) it8172_tune_chipset(drive, speed);

    return ((int)((id->dma_ultra >> 11) & 7) ? ide_dma_on :
	    ((id->dma_ultra >> 8) & 7) ? ide_dma_on :
	    ((id->dma_mword >> 8) & 7) ? ide_dma_on :
	    ((id->dma_1word >> 8) & 7) ? ide_dma_on :
	    ide_dma_off_quietly);
}

static int it8172_dmaproc(ide_dma_action_t func, ide_drive_t *drive)
{
    switch (func) {
    case ide_dma_check:
	return ide_dmaproc((ide_dma_action_t)it8172_config_chipset_for_dma(drive),
			   drive);
    default :
	break;
    }
    /* Other cases are done by generic IDE-DMA code. */
    return ide_dmaproc(func, drive);
}

#endif /* defined(CONFIG_BLK_DEV_IDEDMA) && (CONFIG_IT8172_TUNING) */


unsigned int __init pci_init_it8172 (struct pci_dev *dev, const char *name)
{
    unsigned char progif;
    
    /*
     * Place both IDE interfaces into PCI "native" mode
     */
    (void)pci_read_config_byte(dev, PCI_CLASS_PROG, &progif);
    (void)pci_write_config_byte(dev, PCI_CLASS_PROG, progif | 0x05);    

    return IT8172_IDE_IRQ;
}


void __init ide_init_it8172 (ide_hwif_t *hwif)
{
    struct pci_dev* dev = hwif->pci_dev;
    unsigned long cmdBase, ctrlBase;
    
    hwif->tuneproc = &it8172_tune_drive;
    hwif->drives[0].autotune = 1;
    hwif->drives[1].autotune = 1;

    if (!hwif->dma_base)
	return;

#ifndef CONFIG_BLK_DEV_IDEDMA
    hwif->autodma = 0;
#else /* CONFIG_BLK_DEV_IDEDMA */
#ifdef CONFIG_IT8172_TUNING
    hwif->autodma = 1;
    hwif->dmaproc = &it8172_dmaproc;
    hwif->speedproc = &it8172_tune_chipset;
#endif /* CONFIG_IT8172_TUNING */
#endif /* !CONFIG_BLK_DEV_IDEDMA */

    cmdBase = dev->resource[0].start;
    ctrlBase = dev->resource[1].start;
    
    ide_init_hwif_ports(&hwif->hw, cmdBase, ctrlBase | 2, NULL);
    memcpy(hwif->io_ports, hwif->hw.io_ports, sizeof(hwif->io_ports));
    hwif->noprobe = 0;

    ide_device_ldm.driver_data = hwif;
    ide_ldm_device_register();
    ide_ldm_driver_register();
}
