/*
 * ATI Xilleon 220/225 UDMA controller driver
 *
 * Copyright (C) 2001, 2002 Metro Link, Inc.
 *
 * May be copied or modified under the terms of the GNU General Public License
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/pci.h>
#include <linux/ide.h>

#include <linux/stat.h>
#include <linux/proc_fs.h>

#include <asm/ati/xilleon.h>
#include <asm/ati/xilleonreg_kernel.h>
#include <asm/ati/xilleonint.h>
#include <asm/ati/x225_dif.h>
#include <asm/io.h>

#include "ide_modes.h"

extern long int chipId;

int xilleon_ide_proc;

extern char *ide_xfer_verbose (byte xfer_rate);
static struct pci_dev *bmide_dev;
extern int (*xilleon_ide_display_info)(char *, char **, off_t, int); /* ide-proc.c */

#define XILLEON_IDE_DEBUG_DRIVE_INFO 1
#define DEBUG 1


static int xilleon_ide_get_info(char *buffer, char **addr, off_t offset, int count)
{
    char *p = buffer;
    u32 bibma = pci_resource_start(bmide_dev, 4);
    struct pci_dev *dev = bmide_dev;
    u16 udma_mode;
    u8  c0 = 0, c1 = 0, udma_control;

    p += sprintf(p, "\n     ATI Xilleon Chipset.\n");

    pci_read_config_byte(dev, PCI_REVISION_ID, &c1);

    p+= sprintf(p, "       Revision:   %#x\n", c1);

        /*
         * at that point bibma+0x2 et bibma+0xa are byte registers
         * to investigate:
         */

    pci_read_config_byte(bmide_dev, pciideIDE_UDMA_CONTROL, &udma_control);

    c0 = inb_p((unsigned short)bibma + 0x02);
    c1 = inb_p((unsigned short)bibma + 0x0a);

    p += sprintf(p, "--------------- Primary Channel ----------------\n");
    p += sprintf(p, "                %sabled                         \n",
             (c0&0x80) ? "dis" : " en");

    p += sprintf(p, "--------------- drive0 --------- drive1 --------\n");
    p += sprintf(p, "DMA enabled:    %s              %s             \n",
             (c0&0x20) ? "yes" : "no ",
             (c0&0x40) ? "yes" : "no ");
#if 0
    p += sprintf(p, "UDMA enabled:   %s              %s             \n",
             (udma_control & 0x01) ? "yes" : "no ",
             (udma_control & 0x02) ? "yes" : "no ");
#endif
    p += sprintf(p, "UDMA mode:   %s                %s           \n",
             (udma_mode & 0x0002) ? "2" : ((udma_mode & 0x0001) ? "1" : 
                           ((udma_mode & 0x000f) ? "X" : "0")),
             (udma_mode & 0x0020) ? "2" : ((udma_mode & 0x0010) ? "1" : 
                           ((udma_mode & 0x00f0) ? "X" : "0")));
    return p-buffer;     /* => must be less than 4k! */
}

/*
 * initialize the x220 chip
 */

unsigned int __init pci_init_x220_ide (struct pci_dev *dev, const char *name)
{
    u8 rev  = 0;
    u32 bmbase  = 0;

    pci_read_config_byte(dev, PCI_REVISION_ID, &rev);
    printk(KERN_INFO "XILLEON 220 IDE: rev %02x IDE UDMA controller on pci%s\n", rev, dev->slot_name);

    bmide_dev = dev;

    /* Indicate drives are capable of dma transfers */
    pci_read_config_dword(bmide_dev, pciideIDE_BM_BASE, &bmbase);
    bmbase &= 0xfffffff0;
    OUT_BYTE((IN_BYTE(bmbase + ioideIDE_PRI_BM_STATUS) | 0x60), bmbase + ioideIDE_PRI_BM_STATUS);


#ifdef CONFIG_PROC_FS
    if (!xilleon_ide_proc) {
        xilleon_ide_proc = 1;
        xilleon_ide_display_info = &xilleon_ide_get_info;
    }
#endif

    return 0;
}


/*
 * initialize the x225 chip
 */

unsigned int __init pci_init_x225_ide (struct pci_dev *dev, const char *name)
{
    u8 rev  = 0;
    u32 bmbase  = 0;

    pci_read_config_byte(dev, PCI_REVISION_ID, &rev);
    printk(KERN_INFO "XILLEON 225 IDE: rev %02x IDE UDMA controller on pci%s\n", rev, dev->slot_name);

    bmide_dev = dev;

    /* Indicate drives are capable of dma transfers */
    pci_read_config_dword(bmide_dev, pciideX225_IDE_BM_BASE, &bmbase);
    bmbase &= 0xfffffff0;
    OUT_BYTE((IN_BYTE(bmbase + ioideX225_IDE_PRI_BM_STATUS) | 0x60), bmbase + ioideX225_IDE_PRI_BM_STATUS);
    OUT_BYTE((IN_BYTE(bmbase + ioideX225_IDE_SEC_BM_STATUS) | 0x60), bmbase + ioideX225_IDE_SEC_BM_STATUS);
    

/*
#ifdef CONFIG_PROC_FS
    if (!xilleon_ide_proc) {
        xilleon_ide_proc = 1;
        xilleon_ide_display_info = &xilleon_ide_get_info;
    }
#endif
*/
    return 0;
}

/********************************************************
*
* max_drive_pio_mode
*
* Determine the maximum pio mode supported by this drive
*
*********************************************************/
static byte max_drive_pio_mode (ide_drive_t *drive)
{
    unsigned short pio_timing[6] = {960, 480, 240, 180, 120, 90};
    unsigned short adv_modes     = drive->id->eide_pio_modes;
    unsigned short iordy_time    = drive->id->eide_pio_iordy;
    byte mode_mask               = 0x01;
    byte max_pio = 2;

    if (drive->id->field_valid & 0x02) {
        /* If the drive supports flow control, use the minimum iordy time */
        /* to find the best pio mode supported by the drive */
        if (drive->id->eide_pio_iordy > 0) {
        /* iordy supported, use minimum drive timing to find best mode */
            for (max_pio = 5; max_pio && iordy_time > pio_timing[max_pio]; max_pio--);
        } else {
        /* iordy not supported, use advanced mode support flags to find best mode */
            for (max_pio = 2; adv_modes & mode_mask; mode_mask <<= 1) {
                max_pio++;
            }
        }
    }     
#ifdef DEBUG
    printk("XILLEON 225 IDE drive %d, max pio mode = %d\n", drive->dn, max_pio);
#endif
    return max_pio;
}

/********************************************************
*
* max_drive_dma_mode
*
* Determine the maximum dma mode supported by this drive
*
*********************************************************/
static byte max_drive_dma_mode (ide_drive_t *drive)
{
    unsigned short dma_modes     = drive->id->dma_mword;
    unsigned short mode_mask     = 0x80;
    byte max_dma                 = 0;

    if (drive->id->field_valid & 0x02 && dma_modes & 0xff) {
        for (max_dma = 7; (dma_modes & mode_mask) == 0; mode_mask >>= 1) {
                max_dma--;
        }
#ifdef DEBUG
        printk("XILLEON 225 IDE drive %d, max dma mode = %d\n", drive->dn, max_dma);
#endif        
        return max_dma;
    } else {
#ifdef DEBUG
        printk("XILLEON 225 IDE drive %d, dma not supported\n", drive->dn);
#endif
        return 0x0f;
    }
}

/********************************************************
*
* max_drive_udma_mode
*
* Determine the maximum dma mode supported by this drive
*
*********************************************************/
static byte max_drive_udma_mode (ide_drive_t *drive)
{
    unsigned short udma_modes    = drive->id->dma_ultra;
    unsigned short mode_mask     = 0x80;
    byte max_udma                = 0;

    if (drive->id->field_valid & 0x04 && udma_modes & 0xff) {
        for (max_udma = 7; (udma_modes & mode_mask) == 0; mode_mask >>= 1) {
                max_udma--;
	}
        if (eighty_ninty_three(drive)) {
#ifdef DEBUG
            printk("XILLEON 225 IDE drive %d, max udma mode = %d\n", drive->dn, max_udma);
#endif        
            return max_udma;
        } else {
#ifdef DEBUG
	    printk("XILLEON 225 IDE IDE cable does not support UDMA modes > 2.  Drive %d, max udma mode = %d\n", drive->dn, max_udma > 2 ? 2 : max_udma);
#endif        
            return max_udma > 2 ? 2 : max_udma;
        }
    } else {
#ifdef DEBUG
        printk("XILLEON 225 IDE drive %d, udma not supported\n", drive->dn);
#endif
        return 0x0f;
    }
}

static int x220_ide_tune_chipset (ide_drive_t *drive, byte speed)
{
    byte dma_timing[]= {  // dma timing parameters
        0x77,   // mode 0 = 240 ns / 240 ns
        0x21,   // mode 1 =  90 ns /  60 ns
        0x20    // mode 2 =  90 ns /  30 ns
    };

    byte pio_timing[] = {  // pio timing parameters
        0x5d,   // mode 0 = 180 ns  /  420 ns
        0x47,   // mode 1 = 150 ns  /  240 ns
        0x34,   // mode 2 = 120 ns  /  150 ns
        0x22,   // mode 3 =  90 ns  /   90 ns
        0x20    // mode 4 =  90 ns  /   30 ns
    };

    ide_hwif_t *hwif    = HWIF(drive);
    struct pci_dev *dev = hwif->pci_dev;
    int err;

    byte timing_reg_off;    // timing register offset
    byte pio_mode_reg;      // pio mode register
    byte pio_mode;      // pio mode
    byte pio_control;   // pio control register
    byte dma_mode;      // dma mode
    byte udma_mode;         // udma mode
    byte udma_mode_reg;     // udma mode register
    byte udma_control_reg;  // udma control register
    byte udma_enable = 0;   // udma enable


    if (drive->dn > 1)      // we only support two drives
        return -1;

    /* All three transfer types require pio mode to be set to */
    /* maximum mode supported by both the drive and controller */
    /* Get the current contents and clear the mode field for this drive*/
    pci_read_config_byte(dev, pciideIDE_PIO_MODE, &pio_mode_reg);
    pio_mode_reg = drive->dn ? pio_mode_reg & 0x0f : pio_mode_reg & 0xf0;
    /* Get the highest mode the drive can support */
    pio_mode   = max_drive_pio_mode(drive);

    /* All three transfer types require udma control to be set/reset */
    /* according to the transfer mode to be used */
    /* Get the current contents and clear the enable flag for this drive*/
    pci_read_config_byte(dev, pciideIDE_UDMA_CONTROL, &udma_control_reg);
    udma_control_reg = drive->dn ? udma_control_reg & 0xfd : pio_mode_reg & 0xfe;
    switch(speed) {
        case XFER_PIO_4:
        case XFER_PIO_3:
        case XFER_PIO_2:
        case XFER_PIO_1:
        case XFER_PIO_0:
        /* Setting transfer mode to PIO */
            /* If requested mode is higher than drive supports, set to highest supported */
            pio_mode = pio_mode > (speed - XFER_PIO_0) ? speed - XFER_PIO_0 : pio_mode;
            /* Set timing reg offset for master/slave drive */
            timing_reg_off = drive->dn ? pciideIDE_PIO_TIMING + 1 : pciideIDE_PIO_TIMING;
            pci_write_config_byte(dev, timing_reg_off, pio_timing[pio_mode]);
#ifdef DEBUG
            printk("xilleon_ide_tune_chipset for drive %s (%d), pio mode %d\n", drive->name, drive->dn, pio_mode);
#endif
            /* Enable prefetch and write posting */
            pci_read_config_byte(dev, pciideIDE_PIO_CONTROL, &pio_control);
            pio_control = drive->dn ? pio_control | 0x50 : pio_control | 0xa0;
            pci_write_config_byte(dev, pciideIDE_PIO_CONTROL, pio_control);
            break;

        case XFER_MW_DMA_2:
        case XFER_MW_DMA_1:
        case XFER_MW_DMA_0:
        /* Setting transfer mode to Multiword DMA */
            dma_mode   = speed - XFER_MW_DMA_0;
            timing_reg_off = drive->dn ? pciideIDE_DMA_TIMING + 1 : pciideIDE_DMA_TIMING;
            pci_write_config_byte(dev, timing_reg_off, dma_timing[dma_mode]);
#ifdef DEBUG
            printk("xilleon_ide_tune_chipset for drive %s (%d), dma mode %d, pio mode %d\n",
                    drive->name, drive->dn, dma_mode, pio_mode);
#endif
            break;

        case XFER_UDMA_5:
        case XFER_UDMA_4:
        case XFER_UDMA_3:
        case XFER_UDMA_2:
        case XFER_UDMA_1:
        case XFER_UDMA_0:
        /* Setting transfer mode to Ultra DMA */
            udma_enable       = drive->dn ? 0x82 : 0x81;
            pci_read_config_byte(dev, pciideIDE_UDMA_MODE, &udma_mode_reg);
            udma_mode_reg |= drive->dn ? (speed & 0xf) << 4 : speed & 0xf;
            pci_write_config_byte(dev, pciideIDE_UDMA_MODE, udma_mode_reg);
#ifdef DEBUG
            printk("xilleon_ide_tune_chipset for drive %s (%d), udma mode %d, pio mode %d\n",
                    drive->name, drive->dn, (speed - XFER_UDMA_0), pio_mode);
#endif
        default:
            break;
    }
#if XILLEON_IDE_DEBUG_DRIVE_INFO
        printk("%s: %s drive%d\n", drive->name, ide_xfer_verbose(speed), drive->dn);
#endif /* XILLEON_IDE_DEBUG_DRIVE_INFO */

    if (!drive->init_speed)
        drive->init_speed = speed;

#if XILLEON_IDE_DEBUG_DRIVE_INFO
    printk("PIO: %x, DMA: %x, UDMA: %x, UDMA_EN: %x\n",
               pio_timing, dma_timing, udma_mode, udma_enable);
#endif /* XILLEON_IDE_DEBUG_DRIVE_INFO */

    pci_write_config_byte(dev, pciideIDE_PIO_MODE, pio_mode_reg);
    pci_write_config_byte(dev, pciideIDE_UDMA_CONTROL, udma_control_reg | udma_enable);
    err = ide_config_drive_speed(drive, speed);
    drive->current_speed = speed;

    return err;
}


static int x225_ide_tune_chipset (ide_drive_t *drive, byte speed)
{

    ide_hwif_t *hwif    = HWIF(drive);
    struct pci_dev *dev = hwif->pci_dev;
    int err;
   
    u32 work_dword;
    u32 work_word;

    u8 tmg_cntl_reg_off;  // timing register offset
    u8 pio_cntl_reg_off;  // pio control register offset
    u8 pio_mode_reg_off;  // pio mode register offset
    u8 pio_mode_reg;      // pio mode register contents
    u8 pio_mode;          // pio mode 
    u8 pio_control;       // pio control register
    u8 dma_mode_reg_off;  // dma mode register offset
    u8 dma_mode;          // dma mode

    if (drive->dn > 3)      // we  support four drives
        return -1;

    /* Setup register offsets for current drive */
    /* Assume drives 0/1 are on primary, 2/3 are on secondary */
    if (drive->dn > 1)
    {
        pio_mode_reg_off = pciideX225_IDE_PIO_MODE+1;
        pio_cntl_reg_off = pciideX225_IDE_PIO_CONTROL+1;
        tmg_cntl_reg_off = drive->dn&1 ? pciideX225_IDE_TMG_CNTL+2 : pciideX225_IDE_TMG_CNTL+3;
        dma_mode_reg_off = drive->dn&1 ? pciideX225_IDE_DMA_MODE+3 : pciideX225_IDE_DMA_MODE+2;
    }
    else
    {
        pio_mode_reg_off = pciideX225_IDE_PIO_MODE;
        pio_cntl_reg_off = pciideX225_IDE_PIO_CONTROL;
        tmg_cntl_reg_off = drive->dn&1 ? pciideX225_IDE_TMG_CNTL : pciideX225_IDE_TMG_CNTL+1;
        dma_mode_reg_off = drive->dn&1 ? pciideX225_IDE_DMA_MODE+1 : pciideX225_IDE_DMA_MODE;
    }

    /* Reset auto-calc override so that controller calculates mode timing */
    pci_write_config_byte(dev, tmg_cntl_reg_off, 0x7f);

    /* All three transfer types require pio mode to be set to */
    /* maximum mode supported by both the drive and controller */
    /* Get the current contents and clear the mode field for this drive*/
    pci_read_config_byte(dev, pio_mode_reg_off, &pio_mode_reg);
    pio_mode_reg = drive->dn&1 ? pio_mode_reg & 0x07 : pio_mode_reg & 0x70;
    /* Get the highest mode the drive can support */
    pio_mode   = max_drive_pio_mode(drive);

    /* All three transfer types require udma control to be set/reset */
    /* according to the transfer mode to be used */
    /* Get the current contents and clear the enable flag for this drive*/
#if 0
    pci_read_config_byte(dev, pciideIDE_UDMA_CONTROL, &udma_control_reg);
    udma_control_reg = drive->dn ? udma_control_reg & 0xfd : pio_mode_reg & 0xfe;
#endif
    switch(speed) {
        case XFER_PIO_4:
        case XFER_PIO_3:
        case XFER_PIO_2:
        case XFER_PIO_1:
        case XFER_PIO_0:
        /* Setting transfer mode to PIO */
            /* If requested mode is higher than drive supports, set to highest supported */
            pio_mode = pio_mode > (speed - XFER_PIO_0) ? speed - XFER_PIO_0 : pio_mode;

#ifdef DEBUG
            printk("xilleon_ide_tune_chipset for drive %s (%d), pio mode %d\n", drive->name, drive->dn, pio_mode);
#endif
	    /* Enable prefetch and write posting */
#if 0
	    pci_read_config_byte(dev, pio_cntl_reg_off, &pio_control);
            pio_control = drive->dn ? pio_control | 0x50 : pio_control | 0xa0;
	    pci_write_config_byte(dev, pio_cntl_reg_off, pio_control);
#endif
	    break;

        case XFER_MW_DMA_2:
        case XFER_MW_DMA_1:
        case XFER_MW_DMA_0:
        /* Setting transfer mode to Multiword DMA */
            dma_mode   = speed - XFER_MW_DMA_0;        
            pci_write_config_byte(dev, dma_mode_reg_off, dma_mode); 
#ifdef DEBUG
            printk("xilleon_ide_tune_chipset for drive %s (%d), dma mode %d, pio mode %d\n", 
                    drive->name, drive->dn, dma_mode, pio_mode);
#endif
            break;

        case XFER_UDMA_5:
        case XFER_UDMA_4:
        case XFER_UDMA_3:
        case XFER_UDMA_2:
        case XFER_UDMA_1:
        case XFER_UDMA_0:
        /* Setting transfer mode to Ultra DMA */
	  dma_mode  = (speed - XFER_UDMA_0)|0x10;
            pci_write_config_byte(dev, dma_mode_reg_off, dma_mode);
#ifdef DEBUG
            printk("xilleon_ide_tune_chipset for drive %s (%d), udma mode %d, pio mode %d\n", 
                    drive->name, drive->dn, (speed - XFER_UDMA_0), pio_mode);
#endif
        default:
            break;
    }

    if (!drive->init_speed)
        drive->init_speed = speed;

    /* Set Read and Write Combining */
    pci_read_config_dword(dev, pciideX225_IDE_PCI_BUSMASTER_CNTL, &work_dword);
    pci_write_config_dword(dev, pciideX225_IDE_PCI_BUSMASTER_CNTL, work_dword|0x60);
       
    pio_mode_reg |= drive->dn&1 ? pio_mode << 4 : pio_mode;
    pci_write_config_byte(dev, pio_mode_reg_off, pio_mode_reg);
    err = ide_config_drive_speed(drive, speed);
    drive->current_speed = speed;

#if 0

    pci_read_config_word(dev, pciideIDE_DEVICE_ID, &work_word);
    printk("IDE_DEVICE_ID          =     %04x\n", work_word);

    pci_read_config_dword(dev, pciideIDE_TMG_CNTL, &work_dword);
    printk("IDE_TMG_CNTL           = %08x\n", work_dword);

    pci_read_config_word(dev, pciideIDE_PIO_CONTROL, &work_word);
    printk("IDE_PIO_CONTROL        =     %04x\n", work_word);

    pci_read_config_word(dev, pciideIDE_PIO_MODE, &work_word);
    printk("IDE_PIO_MODE           =     %04x\n", work_word);
 
    pci_read_config_dword(dev, pciideIDE_DMA_MODE, &work_dword);
    printk("IDE_DMA_MODE           = %08x\n", work_dword);

    pci_read_config_dword(dev, pciideIDE_PCI_BUSMASTER_CNTL, &work_dword);
    printk("IDE_PCI_BUSMASTER_CNTL = %08x\n", work_dword);


#endif

    return err;
}

/************************************

config_chipset_for_pio              

Set the controller and drive to the 
highest pio mode supported by both  

*************************************/

static void config_chipset_for_pio (ide_drive_t *drive)
{
    byte speed = XFER_PIO_0 + max_drive_pio_mode (drive);

    if ( chipId == 0x220 )
       (void) x220_ide_tune_chipset(drive, speed);
    else
       (void) x225_ide_tune_chipset(drive, speed);

    drive->current_speed = speed;

#ifdef DEBUG
    printk("config_chipset_for_pio, speed is %d \n", speed);
#endif
}

static void x220_ide_tune_drive (ide_drive_t *drive, byte pio)
{
#ifdef DEBUG
    printk("tune drive %s (%d) for pio %d\n", drive->name, drive->dn, pio);
#endif
    (void) x220_ide_tune_chipset(drive, XFER_PIO_0 + pio);
}

static void x225_ide_tune_drive (ide_drive_t *drive, byte pio)
{
#ifdef DEBUG
    printk("tune drive %s (%d) for pio %d\n", drive->name, drive->dn, pio);
#endif
    (void) x225_ide_tune_chipset(drive, XFER_PIO_0 + pio);
}


static int config_chipset_for_dma (ide_drive_t *drive)
{
    struct hd_driveid *id   = drive->id;
    byte            speed = 0x0f;

#ifdef DEBUG
    printk("config_chipset_for_dma for drive %s (%d)\n", drive->name, drive->dn);
#endif

    /* Check if drive supports ultra dma - get highest supported mode */
    if ((speed = max_drive_udma_mode (drive)) != 0x0f) {
       speed = speed > 5 ? 5 + XFER_UDMA_0 : speed + XFER_UDMA_0;
       /* Check if drive supports dma - get highest supported mode */
    } else if ((speed = max_drive_dma_mode (drive)) != 0x0f) {
       speed = speed > 2 ? 2 + XFER_MW_DMA_0 : speed + XFER_MW_DMA_0;
    } 
    if(speed == 0x0f) {
       /* Speed not set yet, get max pio mode supported by the drive */
        speed = max_drive_pio_mode (drive);
        speed = speed > 4 ? 4 + XFER_PIO_0 : speed + XFER_PIO_0;
    }
    if ( chipId == 0x220 )
        (void) x220_ide_tune_chipset(drive, speed);
    else
        (void) x225_ide_tune_chipset(drive, speed);

    /* Set return code based on drive id information */
    return (((id->dma_ultra & 0xff00 && id->field_valid & 0x04) || 
             (id->dma_mword & 0xff00 && id->field_valid & 0x02)) ? ide_dma_on : ide_dma_off_quietly);
}

static int config_drive_xfer_rate (ide_drive_t *drive)
{
    struct hd_driveid *id = drive->id;
    ide_dma_action_t dma_func = ide_dma_on;

#ifdef DEBUG
    printk("config_drive_xfer_rate for drive %s (%d)\n", drive->name, drive->dn);
#endif

    if (id && (id->capability & 1) && HWIF(drive)->autodma) {

        /* Consult the list of known "bad" drives */
        if (ide_dmaproc(ide_dma_bad_drive, drive)) {
            dma_func = ide_dma_off;
        }
        /* Not in list of bad drives - check udma */
        else if (id->field_valid & 0x06) {
            /* Try to config ultra dma or multiword dma */
            dma_func = config_chipset_for_dma(drive);
        }
        if (dma_func != ide_dma_on) {
            config_chipset_for_pio(drive);
        }
    }
    return HWIF(drive)->dmaproc(dma_func, drive);
}


/*
 * 1 dmaing, 2 error, 4 intr
 */
static int dma_timer_expiry (ide_drive_t *drive)
{
    byte dma_stat = inb(HWIF(drive)->dma_base+2);

#ifdef DEBUG
    printk("%s: dma_timer_expiry: dma status == 0x%02x\n", drive->name, dma_stat);
#endif /* DEBUG */

    HWGROUP(drive)->expiry = NULL;  /* one free ride for now */
    if (dma_stat & 2) { /* ERROR */
        byte stat = GET_STAT();
        return ide_error(drive, "dma_timer_expiry", stat);
    }
    if (dma_stat & 1)   /* DMAing */
        return WAIT_CMD;
    return 0;
}

static int x220_ide_dmaproc(ide_dma_action_t func, ide_drive_t *drive)
{

    ide_hwif_t *hwif = HWIF(drive);
    unsigned long dma_base = hwif->dma_base;
    unsigned int count, reading = 0;

#ifdef LINUX_ATI_X220_A1X
    /* Needed for A1x versions of the X220 */
    SETFLAG_REGMM32(PCIC_BUSSLAVE_CNTL, CACHE_INVALIDATE_EN);
#endif

    switch (func) {
    case ide_dma_check:
        return config_drive_xfer_rate(drive);

    case ide_dma_read:
            reading = 1 << 3;

    case ide_dma_write:
            SELECT_READ_WRITE(hwif,drive,func);
            if (!(count = ide_build_dmatable(drive, func)))
                return 1;                                   /* try PIO instead of DMA */
            outl(hwif->dmatable_dma, dma_base + 4);         /* PRD table */
            outb(reading, dma_base);                        /* specify r/w */
            outb(inb(dma_base + 2) | 6, dma_base + 2);      /* clear INTR & ERROR flags */
            drive->waiting_for_dma = 1;
#ifdef CONFIG_MIPS
#ifdef CONFIG_ATI_XILLEON
            asm volatile ("sync");
#endif
#endif
            outb(inb(dma_base) | 1, dma_base);      /* start DMA */
            if (drive->media != ide_disk)
                return 0;
            ide_set_handler(drive, &ide_dma_intr, WAIT_CMD, dma_timer_expiry);  /* issue cmd to drive */
            OUT_BYTE(reading ? WIN_READDMA : WIN_WRITEDMA, IDE_COMMAND_REG);
            return 0;

    default :
        break;
    }

    /* Other cases are done by generic IDE-DMA code. */
    return ide_dmaproc(func, drive);

}


static int x225_ide_dmaproc(ide_dma_action_t func, ide_drive_t *drive)
{

    ide_hwif_t *hwif = HWIF(drive);
    unsigned long dma_base = hwif->dma_base;
    byte unit = (drive->select.b.unit & 0x01);
    unsigned int count, reading = 0;
    byte dma_stat;

    switch (func) {
    case ide_dma_check: 
        return config_drive_xfer_rate(drive);

	case ide_dma_read:
		reading = 1 << 3;
	case ide_dma_write:
		SELECT_READ_WRITE(hwif,drive,func);
		if (!(count = ide_build_dmatable(drive, func)))
			return 1;	/* try PIO instead of DMA */
        __asm__ volatile ("sync");
		outl(hwif->dmatable_dma, dma_base + 4); /* PRD table */
		outb(reading, dma_base);			/* specify r/w */
		outb(inb(dma_base+2)|6, dma_base+2);		/* clear INTR & ERROR flags */
		drive->waiting_for_dma = 1;
		if (drive->media != ide_disk)
			return 0;
		ide_set_handler(drive, &ide_dma_intr, WAIT_CMD, dma_timer_expiry);	/* issue cmd to drive */
		OUT_BYTE(reading ? WIN_READDMA : WIN_WRITEDMA, IDE_COMMAND_REG);
	case ide_dma_begin:
#ifdef CONFIG_MIPS
#ifdef CONFIG_ATI_XILLEON
            asm volatile ("sync");
#endif
#endif
		outb(inb(dma_base)|1, dma_base);		/* start DMA */
		return 0;

    default :
        break;
    }

    /* Other cases are done by generic IDE-DMA code. */
    return ide_dmaproc(func, drive);

}

/*
 * Init the hwif structure for this controller
 */
void __init ide_init_x220 (ide_hwif_t *hwif)
{
    if (!hwif->irq)
                hwif->irq = XILLEON_IDE_INT;

    hwif->tuneproc = &x220_ide_tune_drive;
    hwif->speedproc = &x220_ide_tune_chipset;

    if (hwif->dma_base) {
        if (!noautodma)
            hwif->autodma = 1;
        hwif->dmaproc = &x220_ide_dmaproc;
    } else {
        hwif->autodma = 0;
        hwif->drives[0].autotune = 1;
        hwif->drives[1].autotune = 1;
    }

    return;
}



/*
 * Init the hwif structure for this controller
 */
void __init ide_init_x225 (ide_hwif_t *hwif)
{
    if (!hwif->irq)
		hwif->irq = X225_IDE_INT;

    hwif->tuneproc = &x225_ide_tune_drive;
    hwif->speedproc = &x225_ide_tune_chipset;
    hwif->serialized = 1;

    if (hwif->dma_base) {
        if (!noautodma)
            hwif->autodma = 1;
        hwif->dmaproc = &x225_ide_dmaproc;
    } else {
        hwif->autodma = 0;
        hwif->drives[0].autotune = 1;
        hwif->drives[1].autotune = 1;
    }
    return;
}

unsigned int __init ata66_x220(ide_hwif_t *hwif)
{
    return 1;
}


unsigned int __init ata66_x225(ide_hwif_t *hwif)
{
    return 1;
}

void __init ide_dmacapable_x220(ide_hwif_t *hwif, unsigned long dmabase)
{
    hwif->dma_base = dmabase;
    ide_setup_dma(hwif, dmabase, 8);
}

void __init ide_dmacapable_x225(ide_hwif_t *hwif, unsigned long dmabase)
{
    hwif->dma_base = dmabase;
    ide_setup_dma(hwif, dmabase, 8);
}
