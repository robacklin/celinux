
/************************************************************************/
/*
  Copyright (C) 2003  Chin Zhou (czhou@ati.com), ATI Technologies Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; version 2
  of the License, June 1991.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, 
  MA  02111-1307, USA.
*/
/************************************************************************/

/************************************************************************/
/*
   drivers/mtd/nand/stw_sflash.c
 
   Descriptions:
 
   Support for a serial flash module on ATI Xilleon 225 Set Top Wonder.
 
   A hardware specific device driver for the serial flash device which 
   is used on ATI's STW evaluation systems. The serial flash device is 
   M25PXX from ST Microelectronics, with SPI-compatible serial interface. 
 
   The driver provides basic interface between the device specific layer 
   and standard MTD layer. 
 */
/************************************************************************/

/************************************************************************/
/*

  Memory Orgnization of M25PXX
    - Total 128KB (131,072 bytes)
    - 4 sectors, 32KB (32768 bytes) each
    - 1024 pages, 128 bytes per page

    sector 0:  0x00000 - 0x07fff
    sector 1:  0x08000 - 0x0ffff
    sector 2:  0x10000 - 0x17fff
    sector 3:  0x18000 - 0x1ffff

  Features:
    - page programmable ( 1-->0 )
    - bulk and sector erasable, not page erasable ( 0-->1 )

  One-byte Instruction Code
    - Write Enable                 (WREN) 0b 0000 0110 (0x06)
    - Write Disable                (WRDI) 0b 0000 0100 (0x04)
    - Read Status Register         (RDSR) 0b 0000 0101 (0x05)
    - Write Status Register        (WRSR) 0b 0000 0001 (0x01)
    - Read Data Bytes              (READ) 0b 0000 0011 (0x03)
    - Page Program                   (PP) 0b 0000 0010 (0x02)
    - Sector/block Erase             (SE) 0b 1101 1000 (0xD8)
    - Bulk Erase                     (BE) 0b 1100 0111 (0xC7)
    - Deep Power-Down                (DP) 0b 1011 1001 (0xB9)
    - Release from DP, 
      and Read Electronic Signature (RES) 0b 1010 1011 (0xAB)
*/
/************************************************************************/

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/ati/xilleon.h>

#include "stw_sflash.h"


#define SFLASH_BURST_SIZE      1

#define NUM_SF_PARTITIONS      1


static struct mtd_info *stwsf_mtd = NULL;

static struct mtd_partition stwsf_par_info[] = 
{
  { name: "STW MTD Serial Flash Partition 0",
    offset: 0x0,
    size: 128*1024 
  }

};


int    sflash_exists = 0;
int    chip_erased   = 0;

/************************************************************************

 STW serial flash MTD inteface routines.

*************************************************************************/

/* serial flash block/sector erase */

static int sflash_erase(struct mtd_info *mtd,
			struct erase_info *instr)
{

  int        page, len, pages_per_block,ret;
  uint32_t   sflash_start_addr, sflash_blk_num;
  int        total_blks, block_shift;

  struct sflash_chip   *this = mtd->priv;

  DECLARE_WAITQUEUE(wait, current);

  DEBUG (MTD_DEBUG_LEVEL3,
	 "sflash_erase: start = 0x%08x, len = %i\n",
	 (unsigned int) instr->addr, (unsigned int) instr->len);

  /* Start address must align on block boundary */
  if (instr->addr & (mtd->erasesize - 1)) {
    DEBUG (MTD_DEBUG_LEVEL0,
	   "sflash_erase: Unaligned address\n");
    return -EINVAL;
  }

  /* Length must align on block boundary */
  if (instr->len & (mtd->erasesize - 1)) {
    DEBUG (MTD_DEBUG_LEVEL0,
	   "sflash_erase: Length not block aligned\n");
    return -EINVAL;
  }

  /* Do not allow erase past end of device */
  if ((instr->len + instr->addr) > mtd->size) {
    DEBUG (MTD_DEBUG_LEVEL0,
	   "sflash_erase: Erase past end of device\n");
    return -EINVAL;
  }

 retry:
  /* Grab the lock and see if the device is available */
  spin_lock_bh (&this->chip_lock);

  switch (this->state) {
  case SF_READY:
    this->state = SF_ERASI;
    break;

  default:
    set_current_state (TASK_UNINTERRUPTIBLE);
    add_wait_queue (&this->wq, &wait);
    spin_unlock_bh (&this->chip_lock);
    schedule();

    remove_wait_queue (&this->wq, &wait);
    goto retry;
  };

  /* Shift to get first page */
  page = (int) (instr->addr >> this->page_shift);

  /* Calculate pages in each block */
  pages_per_block = mtd->erasesize / (1 << this->page_shift);

  /* Loop through the pages */
  len = instr->len;
  sflash_start_addr = instr->addr;
  instr->state = MTD_ERASING;

  while (len) {
    /* erase the blocks, based on ATI's serial flash state-machine */
    total_blks = stwsf_get_total_blocks();

    /* get the block number */
    block_shift = this->chip_shift - (total_blks >> 1);
    sflash_blk_num = (sflash_start_addr >> block_shift); 

    DEBUG (MTD_DEBUG_LEVEL3,  
	   "Sector start addr: %x total_blks: %x blk_num: %x \n", 
	   sflash_start_addr, total_blks, sflash_blk_num );

    /* erase the serial flash blocks */
    if (!chip_erased) {
      if ( stwsf_chip_erase() != 0) {
	printk(KERN_ERR "block_erase error.\n");
      }
      chip_erased = 1;
    }

    if ( stwsf_block_erase_verify(sflash_blk_num) != 0) {
      printk(KERN_ERR "block_erase_verify error.\n");
    }

    /* Increment page address and decrement length */
    len -= mtd->erasesize;
    /* increment to next block */
    sflash_start_addr += mtd->erasesize;
    page += pages_per_block;

    /* Release the spin lock */
    spin_unlock_bh (&this->chip_lock);

  erase_retry:
    /* Check the state and sleep if it changed */
    spin_lock_bh (&this->chip_lock);
    if (this->state == SF_ERASI) {
      continue;
    }
    else {
      set_current_state (TASK_UNINTERRUPTIBLE);
      add_wait_queue (&this->wq, &wait);
      spin_unlock_bh (&this->chip_lock);
      schedule();

      remove_wait_queue (&this->wq, &wait);
      goto erase_retry;
    }
  }
  instr->state = MTD_ERASE_DONE;
  spin_unlock_bh (&this->chip_lock);
	
  ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;;
  /* Do call back function */
  if (!ret && instr->callback)
    instr->callback (instr);

	/* The device is ready */
  spin_lock_bh (&this->chip_lock);
  this->state = SF_READY;
  spin_unlock_bh (&this->chip_lock);

  /* Return OK */
  return ret;

}


static int sflash_read( struct mtd_info *mtd,
			loff_t from,
			size_t len,
			size_t *retlen,
			u_char *buf )
{

  int erase_state = 0;
  struct sflash_chip *this = mtd->priv;

  DECLARE_WAITQUEUE(wait, current);

  DEBUG ( MTD_DEBUG_LEVEL3,
	  "sflash_read: from = 0x%08x, len = %i\n", 
	  (unsigned int) from, (int) len );

  /* Do not allow reads past end of device */
  if ((from + len) > (mtd->size) ) {
    DEBUG ( MTD_DEBUG_LEVEL0,
	    "sflash_read: Attempt read beyond end of device\n" );
    *retlen = 0;
    spin_lock_bh (&this->chip_lock);
    this->state = SF_READY;
    wake_up (&this->wq);
    spin_unlock_bh (&this->chip_lock);
    return -EINVAL;
  }

  /* Grab the lock and see if the device is available */
 retry:
  spin_lock_bh (&this->chip_lock);
  switch (this->state) {
  case SF_READY:
    this->state = SF_READI;
    spin_unlock_bh (&this->chip_lock);
    break;

  case SF_ERASI:
    this->state = SF_READI;
    erase_state = 1;
    spin_unlock_bh (&this->chip_lock);
    break;

  default:
    set_current_state (TASK_UNINTERRUPTIBLE);
    add_wait_queue (&this->wq, &wait);
    spin_unlock_bh (&this->chip_lock);
    schedule();
    remove_wait_queue (&this->wq, &wait);
    goto retry;
  };

  /* Initialize return value */
  *retlen = 0;

  if (sflash_exists) {
    stwsf_read_enable_sequence();
  }
  else {
    printk(KERN_ERR "sflash_read - serial flash NOT initialized.\n");
    spin_lock_bh (&this->chip_lock);
    this->state = SF_READY;
    wake_up (&this->wq);
    spin_unlock_bh (&this->chip_lock);
    return -EINVAL;
  }

  /* Loop until all data read */
  while (*retlen < len) {
    /* Read the data directly into the return buffer */ 
    buf[(*retlen)++] = GETMEM_SFLASH_8((uint32_t)from+(*retlen));
  }

  /* Wake up anyone waiting on the device */
  spin_lock_bh (&this->chip_lock);
  if (erase_state)
    this->state = SF_ERASI;
  else
    this->state = SF_READY;
  wake_up (&this->wq);
  spin_unlock_bh (&this->chip_lock);

  /* Return OK */
  return 0;

}

static int sflash_write( struct mtd_info *mtd,
			 loff_t to,
			 size_t len,
			 size_t *retlen,
			 const u_char *buf )
{


  int       i, cnt, sflash_blk_size;
  uint32_t  regVal,sf_page_size,offset;
  
  struct sflash_chip *this = mtd->priv;

  DECLARE_WAITQUEUE(wait, current);

  DEBUG (MTD_DEBUG_LEVEL3,
	 "sflash_write: to = 0x%08x, len = 0x%x\n", 
	 (unsigned int) to, (int) len);

  /* Do not allow write past end of page */
  if ((to + len) > mtd->size) {
    DEBUG (MTD_DEBUG_LEVEL0,
	   "sflash_write: Attempted write past end of device\n");
    return -EINVAL;
  }

 retry:
  /* Grab the lock and see if the device is available */
  spin_lock_bh (&this->chip_lock);
  switch (this->state) {
  case SF_READY:
    this->state = SF_WRITI;
    spin_unlock_bh (&this->chip_lock);
    break;

  default:
    set_current_state (TASK_UNINTERRUPTIBLE);
    add_wait_queue (&this->wq, &wait);
    spin_unlock_bh (&this->chip_lock);
    schedule();

    remove_wait_queue (&this->wq, &wait);
    goto retry;
  };

  sf_page_size = (1<<this->page_shift);

  /* Initialize return length value */
  *retlen = 0;

  /* 
     DEBUG( MTD_DEBUG_LEVEL3, 
     "sflash_write - page_size: %x\n", sf_page_size );
  */

  if(sflash_exists) {
    sflash_blk_size = stwsf_get_block_size();
  }
  else {
    printk(KERN_ERR "sflash_write - serial flash NOT initialized.\n");
    spin_lock_bh (&this->chip_lock);
    this->state = SF_READY;
    wake_up (&this->wq);
    spin_unlock_bh (&this->chip_lock);
    return -EIO;
  }

  /* Loop until all data is written */
  while (*retlen < len) {
    /* Write data into buffer */
    for(cnt=0 ; cnt < sf_page_size ; cnt++) {
      this->data_buf[cnt] = buf[(*retlen + cnt)];
    }

    /* Write out a page of data */
    for (offset=0; offset < sf_page_size; offset+=SFLASH_BURST_SIZE) {
      /* DEBUG(MTD_DEBUG_LEVEL3, "offset: 0x%x\n", offset); */
      /* pre-write sequence */
      regVal = GETREG_REGMM32(SFLASH_CNTL1);
      MODIFYFLD(regVal,SFLASH_CNTL1,WRITE_ENABLE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,BCNT_OVER_WTE_EN, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,READ_STATUS, 0x0);
      MODIFYFLD(regVal,SFLASH_CNTL1,BYTE_CNT, 0x0);
      MODIFYFLD(regVal,SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,SCK_PRESCALE, 0x2);
      SETREG_REGMM32(SFLASH_CNTL1,regVal);

      SETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEC_COMMAND, 0x6);
      SETMEM_SFLASH_8(0x0, 0x0);

      if(stwsf_check_status_register(100)) return -1;
      //if(stwsf_check_idle_soft_reset(1000)) return -1;

      /* set up the byte count */
      regVal=GETREG_REGMM32(SFLASH_CNTL1);
      MODIFYFLD(regVal,SFLASH_CNTL1, WRITE_ENABLE, 0x0);
      MODIFYFLD(regVal,SFLASH_CNTL1, BCNT_OVER_WTE_EN, 0x1);
      /* 0x0 for 1 byte */
      MODIFYFLD(regVal,SFLASH_CNTL1, BYTE_CNT, SFLASH_BURST_SIZE-1); 
      SETREG_REGMM32(SFLASH_CNTL1,regVal);

      /* Write Enable Finished */
      SETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEC_COMMAND, 0x0); 

      for(i=0; i<SFLASH_BURST_SIZE; i++)
	SETMEM_SFLASH_8((uint32_t)to+(*retlen)+offset+i, this->data_buf[offset+i]);

      if(stwsf_check_write_in_progress(100)) return -1;
      if(stwsf_check_status_register(100)) return -1;
      //if(stwsf_check_idle_soft_reset(1000)) return -1;
    }
    
    /* heart beat... */
    printk(".");

    /* 
       DEBUG ( MTD_DEBUG_LEVEL3, 
       "sflash__write - *retlen=0x%x\n", *retlen ); 
    */
  
    /* Update written bytes count */
    (*retlen) = (*retlen) + sf_page_size;

  } /* end of while loop */

  /* after writing re-enable the read sequence */
  stwsf_read_enable_sequence();

  /* Wake up anyone waiting on the device */
  spin_lock_bh (&this->chip_lock);
  this->state = SF_READY;
  wake_up (&this->wq);
  spin_unlock_bh (&this->chip_lock);

  /* Return OK */
  *retlen = len;
  return 0;

}


static void sflash_sync( struct mtd_info *mtd )
{

  struct sflash_chip *this = mtd->priv;
  DECLARE_WAITQUEUE(wait, current);

  DEBUG (MTD_DEBUG_LEVEL3, "sflash_sync: called\n");

 retry:
  /* Grab the spinlock */
  spin_lock_bh(&this->chip_lock);

  /* See what's going on */
  switch(this->state) {
  case SF_READY:
  case SF_SYNCI:
    this->state = SF_SYNCI;
    spin_unlock_bh (&this->chip_lock);
    break;

  default:
    /* Not an idle state */
    add_wait_queue (&this->wq, &wait);
    spin_unlock_bh (&this->chip_lock);
    schedule ();

    remove_wait_queue (&this->wq, &wait);
    goto retry;
  }

  /* Lock the device */
  spin_lock_bh (&this->chip_lock);

  /* Set the device to be ready again */
  if (this->state == SF_SYNCI) {
    this->state = SF_READY;
    wake_up (&this->wq);
  }

  /* Unlock the device */
  spin_unlock_bh (&this->chip_lock);

  return;
}


/************************************************************************

  STW serial flash support routines.
 
*************************************************************************/


void stwsf_dump_chip_info( void )
{

  DEBUG( MTD_DEBUG_LEVEL3,
	 "STW serial flash chip info struct:\n");

}


void stwsf_dump_mtd_info (struct mtd_info *mtd)
{

  DEBUG( MTD_DEBUG_LEVEL3, "mtd_info struct:\n");
  DEBUG( MTD_DEBUG_LEVEL3, "type: 0x%x\n", mtd->type);
  DEBUG( MTD_DEBUG_LEVEL3, "flags: 0x%x\n", mtd->flags);
  DEBUG( MTD_DEBUG_LEVEL3, "total size: 0x%x\n", mtd->size);
  DEBUG( MTD_DEBUG_LEVEL3, "erasesize: 0x%x\n", mtd->erasesize);
  DEBUG( MTD_DEBUG_LEVEL3, "index: 0x%x\n", mtd->index);

}


/************************************************************************

 STW serial flash driver low-level device routines.

*************************************************************************/




static int stwsf_get_total_blocks(void)
{
    switch(SFLASH_GET_DEVID())
    {
    case SFLASH_DEVID_M25P10:  return 4;
    case SFLASH_DEVID_M25P05:  return 2;
    case SFLASH_DEVID_NX25F011:  return 16;
    case SFLASH_DEVID_AT25F1024:  return 4;
    }

    return 0;
}


static int stwsf_get_chip_size( void )
{
  switch(SFLASH_GET_DEVID()) 
    {
    case SFLASH_DEVID_M25P10:  return 128*1024;
    case SFLASH_DEVID_M25P05:  return 64*1024;
    case SFLASH_DEVID_NX25F011:  return 132*1024;
    case SFLASH_DEVID_AT25F1024:  return 128*1024;
    }

  return 0;
    
}


static int stwsf_get_block_size( void )
{

  switch(SFLASH_GET_DEVID()) 
    {
    case SFLASH_DEVID_M25P10:  return 0x8000;
    case SFLASH_DEVID_M25P05:  return 0x8000;
    case SFLASH_DEVID_NX25F011:  return 0x2100;
    case SFLASH_DEVID_AT25F1024:  return 0x8000;
    }
  
  return 0;

}


static int stwsf_get_block_offset(uint32_t blk_num)
{
    switch(SFLASH_GET_DEVID())
    {
    case SFLASH_DEVID_M25P10:
        switch(blk_num)
        {
        case 0: return 0x00000;
        case 1: return 0x08000;
        case 2: return 0x10000;
        case 3: return 0x18000;
        }
        break;

    case SFLASH_DEVID_M25P05:
        switch(blk_num)
        {
        case 0: return 0x00000;
        case 1: return 0x08000;
        }
        break;

    case SFLASH_DEVID_NX25F011:
      /* not support yet */
      break;

    case SFLASH_DEVID_AT25F1024:
      /* not support yet */
      break;
    }

    return 0;
}

static int stwsf_block_erase_verify( uint32_t blk_num )
{
  uint32_t    offset;
  uint32_t    blocksize;
  uint32_t    blockoffset;
  uint8_t     actual;

  if(sflash_exists) {
    blocksize = stwsf_get_block_size();
    blockoffset = stwsf_get_block_offset(blk_num);

    stwsf_read_enable_sequence();

    for(offset=0; offset<blocksize; offset++)
      {
	actual = GETMEM_SFLASH_8(blockoffset + offset);

	if(actual != 0xFF) {
	  printk(KERN_INFO 
		 "erase_verify: block=%d, offset=%d, actual=0x%02X(0xFF)\n", 
		 blk_num, offset, actual);
	  return -1;
	}
      }

    return 0;
  }
  else {
    printk(KERN_ERR "block_erase_verify: flash NOT initialized\n");
  }

  return -1;
}


static int stwsf_chip_probe( struct mtd_info *mtd )
{

  uint32_t chip_size, sector_size;
  int      i;
  struct sflash_chip *this = mtd->priv;

  /* get serial flash chip size and sector size */
  chip_size = stwsf_get_chip_size();
  sector_size = stwsf_get_block_size();

  /* Print and store flash device information */
  for (i = 0; sflash_ids[i].name != NULL; i++) {
    if (!mtd->size) {
      mtd->name = sflash_ids[i].name;
      mtd->erasesize = sflash_ids[i].sectorsize;
      mtd->size = (1 << sflash_ids[i].chipshift);
      this->chip_shift = sflash_ids[i].chipshift;
      this->page_shift = sflash_ids[i].pageshift;
    }
    DEBUG (MTD_DEBUG_LEVEL3, 
	   "Serial flash device: (%s) Chip size: 0x%x(0x%x) blk size: 0x%x\n",
	   mtd->name, mtd->size, chip_size, sector_size);
  }
  
  /* Initialize state and spinlock */
  this->state = SF_READY;
  init_waitqueue_head(&this->wq);
  spin_lock_init(&this->chip_lock);

  /* Print warning message for no device */
  if (!mtd->size) {
    printk (KERN_WARNING "No serial flash device was found!!!\n");
    return 1;
  }

  /* Fill in remaining MTD driver data */
  mtd->type = MTD_SFLASH;
  mtd->flags = MTD_CAP_SFLASH;
  mtd->owner = THIS_MODULE;
  mtd->erase = sflash_erase;
  mtd->sync = sflash_sync;
  mtd->read = sflash_read;
  mtd->write = sflash_write;
  mtd->point = NULL;
  mtd->unpoint = NULL;
  mtd->lock = NULL;
  mtd->unlock = NULL;
  mtd->suspend = NULL;
  mtd->resume = NULL;

  /* OK */
  return 0;

}

/* turn on STW serial flash aperture */
static int stwsf_chip_init(void)
{
  uint32_t r;

  /* turn off pflash aperture */
  SETFLD_REGMM32(APER_PCU_PFLASH_CNTL, APERSIZE, 0);

  /* disable pflash registers */
  r = GETREG_REGMM32(PFLASH_CNTL);
  MODIFYFLD(r, PFLASH_CNTL, PF32MBIT_EN, 0);
  MODIFYFLD(r, PFLASH_CNTL, PF64MBIT_EN, 0);
  MODIFYFLD(r, PFLASH_CNTL, PF128MBIT_EN, 0); 
  SETREG_REGMM32(PFLASH_CNTL, r); 

  /* set flexbus muxing to sflash */
  SETREG_REGMM32(FBUS_SELECT, 0x4);

  /* set GPIO10 (select sflash) */
  r = GETREG_REGMM32(GPIO_SEL);
  r &= ~((1<<GPIOSEL__1394_TRANSPORT)|(1<<GPIOSEL__DEBUG_BUS_A));
  r |= (1<<GPIOSEL__CLK_DEBUG_OUT);
  SETREG_REGMM32(GPIO_SEL, r);

  r = GETREG_REGMM32(GPIOA_DIR);
  r |= (1<<10);
  SETREG_REGMM32(GPIOA_DIR, r);
  
  r = GETREG_REGMM32(GPIOA_MASK);
  r |= (1<<10);
  SETREG_REGMM32(GPIOA_MASK, r);

  r = GETREG_REGMM32(GPIOA_DATA);
  r |= (1<<10);
  SETREG_REGMM32(GPIOA_DATA, r);

  /* enable sflash registers for ST M25P10 */
  r = GETREG_REGMM32(SFLASH_CNTL1);
  MODIFYFLD(r, SFLASH_CNTL1, SEPST10_EN, 1);
  MODIFYFLD(r, SFLASH_CNTL1, SEPST05_EN, 0);
  MODIFYFLD(r, SFLASH_CNTL1, SEPISSI_EN, 0);
  MODIFYFLD(r, SFLASH_CNTL1, SEPATMEL_EN, 0);
  SETREG_REGMM32(SFLASH_CNTL1, r); 

  /* turn on sflash aperture */
  SETFLD_REGMM32(APER_PCU_SFLASH_CNTL, APERSIZE, 0xB);

  stwsf_check_idle_soft_reset(1000);
  stwsf_read_enable_sequence();

  sflash_exists = 1;

  return sflash_exists;

}


static int stwsf_chip_erase(void)
{
  uint32_t regVal;

  if(sflash_exists)
    {
      regVal = GETREG_REGMM32(SFLASH_CNTL1);
      MODIFYFLD(regVal,SFLASH_CNTL1,WRITE_ENABLE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,BCNT_OVER_WTE_EN, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,BYTE_CNT, 0x0);
      MODIFYFLD(regVal,SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,SCK_PRESCALE, 0x2);
      SETREG_REGMM32(SFLASH_CNTL1,regVal);

      SETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEC_COMMAND, 0x6);
      SETMEM_SFLASH_8(0x0,0x0);

      if(stwsf_check_idle_soft_reset(1000)) return -1;

      regVal = GETREG_REGMM32(SFLASH_CNTL1);
      MODIFYFLD(regVal,SFLASH_CNTL1,WRITE_ENABLE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,BYTE_CNT, 0x0);
      MODIFYFLD(regVal,SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,SCK_PRESCALE, 0x2);
      SETREG_REGMM32(SFLASH_CNTL1,regVal);

      SETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEC_COMMAND, 0xC7); //chip erase command
      SETMEM_SFLASH_8(0x0,0x0); //command trigger

      if(stwsf_check_idle_soft_reset(1000)) return -1;

      regVal=GETREG_REGMM32(SFLASH_CNTL1);
      MODIFYFLD(regVal,SFLASH_CNTL1,WRITE_ENABLE, 0x0);
      MODIFYFLD(regVal,SFLASH_CNTL1,BCNT_OVER_WTE_EN, 1);
      MODIFYFLD(regVal,SFLASH_CNTL1,BYTE_CNT, 0x3);
      MODIFYFLD(regVal,SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,SCK_PRESCALE, 0x2);
      SETREG_REGMM32(SFLASH_CNTL1,regVal);

      SETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEC_COMMAND, 0x0);

      regVal = GETREG_REGMM32(SFLASH_CNTL1);
      MODIFYFLD(regVal,SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x1);
      MODIFYFLD(regVal,SFLASH_CNTL1,SCK_PRESCALE, 0x1);
      SETREG_REGMM32(SFLASH_CNTL1,regVal);
      SETFLD_REGMM32(SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE,0x0);

      if(stwsf_check_write_in_progress(4000)) return -1;
      if(stwsf_check_status_register(1000))  return -1;

      return 0;
    }
  else
    {
      printk(KERN_ERR "chip_erase: flash NOT initialized\n");
    }

  return -1;
}


void stwsf_read_enable_sequence(void)
{
  uint32_t val;

  val = GETREG_REGMM32(SFLASH_CNTL1);
  MODIFYFLD(val,SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x1);
  MODIFYFLD(val,SFLASH_CNTL1,SCK_PRESCALE, 0x1);
  SETREG_REGMM32(SFLASH_CNTL1,val);

  SETFLD_REGMM32(SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x0);
}


static int stwsf_check_idle_soft_reset(uint32_t timeout)
{
    return 0;
}


static int stwsf_check_write_in_progress(uint32_t timeout)
{
  uint32_t reg;

  if(sflash_exists) {
    timeout *= 1000;

    while(1) {
      reg = GETREG_REGMM32(SFLASH_CNTL1);
      MODIFYFLD(reg,SFLASH_CNTL1,WRITE_ENABLE, 0x0);
      MODIFYFLD(reg,SFLASH_CNTL1,BCNT_OVER_WTE_EN, 0x1);
      MODIFYFLD(reg,SFLASH_CNTL1,READ_STATUS, 0x1);
      MODIFYFLD(reg,SFLASH_CNTL1,BYTE_CNT, 0x0);
      MODIFYFLD(reg,SFLASH_CNTL1,WTRIG_PRE_FLD_SCK_PRESCALE, 0x1);
      MODIFYFLD(reg,SFLASH_CNTL1,SCK_PRESCALE, 0x2);
      SETREG_REGMM32(SFLASH_CNTL1,reg);

      SETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEC_COMMAND, 0x5);
      reg = GETMEM_SFLASH_8(0x0);

      SETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEC_COMMAND, 0x0);
      SETFLD_REGMM32(SFLASH_CNTL1,READ_STATUS,0x0);

      if(!(reg & 0x1)) {
	return 0;
      }

      if(timeout-- == 0) {
	printk(KERN_ERR "check_write_in_progress timeout\n");
	return -1;
      }

      udelay(1000);
    }
  }
  else {
    printk(KERN_ERR "check_write_in_progress: serial flash NOT initialized\n");
  }

  return 0;
}

/************************************************************************/


static int stwsf_check_status_register(uint32_t timeout)
{
  if(sflash_exists) {
    timeout *= 1000;

    while (1) {
      if((GETFLD_REGMM32(SFLASH_CNTL2_STATUS,SFLASH_BUSY) |
	  GETFLD_REGMM32(SFLASH_CNTL2_STATUS,ROMPARIF_BUSY) |
	  GETFLD_REGMM32(SFLASH_CNTL2_STATUS,PARIF_BUSY) |
	  GETFLD_REGMM32(SFLASH_CNTL2_STATUS,PARIFROM_BUSY) |
	  GETFLD_REGMM32(SFLASH_CNTL2_STATUS,READY_BUSY) |
	  GETFLD_REGMM32(SFLASH_CNTL2_STATUS,SEPROM_BUSY)) == 0) {
	return 0;
      }

      if (timeout-- == 0) {
	printk(KERN_ERR "check_status_register timeout\n");
	return -1;
      }

      udelay(1000);
    }
  }
  else {
    printk(KERN_ERR "check_status_register: serial flash NOT initialized\n");
  }

  return -1;
}


/************************************************************************

  PLL access routines ported from MMON code									 
 ***********************************************************************/


uint32_t stw_pll_read32(uint32_t offset)
{
  SETREG_REGMM32(CLOCK_CNTL_INDEX, offset);
  return GETREG_REGMM32(CLOCK_CNTL_DATA);
}


void stw_pll_write32(uint32_t offset, uint32_t data)
{
  SETREG_REGMM32(CLOCK_CNTL_INDEX, offset);
  SETREG_REGMM32(CLOCK_CNTL_DATA, data);
}


/************************************************************************

 Linux module initialization routine

*************************************************************************/


static int __init sflash_init( void )
{

  struct sflash_chip  *this;

  printk(KERN_INFO "STW Serial Flash initializing...\n");
  
  /* allocate memory for MTD device struct and private date */
  stwsf_mtd = kmalloc( sizeof(struct mtd_info) +
		       sizeof(struct sflash_chip), GFP_KERNEL);
  if (!stwsf_mtd) {
    printk("Unable to allocate memory for MTD Serial Flash Device.\n");
    return -ENOMEM;
  }

  /* get pointer to chip private data */
  this = (struct sflash_chip *)(&stwsf_mtd[1]);

  /* zeroing the structs */
  memset((char *)stwsf_mtd, 0, sizeof(struct mtd_info));
  memset((char *)this, 0, sizeof(struct sflash_chip));
  stwsf_mtd->priv = this;

  /* turn on serial flash aperture */
  stwsf_chip_init(); 

  /* scan the serial flash device and set parameters */
  if (stwsf_chip_probe(stwsf_mtd)) {
    kfree(stwsf_mtd);
    return -ENXIO;
  }

  /* dump debug information */
  stwsf_dump_chip_info();
  stwsf_dump_mtd_info(stwsf_mtd);

  /* allocate memory for internal data buffer */
  this->data_buf = kmalloc (sizeof(u_char) * 
			    (stwsf_mtd->oobblock + stwsf_mtd->oobsize), 
			    GFP_KERNEL);
  if (!this->data_buf) {
    printk ("Unable to allocate serial flash data buffer.\n");
    kfree (stwsf_mtd);
    return -ENOMEM;
  }

  /* Register the partitions */
  add_mtd_partitions(stwsf_mtd, 
		     (struct mtd_partition*)stwsf_par_info, 
		     (int)NUM_SF_PARTITIONS);

  printk(KERN_INFO "ATI STW Serial Flash Device initialization done.\n");	

  return 0;
  
}
module_init(sflash_init);


/************************************************************************

 Linux module clean-up/exit routine

************************************************************************/


#ifdef MODULE
static void __exit sflash_cleanup( void )
{

  uint32_t r;
  struct sflash_chip *this = (struct sflash_chip *) &stwsf_mtd[1];
 
  /* return control to NAND flash */
  /* &&&&&&&&&&&&&&&&&&&&&&&&&&&& */

  /* turn off sflash aperture */
  SETFLD_REGMM32(APER_PCU_SFLASH_CNTL, APERSIZE, 0);

  /* disable sflash registers */
  r = GETREG_REGMM32(SFLASH_CNTL1);
  MODIFYFLD(r, SFLASH_CNTL1, SEPST10_EN, 0);
  MODIFYFLD(r, SFLASH_CNTL1, SEPST05_EN, 0);
  MODIFYFLD(r, SFLASH_CNTL1, SEPISSI_EN, 0);
  MODIFYFLD(r, SFLASH_CNTL1, SEPATMEL_EN, 0);
  SETREG_REGMM32(SFLASH_CNTL1, r); 

  /* set flexbus muxing to pflash */
  SETREG_REGMM32(FBUS_SELECT, 0x8);

  /* enable external synchronizer (required for A11) */
  r = GETREG_REGMM32(GPIO_SEL);
  r &= ~((1<<GPIOSEL__1394_TRANSPORT)|(1<<GPIOSEL__DEBUG_BUS_A));
  r |= (1<<GPIOSEL__CLK_DEBUG_OUT);
  SETREG_REGMM32(GPIO_SEL, r);

  r = GETREG_REGMM32(TEST_DEBUG_MUX);
  MODIFYFLD(r, TEST_DEBUG_MUX, TEST_DEBUG_CLK_INV, 1);
  MODIFYFLD(r, TEST_DEBUG_MUX, TEST_DEBUG_CLK, 0xF);
  SETREG_REGMM32(TEST_DEBUG_MUX, r);

  /* clear GPIO10 (select pflash) and set GPIO11 (NAND_SYNC_CLK) */
  r = GETREG_REGMM32(GPIOA_DIR);
  r |= (1<<10)|(1<<11);
  SETREG_REGMM32(GPIOA_DIR, r);

  r = GETREG_REGMM32(GPIOA_MASK);
  r |= (1<<10)|(1<<11);
  SETREG_REGMM32(GPIOA_MASK, r);

  r = GETREG_REGMM32(GPIOA_DATA);
  r &= ~(1<<10);
  r |= (1<<11);
  SETREG_REGMM32(GPIOA_DATA, r);

  /* set clock to divide by 4 */
  SETFLD_PLL32(PLL_TEST_CNTL, TST_DIV_SEL, 1);

  /* enable pflash registers for 32Mb chip */
  r = GETREG_REGMM32(PFLASH_CNTL);
  MODIFYFLD(r, PFLASH_CNTL, PF32MBIT_EN, 1);
  MODIFYFLD(r, PFLASH_CNTL, PF64MBIT_EN, 0);
  MODIFYFLD(r, PFLASH_CNTL, PF128MBIT_EN, 0); 
  SETREG_REGMM32(PFLASH_CNTL, r); 

  /* turn on pflash aperture */
  SETFLD_REGMM32(APER_PCU_PFLASH_CNTL, APERSIZE, 0xB);

  /* &&&&&&&&&&&&&&&&&&&&&&&&&&&& */

  /* Unregister the device */
  del_mtd_device (stwsf_mtd);
  del_mtd_partitions(stwsf_mtd);

  /* Free internal data buffer */
  kfree (this->data_buf);

  /* Free the MTD device structure */
  kfree (stwsf_mtd);

  printk(KERN_INFO "Exit...STW Serial Flash driver module clean-up done.\n");

}
module_exit(sflash_cleanup);
#endif


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chin Zhou, czhou@ati.com");
MODULE_DESCRIPTION("ATI STW Serial Flash Device Interface");
