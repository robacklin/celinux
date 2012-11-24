#ifndef STW_SFLASH_H
#define STW_SFLASH_H

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
   Device driver for M25PXX from ST Microelectronics 
*/
/************************************************************************/


/* these should move to mtd.h later */
#define MTD_CAP_SFLASH       (MTD_CLEAR_BITS|MTD_ERASEABLE)
#define MTD_SFLASH           6

/* STW serial flash device ID */ 
#define SFLASH_DEVID_M25P10             0x8
#define SFLASH_DEVID_M25P05             0x9
#define SFLASH_DEVID_NX25F011           0xB
#define SFLASH_DEVID_AT25F1024          0xC

#define SFLASH_M25P10_PAGE_SHIFT        0x7

#define SFLASH_GET_DEVID() ((GETREG_REGMM32(STRAPS_VALUE) & 0x00001E00) >> 9)


/* GPIOSEL macros */

#define GPIOSEL__1394_TRANSPORT                     0
#define GPIOSEL__NA_TRANSPORT                       1
#define GPIOSEL__I2S_B_OUT                          2 
#define GPIOSEL__I2S_A_OUT                          3 
#define GPIOSEL__SPDIF_A_B_OUT                      4 
#define GPIOSEL__DVI_OUT                            5 
#define GPIOSEL__OUT_OF_BAND                        6 
#define GPIOSEL__HOST_VIP_INTERRUPT                 7 
#define GPIOSEL__ITU656_IN_PORT_B                   8 
#define GPIOSEL__ITU656_OUT_PORT_C                  9 
#define GPIOSEL__CRT                                10
#define GPIOSEL__NA_EJTAG                           11
#define GPIOSEL__DEBUG_BUS_A                        12
#define GPIOSEL__DEBUG_BUS_B                        13
#define GPIOSEL__SERIAL_PORT_A_1                    14
#define GPIOSEL__CLK_DEBUG_OUT                      15
#define GPIOSEL__DREQBB_GPIO6                       16
#define GPIOSEL__DVS_DATA_PCI                       17
#define GPIOSEL__SC_A_NRSS_DATA__FB_INTERRUPTS_PCI  18
#define GPIOSEL__IDE                                21
#define GPIOSEL__IR                                 22
#define GPIOSEL__SERIAL_PORT_A_2                    23
#define GPIOSEL__SC_B__FB_INTERRUPTS                24
#define GPIOSEL__SC_A__FB_INTERRUPTS                25
#define GPIOSEL__ITU656_IN_PORT_A                   26


typedef enum {
  SF_READY,
  SF_READI,
  SF_WRITI,
  SF_ERASI,
  SF_SYNCI
} sf_state_t;

struct sflash_chip {
  spinlock_t chip_lock;
  wait_queue_head_t wq;
  sf_state_t state;
  unsigned int chip_shift;
  unsigned int page_shift;
  u_char *data_buf;
};

struct sflash_dev {
  char *name;
  int device_id;
  int chipshift;
  int pageshift;
  int pagesize;
  unsigned long sectorsize;
};

static struct sflash_dev sflash_ids[] = {
  {"ST M25P10", SFLASH_DEVID_M25P10, 17, 0x7, 0x80, 0x8000},
  {NULL,}
};


/* some MACROS for PLL access */

void     stw_pll_write32(uint32_t offset, uint32_t data);
uint32_t stw_pll_read32(uint32_t offset);

#define STW_PLL_WRITE_REG32(offset,data)   stw_pll_write32(offset,data)
#define STW_PLL_READ_REG32(offset)         stw_pll_read32(offset)

#define SETREG_PLL32(reg,value)            STW_PLL_WRITE_REG32(ix##reg,value)
#define GETREG_PLL32(reg)                  STW_PLL_READ_REG32(ix##reg)

#define SETFLD_PLL32(reg, field, val)      (SETREG_PLL32(reg, (GETREG_PLL32(reg) & ~reg##__##field##__MASK) | VAL2FLD(reg, field, val)))


/* prototypes of the STW serial flash driver */

static int   sflash_erase( struct mtd_info *mtd,
			  struct erase_info *instr );
static int   sflash_read( struct mtd_info *mtd,
			 loff_t from,
			 size_t len,
			 size_t *retlen,
			 u_char *buf );
static int   sflash_write( struct mtd_info *mtd,
			  loff_t to,
			  size_t len,
			  size_t *retlen,
			  const u_char *buf );
static void  sflash_sync( struct mtd_info *mtd );

static void  sflash_cleanup( void );
static int   sflash_init( void );


static int   stwsf_chip_init( void );
static int   stwsf_chip_probe( struct mtd_info *mtd );

static int   stwsf_check_write_in_progress( uint32_t timeout );
static int   stwsf_check_status_register( uint32_t timeout );

static int   stwsf_get_block_size( void );
static int   stwsf_get_chip_size( void );
static int   stwsf_get_block_offset( uint32_t blk_num );
static int   stwsf_get_total_blocks( void );
static int   stwsf_block_erase_verify( uint32_t blk_num );

static int   stwsf_chip_erase( void );

static void  stwsf_read_enable_sequence( void );
static int   stwsf_check_idle_soft_reset( uint32_t timeout );

#endif /* STW_SFLASH_H */




















