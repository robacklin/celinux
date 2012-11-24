/*
 *
 * FILE NAME ti1500_fpga0.h
 *
 * BRIEF MODULE DESCRIPTION
 *  FPGA0 Driver for LCD, Buttons, GPIO and IRQ control for the XXS_1500 board
 *
 * Copyright 2003 mycable GmbH
 * Author: mycable GmbH
 *         linux@mycable.de
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Notes:
 *
 *  Revision history
 *  Revision History:
 *  -----------------
 *  18.06.2003  TK  0.1   Initial release
 *  15.07.2003  TK  0.2   Update to FPGA 0.51
 *  21.07.2003  TK  0.21  Update to FPGA 0.52
 *  28.07.2003  TK  0.53.1  Update to FPGA 0.53  New version numbers
 */

//Device and Vendor ID for FPGA 0.50
#define AMM_VENDOR_ID_050 0x1234
#define AMM_DEVICE_ID_050 0x5678

//FPGA 0.51 never really existed

//Device and Vendor ID for FPGA 0.52
#define AMM_VENDOR_ID_052 0x1755
#define AMM_DEVICE_ID_052 0x0100

#include <linux/ioctl.h>

#define DEV_MAJOR 12
#define TI1500_FUNC0_VERSION 0.21

/*Register Settings*/
#define UI_OFFSET      0x08
#define INT_OFFSET     0x20
#define GPIO_OFFSET    0x40
#define GPIOENA_OFFSET 0x50

#define LCD_CTRL 0
#define LCD_DATA 1
#define PB_DATA  2

#define INT_LATCH 0
#define INT_MASK  1
#define INT_MRES  2
#define INT_MSET  3

#define GPIO_IN   0
#define GPIO_O    1
#define GPIO_ORES 2
#define GPIO_OSET 3

#define GPIO_EN    0
#define GPIO_ENRES 1
#define GPIO_ENSET 2

/*Bitmasks*/
#define FPGA_MASK_LOW  0x0000003F
#define FPGA_MASK_HIGH 0x3F000000
#define GPS_MASK_LOW   0x00000FFF
#define GPS_MASK_HIGH  0x00FFF000
#define VMX_MASK_LOW   0x00000FFF
#define VMX_MASK_HIGH  0x00000FFF

/*Bitshifts*/
#define FPGA_SHIFT 24
#define GPS_SHIFT  12
#define VMX_SHIFT  0

/*Due to some strange error all values are set to 20*/
#define UI_LENGTH      20
#define INT_LENGTH     20
#define GPIO_LENGTH    20
#define GPIOENA_LENGTH 20


/*Start of IOCTL Numbers*/
#define TI1500_IOC_MAGIC  'K'

/*LCD control*/
#define TI1500_LCD_CLEAR    _IO(TI1500_IOC_MAGIC,  0)
#define TI1500_LCD_GOTOXY   _IO(TI1500_IOC_MAGIC,  1)

/*Pushbutton control*/
#define TI1500_KEY_READ     _IO(TI1500_IOC_MAGIC,  2)

/*IRQ control*/
#define TI1500_IRQ_LATCH    _IO(TI1500_IOC_MAGIC,  3)
#define TI1500_IRQ_MASKR    _IO(TI1500_IOC_MAGIC,  4)
#define TI1500_IRQ_MASKW    _IO(TI1500_IOC_MAGIC,  5)
#define TI1500_IRQ_MRES     _IO(TI1500_IOC_MAGIC,  6)
#define TI1500_IRQ_MSET     _IO(TI1500_IOC_MAGIC,  7)

/*FPGA GPIO control*/
/*These ioctl act on bits 24-29 of the GPIO control Registers*/
#define TI1500_FPGA_INPUT   _IO(TI1500_IOC_MAGIC,  8)
#define TI1500_FPGA_OUTR    _IO(TI1500_IOC_MAGIC,  9)
#define TI1500_FPGA_OUTW    _IO(TI1500_IOC_MAGIC, 10)
#define TI1500_FPGA_OUTRES  _IO(TI1500_IOC_MAGIC, 11)
#define TI1500_FPGA_OUTSET  _IO(TI1500_IOC_MAGIC, 12)
#define TI1500_FPGA_ENAR    _IO(TI1500_IOC_MAGIC, 13)
#define TI1500_FPGA_ENAW    _IO(TI1500_IOC_MAGIC, 14)
#define TI1500_FPGA_ENARES  _IO(TI1500_IOC_MAGIC, 15)
#define TI1500_FPGA_ENASET  _IO(TI1500_IOC_MAGIC, 16)

/*GPS GPIO control*/
/*These ioctl act on bits 12-23 of the GPIO control Registers*/
#define TI1500_GPS_INPUT   _IO(TI1500_IOC_MAGIC, 17)
#define TI1500_GPS_OUTR    _IO(TI1500_IOC_MAGIC, 18)
#define TI1500_GPS_OUTW    _IO(TI1500_IOC_MAGIC, 19)
#define TI1500_GPS_OUTRES  _IO(TI1500_IOC_MAGIC, 20)
#define TI1500_GPS_OUTSET  _IO(TI1500_IOC_MAGIC, 21)
#define TI1500_GPS_ENAR    _IO(TI1500_IOC_MAGIC, 22)
#define TI1500_GPS_ENAW    _IO(TI1500_IOC_MAGIC, 23)
#define TI1500_GPS_ENARES  _IO(TI1500_IOC_MAGIC, 24)
#define TI1500_GPS_ENASET  _IO(TI1500_IOC_MAGIC, 25)

/*VMX GPIO control*/
/*These ioctl act on bits 0-11 of the GPIO control Registers*/
#define TI1500_VMX_INPUT   _IO(TI1500_IOC_MAGIC, 26)
#define TI1500_VMX_OUTR    _IO(TI1500_IOC_MAGIC, 27)
#define TI1500_VMX_OUTW    _IO(TI1500_IOC_MAGIC, 28)
#define TI1500_VMX_OUTRES  _IO(TI1500_IOC_MAGIC, 29)
#define TI1500_VMX_OUTSET  _IO(TI1500_IOC_MAGIC, 30)
#define TI1500_VMX_ENAR    _IO(TI1500_IOC_MAGIC, 31)
#define TI1500_VMX_ENAW    _IO(TI1500_IOC_MAGIC, 32)
#define TI1500_VMX_ENARES  _IO(TI1500_IOC_MAGIC, 33)
#define TI1500_VMX_ENASET  _IO(TI1500_IOC_MAGIC, 34)

#define TI1500_IOC_MAXNR 34
/*End of IOCTL Numbers*/

static char *options;
MODULE_PARM(options, "s");

/*-------------------------------------------------------------------------*/
/*  module id                                                              */
/*-------------------------------------------------------------------------*/

#define _AMM_PCI_FUNC0


/*-------------------------------------------------------------------------*/
/*  defines                                                                */
/*-------------------------------------------------------------------------*/
#define ALCD_CMD_WRITE(cmd)   writel(cmd, amm_alcd_base);  mdelay(20)
#define ALCD_DATA_WRITE(data) writel(data, amm_alcd_base + 0x04); mdelay(20)

#define ALCD_BUSY_DELAY       20


/*-------------------------------------------------------------------------*/
/*  module global variables                                                */
/*-------------------------------------------------------------------------*/

static const char _AMM_PB_revision[] = "0.02.010";
static const char _AMM_ALCD_revision[] = "0.02.010";
static const char   *amm_pb_revision;
static const char *amm_alcd_revision;
static uint8_t     amm_alcd_status;                  /* display and cursor settings */
uint32_t     amm_alcd_base;
int rowmarker=0, colmarker=0;

static uint32_t amm_pb_base;
static uint32_t amm_gpio_address;

unsigned long * amm_pci_mem_ui_remap ;
unsigned long * amm_pci_mem_irq_remap;
unsigned long * amm_pci_mem_gpio_remap;
unsigned long * amm_pci_mem_gpioena_remap;

int amm_fpga_ver = 0;

void * ti1500_func0_devices;
