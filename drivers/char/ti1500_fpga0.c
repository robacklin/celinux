/*
 *
 * FILE NAME ti1500_fpga0.c
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


#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

/*-------------------------------------------------------------------------*/
/*compiler libraries */
/*-------------------------------------------------------------------------*/
#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/selection.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/nvram.h>
#include <linux/kd.h>
#include <linux/vt_kern.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/pagemap.h>

#include <asm/pgalloc.h>
#include <asm/uaccess.h>
#include <asm/tlb.h>

#ifdef CONFIG_MTRR
#include <asm/mtrr.h>
#endif

#include <linux/spinlock.h>
//#ifdef CONFIG_MIPS_AU1000
#include <asm/au1000.h>
//#endif

#include "ti1500_key.h"
#include "ti1500_lcd.h"
#include "ti1500_fpga0.h"


/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_init_alcd                                                            */
/*                                                                         */
/*action: Alphanumeric LCD initialization.                                 */
/*                                                                         */
/*passing param.: base_addressbase address of alphanumeric LCD             */
/*globalparam.: amm_alcd_revision initialized                              */
/*amm_alcd_base initialized                                                */
/*returnparam.: FR_NOERR                                                   */
/*                                                                         */
/*programmer: Michael Carstens-Behrens                                     */
/*                                                                         */
/*26.11.02MCB :build this function                                         */
/*11.05.03 TK :replaced delay() by mdelay() for Linux                      */
/*                                                                         */
/*-------------------------------------------------------------------------*/

int amm_init_alcd (uint32_t base_address)
{
	//PRINT_FUNCTIONNAME;

	amm_alcd_revision = _AMM_ALCD_revision;
	amm_alcd_base = base_address;

	ALCD_CMD_WRITE(ALCD_FUNCTION_SET);
	ALCD_CMD_WRITE(ALCD_CLEAR_DISPLAY);
	mdelay(ALCD_BUSY_DELAY);

	ALCD_CMD_WRITE(ALCD_CURSOR_HOME);
	mdelay(ALCD_BUSY_DELAY);

	amm_alcd_status = ALCD_DISPLAY_OFF | ALCD_CURSOR_OFF | ALCD_CBLINK_OFF;
	ALCD_CMD_WRITE(ALCD_DISPLAY_CONTROL | amm_alcd_status);

	return 0;
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_switch_alcd                                                          */
/*                                                                         */
/*action: Switch the alphanumeric display on or off.                       */
/*                                                                         */
/*passing param.: switch_on TRUE | FALSE                                   */
/*globalparam.: -none-                                                     */
/*returnparam.: -none-                                                     */
/*                                                                         */
/*programmer: Michael Carstens-Behrens                                     */
/*                                                                         */
/*26.11.02MCB : 1.00 build this function                                   */
/*                                                                         */
/*-------------------------------------------------------------------------*/


void amm_switch_alcd (int switch_on)
{
	if (switch_on){
		amm_alcd_status |= ALCD_DISPLAY_ON;
		ALCD_CMD_WRITE(ALCD_DISPLAY_CONTROL | amm_alcd_status);
	}
	else{
		amm_alcd_status &= ~ALCD_DISPLAY_ON;
		ALCD_CMD_WRITE(ALCD_DISPLAY_CONTROL | amm_alcd_status);
	}
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_clear_alcd                                                           */
/*                                                                         */
/*action: Clear the alphanumeric display at set the cursor to 0,0.         */
/*                                                                         */
/*passing param.: -none-                                                   */
/*globalparam.: -none-                                                     */
/*returnparam.: -none-                                                     */
/*                                                                         */
/*programmer: Michael Carstens-Behrens                                     */
/*                                                                         */
/*26.11.02MCB : 1.00 build this function                                   */
/*11.05.03 TK :replaced delay() by mdelay() for Linux                      */
/*                                                                         */
/*-------------------------------------------------------------------------*/
void amm_clear_alcd (void)
{
	ALCD_CMD_WRITE(ALCD_CLEAR_DISPLAY);
	mdelay(ALCD_BUSY_DELAY);
	ALCD_CMD_WRITE(ALCD_CURSOR_HOME);
	mdelay(ALCD_BUSY_DELAY);
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_alcd_gotoxy                                                          */
/*                                                                         */
/*action: Set the cursor position.                                         */
/*                                                                         */
/*passing param.: xcursor x position, 0 -15                                */
/*ycursor x position, 0 -1                                                 */
/*globalparam.: -none-                                                     */
/*returnparam.: -none-                                                     */
/*                                                                         */
/*programmer: Michael Carstens-Behrens                                     */
/*                                                                         */
/*26.11.02MCB : build this function                                        */
/*                                                                         */
/*-------------------------------------------------------------------------*/

void amm_alcd_gotoxy (uint8_t x, uint8_t y)
{
	uint8_t alcd_cmd;

	if ((x < 16) && (y < 2)){

		alcd_cmd = ALCD_DDRAM_SET + x;
		if (y) alcd_cmd += 0x40;
		ALCD_CMD_WRITE(alcd_cmd);
	}
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_alcd_putc                                                            */
/*                                                                         */
/*action: Alphanumeric LCD put character function.                         */
/*                                                                         */
/*passing param.: alcd_char character                                      */
/*globalparam.: -none-                                                     */
/*returnparam.: -none-                                                     */
/*                                                                         */
/*programmer: Michael Carstens-Behrens                                     */
/*                                                                         */
/*26.11.02MCB : build this function                                        */
/*                                                                         */
/*-------------------------------------------------------------------------*/
void amm_alcd_putc (uint8_t alcd_char)
{
	ALCD_DATA_WRITE(alcd_char);
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_alcd_print                                                           */
/*                                                                         */
/*action: Alphanumeric LCD print function.                                 */
/*                                                                         */
/*passing param.: msgptr to msg to print                                   */
/*globalparam.: -none-                                                     */
/*returnparam.: -none-                                                     */
/*                                                                         */
/*programmer: Michael Carstens-Behrens, Joerg Ritter                       */
/*                                                                         */
/*26.11.02MCB : build this function                                        */
/*05.03.03JR: return and new line ignored                                  */
/*                                                                         */
/*-------------------------------------------------------------------------*/
void amm_alcd_print (const char *msg)
{
	while(*msg != '\0'){
		if (*msg == '\r' || *msg == '\n'){
			msg++;
			continue;
		}
		amm_alcd_putc(*msg);
		msg ++;
	}
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_init_pb                                                              */
/*                                                                         */
/*action: Push button base address initialization.                         */
/*                                                                         */
/*passing param.: base_addressbase address of push button check            */
/*gpio_addressaddress og sys_pinstaterd                                    */
/*globalparam.: amm_pb_base initialized                                    */
/*amm_gpio_addressinitialized                                              */
/*amm_pb_revision initialized                                              */
/*returnparam.: 0                                                          */
/*                                                                         */
/*programmer: Michael Carstens-Behrens                                     */
/*                                                                         */
/*05.03.03JR: gpio address added                                           */
/*12.05.03TK: On-Button init removed for Linux                             */
/*                                                                         */
/*-------------------------------------------------------------------------*/
int amm_init_pb (uint32_t base_address)
{
	amm_pb_revision= _AMM_PB_revision;
	amm_pb_base= base_address;
        if (amm_fpga_ver == 52 )AMM_PB_STATUS = 0x7F;
	return 0;
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_check_pb                                                             */
/*                                                                         */
/*action: Checks all pushbuttons at the same time.                         */
/*                                                                         */
/*passing param.: -none-                                                   */
/*globalparam.: -none-                                                     */
/*returnparam.: pressed buttons                                            */
/*                                                                         */
/*programmer: Michael Carstens-Behrens                                     */
/*                                                                         */
/*30.11.02MCB : build this function                                        */
/*16.01.03MCB : add ON button                                              */
/*05.03.03JR: ON button checked without gpio function                      */
/*12.05.03TK: Changed ON-Button for Linux                                  */
/*16.05.03TK: Changed BitMasks and logic operators for FPGA 0.50           */
/*                                                                         */
/*-------------------------------------------------------------------------*/
uint8_t amm_check_pb (void)
{
	uint8_t rc;

	rc = ~AMM_PB_STATUS;
	rc |= 0x80;
	if ((au_readl(SYS_PINSTATERD) & 0x00000004) == 0) {
		rc &= ~AMM_PB_ON;
	}
        if (amm_fpga_ver == 52 )AMM_PB_STATUS = 0x7F;
	return (rc);
}

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*amm_test_pb                                                              */
/*                                                                         */
/*action: Tests all pushbuttons                                            */
/*                                                                         */
/*passing param.: -none-                                                   */
/*globalparam.: -none-                                                     */
/*returnparam.: -none-                                                     */
/*                                                                         */
/*programmer: Tiemo Krueger                                                */
/*                                                                         */
/*16.05.03TK: Build this function                                          */
/*                                                                         */
/*-------------------------------------------------------------------------*/
void amm_test_pb (void)
{
	amm_clear_alcd();
	amm_alcd_print("Press Cross-Left");
	while (amm_check_pb()!=0xDF) mdelay(5);
	amm_clear_alcd();
	amm_alcd_print("Press CrossRight");
	while (amm_check_pb()!=0xFB) mdelay(5);
	amm_clear_alcd();
	amm_alcd_print("Press Cross-Up");
	while (amm_check_pb()!=0xEF) mdelay(5);
	amm_clear_alcd();
	amm_alcd_print("Press Cross-Down");
	while (amm_check_pb()!=0xF7) mdelay(5);
	amm_clear_alcd();
	amm_alcd_print("Press LCD-Up");
	while (amm_check_pb()!=0xFE) mdelay(5);
	amm_clear_alcd();
	amm_alcd_print("Press LCD-Down");
	while (amm_check_pb()!=0xFD) mdelay(5);
	amm_clear_alcd();
	amm_alcd_print("Test complete");
	mdelay(2000);
	amm_clear_alcd();

	return;
}

/***************************************/
/* Char Device Driver Functions*/
/***************************************/

loff_t ti1500_func0_llseek (struct file * pFile , loff_t loff, int arg)
{
	return 0;
}

ssize_t ti1500_func0_read (struct file * pFile , char * pChar, 
		size_t size, loff_t * pArg)
{
	return 0;
}

ssize_t ti1500_func0_write (struct file * pfile, const char * pChar, 
		size_t size, loff_t * pArg)
{
	int restc=0;

	if (memcmp(pChar, "%cls", 4) == 0){
		amm_clear_alcd();
		colmarker=0;
		rowmarker=0;
	}
	else if (memcmp(pChar, "%line0", 6) == 0) {
		amm_alcd_gotoxy(0, 0);
		colmarker=0;
		rowmarker=0;
	}
	else if (memcmp(pChar, "%line1", 6) == 0) {
		amm_alcd_gotoxy(0, 1);
		colmarker=0;
		rowmarker=1;
	}
	else {
		if ( ((16-((int)size-1)-colmarker)>=0 || rowmarker==1)){
			amm_alcd_print(pChar);
			colmarker=colmarker+size-1;

		}
		else {
			amm_alcd_print(pChar);
			restc=size-(16-colmarker);
			amm_alcd_gotoxy(0, 1);
			amm_alcd_print(&pChar[size-restc]);
			rowmarker=1;
		}
	}
	return size;
}

int ti1500_func0_ioctl(struct inode * inode,struct file * pFile,unsigned int cmd, unsigned long arg)
{
	int err= 0, ret = 0, tmp;

	/* don't even decode wrong cmds: better returningENOTTY than EFAULT */
	if (_IOC_TYPE(cmd) != TI1500_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > TI1500_IOC_MAXNR) return -ENOTTY;

	/*
	* the type is a bitmask, and VERIFY_WRITE catches R/W
	* transfers. Note that the type is user-oriented, while
	* verify_area is kernel-oriented, so the concept of "read" and
	* "write" is reversed
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	err =!access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));

if (err)
	return -EFAULT;

	switch(cmd) {

		/*LCD IOCTL*/
		case TI1500_LCD_CLEAR:
		amm_clear_alcd();
			rowmarker=0;
			colmarker=0;
		break;
		case TI1500_LCD_GOTOXY:
		amm_alcd_gotoxy ((arg & 0x00000f00) >> 8, arg & 0x0000000f);
			colmarker=arg & 0x0000000f;
			rowmarker=(arg & 0x00000f00) >> 8;
		break;
		/*PB IOCTL*/
		case TI1500_KEY_READ:
			return amm_check_pb();
		/*IRQ IOCTL*/
		case TI1500_IRQ_LATCH:
		return amm_pci_mem_irq_remap[INT_LATCH];
		case TI1500_IRQ_MASKR:
		return amm_pci_mem_irq_remap[INT_MASK];
		case TI1500_IRQ_MASKW:
			amm_pci_mem_irq_remap[INT_MASK]=(unsigned long)arg;
		return amm_pci_mem_irq_remap[INT_MASK];
		case TI1500_IRQ_MRES:
			amm_pci_mem_irq_remap[INT_MRES]=(unsigned long)arg;
		return amm_pci_mem_irq_remap[INT_MASK];
		case TI1500_IRQ_MSET:
			amm_pci_mem_irq_remap[INT_MSET]=(unsigned long)arg;
		return amm_pci_mem_irq_remap[INT_MASK];
		/*FPGA GPIO IOCTL*/
		case TI1500_FPGA_INPUT:
		return (amm_pci_mem_gpio_remap[GPIO_IN] >> FPGA_SHIFT) & 
			FPGA_MASK_LOW;
		case TI1500_FPGA_OUTW:
			amm_pci_mem_gpio_remap[GPIO_O] =
			((arg << FPGA_SHIFT) & FPGA_MASK_HIGH ) | 
			(amm_pci_mem_gpio_remap[GPIO_O] & ~FPGA_MASK_HIGH);
			//explicitly nobreak;
		case TI1500_FPGA_OUTR:
		return (amm_pci_mem_gpio_remap[GPIO_O] >> FPGA_SHIFT) & 
			FPGA_MASK_LOW;
		case TI1500_FPGA_OUTRES:
			amm_pci_mem_gpio_remap[GPIO_ORES] = 
				(arg << FPGA_SHIFT) & FPGA_MASK_HIGH ;
		return (amm_pci_mem_gpio_remap[GPIO_O] >> FPGA_SHIFT) & 
			FPGA_MASK_LOW;
		case TI1500_FPGA_OUTSET:
			amm_pci_mem_gpio_remap[GPIO_OSET] = 
				(arg << FPGA_SHIFT) & FPGA_MASK_HIGH ;
		return (amm_pci_mem_gpio_remap[GPIO_O] >> FPGA_SHIFT) & 
			FPGA_MASK_LOW;
		case TI1500_FPGA_ENAW:
			amm_pci_mem_gpioena_remap[GPIO_EN] =
			((arg << FPGA_SHIFT) & FPGA_MASK_HIGH ) | 
			(amm_pci_mem_gpioena_remap[GPIO_EN] & ~FPGA_MASK_HIGH);
			//explicitly nobreak;
		case TI1500_FPGA_ENAR:
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> FPGA_SHIFT) & 
			FPGA_MASK_LOW;
		case TI1500_FPGA_ENARES:
			amm_pci_mem_gpioena_remap[GPIO_ENRES] = 
				(arg << FPGA_SHIFT) & FPGA_MASK_HIGH ;
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> FPGA_SHIFT) & 
			FPGA_MASK_LOW;
		case TI1500_FPGA_ENASET:
			amm_pci_mem_gpioena_remap[GPIO_ENSET] = 
				(arg << FPGA_SHIFT) & FPGA_MASK_HIGH ;
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> FPGA_SHIFT) & 
			FPGA_MASK_LOW;
		/*GPS GPIO IOCTL*/
		case TI1500_GPS_INPUT:
		return (amm_pci_mem_gpio_remap[GPIO_IN] >> GPS_SHIFT) & 
			GPS_MASK_LOW;
		case TI1500_GPS_OUTW:
			amm_pci_mem_gpio_remap[GPIO_O] = 
				((arg << GPS_SHIFT) & GPS_MASK_HIGH ) | 
				(amm_pci_mem_gpio_remap[GPIO_O] & 
				 ~GPS_MASK_HIGH);
			//explicitly nobreak;
		case TI1500_GPS_OUTR:
		return (amm_pci_mem_gpio_remap[GPIO_O] >> GPS_SHIFT) & 
			GPS_MASK_LOW;
		case TI1500_GPS_OUTRES:
			amm_pci_mem_gpio_remap[GPIO_ORES] = 
				(arg << GPS_SHIFT) & GPS_MASK_HIGH ;
		return (amm_pci_mem_gpio_remap[GPIO_O] >> GPS_SHIFT) & 
			GPS_MASK_LOW;
		case TI1500_GPS_OUTSET:
			amm_pci_mem_gpio_remap[GPIO_OSET] = 
				(arg << GPS_SHIFT) & GPS_MASK_HIGH ;
		return (amm_pci_mem_gpio_remap[GPIO_O] >> GPS_SHIFT) & 
			GPS_MASK_LOW;
		case TI1500_GPS_ENAW:
			amm_pci_mem_gpioena_remap[GPIO_EN] = 
				((arg << GPS_SHIFT) & GPS_MASK_HIGH ) | 
				(amm_pci_mem_gpioena_remap[GPIO_EN] & ~GPS_MASK_HIGH);
			//nobreak;
		case TI1500_GPS_ENAR:
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> GPS_SHIFT) & 
			GPS_MASK_LOW;
		case TI1500_GPS_ENARES:
			amm_pci_mem_gpioena_remap[GPIO_ENRES] = 
				(arg << GPS_SHIFT) & GPS_MASK_HIGH ;
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> GPS_SHIFT) & 
			GPS_MASK_LOW;
		case TI1500_GPS_ENASET:
			amm_pci_mem_gpioena_remap[GPIO_ENSET] = 
				(arg << GPS_SHIFT) & GPS_MASK_HIGH ;
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> GPS_SHIFT) & 
			GPS_MASK_LOW;
		/*VMX GPIO IOCTL*/
		case TI1500_VMX_INPUT:
		return (amm_pci_mem_gpio_remap[GPIO_IN] >> VMX_SHIFT) & 
			VMX_MASK_LOW;
		case TI1500_VMX_OUTW:
			amm_pci_mem_gpio_remap[GPIO_O] = 
				((arg << VMX_SHIFT) & VMX_MASK_HIGH ) | 
				(amm_pci_mem_gpio_remap[GPIO_O] & 
				 ~VMX_MASK_HIGH);
		//nobreak;
		case TI1500_VMX_OUTR:
		return (amm_pci_mem_gpio_remap[GPIO_O] >> VMX_SHIFT) & 
			VMX_MASK_LOW;
		case TI1500_VMX_OUTRES:
			amm_pci_mem_gpio_remap[GPIO_ORES] = 
				(arg << VMX_SHIFT) & VMX_MASK_HIGH ;
		return (amm_pci_mem_gpio_remap[GPIO_O] >> VMX_SHIFT) & 
			VMX_MASK_LOW;
		case TI1500_VMX_OUTSET:
			amm_pci_mem_gpio_remap[GPIO_OSET] = 
				(arg << VMX_SHIFT) & VMX_MASK_HIGH ;
		return (amm_pci_mem_gpio_remap[GPIO_O] >> VMX_SHIFT) & 
			VMX_MASK_LOW;
		case TI1500_VMX_ENAW:
			amm_pci_mem_gpioena_remap[GPIO_EN] = 
				((arg << VMX_SHIFT) & VMX_MASK_HIGH ) | 
				(amm_pci_mem_gpioena_remap[GPIO_EN] & 
				 ~VMX_MASK_HIGH);
			//nobreak;
		case TI1500_VMX_ENAR:
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> VMX_SHIFT) & 
			VMX_MASK_LOW;
		case TI1500_VMX_ENARES:
			amm_pci_mem_gpioena_remap[GPIO_ENRES] = 
				(arg << VMX_SHIFT) & VMX_MASK_HIGH ;
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> VMX_SHIFT) & 
			VMX_MASK_LOW;
		case TI1500_VMX_ENASET:
			amm_pci_mem_gpioena_remap[GPIO_ENSET] = 
				(arg << VMX_SHIFT) & VMX_MASK_HIGH ;
		return (amm_pci_mem_gpioena_remap[GPIO_EN] >> VMX_SHIFT) & 
			VMX_MASK_LOW;
		default:/* redundant, as cmd was checked against MAXNR */
		return ret;
	}
	return ret;
}

int ti1500_func0_open(struct inode * pInode, struct file * pFile)
{
	return 0;
}

int ti1500_func0_release (struct inode * pInode, struct file * pFile)
{
	return 0;
}

/**********************************************/
/* Table of symbols used for access of driver */
/**********************************************/
struct file_operations ti1500_func0_fops = {
	llseek:		ti1500_func0_llseek,
	read:		ti1500_func0_read,
	write:		ti1500_func0_write,
	ioctl:		ti1500_func0_ioctl,
	open:		ti1500_func0_open,
	release:	ti1500_func0_release,
};

/* have seven sins to export */
#ifndef __USE_OLD_SYMTAB__
EXPORT_SYMBOL (amm_clear_alcd);
EXPORT_SYMBOL (amm_alcd_gotoxy);
EXPORT_SYMBOL (amm_alcd_print);
EXPORT_SYMBOL (amm_check_pb);
EXPORT_SYMBOL (amm_test_pb);
EXPORT_SYMBOL (ti1500_func0_write);
EXPORT_SYMBOL (ti1500_func0_ioctl);
#endif

/*******************************/
/*Register Exported Symbols*/
/* grep ti1500 /proc/ksyms */
/*makes them visible */
/*******************************/
static int ti1500_func0_register(void) /* and export them */
{
	#ifdef __USE_OLD_SYMTAB__
	static struct symbol_table skull_syms = {

		#include <linux/symtab_begin.h>
	X(amm_clear_alcd),
	X(amm_alcd_gotoxy),
	X(amm_alcd_print),
	X(amm_check_pb),
	X(amm_test_pb),
	X(ti1500_func0_write),
	X(ti1500_func0_ioctl),
		#include <linux/symtab_end.h>
	};

	register_symtab(&skull_syms);
	#endif /* __USE_OLD_SYMTAB__ */
	return 0;
}

/*****************************/
/* Initialization procedure*/
/* for the Driver*/
/*****************************/

int xxs1500_fpga0_init_module(void)
{
	struct pci_dev *dev = NULL;
	unsigned long amm_pci_mem_start;
	volatile unsigned long amm_pci_mem_config ;
	int regResult;

	/*Search for the device*/
	dev = pci_find_device(AMM_VENDOR_ID_052, AMM_DEVICE_ID_052, dev);
	if (!dev) {
		dev = pci_find_device(AMM_VENDOR_ID_050, AMM_DEVICE_ID_050, dev);
		if (!dev) {
			printk("<1>AMD PCI FUNC0: Device NOT found\n");
			return -1;
		}
                else {
                	amm_fpga_ver = 50;
                }
	}
        else {
        	amm_fpga_ver = 52;
        }

	/*Enable PCI Device*/
	if (pci_enable_device(dev)) {
		printk("<1> AMM PCI FUNC0 dev enable failed\n");
		return -1;
	}

	/*Read the mapped memory space address*/
	pci_read_config_dword (dev, PCI_BASE_ADDRESS_0, &amm_pci_mem_start);
	pci_write_config_dword (dev, PCI_COMMAND, PCI_COMMAND_MEMORY);
	pci_read_config_dword (dev, PCI_COMMAND, &amm_pci_mem_config);
	if (!amm_pci_mem_start) {
		printk("<1>AMD PCI FUNC0: Memory space NOT found!\n");
		return -1;
	}

	/*Register the driver to the device node*/
	regResult = register_chrdev(DEV_MAJOR,"xxs1500_fpga0", &ti1500_func0_fops);
	if(regResult) {
		printk("<1>AMD PCI FUNC0: Registering of device failed \n");
		return regResult;
	}

	/*Map the different memory space regions for this function into kernel mem space*/
	amm_pci_mem_ui_remap= (unsigned long *)ioremap(amm_pci_mem_start + 
			UI_OFFSET , UI_LENGTH);
	amm_pci_mem_irq_remap= (unsigned long *)ioremap(amm_pci_mem_start + 
			INT_OFFSET , INT_LENGTH);
	amm_pci_mem_gpio_remap= (unsigned long *)ioremap(amm_pci_mem_start + 
			GPIO_OFFSET , GPIO_LENGTH);
	amm_pci_mem_gpioena_remap= (unsigned long *)ioremap(amm_pci_mem_start + 
			GPIOENA_OFFSET , GPIOENA_LENGTH);

	/*Initialization of LC-Display*/
	amm_init_alcd(amm_pci_mem_ui_remap);
	amm_switch_alcd (0);
	mdelay(100);
	amm_switch_alcd (1);
	mdelay(100);
	amm_clear_alcd();
	amm_alcd_gotoxy(0, 0);
	amm_alcd_print("Ti1500 PCI 0");
	amm_alcd_gotoxy(0, 1);
	amm_alcd_print("starting up");
	mdelay(1000);
	amm_clear_alcd();

	/*Initialization of Push-Buttons*/
	amm_init_pb (&amm_pci_mem_ui_remap[PB_DATA]);

	/*Register the exported symbols*/
	ti1500_func0_register();

	/*If OnButton is pressed, then execute PB-Test procedure*/
	if (amm_check_pb()==0x7f)
	amm_test_pb();

	return 0;
}

/*****************************/
/* CleanUp procedure */
/* for the Driver*/
/*****************************/
void xxs1500_fpga0_cleanup_module(void)
{
	/*Say Good Bye*/
	amm_clear_alcd();
	amm_alcd_gotoxy(0, 0);
	amm_alcd_print("Good Bye...");
	mdelay(1000);
	amm_clear_alcd();
	/*Unregister the driver from the device node*/
	unregister_chrdev(DEV_MAJOR,"ti1500pci0");
	/*Free the mapped memory regions*/
	iounmap(amm_pci_mem_ui_remap);
	iounmap(amm_pci_mem_irq_remap);
	iounmap(amm_pci_mem_gpio_remap);
	iounmap(amm_pci_mem_gpioena_remap);
}

/* Module information */
MODULE_AUTHOR("www.mycable.de");
MODULE_DESCRIPTION("XXS1500/FPGA0 FPGA Control Driver");
MODULE_LICENSE("GPL");

module_init(xxs1500_fpga0_init_module);
module_exit(xxs1500_fpga0_cleanup_module);
