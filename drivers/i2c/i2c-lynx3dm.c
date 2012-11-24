/*
    lynx3dm.c - I2C driver for Lynx3DM graphics chip
                Joerg Ritter, mycable GmbH
    
    Copyright (c) 2003 mycable GmbH <jr@mycable.de>,
    
    
    based on voodoo3.c

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/* This interfaces to the I2C bus of the Lynx3dm to gain access to
    the BT869 and possibly other I2C devices. */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/init.h>
#include <linux/delay.h>

#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

#define LM_VERSION "0.1"
#define LM_DATE "2003-07-31"

#define PCI_SMI_VENDORID                0x126F
#define PCI_LYNX3DMP_DEVICEID           0x0720
 	

#define DEB(x) if (i2c_debug>=1) x;
#define DEB2(x) if (i2c_debug>=2) x;
	
	
/* bit locations in the register */
#define I2C_SCL_OUT	(1 << 0)
#define I2C_SDA_OUT	(1 << 1)
#define I2C_SCL_IN	(1 << 2)
#define I2C_SDA_IN	(1 << 3)

/* initialization states */
#define I2C_ENAB	0x33


/* delays */
#define CYCLE_DELAY	100 //10
#define TIMEOUT		200 //50

#ifdef MODULE
static
#else
extern
#endif
int __init i2c_lynx3dm_init(void);
static int __init lynx3dm_cleanup(void);
static int lynx3dm_setup(void);
static void config_lynx3dm_i2c(void);
static void lynx3dm_inc(struct i2c_adapter *adapter);
static void lynx3dm_dec(struct i2c_adapter *adapter);

#ifdef MODULE
extern int init_module(void);
extern int cleanup_module(void);
#endif				/* MODULE */


static int __initdata lynx3dm_initialized;
static volatile u8    *mem;
static volatile u8    *MMIO;
static int i2c_debug = 0;

static void bit_i2c_setscl(void *data, int val)
{
	u8 r;
	DEB(printk("%s: %d   ", __FUNCTION__, val));
	MMIO[0x3C4] = 0x72; 
	r = MMIO[0x3C5];
	if(val) {
		r |= I2C_SCL_OUT;
	} else {
		r &= ~I2C_SCL_OUT;
	}
	MMIO[0x3c5] = r;
}

static void bit_i2c_setsda(void *data, int val)
{
	u8 r;
	DEB(printk("%s:    %d", __FUNCTION__, val));
	MMIO[0x3C4] = 0x72; 
	r = MMIO[0x3C5];
	if(val) {
		r |= I2C_SDA_OUT;
	} else {
		r &= ~I2C_SDA_OUT;
	}
	MMIO[0x3c5] = r;
}

/* The GPIO pins are open drain, so the pins always remain outputs.
   We rely on the i2c-algo-bit routines to set the pins high before
   reading the input from other chips. */

static int bit_i2c_getscl(void *data)
{
	u8 r, b;
	MMIO[0x3C4] = 0x72; 
	udelay(10);
	r = MMIO[0x3C5];
	b = (0 != (r & I2C_SCL_IN));
	DEB(printk("%s:       %d\n", __FUNCTION__, b));
	return b;
}

static int bit_i2c_getsda(void *data)
{
	u8 r, b;
	MMIO[0x3C4] = 0x72; 
	udelay(10);
	r = MMIO[0x3C5];
	b = (0 != (r & I2C_SDA_IN));
	DEB(printk("%s:          %d r=0x%x\n", __FUNCTION__, b, r));
	return b;
}

static struct i2c_algo_bit_data i2c_bit_data = {
	NULL,
	bit_i2c_setsda,
	bit_i2c_setscl,
	bit_i2c_getsda,
	bit_i2c_getscl,
	CYCLE_DELAY, CYCLE_DELAY, TIMEOUT
};

static struct i2c_adapter lynx3dm_i2c_adapter = {
	"I2C Lynx3dm adapter",
	0x18,                          //  FIXME
	NULL,
	&i2c_bit_data,
	lynx3dm_inc,
	lynx3dm_dec,
	NULL,
	NULL,
};

static void config_lynx3dm_i2c (void)
{
	MMIO[0x3C4] = 0x72; MMIO[0x3C5] = 0x33; /* enable I2C pins */
	
}

/* Detect whether a Lynx3dm can be found,
   and initialize it. */
static int lynx3dm_setup(void)
{
	struct pci_dev *pdev = NULL;

	if (!(pdev = pci_find_device(PCI_SMI_VENDORID,
				     PCI_LYNX3DMP_DEVICEID, pdev))) {
		printk(KERN_INFO "  Lynx3M+ not found\n");
		return -ENODEV;
	} else {
		printk(KERN_INFO "  Lynx3DM+ found\n");
		mem = ioremap_nocache(pci_resource_start(pdev, 0), 8*1024*1024);
		MMIO = mem + 0x0c0000;
		if (!MMIO) {
			printk(KERN_WARNING "couldn't map Lynx3DM mem.\n");
			return -ENODEV;
		}
		config_lynx3dm_i2c();
		return 0;
	}
}

void lynx3dm_inc(struct i2c_adapter *adapter)
{
	MOD_INC_USE_COUNT;
}

void lynx3dm_dec(struct i2c_adapter *adapter)
{
	MOD_DEC_USE_COUNT;
}

int __init i2c_lynx3dm_init(void)
{
	int res;
	printk(KERN_INFO "i2c-lynx3dm.o version %s (%s)\n", LM_VERSION, LM_DATE);

	if ((res = lynx3dm_setup())) {
		printk(KERN_INFO "i2c-lynx3dm.o: Lynx3dm not detected, module not inserted.\n");
		lynx3dm_cleanup();
		return res;
	}
	if ((res = i2c_bit_add_bus(&lynx3dm_i2c_adapter))) {
		printk(KERN_INFO "i2c-lynx3dm.o: I2C adapter registration failed\n");
		lynx3dm_cleanup();
		return res;
	} else {
		lynx3dm_initialized = 1;
		printk(KERN_INFO "i2c-lynx3dm.o: I2C bus initialized\n");
	}

	return 0;
}

int __init lynx3dm_cleanup(void)
{
	int res;

	iounmap((void *) mem);
	if (lynx3dm_initialized) {
		if ((res = i2c_bit_del_bus(&lynx3dm_i2c_adapter))) {
			printk(KERN_INFO "i2c-lynx3dm.o: i2c_bit_del_bus failed, module not removed\n");
			return res;
		}
	}
	return 0;
}

EXPORT_NO_SYMBOLS;

#ifdef MODULE

MODULE_AUTHOR
    ("Joerg Ritter <jr@mycable.de>");
MODULE_DESCRIPTION("Lynx3dm I2C driver");


int init_module(void)
{
	return i2c_lynx3dm_init();
}

int cleanup_module(void)
{
	return lynx3dm_cleanup();
}

#endif				/* MODULE */
