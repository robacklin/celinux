/*
 * sii164.c: I2C chip driver for SiI164 Panel Link Transmitter
 *           
 * 
 * Copyright 2003 mycable GmbH
 * Author: mycable GmbH
 *         	 Joerg Ritter jr@mycable.de 
 *
 *  based on Montavista's m41t11 driver
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
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/uaccess.h>
#include <asm/system.h>

#include <linux/ioctl.h>
#include "sii164.h"

#include <linux/delay.h>



#undef	DEBUG
#define	DEBUG
#define	DEBUG_SII164
#if	defined(DEBUG_SII164)
#	define	DPRINTK(fmt, args...)	printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
//#	define	DPRINTK(fmt, args...)	printk(KERN_INFO "%s: " fmt, __FUNCTION__ , ## args)
#else
#	define	DPRINTK(fmt, args...)
#endif

struct sii_init_tab {
        u8      reg;
        u8      val;
} init_tab[] = {
	{0x08, 0x37},
	{0x09, 0x07},
	{0x0a, 0x06},
	{0x0c, 0x89},
	{0x0d, 0x03},
	{0x0e, 0x00},
	{0x0f, 0x00}
};

struct sii_init_tab init_tab_off[] = {
	{0x08, 0x36},
	{0x09, 0x07},
	{0x0a, 0x06},
	{0x0c, 0x89},
	{0x0d, 0x03},
	{0x0e, 0x00},
	{0x0f, 0x00}
};

#define NUM_INIT_TAB 7

static struct i2c_driver sii164_driver;

static int sii164_use_count = 0;

/*
 *	If this driver ever becomes modularised, it will be really nice
 *	to make the epoch retain its value across module reload...
 */


#define MY_I2C_ADDR 0x38
static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { ANY_I2C_BUS, MY_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END};

static struct i2c_client_address_data addr_data = {
	normal_i2c:		ignore, //normal_addr,
	normal_i2c_range:	ignore,
	probe:			ignore,
	probe_range:		ignore,
	ignore:			ignore,
	ignore_range:		ignore,
	force:			normal_addr// ignore,
};

static struct i2c_client *this_client;


#ifdef DEBUG_SII164 
static void sii_dump_regs(void)
{
	unsigned char data[2], buf[2];
	struct i2c_msg msgs[2] = {
		{ MY_I2C_ADDR, I2C_M_WR, 1, data },
		{ MY_I2C_ADDR, I2C_M_RD, 1, buf }
	};
	int i;
	printk("SiI164 regs:\n");	
	for (i = 0; i < 16; ++i) {
		data[0] = i;
		i2c_transfer(this_client->adapter, msgs, 2);
		printk("  addr %d 0x%02x\n", i, buf[0]);
	}

}
#endif

static int
sii164_attach(struct i2c_adapter *adap, int addr, unsigned short flags,
	int kind)
{
	int res;
	DPRINTK("\n");
	this_client = kmalloc(sizeof(*this_client), GFP_KERNEL);

	if (!this_client) {
		return -ENOMEM;
	}

	strcpy(this_client->name, "SII164");
	this_client->id		= sii164_driver.id;
	this_client->flags	= 0;
	this_client->addr	= addr;
	this_client->adapter	= adap;
	this_client->driver	= &sii164_driver;
	this_client->data	= NULL;
	res = i2c_attach_client(this_client);

	return res; 
}

static int
sii164_probe(struct i2c_adapter *adap)
{
	DPRINTK("\n");
	return i2c_probe(adap, &addr_data, sii164_attach);
}

static int
sii164_detach(struct i2c_client *client)
{
	DPRINTK("\n");
	i2c_detach_client(client);

	return 0;
}


static int
sii164_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	DPRINTK("\n");
	switch (cmd) {
#if 0                
	case MEM_READ:
		return sii164_read_mem(client, arg);
	case MEM_WRITE:
		return sii164_write_mem(client, arg);
#endif                
	default:
		return -EINVAL;
	}
}

int sii_open(struct inode *minode, struct file *mfile)
{
	/*if(MOD_IN_USE)*/
	DPRINTK("\n");
	if(sii164_use_count > 0) {
		return -EBUSY;
	}
	MOD_INC_USE_COUNT;
	++sii164_use_count;
	return 0;
}

int sii_release(struct inode *minode, struct file *mfile)
{
	DPRINTK("\n");
	MOD_DEC_USE_COUNT;
	--sii164_use_count;
	return 0;
}


static int sii_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
			unsigned long arg)
{
	unsigned char data[2];
	struct i2c_msg msgs[2] = {
		{ MY_I2C_ADDR, I2C_M_WR, 2, data }
	};
	int i;
	DPRINTK("sii_ioctl(0x%x)\n", cmd);
	switch (cmd) {
	case TI1500_DVI_ON:
		DPRINTK(" TI1500_DVI_ON\n");
		for (i = 0; i < (NUM_INIT_TAB); ++i) {
			data[0] = init_tab[i].reg;
			data[1] = init_tab[i].val;
			i2c_transfer(this_client->adapter, msgs, 1);
		}
		break;
	case TI1500_DVI_OFF:
		DPRINTK(" TI1500_DVI_OFF\n");
		for (i = 0; i < (NUM_INIT_TAB); ++i) {
			data[0] = init_tab_off[i].reg;
			data[1] = init_tab_off[i].val;
			i2c_transfer(this_client->adapter, msgs, 1);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned int sii_poll(struct file *file, poll_table *wait)
{
	DPRINTK("\n");
	return -ENOSYS;
}


static struct i2c_driver sii164_driver = {
	name:		"SII164",
	id:		I2C_DRIVERID_EXP0,
	flags:		I2C_DF_NOTIFY,
	attach_adapter:	sii164_probe,
	detach_client:	sii164_detach,
	command:	sii164_command
};

static struct file_operations sii_fops = {
	owner:		THIS_MODULE,
	llseek:		NULL,
	read:		NULL,
	poll:		NULL,
	ioctl:		sii_ioctl,
	open:		sii_open,
	release:	sii_release,
	fasync:		NULL
};

static struct miscdevice sii164_miscdev = {
	222,
	"sii164",
	&sii_fops
};

static __init int sii164_init(void)
{
	int err;

	DPRINTK("\n");
	printk("sii164.o: SII164 I2C based driver.\n");
	err = i2c_add_driver(&sii164_driver);
	if (err) {
		printk("sii164.o: Register I2C driver failed, errno is %d\n"
			,err);
		return err;
	}
	err = misc_register(&sii164_miscdev);
	if (err) {
		printk("sii164.o: Register misc driver failed, errno is %d\n"
			,err);
		err = i2c_del_driver(&sii164_driver);
		if (err) {
			printk("sii164.o: Unregister I2C driver failed, "
				"errno is %d\n" ,err);
		}
		return err;
	}
	sii_ioctl(NULL, NULL, TI1500_DVI_ON, 0);
	return 0;
}

MODULE_AUTHOR("Joerg Ritter, mycable GmbH");
MODULE_DESCRIPTION("I2C-Bus chip driver for SiI164");
MODULE_LICENSE("GPL");

module_init(sii164_init);
//module_exit(sii164_exit); 

