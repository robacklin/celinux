/*
 *  sbus_sccb.c
 *
 *  Implementation of the Camera Serial Bus for the Serial Camera Control
 *  Bus (SCCB) Specification for the OmniVision camera chips
 *  (OmniVision doc # AppNote 101 Version 2.0).
 *
 *  The SCCB control and data lines are connected to the OMAP1510
 *  I2C lines, but apparently the SCCB is not compatible with I2C bus
 *  timing, so we have to manually bit-bang the I2C lines to generate
 *  the correct SCCB timing.
 *  
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/sccb.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#define MODULE_NAME "sccb"
#include "common.h"
#include "camif.h"

#define DEBUG_SCCB

#define SCCB_NAME "sccb"
#define SCCB_MAJOR 12

static struct camera_serial_bus * this;

static spinlock_t sccb_lock;

/* I2C System Test Register */

#define I2C_BASE	(0xfffb3800)
#define I2C_SYSTEST	(*(volatile u16 *)(I2C_BASE + 0x3c))

#define ST_EN       (1 << 15) // System test enable
#define FREE        (1 << 14) // Free running mode (on breakpoint)
#define TMODE_MASK  (3 << 12) // Test mode select
#define TMODE_SHIFT (12)      // Test mode select
#define SCL_I       (1 << 3)  // SCL line sense input value
#define SCL_O       (1 << 2)  // SCL line drive output value
#define SDA_I       (1 << 1)  // SDA line sense input value
#define SDA_O       (1 << 0)  // SDA line drive output value


static inline void delay_full_clock(void)
{
	udelay(16);
}
static inline void delay_half_clock(void)
{
	udelay(8);
}
static inline void delay_quarter_clock(void)
{
	udelay(4);
}

static inline void sccb_busidle(void)
{
	/*
	 * float the SCL and SDA lines. The lines have 10K
	 * pull-ups on the Innovator so this will drive the
	 * lines high.
	 */
	I2C_SYSTEST |= SCL_O;
	delay_quarter_clock();
	I2C_SYSTEST |= SDA_O;
}

static inline void sccb_start(void)
{
	sccb_busidle();
	delay_full_clock();
	I2C_SYSTEST &= ~SDA_O;
	delay_quarter_clock();
	I2C_SYSTEST &= ~SCL_O;
}


static inline void setscl(int state)
{
	if (state) {
		I2C_SYSTEST |= SCL_O;
	} else {
		I2C_SYSTEST &= ~SCL_O;
	}
}

static inline void setsda(int state)
{
	if (state) {
		I2C_SYSTEST |= SDA_O;
	} else {
		I2C_SYSTEST &= ~SDA_O;
	}
} 

static inline int getscl(void)
{
	return (I2C_SYSTEST & SCL_I) ? 1 : 0;
}

static inline int getsda(void)
{
	return (I2C_SYSTEST & SDA_I) ? 1 : 0;
}


static inline void write_bit(int bit)
{
	setsda(bit);
	delay_quarter_clock();
	setscl(1);
	delay_half_clock();
	setscl(0);
	delay_quarter_clock();
}

static inline int read_bit(void)
{
	int bit;
	
	delay_quarter_clock();
	setscl(1);
	delay_half_clock();
	setscl(0);
	bit = getsda();
	delay_quarter_clock();
	return bit;
}

static void sccb_phase1(int read_cyc)
{
	int i;

	sccb_start();
	delay_quarter_clock();

	for (i=7; i>0; i--)
		write_bit(((u8)this->dev_id) & (1<<i));
	write_bit(read_cyc);
	read_bit(); // Don't-Care bit
}

static void sccb_phase2_subaddr(u8 subaddr, int two_phase_write)
{
	int i;

	for (i=7; i>=0; i--)
		write_bit(subaddr & (1<<i));
	read_bit(); // Don't-Care bit

	if (two_phase_write) {
		setsda(0);
		delay_quarter_clock();
		sccb_busidle();
	}
}

static u8 sccb_phase2_read(void)
{
	int i;
	u8 ret=0;

	for (i=7; i>=0; i--)
		ret |= (read_bit() << i);
	write_bit(1); // NA bit

	setsda(0);
	delay_quarter_clock();
	sccb_busidle();
	
	return ret;
}

static void sccb_phase3(u8 data)
{
	int i;

	for (i=7; i>=0; i--)
		write_bit(data & (1<<i));
	read_bit(); // Don't-Care bit

	setsda(0);
	delay_quarter_clock();
	sccb_busidle();
}


static int sccb_read (u8 subaddr, u8* buf, int len)
{
	unsigned long flags;
	int i;
	
	for (i=0; i<len; i++) {
		spin_lock_irqsave(&sccb_lock, flags);
		sccb_phase1(0);
		sccb_phase2_subaddr(subaddr++, 1);
		delay_full_clock();
		delay_full_clock();
		spin_unlock_irqrestore(&sccb_lock, flags);

		spin_lock_irqsave(&sccb_lock, flags);
		sccb_phase1(1);
		buf[i] = sccb_phase2_read();
		delay_full_clock();
		delay_full_clock();
		spin_unlock_irqrestore(&sccb_lock, flags);
	}

	return 0;
}

static int sccb_write (u8 subaddr, u8* buf, int len)
{
	unsigned long flags;
	int i;

	for (i=0; i<len; i++) {
		spin_lock_irqsave(&sccb_lock, flags);
		sccb_phase1(0);
		sccb_phase2_subaddr(subaddr++, 0);
		sccb_phase3(buf[i]);
		delay_full_clock();
		delay_full_clock();
		spin_unlock_irqrestore(&sccb_lock, flags);
	}

	return 0;
}

#define WRITE_VERIFY_RETRIES 5

static int sccb_write_verify(u8 subaddr, u8 val)
{
	int count=0;
	u8 readval;
	
	do {
		sccb_write(subaddr, &val, 1);
		readval = ~val;
		sccb_read(subaddr, &readval, 1);
	} while (readval != val && count++ < WRITE_VERIFY_RETRIES);

	if (readval != val && count >= WRITE_VERIFY_RETRIES) {
		err("%d attempts to write %02x to reg %02x failed\n",
		    WRITE_VERIFY_RETRIES, val, subaddr);
		return -ENXIO;
	}

	return 0;
}


static int sccb_set_devid(int id)
{
	this->dev_id = id;
	return 0;
}


/* Char Device Interface follows */
static int
sccb_ioctl(struct inode * inode, struct file *filp,
	   unsigned int cmd, unsigned long arg)
{
	int len, ret = 0;
	sccb_ioctl_t info;
	u8 buf[0x60];
	
	switch(cmd) {
	case SCCB_READ:
		copy_from_user(&info, (sccb_ioctl_t *)arg,
			       sizeof(sccb_ioctl_t));
		len = info.len <= sizeof(buf) ? info.len : sizeof(buf);
		if (verify_area(VERIFY_WRITE, (void*)info.buf, len))
			return -EFAULT;
		sccb_read(info.subaddr, buf, len);
		copy_to_user((void*)info.buf, buf, len);
		break;
	case SCCB_WRITE:
		copy_from_user(&info, (sccb_ioctl_t *)arg,
			       sizeof(sccb_ioctl_t));
		len = info.len <= sizeof(buf) ? info.len : sizeof(buf);
		if (verify_area(VERIFY_READ, (void*)info.buf, len))
			return -EFAULT;
		copy_from_user(buf, info.buf, len);
		if (info.write_verify) {
			int i;
			for (i=0; i<len; i++)
				ret |= sccb_write_verify(info.subaddr+i,
							 buf[i]);
		} else {
			sccb_write(info.subaddr, buf, len);
		}
		break;
	default:
		err("unknown SCCB ioctl %04x\n", cmd);
		return -EINVAL;
	}

	return ret;
}

static int
sccb_open(struct inode * inode, struct file * filp)
{
	MOD_INC_USE_COUNT;
	return 0;
}

static int
sccb_release(struct inode * inode, struct file * filp)
{
	MOD_DEC_USE_COUNT;
	return 0;
}

static struct file_operations sccb_fops = {
	ioctl:   sccb_ioctl,
	open:    sccb_open,
	release: sccb_release,
};

/*
 * Initialize the interface
 */
static int sccb_init(void)
{
	int ret = 0;

	this = &camera_sbus_sccb;

	spin_lock_init(&sccb_lock);

	/* make sure we're using the "new" I2C interface */
	outl(inl(MOD_CONF_CTRL_0) & ~(1<<16), MOD_CONF_CTRL_0);

	I2C_SYSTEST = ST_EN | FREE | (3 << TMODE_SHIFT);
	sccb_busidle();

	/* register our character device interface to SCCB bus */
	if ((ret = register_chrdev(SCCB_MAJOR, SCCB_NAME, &sccb_fops)) < 0) {
		err("can't get SCCB major number\n");
		return ret;
	}

	info("registered SCCB\n");
	return 0;
}


static void sccb_cleanup(void)
{
	I2C_SYSTEST = 0; // get out of test mode
	unregister_chrdev(SCCB_MAJOR, SCCB_NAME);
}


struct camera_serial_bus camera_sbus_sccb = {
	init:         sccb_init,
	cleanup:      sccb_cleanup,
	set_devid:    sccb_set_devid,
	read:         sccb_read,
	write:        sccb_write,
	write_verify: sccb_write_verify,
};
