/*
 * David Long, dave@metrolink.com
 * Copyright (C) 2001 Metro Link, Inc.  All rights reserved.
 *
 * ########################################################################
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * ########################################################################
 *
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/major.h>
#include <linux/tty.h>
#include <linux/kbd_ll.h>
#include <linux/kbd_kern.h>
#include <asm/semaphore.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/keyboard.h>
#include <asm/ati/xilleon.h>
#include <asm/ati/xilleonint.h>
#include <asm/ati/xilleonreg_kernel.h>
#include <asm/ati/xilleon_ir.h>

#define	NUM_KEY_CODES	256

static struct uirt_port uirt_devs[NUM_UIRT];
static struct semaphore uirt_sem;

/*
 * Scancode remapping table.
 */
static int keytab[NUM_KEY_CODES] = {
0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 29, 20, 21, 22,
23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82,
83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102,
103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118,
119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134,
135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150,
151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166,
167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198,
199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214,
215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230,
231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246,
247, 248, 249, 250, 251, 252, 253, 254, 255};

#ifdef	XILLEON_IR_KEYBOARD

void kbd_init_hw(void)
{}
/*
 * Used to alter contents of scancode remapping table.
 */
int kbd_setkeycode(unsigned int scancode, unsigned int keycode)
{
	if(scancode > NUM_KEY_CODES)
		return -EINVAL;
	keytab[scancode] = keycode;
	return 0;
}
/*
 * Used to lookup scancode conversions.
 */
int kbd_getkeycode(unsigned int scancode)
{
	if(scancode < 0 || scancode > NUM_KEY_CODES)
		return -EINVAL;
	else
		return keytab[scancode];
}

void kbd_leds(unsigned char leds)
{
}

int kbd_translate(unsigned char scancode, unsigned char *keycode, char raw_mode)
{
	*keycode = keytab[scancode];
	return 1;
}

char kbd_unexpected_up(unsigned char keycode)
{
	return 0;
}
#endif

/*
 * Determine how many samples in receive fifo.
 */
static inline int uirt_rx_count(struct uirt_port *uirt)
{
	return 32 - ((GETREG_PCUIPAMM32(UIRT_STATUS) >> 16) & 0x3f);
}
/*
 * Pull one sample from receive fifo
 */
static inline int uirt_read_data(struct uirt_port *uirt)
{
	return GETREG_PCUIPAMM32(UIRT_RCVR_FIFO);
}
/*
 * Hardware initialization
 */
static int uirt_port_init(int devnum, void (*uirt_handler)(int, void *, struct pt_regs *))
{
	struct uirt_port *uirt=&uirt_devs[devnum];
	struct uirt_dev *dev;
	unsigned long i;

	uirt->port = devnum;
	uirt->users = 0;
	uirt->rx_count = 0;
	uirt->savedcode = -1;
	uirt->savedcount = 0;
	uirt->recv_time = 0;
	dev = uirt->dev = ioremap_nocache(PHYSADDR(XILLEON_IPA_BASE+mmpcuipaUIRT_XMIT_FIFO), 64)-mmpcuipaUIRT_XMIT_FIFO;
#ifdef	DEBUG
	printk("UIRT init device %d at 0x%x\n", devnum, dev);
#endif
	SETREG_PCUIPAMM32(UIRT_INTR_ENABLE, 0);
	uirt->irq = XILLEON_UIRT_INT;
	if(request_irq(uirt->irq, uirt_handler, SA_INTERRUPT, "uirt", uirt)) {
		printk(KERN_INFO "uirt: can't get irq %d\n", uirt->irq);
	}
	uirt->rx_count = 0;
	uirt->tx_count = 0;
/*
 * Soft reset of UIRT hardware.
 */
//	SETFLAG_PCUIPAMM32(UIRT_CNTL, XMIT_SOFTRESET);
//	SETFLAG_PCUIPAMM32(UIRT_CNTL, RCVR_SOFTRESET);
	SETREG_PCUIPAMM32(UIRT_CNTL, (1<<17) | (1<<16) | 0xffff);
	i = jiffies+1;
/* This is ugly, but it only happens once */
	while(i <= jiffies) ;
/*
 * Set receive timeout to 2048*500/48000000 seconds.
 */
	SETREG_PCUIPAMM32(UIRT_CNTL, 500);
/*
 * Configure transmitter.  Not used for keyboards and remote control devices.
 */
#if 0
	SETFLD_PCUIPAMM32(UIRT_SAMPLE_CLKDIV, XMIT_SAMPLE_DIV, 9600);
	SETREG_PCUIPAMM32(UIRT_XMIT_MOD_CNTL, (1<<28) | (400<<16) | 1200);
	SETREG_PCUIPAMM32(UIRT_XMIT_CNTL, (1<<31) | (1<<30));
#endif
/*
 * Configure receiver.
 */
	SETFLD_PCUIPAMM32(UIRT_SAMPLE_CLKDIV, RCVR_SAMPLE_DIV, 9600);
#ifdef	DEMOD
	SETREG_PCUIPAMM32(UIRT_RCVR_DEMOD_CNTL, (1<<28) | (0x013<<16) | (1<12) | 300);
#else
	SETREG_PCUIPAMM32(UIRT_RCVR_DEMOD_CNTL, 0);
#endif
	SETREG_PCUIPAMM32(UIRT_RCVR_NOISE_CNTL, 0);
	SETREG_PCUIPAMM32(UIRT_RCVR_CNTL, 0);
	SETFLD_PCUIPAMM32(UIRT_RCVR_CNTL, RCVR_STOP_SAMPLES, 1);
	SETFLD_PCUIPAMM32(UIRT_RCVR_CNTL, RCVR_START_STOP, 6);
	SETFLAG_PCUIPAMM32(UIRT_RCVR_CNTL, RCVR_OVERRUN_CNTL);
	SETFLD_PCUIPAMM32(UIRT_RCVR_CNTL, RCVR_FIFO_THRESHOLD, 8);
/*
 * Enable device interrupts.
 */
	SETFLAG_PCUIPAMM32(UIRT_INTR_CNTL, RCVR_OVERRUN_CLR);
	SETFLAG_PCUIPAMM32(UIRT_INTR_CNTL, XMIT_EOT_CLR);
	SETFLAG_PCUIPAMM32(UIRT_INTR_CNTL, XMIT_UNDERRUN_CLR);
	SETFLAG_PCUIPAMM32(UIRT_INTR_ENABLE, RCVR_FIFO_EN);
	SETFLAG_PCUIPAMM32(UIRT_INTR_ENABLE, RCVR_OVERRUN_EN);
	SETFLAG_PCUIPAMM32(UIRT_INTR_ENABLE, RCVR_TIMEOUT_EN);
#ifndef	XILLEON_IR_KEYBOARD
	uirt->in_head = uirt->in_tail = 0;
	sema_init(&uirt->semaphore, 1);
	init_waitqueue_head(&uirt->in_waiters);
#endif
	SETFLAG_PCUIPAMM32(UIRT_INTR_ENABLE, MASTER_EN);
	return 0;
}
/*
 * Hardware shutdown
 */
static void uirt_port_exit(int devnum)
{
	struct uirt_port *uirt=&uirt_devs[devnum];
	struct uirt_dev *dev=uirt->dev;

	SETREG_PCUIPAMM32(UIRT_INTR_ENABLE, 0);
	free_irq(uirt->irq, uirt);
	iounmap(dev);
}
/*
 * Decode one complete sequence of pulses into a scancode.
 */
static int pulses_to_scancode(u16 *pulses, int count)
{
	int scancode=0, i;
int start;

/*
 * Find start pulse.
 */
	for(i=0; i < count; i++) {
		if(pulses[i] > 0x8012)
			break;
	}
	i++;
	if(i >= count)
		return -1;
/*
 * Verify start space following start pulse.
 */
	if(pulses[i] < 0x12 || pulses[i] > 0x17) {
		return -1;
	}
	i++;
	if((count-i) <= (2*PULSES_PER_KEY))
		return -1;
start=i;
/*
 * Examine spaces between pulses up to trailer pulse/space.
 */
	for(; i < count; i++) {
		if((pulses[i] & 0x7fff) > 11)
			break;
		if(pulses[i] & 0x8000)
			continue;
		scancode <<= 1;
		scancode |= (pulses[i] <= 7);
	}
/*
 * RCA scancode is in the low byte.  Bits 12 through 19 must contain the
 * complement of the low byte, and bits 8 through 11 must all be one.
 */
	if((scancode & 0xf00) != 0xf00 || (scancode>>12) != ((~scancode)&0xff))
		return -1;
	return keytab[scancode & 0xff];
}
/*
 * Interrupt handler
 */
static void handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uirt_port *uirt=dev_id;
	unsigned int sample;
	int scancode, i;

#if 0
/*
 * Service the transmitter.
 */
	if(GETFLAG_PCUIPAMM32(UIRT_INTR_STATUS, XMIT_FIFO_STAT)) {
	}
#endif
/*
 * Handle receiver overrun.
 */
	if(GETFLAG_PCUIPAMM32(UIRT_INTR_STATUS, RCVR_OVERRUN_STAT)) {
#ifdef	DEBUG
		printk("uirt: rcvr overrun\n");
#endif
		while(uirt_rx_count(uirt) > 0)
			sample = uirt_read_data(uirt);
		return;
	}
/*
 * Process received pulses.
 */
	while(uirt_rx_count(uirt) > 0) {
		sample = uirt_read_data(uirt);
		if(uirt->rx_count < UIRT_MAX_SAMPLES) {
			uirt->rx_pulses[uirt->rx_count++] = sample;
/*
 * A bit time greater than 11 clocks indicates a header bit or a timeout.
 */
			if((sample&0x7fff) > 11 && uirt->rx_count > 2) {
/*
 * Process complete pulse train
 */
				scancode = pulses_to_scancode(uirt->rx_pulses, uirt->rx_count);
				uirt->rx_count = 0;
				if(scancode >= 0) {
/*
 * A successful code transmission will consist of three identical codes.
 * The codes will be sent at a rate of about one every 100ms, so we restart
 * if we see a time gap of more than say 160ms.
 */
					if(scancode == uirt->savedcode && (jiffies-uirt->recv_time) < (HZ/6)) {
						uirt->savedcount++;
						if(uirt->savedcount >= 3) {
/*
 * At this point we have a scan code and need to send it on to the higher layers.
 */
#ifdef	XILLEON_IR_KEYBOARD
							handle_scancode(scancode, 1);
							handle_scancode(scancode, 0);
							tasklet_schedule(&keyboard_tasklet);
#else
							i = (uirt->in_tail + 1) % IN_QUEUE_SIZE;
							if(i != uirt->in_head) {
								uirt->in_queue[uirt->in_tail] = scancode;
								uirt->in_tail = i;
								wake_up_interruptible(&uirt->in_waiters);
							}
#endif
							uirt->savedcount = 0;
							uirt->savedcode = -1;
#ifdef	DEBUG
							printk("sc=0x%x\n", scancode);
#endif
						}
					} else {
/*
 * If the current code doesn't equal the last one, start over looking for three
 * in a row.
 */
						uirt->savedcode = scancode;
						uirt->savedcount = 1;
					}
					uirt->recv_time = jiffies;
				}
			}
		} else
			uirt->rx_count = 0;
	}
}

#ifndef	XILLEON_IR_KEYBOARD

#if 0
static inline int min(int a, int b)
{
	return ((a<b)?a:b);
}
#endif

static int uirt_open(struct inode *inode, struct file *file)
{
	int unit=inode->i_rdev&1;
	struct uirt_port *uirt=&uirt_devs[unit];

	MOD_INC_USE_COUNT;
	if(down_interruptible(&uirt_sem)) {
		MOD_DEC_USE_COUNT;
		return -ERESTARTSYS;
	}
	if(uirt->users <= 0) {
		uirt_port_init(unit, handler);
		uirt->users = 1;
	} else
		uirt->users++;
	up(&uirt_sem);
	file->private_data = uirt;
	return 0;
}

static int uirt_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static int uirt_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct uirt_port *uirt=file->private_data;
	int i, got=0;

	if(down_interruptible(&uirt->semaphore))
		return -ERESTARTSYS;
	while(got < count) {
		while(uirt->in_head == uirt->in_tail) {
			up(&uirt->semaphore);
			if(file->f_flags & O_NONBLOCK)
				return -EAGAIN;
			if(wait_event_interruptible(uirt->in_waiters, (uirt->in_head != uirt->in_tail)))
				return -ERESTARTSYS;
			if(down_interruptible(&uirt->semaphore))
				return -ERESTARTSYS;
		}
		if(uirt->in_tail < uirt->in_head) {
			i = min(count, IN_QUEUE_SIZE - uirt->in_head);
			if(copy_to_user(buf, &uirt->in_queue[uirt->in_head], i)) {
				up(&uirt->semaphore);
				return -EFAULT;
			}
			uirt->in_head = 0;
			buf += i;
			got += i;
			i = count - i;
		} else
			i = count;
		i = min(i, uirt->in_tail - uirt->in_head);
		if(i > 0) {
			if(copy_to_user(buf, &uirt->in_queue[uirt->in_head], i)) {
				up(&uirt->semaphore);
				return -EFAULT;
			}
			uirt->in_head = (uirt->in_head + i) % IN_QUEUE_SIZE;
			buf += i;
			got += i;
		}
	}
	up(&uirt->semaphore);

	return got;
}

static unsigned int uirt_poll(struct file *file, poll_table *pt)
{
	struct uirt_port *uirt=file->private_data;

	poll_wait(file, &uirt->in_waiters, pt);
	if(uirt->in_head != uirt->in_tail)
		return POLLIN | POLLRDNORM;
	else
		return 0;
}

static int uirt_release(struct inode *inode, struct file *file)
{
	int unit=inode->i_rdev&1;
	struct uirt_port *uirt=file->private_data;

	if(down_interruptible(&uirt_sem))
		return -ERESTARTSYS;
	if(--uirt->users <= 0) {
		uirt->users = 0;
		uirt_port_exit(unit);
		file->private_data = 0;
	}
	up(&uirt_sem);
	MOD_DEC_USE_COUNT;
	return 0;
}

struct file_operations uirt_ops = {
	read:		uirt_read,
	write:		uirt_write,
	poll:		uirt_poll,
	open:		uirt_open,
	release:	uirt_release
};

static struct miscdevice uirt_miscdev = {
	MISC_DYNAMIC_MINOR,
	"xilleon_ir",
	&uirt_ops
};

#endif
/*
 * Init wrapper function.  Calls real init routine for specific UIRT port.
 */
static int __init uirt_init(void)
{
	int result;

	sema_init(&uirt_sem, 1);
#ifdef	XILLEON_IR_KEYBOARD
	uirt_port_init(0, handler);
#else
	if((result=misc_register(&uirt_miscdev)) != 0) {
		printk(KERN_ERR "uirt: Couldn't register misc UIRT driver, error = %d\n", result);
		return result;
	}
#endif
	return 0;
}
/*
 * Shutdown wrapper function.  Calls real shutdown routine for specific UIRT port.
 */
static void __exit uirt_exit(void)
{
#ifdef	XILLEON_IR_KEYBOARD
	uirt_port_exit(0);
#else
	misc_deregister(&uirt_miscdev);
#endif
}

MODULE_AUTHOR("David A. Long");
MODULE_DESCRIPTION("ATI Xilleon UIRT driver");

module_init(uirt_init);
module_exit(uirt_exit);
