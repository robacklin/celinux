/*
 *	linux/drivers/char/tcube-rtc.c
 *
 *	Real Time Clock interface for Linux on T-Cube.
 *	(RICOH Co., Ltd., Rx5C348B)
 *
 *	Copyright (c) 2003 Lineo Solutions, Inc.
 *
 *	Based on sa1100-rtc.c
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>

#undef	RTC_ALARM_SUPPORT
//#define	RTC_ALARM_SUPPORT	1

#define	DRIVER_VERSION		"1.00"

/*
 *  * Check machine
 *   */
#if !defined(CONFIG_MIPS) || !defined(CONFIG_NEW_TIME_C)
#error "This driver is for MIPS machines with CONFIG_NEW_TIME_C defined"
#endif

#include <asm/time.h>

#define TIMER_FREQ		3686400

#define RTC_DEF_DIVIDER		32768 - 1
#define RTC_DEF_TRIM		0

/* Those are the bits from a classic RTC we want to mimic */
#define RTC_IRQF		0x80	/* any of the following 3 is active */
#define RTC_PF			0x40
#define RTC_AF			0x20
#define RTC_UF			0x10

int (*rtc_get_alm_time)(struct rtc_time *);
int (*rtc_set_alm_time)(struct rtc_time *);
int (*rtc_get_ctrl_reg)(unsigned char *);
int (*rtc_set_ctrl_reg)(unsigned char *);

static unsigned long rtc_status;
static unsigned long rtc_irq_data;

static struct fasync_struct *rtc_async_queue;
static DECLARE_WAIT_QUEUE_HEAD(rtc_wait);

extern spinlock_t rtc_lock;

static const unsigned char days_in_mo[] =
	{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

#define is_leap(year) \
	((year) % 4 == 0 && ((year) % 100 != 0 || (year) % 400 == 0))

#if defined(RTC_ALARM_SUPPORT)
static void tcube_rtc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	/***************************/
	/***************************/

	/* wake up waiting process */
	wake_up_interruptible(&rtc_wait);
	kill_fasync (&rtc_async_queue, SIGIO, POLL_IN);
}
#endif

static int tcube_rtc_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit (1, &rtc_status))
		return -EBUSY;
	rtc_irq_data = 0;
	return 0;
}

static int tcube_rtc_release(struct inode *inode, struct file *file)
{
	spin_lock_irq (&rtc_lock);
	spin_unlock_irq (&rtc_lock);
	rtc_status = 0;
	return 0;
}

static int tcube_rtc_fasync (int fd, struct file *filp, int on)
{
	return fasync_helper (fd, filp, on, &rtc_async_queue);
}

static unsigned int tcube_rtc_poll(struct file *file, poll_table *wait)
{
	poll_wait (file, &rtc_wait, wait);
	return (rtc_irq_data) ? POLLIN | POLLRDNORM : 0;
}

static loff_t tcube_rtc_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

ssize_t tcube_rtc_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long data;
	ssize_t retval;

	if (count < sizeof(unsigned long))
		return -EINVAL;

	add_wait_queue(&rtc_wait, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	for (;;) {
		spin_lock_irq (&rtc_lock);
		data = rtc_irq_data;
		if (data != 0) {
			rtc_irq_data = 0;
			break;
		}
		spin_unlock_irq (&rtc_lock);

		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		}

		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}

		schedule();
	}

	retval = put_user(data, (unsigned long *)buf);
	if (!retval)
		retval = sizeof(unsigned long);

out:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&rtc_wait, &wait);
	return retval;
}

static int tcube_rtc_ioctl(struct inode *inode, struct file *file,
			   unsigned int cmd, unsigned long arg)
{
	struct rtc_time tm;
	unsigned long curr_time;
#if defined(RTC_ALARM_SUPPORT)
	unsigned char ctrl_reg[2];
#endif

	switch (cmd) {
	case RTC_AIE_OFF:
#if defined(RTC_ALARM_SUPPORT)
		spin_lock_irq (&rtc_lock);
		rtc_get_ctrl_reg(&ctrl_reg[0]);
		ctrl_reg[0] ^= 0x40;
		rtc_set_ctrl_reg(&ctrl_reg[0]);
		spin_unlock_irq (&rtc_lock);
#else
		return -EINVAL;
#endif
		break;

	case RTC_AIE_ON:
#if defined(RTC_ALARM_SUPPORT)
		spin_lock_irq (&rtc_lock);
		rtc_get_ctrl_reg(&ctrl_reg[0]);
		ctrl_reg[0] |= 0x40;
		rtc_set_ctrl_reg(&ctrl_reg[0]);
		spin_unlock_irq (&rtc_lock);
#else
		return -EINVAL;
#endif
		break;

	case RTC_UIE_OFF:
		return -EINVAL;
		break;

	case RTC_UIE_ON:
		return -EINVAL;
		break;

	case RTC_ALM_READ:
		rtc_get_alm_time(&tm);
		break;

	case RTC_ALM_SET:
		if (copy_from_user (&tm, (struct rtc_time*)arg, sizeof (tm)))
			return -EFAULT;
		return rtc_set_alm_time(&tm);

	case RTC_RD_TIME:	/* Read the time/date from RTC  */
		curr_time = rtc_get_time();
		to_tm(curr_time, &tm);
		tm.tm_year -= 1900;
		break;

	case RTC_SET_TIME:	/* Set the time/date from RTC  */
		if (!capable(CAP_SYS_TIME))
			return -EACCES;
		if (copy_from_user (&tm, (struct rtc_time*)arg, sizeof (tm)))
			return -EFAULT;
		tm.tm_year += 1900;
		if (tm.tm_year < 1970 || (unsigned)tm.tm_mon >= 12 ||
		    tm.tm_mday < 1 || tm.tm_mday > (days_in_mo[tm.tm_mon] +
				(tm.tm_mon == 1 && is_leap(tm.tm_year))) ||
		    (unsigned)tm.tm_hour >= 24 ||
		    (unsigned)tm.tm_min >= 60 ||
		    (unsigned)tm.tm_sec >= 60)
			return -EINVAL;
		curr_time = mktime (tm.tm_year,
				    tm.tm_mon + 1, /* tm_mon starts from 0 */
				    tm.tm_mday,
				    tm.tm_hour,
				    tm.tm_min,
			       	    tm.tm_sec);
		return rtc_set_time(curr_time);

	case RTC_PIE_ON:
	case RTC_PIE_OFF:
	case RTC_IRQP_READ:
	case RTC_IRQP_SET:
	case RTC_EPOCH_READ:
	case RTC_EPOCH_SET:
	case RTC_WKALM_SET:
	case RTC_WKALM_RD:
	default:
		return -EINVAL;
	}
	return copy_to_user ((void *)arg, &tm, sizeof (tm)) ? -EFAULT : 0;
}

static struct file_operations tcube_rtc_fops = {
	owner:		THIS_MODULE,
	llseek:		tcube_rtc_llseek,
	read:		tcube_rtc_read,
	poll:		tcube_rtc_poll,
	ioctl:		tcube_rtc_ioctl,
	open:		tcube_rtc_open,
	release:	tcube_rtc_release,
	fasync:		tcube_rtc_fasync,
};

static struct miscdevice tcube_rtc_miscdev = {
	RTC_MINOR,
	"rtc",
	&tcube_rtc_fops
};

static int tcube_rtc_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len;
	struct rtc_time tm;
	unsigned long curr_time;
#if defined(RTC_ALARM_SUPPORT)
	unsigned char ctrl_reg[2];
#endif

	curr_time = rtc_get_time();
	to_tm(curr_time, &tm);
	tm.tm_year -= 1900;
	p += sprintf(p, "rtc_time\t: %02d:%02d:%02d\n"
			"rtc_date\t: %04d-%02d-%02d\n"
			"rtc_epoch\t: %04d\n",
			tm.tm_hour, tm.tm_min, tm.tm_sec,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, 1970);
#if defined(RTC_ALARM_SUPPORT)
	rtc_get_alm_time(&tm);
	rtc_get_ctrl_reg(&ctrl_reg[0]);
	p += sprintf(p, "alrm_time\t: %02d:%02d\n", tm.tm_hour, tm.tm_min);
	p += sprintf(p, "alarm_IRQ\t: %s\n",
				     (ctrl_reg[0] & 0x40) ? "yes" : "no" );
#endif

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

static int rtc_setup_interrupts(void)
{
#if defined(RTC_ALARM_SUPPORT)
	int ret;

	ret = request_irq (, rtc_interrupt, SA_INTERRUPT, "rtc Alrm", NULL);
	if (ret) {
		printk(KERN_ERR "rtc: IRQ %d already in use.\n", );
		return ret;
	}

#endif
	return 0;
}

static void rtc_free_interrupts(void)
{
#if defined(RTC_ALARM_SUPPORT)
	free_irq (, NULL);
#endif
}

static int __init tcube_rtc_init(void)
{
	int ret;

	misc_register (&tcube_rtc_miscdev);
	create_proc_read_entry ("driver/rtc", 0, 0, tcube_rtc_read_proc, NULL);
	ret = rtc_setup_interrupts();
	if (ret)
		goto IRQ_failed;

	printk(KERN_INFO "T-Cube Real Time Clock driver v" DRIVER_VERSION "\n");

	return 0;

IRQ_failed:
	remove_proc_entry ("driver/rtc", NULL);
	misc_deregister (&tcube_rtc_miscdev);
	return ret;
}

static void __exit tcube_rtc_exit(void)
{
	rtc_free_interrupts();
	remove_proc_entry ("driver/rtc", NULL);
	misc_deregister (&tcube_rtc_miscdev);
}

module_init(tcube_rtc_init);
module_exit(tcube_rtc_exit);

MODULE_AUTHOR("Lineo Solutions, Inc.");
MODULE_DESCRIPTION("T-Cube Realtime Clock Driver (RTC)");
EXPORT_NO_SYMBOLS;
