/*
 * linux/drivers/char/innovator-keypad.c
 *
 * TI Innovator/OMAP1510 keypad driver
 *
 * Author: MontaVista Software, Inc.
 *         <gdavis@mvista.com> or <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * History:
 *
 * 20030529: George G. Davis <gdavis@mvista.com>
 *	Initial release
 *
 * TODO:
 * 1. Test with a variety of GUIs.
 * 2. Complete DPM/LDM stubs and test.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/slab.h>

#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/arch/irqs.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/irq.h>

#undef	DEBUG
#ifdef	DEBUG
int debug = 0;
#define	DPRINTK(fmt, args...) \
	if (debug) {						\
		printk(KERN_DEBUG "%s:%d: " fmt,		\
		       __FUNCTION__ , __LINE__ , ## args);	\
	}
#else
#define	DPRINTK(fmt, args...)
#endif

static struct input_dev keypad_dev;
static int innovator_keypad_pid;
static DECLARE_COMPLETION(keypadt);
static wait_queue_head_t keypadt_wait;
struct task_struct *innovator_keypad_task;

#if	defined(CONFIG_OMAP_INNOVATOR)	/* MVL-CEE */
#include <linux/device.h>

static int innovator_keypad_suspend(struct device *dev, u32 state, u32 level);
static int innovator_keypad_resume(struct device *dev, u32 level);
static int innovator_keypad_scale(struct bus_op_point *op, u32 level);

static struct device_driver innovator_keypad_driver_ldm = {
	.name		= "innovator-keypad",
	.devclass	= NULL,
	.probe		= NULL,
	.suspend	= innovator_keypad_suspend,
	.resume		= innovator_keypad_resume,
	.scale		= innovator_keypad_scale,
	.remove		= NULL,
	.constraints	= NULL,
};

static struct device innovator_keypad_device_ldm = {
	.name		= "Innovator/OMAP1510 Keypad",
	.bus_id		= "keypad",
	.driver		= NULL,
	.power_state	= DPM_POWER_ON,
};

static void
innovator_keypad_ldm_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);
	mpu_public_driver_register(&innovator_keypad_driver_ldm);
}

static void
innovator_keypad_ldm_device_register(void)
{
	extern void mpu_public_device_register(struct device *device);
	mpu_public_device_register(&innovator_keypad_device_ldm);
}

static void
innovator_keypad_ldm_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	mpu_public_driver_unregister(&innovator_keypad_driver_ldm);
}

static void
innovator_keypad_ldm_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *device);
	mpu_public_device_unregister(&innovator_keypad_device_ldm);
}

static int
innovator_keypad_scale(struct bus_op_point *op, u32 level)
{
	/* REVISIT */
	return 0;
}

static int
innovator_keypad_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		/* REVISIT */
		break;
	}

	return 0;
}

static int
innovator_keypad_resume(struct device *dev, u32 level)
{
	int ret = 0;

	switch (level) {
	case RESUME_POWER_ON:
		/* REVISIT */
		break;
	}

	return ret;
}
#endif				/* MVL-CEE */

static u32
innovator_keypad_read(void)
{
	int i;
	u32 keys = 0;

	outw(0xff, OMAP1510_MPUIO_KBC_REG);
	udelay(1);

	for (i = 0; i < 4; i += 1) {
		outw(~(1 << i) & 0xff, OMAP1510_MPUIO_KBC_REG);
		udelay(1);
		keys |= ((~inw(OMAP1510_MPUIO_KBR_LATCH)) & 0xf) << (i << 2);
		inw(OMAP1510_32kHz_TIMER_BASE);	/* BTS_Errata.22 */
	}

	outw(0x0, OMAP1510_MPUIO_KBC_REG);
	udelay(1);

	DPRINTK("keys: %0x\n", keys);

	return keys;
}

static void
innovator_keypad_report(u32 old, u32 new)
{
	/* The Innovator/OMAP1510 keypad row/column button assignments
	 * below were determined by simply pressing each button and
	 * scanning the keypad to find the R/C assignments. These
	 * assignments do not completely agree with currently available
	 * Innovator/OMAP1510 documentation. However the currently assigned
	 * functions for each row/col pair match the logical function of
	 * each button.
	 */

	if ((old ^ new) & 0x00000001)	/* Col 0/Row 0: SW6 */
		input_report_key(&keypad_dev, KEY_F1, new & 0x00000001);
	if ((old ^ new) & 0x00000008)	/* Col 0/Row 3: SW2-DOWN */
		input_report_key(&keypad_dev, KEY_DOWN, new & 0x00000008);
	if ((old ^ new) & 0x00000020)	/* Col 1/Row 1: SW5 */
		input_report_key(&keypad_dev, KEY_F2, new & 0x00000020);
	if ((old ^ new) & 0x00000040)	/* Col 1/Row 2: */
		input_report_key(&keypad_dev, KEY_RIGHT, new & 0x00000040);
	if ((old ^ new) & 0x00000100)	/* Col 2/Row 0: SW4 */
		input_report_key(&keypad_dev, KEY_F3, new & 0x00000100);
	if ((old ^ new) & 0x00000200)	/* Col 2/Row 1: SW7 */
		input_report_key(&keypad_dev, KEY_RIGHT, new & 0x00000200);
	if ((old ^ new) & 0x00000400)	/* Col 2/Row 0: */
		input_report_key(&keypad_dev, KEY_UP, new & 0x00000400);
	if ((old ^ new) & 0x00004000)	/* Col 3/Row 2: SW2-SEL */
		input_report_key(&keypad_dev, KEY_ENTER, new & 0x00004000);
	if ((old ^ new) & 0x00008000)	/* Col 3/Row 3: */
		input_report_key(&keypad_dev, KEY_LEFT, new & 0x00008000);
}

static void
innovator_keypad_scan(void)
{
	int timeout;
	u32 old = 0;
	u32 new = 0;

	do {
		new = innovator_keypad_read();
		if (old != new) {
			DPRINTK("old: %0x, new: %0x\n", old, new);
			innovator_keypad_report(old, new);
			old = new;
		}
		timeout = interruptible_sleep_on_timeout(&keypadt_wait, 10);
		if (timeout) {
			break;
		}
	} while (new);

	return;
}

static int
innovator_keypad_thread(void *null)
{
	struct task_struct *tsk = current;
	unsigned long flags;

	daemonize();
	reparent_to_init();
	strcpy(tsk->comm, "keypadt");
	tsk->tty = NULL;

	/* only want to receive SIGKILL */
	spin_lock_irq(&tsk->sigmask_lock);
	siginitsetinv(&tsk->blocked, sigmask(SIGKILL));
	recalc_sigpending(tsk);
	spin_unlock_irq(&tsk->sigmask_lock);

	innovator_keypad_task = tsk;

	complete(&keypadt);

	DPRINTK("init\n");

	do {
		/* Careful, a dead lock can occur here if an keypad
		 * interrupt occurs before we're ready and waiting on
		 * the keypadt_wait queue. So we disable interrupts
		 * while unmasking device interrupts prior to putting
		 * the thread to sleep.
		 */
		local_irq_save(flags);
		DPRINTK("sleep\n");
		outw(0x0, OMAP1510_MPUIO_KBD_MASKIT);
		interruptible_sleep_on(&keypadt_wait);
		local_irq_restore(flags);
		if (signal_pending(tsk))
			break;
		DPRINTK("wake\n");
		innovator_keypad_scan();
	} while (!signal_pending(tsk));

	outw(0x1, OMAP1510_MPUIO_KBD_MASKIT);
	complete_and_exit(&keypadt, 0);
}

static void
innovator_keypad_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	outw(1, OMAP1510_MPUIO_KBD_MASKIT);

	DPRINTK("interrupt\n");

	/* Wake up thread to scan keypad. */
	wake_up(&keypadt_wait);
}

static int __init
innovator_keypad_init(void)
{
	printk(KERN_INFO "TI Innovator/OMAP1510 keypad driver.\n");

	outw(1, OMAP1510_MPUIO_KBD_MASKIT);
	outw(0xff, OMAP1510_GPIO_DEBOUNCING_REG);
	outw(0x0, OMAP1510_MPUIO_KBC_REG);

	keypad_dev.evbit[LONG(EV_KEY)] |= BIT(EV_KEY);
	keypad_dev.keybit[LONG(KEY_UP)] |= BIT(KEY_UP);
	keypad_dev.keybit[LONG(KEY_DOWN)] |= BIT(KEY_DOWN);
	keypad_dev.keybit[LONG(KEY_LEFT)] |= BIT(KEY_LEFT);
	keypad_dev.keybit[LONG(KEY_RIGHT)] |= BIT(KEY_RIGHT);
	keypad_dev.keybit[LONG(KEY_ENTER)] |= BIT(KEY_ENTER);
	keypad_dev.keybit[LONG(KEY_F1)] |= BIT(KEY_F1);
	keypad_dev.keybit[LONG(KEY_F2)] |= BIT(KEY_F2);
	keypad_dev.keybit[LONG(KEY_F3)] |= BIT(KEY_F3);
	keypad_dev.keybit[LONG(KEY_F4)] |= BIT(KEY_F4);

	input_register_device(&keypad_dev);

	init_waitqueue_head(&keypadt_wait);
	init_completion(&keypadt);
	innovator_keypad_pid =
	    kernel_thread(innovator_keypad_thread, NULL,
			  CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	if (innovator_keypad_pid <= 0) {
		return -1;
	}
	wait_for_completion(&keypadt);

	if (request_irq(INT_KEYBOARD, innovator_keypad_interrupt,
			SA_INTERRUPT, "keypad", 0) < 0) {
		/* Kill the thread */
		init_completion(&keypadt);
		send_sig(SIGKILL, innovator_keypad_task, 1);
		wake_up(&keypadt_wait);
		wait_for_completion(&keypadt);
		input_unregister_device(&keypad_dev);
		return -EINVAL;
	}

#if	defined(CONFIG_OMAP_INNOVATOR)	/* MVL-CEE */
	innovator_keypad_ldm_driver_register();
	innovator_keypad_ldm_device_register();
#endif				/* MVL-CEE */

	return 0;
}

static void __exit
innovator_keypad_exit(void)
{
	outw(1, OMAP1510_MPUIO_KBD_MASKIT);

#if	defined(CONFIG_OMAP_INNOVATOR)	/* MVL-CEE */
	innovator_keypad_ldm_device_unregister();
	innovator_keypad_ldm_driver_unregister();
#endif				/* MVL-CEE */

	/* Kill the thread */
	init_completion(&keypadt);
	send_sig(SIGKILL, innovator_keypad_task, 1);
	wake_up(&keypadt_wait);
	wait_for_completion(&keypadt);

	input_unregister_device(&keypad_dev);
	free_irq(INT_KEYBOARD, 0);

	return;
}

module_init(innovator_keypad_init);
module_exit(innovator_keypad_exit);

MODULE_AUTHOR("George G. Davis <gdavis@mvista.com>");
MODULE_DESCRIPTION("TI Innovator/OMAP1510 Keypad driver");
MODULE_LICENSE("GPL");
