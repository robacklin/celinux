/*
 * power.c - power management functions for the device tree.
 * 
 * Copyright (c) 2002-3 Patrick Mochel
 *		 2002-3 Open Source Development Lab
 * 
 * This file is released under the GPLv2
 * 
 *  Kai Germaschewski contributed to the list walking routines.
 *
 */

#if 0 /* linux-pm */
#define DEBUG
#endif /* linux-pm */

#include <linux/device.h>
#include <linux/module.h>
#include <asm/semaphore.h>
#include "base.h"
#if 1 /* linux-pm */
#include <linux/sched.h>
#endif /* linux-pm */

#define to_dev(node) container_of(node,struct device,kobj.entry)

extern struct subsystem devices_subsys;

/**
 * device_suspend - suspend/remove all devices on the device ree
 * @state:	state we're entering
 * @level:	what stage of the suspend process we're at
 *    (emb: it seems that these two arguments are described backwards of what
 *          they actually mean .. is this correct?)
 *
 * The entries in the global device list are inserted such that they're in a
 * depth-first ordering.  So, simply interate over the list, and call the 
 * driver's suspend or remove callback for each device.
 */
int device_suspend(u32 state, u32 level)
{
	struct list_head * node;
	int error = 0;

#if 0 /* linux-pm */
	printk(KERN_EMERG "Suspending devices\n");
#endif /* linux-pm */

	down_write(&devices_subsys.rwsem);
	list_for_each_prev(node,&devices_subsys.kset.list) {
		struct device * dev = to_dev(node);
		if (dev->driver && dev->driver->suspend) {
			pr_debug("suspending device %s\n",dev->name);
			error = dev->driver->suspend(dev,state,level);
			if (error)
				printk(KERN_ERR "%s: suspend returned %d\n",dev->name,error);
		}
	}
	up_write(&devices_subsys.rwsem);
	return error;
}

/**
 * device_resume - resume all the devices in the system
 * @level:	stage of resume process we're at 
 * 
 * Similar to device_suspend above, though we want to do a breadth-first
 * walk of the tree to make sure we wake up parents before children.
 * So, we iterate over the list backward. 
 */
void device_resume(u32 level)
{
	struct list_head * node;

	down_write(&devices_subsys.rwsem);
	list_for_each(node,&devices_subsys.kset.list) {
		struct device * dev = to_dev(node);
		if (dev->driver && dev->driver->resume) {
			pr_debug("resuming device %s\n",dev->name);
			dev->driver->resume(dev,level);
#if 1 /* linux-pm */
                        /*
                         * In case the driver resume code does something that
                         * sends a signal to the resume thread (like a
                         * SIGCHLD for /sbin/hotplug exit), flush it so the
                         * next driver's resume code can issue blocking
                         * operations without the signal interfering.
                         */

                        flush_signals(current);
#endif /* linux-pm */
		}
	}
	up_write(&devices_subsys.rwsem);

#if 0 /* linux-pm */
	printk(KERN_EMERG "Devices Resumed\n");
#endif /* linux-pm */
}

/**
 * device_shutdown - call ->remove() on each device to shutdown. 
 */
void device_shutdown(void)
{
	struct list_head * entry;
	
	printk(KERN_EMERG "Shutting down devices\n");

	down_write(&devices_subsys.rwsem);
	list_for_each(entry,&devices_subsys.kset.list) {
		struct device * dev = to_dev(entry);
		pr_debug("shutting down %s: ",dev->name);
		if (dev->driver && dev->driver->shutdown) {
			pr_debug("Ok\n");
			dev->driver->shutdown(dev);
		} else
			pr_debug("Ignored.\n");
	}
	up_write(&devices_subsys.rwsem);
}

#if 1 /* linux-pm */
#include <linux/sched.h>
#include <linux/init.h>

/*
 * Threaded driver resume stuff...
 */

static int kdpmd_pid;
static DECLARE_COMPLETION(kdpmd_complete);
static int kdpmd_req;
static wait_queue_head_t kdpmd_wait;

static void kdpmd_suspend_devices(void)
{
	unsigned long flags;

	local_irq_save(flags);
	device_suspend(0, SUSPEND_NOTIFY);
	current->state = TASK_UNINTERRUPTIBLE;
	schedule_timeout(2);
	device_suspend(0, SUSPEND_SAVE_STATE);
	device_suspend(0, SUSPEND_POWER_DOWN);
	local_irq_restore(flags);
}

static void kdpmd_resume_devices(void)
{
	unsigned long flags;

	local_irq_save(flags);
	device_resume(RESUME_RESTORE_STATE);
	device_resume(RESUME_POWER_ON);
	local_irq_restore(flags);
}

static int kdpmd_thread(void *arg)
{
	struct task_struct *tsk = current;

	/*
	 * We don't want /any/ signals, not even SIGKILL
	 */
	sigfillset(&tsk->blocked);
	sigemptyset(&tsk->pending.signal);
	recalc_sigpending(tsk);
	daemonize();
	reparent_to_init();
	strcpy(tsk->comm, "kdpmd");
	tsk->tty = NULL;

	while (1) {
		int req;

		do {
			req = xchg(&kdpmd_req, 0);
			if (req == 0) {
				sigemptyset(&tsk->pending.signal);
				interruptible_sleep_on(&kdpmd_wait);
			}
		} while (req == 0);

		if (req == SIGTERM) {
			break;
		} else if (req == SIGUSR1) {
			kdpmd_suspend_devices();
			complete(&kdpmd_complete);
		} else if (req == SIGHUP) {
			kdpmd_resume_devices();
			complete(&kdpmd_complete);
		} else {
			printk(KERN_DEBUG "%s: Unknown Request: %d\n",
			       __FUNCTION__, req);
		}
	}

	complete_and_exit(&kdpmd_complete, 0);
}


void dpm_device_suspend(void)
{
	/* SIGUSR1 to kdpmd signals device suspend. */

	init_completion(&kdpmd_complete);
	kdpmd_req = SIGUSR1;
	wake_up(&kdpmd_wait);
	wait_for_completion(&kdpmd_complete);
}

void dpm_device_resume(void)
{
	/* SIGHUP to kdpmd signals device resume. */

	init_completion(&kdpmd_complete);
	kdpmd_req = SIGHUP;
	wake_up(&kdpmd_wait);
}

void __init dpm_kdpmd_init(void)
{
	init_waitqueue_head(&kdpmd_wait);
	init_completion(&kdpmd_complete);
	kdpmd_req = 0;
	if ((kdpmd_pid = kernel_thread(kdpmd_thread,
					  (void *)NULL,
					  CLONE_FS | CLONE_FILES |
					  CLONE_SIGHAND)) <= 0) {
		return;
	}

	return;
}

int validate_constraints(struct bus_type *bus, struct constraints *constraints)
{
	int i, j;

	if (! constraints || ! bus->bus_op)
		return 1;

	for (i = 0; i < constraints->count; i++) {
		for (j = 0; j < bus->bus_op->count; j++) {
			if (constraints->param[i].id == 
			    bus->bus_op->param[j].id)
				if ((constraints->param[i].min >
				     bus->bus_op->param[j].val) ||
				    (constraints->param[i].max <
				     bus->bus_op->param[j].val))
					return 0;
		}
	}

	return 1;
}

int device_powerup(struct device *dev)
{
	return driver_powerup(dev->driver, dev, DPM_POWER_ON);
}

int device_powerdown(struct device *dev)
{
	return driver_powerdown(dev->driver, dev, DPM_POWER_OFF);
}

EXPORT_SYMBOL(validate_constraints);
EXPORT_SYMBOL(device_powerup);
EXPORT_SYMBOL(device_powerdown);
#endif /* linux-pm */

EXPORT_SYMBOL(device_suspend);
EXPORT_SYMBOL(device_resume);
EXPORT_SYMBOL(device_shutdown);
