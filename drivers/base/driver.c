/*
 * driver.c - centralized device driver management
 *
 * Copyright (c) 2002-3 Patrick Mochel
 * Copyright (c) 2002-3 Open Source Development Labs
 * 
 * This file is released under the GPLv2
 *
 */

#undef DEBUG

#include <linux/device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include "base.h"

#define to_dev(node) container_of(node,struct device,driver_list)
#define to_drv(obj) container_of(obj,struct device_driver,kobj)

/**
 *	driver_create_file - create sysfs file for driver.
 *	@drv:	driver.
 *	@attr:	driver attribute descriptor.
 */

int driver_create_file(struct device_driver * drv, struct driver_attribute * attr)
{
	int error;
	if (get_driver(drv)) {
		error = sysfs_create_file(&drv->kobj,&attr->attr);
		put_driver(drv);
	} else
		error = -EINVAL;
	return error;
}


/**
 *	driver_remove_file - remove sysfs file for driver.
 *	@drv:	driver.
 *	@attr:	driver attribute descriptor.
 */

void driver_remove_file(struct device_driver * drv, struct driver_attribute * attr)
{
	if (get_driver(drv)) {
		sysfs_remove_file(&drv->kobj,&attr->attr);
		put_driver(drv);
	}
}


/**
 *	get_driver - increment driver reference count.
 *	@drv:	driver.
 */
struct device_driver * get_driver(struct device_driver * drv)
{
	return drv ? to_drv(kobject_get(&drv->kobj)) : NULL;
}


/**
 *	put_driver - decrement driver's refcount.
 *	@drv:	driver.
 */
void put_driver(struct device_driver * drv)
{
	kobject_put(&drv->kobj);
}


/**
 *	driver_register - register driver with bus
 *	@drv:	driver to register
 *
 *	We pass off most of the work to the bus_add_driver() call,
 *	since most of the things we have to do deal with the bus 
 *	structures.
 *
 *	The one interesting aspect is that we initialize @drv->unload_sem
 *	to a locked state here. It will be unlocked when the driver
 *	reference count reaches 0.
 */
int driver_register(struct device_driver * drv)
{
	INIT_LIST_HEAD(&drv->devices);
	init_MUTEX_LOCKED(&drv->unload_sem);
	return bus_add_driver(drv);
}


/**
 *	driver_unregister - remove driver from system.
 *	@drv:	driver.
 *
 *	Again, we pass off most of the work to the bus-level call.
 *
 *	Though, once that is done, we attempt to take @drv->unload_sem.
 *	This will block until the driver refcount reaches 0, and it is
 *	released. Only modular drivers will call this function, and we 
 *	have to guarantee that it won't complete, letting the driver 
 *	unload until all references are gone.
 */

void driver_unregister(struct device_driver * drv)
{
	bus_remove_driver(drv);
	down(&drv->unload_sem);
	up(&drv->unload_sem);
}

#if 1 /* linux-pm */
static int device_power_onoff(struct device *dev, u32 level)
{
	struct device_driver *drv = dev->driver;

	switch(level) {
	case DPM_POWER_ON:
		if (drv->resume)
			drv->resume(dev, RESUME_POWER_ON);
		break;
	case DPM_POWER_OFF:
	case DPM_SUSPEND_FOR_OP:
		if (drv->suspend)
			drv->suspend(dev, 0, SUSPEND_POWER_DOWN);
		break;
	}

	/*
	 * If the device has been previously explicitly powered off
	 * then leave that state intact in case we're suspending due to
	 * scaling to an incompatible operating point.
	 */

	if ((level != DPM_SUSPEND_FOR_OP) ||
	    (dev->power_state != DPM_POWER_OFF))
		dev->power_state = level;

	return 0;
}

int driver_powerup(struct device_driver *drv, struct device *dev, u32 level)
{
	int valid = validate_constraints(drv->bus, drv->constraints);

	if (dev)
		device_power_onoff(dev, valid ? level : DPM_SUSPEND_FOR_OP);
	else {
		struct list_head * entry, * next;

		list_for_each_safe(entry,next,&drv->devices) {
			dev = container_of(entry,struct device,driver_list);
			device_power_onoff(dev, valid ? level : 
					   DPM_SUSPEND_FOR_OP);
		}
	}

	if (drv->constraints && (drv->constraints->asserted != valid)) {
		drv->constraints->asserted = valid;

		if (valid)
			assert_constraints(drv->bus, drv->constraints);
		else
			bus_reeval_constraints(drv->bus);
	} 

	return 0;
}

int driver_powerdown(struct device_driver *drv, struct device *dev, u32 level)
{
	if (dev)
		device_power_onoff(dev, level);
	else {
		struct list_head * entry, * next;

		list_for_each_safe(entry,next,&drv->devices) {
			dev = container_of(entry,struct device,driver_list);
			device_power_onoff(dev, level);
		}
	}

	if (drv->constraints && drv->constraints->asserted) {
		drv->constraints->asserted = 0;
		bus_reeval_constraints(drv->bus);
	} 

	return 0;
}

static int device_op_check(struct device * dev)
{
	if (dev->power_state == DPM_SUSPEND_FOR_OP) {
		device_power_onoff(dev, DPM_POWER_ON);

		if (dev->driver->constraints && 
		    ! dev->driver->constraints->asserted) {
			dev->driver->constraints->asserted = 1;
			assert_constraints(dev->bus, dev->driver->constraints);
		} 
	}

	return 0;
}

int driver_scale(struct device_driver * drv, void * data)
{
	int level = (u32) data;
	int valid = validate_constraints(drv->bus, drv->constraints);
	int error = 0;

	if (drv->scale)
		error = drv->scale(drv->bus->bus_op, level);

	if (! valid && (level == DPM_SCALE))
		driver_powerdown(drv, NULL, DPM_SUSPEND_FOR_OP);
	else {
		struct list_head * entry, * next;
		struct device *dev;

		list_for_each_safe(entry,next,&drv->devices) {
			dev = container_of(entry,struct device,driver_list);
			device_op_check(dev);
		}
	}

        return error;
}

#endif /* linux-pm */

EXPORT_SYMBOL(driver_register);
EXPORT_SYMBOL(driver_unregister);
EXPORT_SYMBOL(get_driver);
EXPORT_SYMBOL(put_driver);

EXPORT_SYMBOL(driver_create_file);
EXPORT_SYMBOL(driver_remove_file);
