/*
 * linux/arch/arm/mach-s3c2440/s3c2440-ldm.c
 *
 * Copyright (C) 2003 Samsung Electronics. 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include <asm/system.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/module.h>

static int s3c2440_bus_match(struct device *dev, struct device_driver *drv);

struct bus_type ahb_bus = {
	name: 	"ahb-bus",
	match: 	s3c2440_bus_match,
};

struct device ahb_device = {
	name: 	"AHB Peripheral BUS Controller",
	bus_id: "ahb-bus",
	parent: NULL,
	bus: 	&ahb_bus,
	driver:	NULL,
};

struct bus_type apb_bus = {
	name: 	"apb-bus",
	match: 	s3c2440_bus_match,
};

struct device apb_device = {
	name: 	"APB Peripheral BUS Controller",
	bus_id: "apb-bus",
	parent: NULL,
	bus: 	&apb_bus,
	driver:	NULL,
};

static int 
s3c2440_bus_match(struct device *dev, struct device_driver *drv)
{
	if (!strcmp(dev->bus_id, "lcd"))
		return !strcmp(drv->name, "s3c2440fb");
	
	return 0;
}

void
ahb_bus_register(struct device *device, struct device_driver *driver)
{
	if(driver) {
		driver->bus = &ahb_bus;
		(void) driver_register(driver);
	}
	if(device) {
		device->parent = &ahb_device;
		device->bus = &ahb_bus;
		device_register(device);
	}
	return;
}

void
ahb_bus_unregister(struct device *device, struct device_driver *driver)
{
	if(driver)
		(void) driver_unregister(driver);
	if(device)
		device_unregister(device);
	return;
}

void
apb_bus_register(struct device *device, struct device_driver *driver)
{
	if(driver) {
		driver->bus = &apb_bus;
		(void) driver_register(driver);
	}
	if(device) {
		device->parent = &apb_device;
		device->bus = &apb_bus;
		device_register(device);
	}
	return;
}

void
apb_bus_unregister(struct device *device, struct device_driver *driver)
{
	if(driver)
		(void) driver_unregister(driver);
	if(device)
		device_unregister(device);
	return;
}

void __init s3c2440_ldm_setup(void)
{
	bus_register(&ahb_bus);
	device_register(&ahb_device);
	bus_register(&apb_bus);
	device_register(&apb_device);
	return;
}

EXPORT_SYMBOL(ahb_bus_register);
EXPORT_SYMBOL(ahb_bus_unregister);
EXPORT_SYMBOL(apb_bus_register);
EXPORT_SYMBOL(apb_bus_unregister);

