/*
 *
 * BRIEF MODULE DESCRIPTION
 *	IT8172-Specific LDM support.
 * 
 * Copyright 2003 Sony Corporation 
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Author: MontaVista Software, Inc.
 *     source@mvista.com
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
 */

#include <asm/system.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>

/*
 * PCI Bus
 */

static int pci_bus_match(struct device *dev, struct device_driver *drv);

static struct constraints pci_constraints = {
	count:		0,
};

static struct bus_op_point pci_op = {
	count:		0,
};

struct bus_type pci_bus = {
        name:		"pci",
        match:		pci_bus_match,
	constraints:	&pci_constraints,
	bus_op:		&pci_op,
};

struct device pci_device = {
	name:		"PCI Bus Controller",
	bus_id:		"pci",
	parent:		NULL,
	bus:		&pci_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

static int
pci_bus_match(struct device *dev, struct device_driver *drv)
{
  if (strcmp(dev->bus_id, "itefb") == 0)
 	return (strcmp(drv->name, "ite8181-frame-buffer") == 0);
  if (strcmp(dev->bus_id, "audio") == 0)
  	return (strcmp(drv->name, "ite8172-audio") == 0); 
  if (strcmp(dev->bus_id, "ide") == 0)
  	return (strcmp(drv->name, "ite8172-ide-cont") == 0);
  if (strcmp(dev->bus_id, "hc_ohci") == 0)
  	return (strcmp(drv->name, "USB_HC_ITE8172") == 0); 
  if (strcmp(dev->bus_id, "rtl8139") == 0)
  	return (strcmp(drv->name, "RTL8139_Drv") == 0);

	return 0;
}

void pci_device_ldm_register(struct device *device)
{
	device->parent = &pci_device;
	device->bus = &pci_bus;
	device_register(device);
	return;
}

void pci_driver_ldm_register(struct device_driver *driver)
{
	driver->bus = &pci_bus;
	(void) driver_register(driver);
	return;
}

void pci_driver_ldm_unregister(struct device_driver *driver)
{
        driver_unregister(driver);  
        return;
}

void pci_device_ldm_unregister(struct device *device)
{
        device_unregister(device);
        return;
}

/*
 * IHB Internal Host Bus
 */

static int ihb_bus_match(struct device *dev, struct device_driver *drv);

static struct constraints ihb_constraints = {
	count:		0,
};

static struct bus_op_point ihb_op = {
	count:		0,
};

struct bus_type ihb_bus = {
        name:		"ihb",
        match:		ihb_bus_match,
	constraints:	&ihb_constraints,
	bus_op:		&ihb_op,
};

struct device ihb_device = {
	name:		"IHB Internal Host Bus Controller",
	bus_id:		"ihb",
	parent:		NULL,
	bus:		&ihb_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

static int
ihb_bus_match(struct device *dev, struct device_driver *drv)
{
  if (strncmp(dev->bus_id, "uart", 4) == 0)
  	return (strcmp(drv->name, "serial") == 0);
  if (strcmp(dev->bus_id, "I2C") == 0)
  	return (strcmp(drv->name, "i2c-adap-ite") == 0);	
  
	return 0;
}

void ihb_device_ldm_register(struct device *device)
{
	device->parent = &ihb_device;
	device->bus = &ihb_bus;
	device_register(device);
	return;
}

void ihb_driver_ldm_register(struct device_driver *driver)
{
	driver->bus = &ihb_bus;
	(void) driver_register(driver);
	return;
}

void ihb_device_ldm_unregister(struct device *device)
{
        device_unregister(device);  
        return;
}

void ihb_driver_ldm_unregister(struct device_driver *driver)
{
        driver_unregister(driver);  
        return;
}

/*
 * PCB Peripheral Control Bus Controller
 */

static int pcb_bus_match(struct device *dev, struct device_driver *drv);

static struct constraints pcb_constraints = {
	count:		0,
};

static struct bus_op_point pcb_op = {
	count:		0,
};

struct bus_type pcb_bus = {
        name:		"pcb",
        match:		pcb_bus_match,
	constraints:	&pcb_constraints,
	bus_op:		&pcb_op,
};

struct device pcb_device = {
	name:		"PCB Peripheral Control Bus Controller",
	bus_id:		"pcb",
	parent:		NULL,
	bus:		&pcb_bus,
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

static int
pcb_bus_match(struct device *dev, struct device_driver *drv)
{
 
	return 0;
}

void pcb_device_ldm_register(struct device *device)
{
	device->parent = &pcb_device;
	device->bus = &pcb_bus;
	device_register(device);
	return;
}

void pcb_driver_ldm_register(struct device_driver *driver)
{
	driver->bus = &pcb_bus;
	(void) driver_register(driver);
	return;
}

void pcb_device_ldm_unregister(struct device *device)
{
        device_unregister(device);  
        return;
}

void pcb_driver_ldm_unregister(struct device_driver *driver)
{
        driver_unregister(driver);  
        return;
}

void __init it8172_ldm_init(void)
{
	bus_register(&pci_bus);
	device_register(&pci_device);
	bus_register(&ihb_bus);
	device_register(&ihb_device);
        bus_register(&pcb_bus);
        device_register(&pcb_device);
	return;
}

EXPORT_SYMBOL(pci_device_ldm_register);
EXPORT_SYMBOL(pci_driver_ldm_register);
EXPORT_SYMBOL(pci_device_ldm_unregister);
EXPORT_SYMBOL(pci_driver_ldm_unregister);
EXPORT_SYMBOL(ihb_device_ldm_register);
EXPORT_SYMBOL(ihb_driver_ldm_register);
EXPORT_SYMBOL(ihb_device_ldm_unregister);
EXPORT_SYMBOL(ihb_driver_ldm_unregister);
EXPORT_SYMBOL(pcb_device_ldm_register);
EXPORT_SYMBOL(pcb_driver_ldm_register);
EXPORT_SYMBOL(pcb_device_ldm_unregister);
EXPORT_SYMBOL(pcb_driver_ldm_unregister);
