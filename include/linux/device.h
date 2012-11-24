/*
 * device.h - generic, centralized driver model
 *
 * Copyright (c) 2001-2003 Patrick Mochel <mochel@osdl.org>
 *
 * This file is released under the GPLv2
 *
 * See Documentation/driver-model/ for more information.
 */

#ifndef _DEVICE_H_
#define _DEVICE_H_

#include <linux/config.h>
#include <linux/ioport.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <asm/semaphore.h>
#include <asm/atomic.h>

#if 1 /* linux-pm */
#include <linux/stringify.h>
#include <linux/string.h>

#ifndef __HAVE_ARCH_STRLCPY
static inline size_t strlcpy(char *dest, const char *src, size_t size)
{
        size_t ret = strlen(src);

        if (size) {
                size_t len = (ret >= size) ? size-1 : ret;
                memcpy(dest, src, len);
                dest[len] = '\0';
        }
        return ret;
}
#endif

/**
 * container_of - cast a member of a structure out to the containing structure
 *
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({			\
        const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
        (type *)( (char *)__mptr - offsetof(type,member) );})

#endif /* linux-pm */

#define DEVICE_NAME_SIZE	50
#define DEVICE_NAME_HALF	__stringify(20)	/* Less than half to accommodate slop */
#define DEVICE_ID_SIZE		32
#define BUS_ID_SIZE		KOBJ_NAME_LEN


enum {
	SUSPEND_NOTIFY,
	SUSPEND_SAVE_STATE,
	SUSPEND_DISABLE,
	SUSPEND_POWER_DOWN,
};

enum {
	RESUME_POWER_ON,
	RESUME_RESTORE_STATE,
	RESUME_ENABLE,
};

enum device_state {
	DEVICE_UNINITIALIZED	= 0,
	DEVICE_INITIALIZED	= 1,
	DEVICE_REGISTERED	= 2,
	DEVICE_GONE		= 3,
};

#if 1 /* linux-pm */
enum {
	DPM_POWER_ON,
	DPM_POWER_OFF,
	DPM_SUSPEND_FOR_OP,
};

enum {
	DPM_SCALE_NOTIFY,
	DPM_SCALE,
	DPM_SCALE_DISABLE,
};

enum {
	DPM_PARAM_BUS_FREQ,
	DPM_PARAM_EXT_CLOCK,
};

#define DPM_PARAM_MAX DPM_PARAM_EXT_CLOCK + 1

struct bus_op_param {
	int id;
	int val;
};

struct bus_op_point {
	int count;
	struct bus_op_param param[DPM_PARAM_MAX];
};

struct constraint_param {
	int id;
	int min;
	int max;
};

struct constraints {
	int asserted;
	int count;
	struct constraint_param param[DPM_PARAM_MAX];
};
#endif /* linux-pm */

struct device;
struct device_driver;
struct class;
struct class_device;

struct bus_type {
	char			* name;

	struct subsystem	subsys;
	struct kset		drivers;
	struct kset		devices;

#if 1 /* linux-pm */
	struct constraints	* constraints;
	struct bus_op_point	* bus_op;
#endif /* linux-pm */

	int		(*match)(struct device * dev, struct device_driver * drv);
	struct device * (*add)	(struct device * parent, char * bus_id);
	int		(*hotplug) (struct device *dev, char **envp, 
				    int num_envp, char *buffer, int buffer_size);
};


extern int bus_register(struct bus_type * bus);
extern void bus_unregister(struct bus_type * bus);

extern int bus_rescan_devices(struct bus_type * bus);

extern struct bus_type * get_bus(struct bus_type * bus);
extern void put_bus(struct bus_type * bus);

extern struct bus_type * find_bus(char * name);

/* iterator helpers for buses */

int bus_for_each_dev(struct bus_type * bus, struct device * start, void * data,
		     int (*fn)(struct device *, void *));

int bus_for_each_drv(struct bus_type * bus, struct device_driver * start, 
		     void * data, int (*fn)(struct device_driver *, void *));


/* driverfs interface for exporting bus attributes */

struct bus_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct bus_type *, char * buf);
	ssize_t (*store)(struct bus_type *, const char * buf, size_t count);
};

#define BUS_ATTR(_name,_mode,_show,_store)	\
struct bus_attribute bus_attr_##_name = { 		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,				\
	.store	= _store,				\
};

extern int bus_create_file(struct bus_type *, struct bus_attribute *);
extern void bus_remove_file(struct bus_type *, struct bus_attribute *);

struct device_driver {
	char			* name;
	struct bus_type		* bus;
#if 1 /* linux-pm temp: devclass is obsolete */
        struct device_class     * devclass;
#endif /* linux-pm */

	struct semaphore	unload_sem;
	struct kobject		kobj;
	struct list_head	devices;

#if 1 /* linux-pm */
	struct constraints	* constraints;
#endif /* linux-pm */

	int	(*probe)	(struct device * dev);
	int 	(*remove)	(struct device * dev);
	void	(*shutdown)	(struct device * dev);
	int	(*suspend)	(struct device * dev, u32 state, u32 level);
	int	(*resume)	(struct device * dev, u32 level);

#if 1 /* linux-pm */
	int	(*scale)	(struct bus_op_point * op, u32 level);
#endif /* linux-pm */
};


extern int driver_register(struct device_driver * drv);
extern void driver_unregister(struct device_driver * drv);

extern struct device_driver * get_driver(struct device_driver * drv);
extern void put_driver(struct device_driver * drv);


/* driverfs interface for exporting driver attributes */

struct driver_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct device_driver *, char * buf);
	ssize_t (*store)(struct device_driver *, const char * buf, size_t count);
};

#define DRIVER_ATTR(_name,_mode,_show,_store)	\
struct driver_attribute driver_attr_##_name = { 		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,				\
	.store	= _store,				\
};

extern int driver_create_file(struct device_driver *, struct driver_attribute *);
extern void driver_remove_file(struct device_driver *, struct driver_attribute *);


/*
 * device classes
 */
struct class {
	char			* name;

	struct subsystem	subsys;
	struct list_head	children;
	struct list_head	interfaces;

	int	(*hotplug)(struct class_device *dev, char **envp, 
			   int num_envp, char *buffer, int buffer_size);

	void	(*release)(struct class_device *dev);
};

extern int class_register(struct class *);
extern void class_unregister(struct class *);

extern struct class * class_get(struct class *);
extern void class_put(struct class *);


struct class_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct class *, char * buf);
	ssize_t (*store)(struct class *, const char * buf, size_t count);
};

#define CLASS_ATTR(_name,_mode,_show,_store)			\
struct class_attribute class_attr_##_name = { 			\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,					\
	.store	= _store,					\
};

extern int class_create_file(struct class *, struct class_attribute *);
extern void class_remove_file(struct class *, struct class_attribute *);


struct class_device {
	struct list_head	node;

	struct kobject		kobj;
	struct class		* class;	/* required */
	struct device		* dev;		/* not necessary, but nice to have */
	void			* class_data;	/* class-specific data */

	char	class_id[BUS_ID_SIZE];	/* unique to this class */
};

static inline void *
class_get_devdata (struct class_device *dev)
{
	return dev->class_data;
}

static inline void
class_set_devdata (struct class_device *dev, void *data)
{
	dev->class_data = data;
}


extern int class_device_register(struct class_device *);
extern void class_device_unregister(struct class_device *);
extern void class_device_initialize(struct class_device *);
extern int class_device_add(struct class_device *);
extern void class_device_del(struct class_device *);

extern struct class_device * class_device_get(struct class_device *);
extern void class_device_put(struct class_device *);

struct class_device_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct class_device *, char * buf);
	ssize_t (*store)(struct class_device *, const char * buf, size_t count);
};

#define CLASS_DEVICE_ATTR(_name,_mode,_show,_store)		\
struct class_device_attribute class_device_attr_##_name = { 	\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,					\
	.store	= _store,					\
};

extern int class_device_create_file(struct class_device *, struct class_device_attribute *);
extern void class_device_remove_file(struct class_device *, struct class_device_attribute *);


struct class_interface {
	struct list_head	node;
	struct class		*class;

	int (*add)	(struct class_device *);
	void (*remove)	(struct class_device *);
};

extern int class_interface_register(struct class_interface *);
extern void class_interface_unregister(struct class_interface *);


struct device {
	struct list_head node;		/* node in sibling list */
	struct list_head bus_list;	/* node in bus's list */
	struct list_head driver_list;
	struct list_head children;
	struct device 	* parent;

	struct kobject kobj;
	char	name[DEVICE_NAME_SIZE];	/* descriptive ascii string */
	char	bus_id[BUS_ID_SIZE];	/* position on parent bus */

	struct bus_type	* bus;		/* type of bus device is on */
	struct device_driver *driver;	/* which driver has allocated this
					   device */
	void		*driver_data;	/* data private to the driver */
	void		*platform_data;	/* Platform specific data (e.g. ACPI,
					   BIOS data relevant to device) */

	u32		power_state;	/* Current operating state. In
					   ACPI-speak, this is D0-D3, D0
					   being fully functional, and D3
					   being off. */

	unsigned char *saved_state;	/* saved device state */
	u64		*dma_mask;	/* dma mask (if dma'able device) */

	void	(*release)(struct device * dev);
};

static inline struct device *
list_to_dev(struct list_head *node)
{
	return list_entry(node, struct device, node);
}

static inline void *
dev_get_drvdata (struct device *dev)
{
	return dev->driver_data;
}

static inline void
dev_set_drvdata (struct device *dev, void *data)
{
	dev->driver_data = data;
}

/*
 * High level routines for use by the bus drivers
 */
extern int device_register(struct device * dev);
extern void device_unregister(struct device * dev);
extern void device_initialize(struct device * dev);
extern int device_add(struct device * dev);
extern void device_del(struct device * dev);
extern int device_for_each_child(struct device *, void *,
		     int (*fn)(struct device *, void *));

/*
 * Manual binding of a device to driver. See drivers/base/bus.c 
 * for information on use.
 */
extern void device_bind_driver(struct device * dev);
extern void device_release_driver(struct device * dev);
extern void driver_attach(struct device_driver * drv);


/* driverfs interface for exporting device attributes */

struct device_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct device * dev, char * buf);
	ssize_t (*store)(struct device * dev, const char * buf, size_t count);
};

#define DEVICE_ATTR(_name,_mode,_show,_store) \
struct device_attribute dev_attr_##_name = { 		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,				\
	.store	= _store,				\
};


extern int device_create_file(struct device *device, struct device_attribute * entry);
extern void device_remove_file(struct device * dev, struct device_attribute * attr);

/*
 * Platform "fixup" functions - allow the platform to have their say
 * about devices and actions that the general device layer doesn't
 * know about.
 */
/* Notify platform of device discovery */
extern int (*platform_notify)(struct device * dev);

extern int (*platform_notify_remove)(struct device * dev);


/**
 * get_device - atomically increment the reference count for the device.
 *
 */
extern struct device * get_device(struct device * dev);
extern void put_device(struct device * dev);

/* drivers/base/sys.c */

struct sys_root {
	u32		id;
	struct device 	dev;
	struct device	sysdev;
};

extern int sys_register_root(struct sys_root *);
extern void sys_unregister_root(struct sys_root *);


struct sys_device {
	char		* name;
	u32		id;
	struct sys_root	* root;
	struct device	dev;
	struct class_device class_dev;
};

extern int sys_device_register(struct sys_device *);
extern void sys_device_unregister(struct sys_device *);

extern struct bus_type system_bus_type;

/* drivers/base/platform.c */

struct platform_device {
	char		* name;
	u32		id;
	struct device	dev;
	u32		num_resources;
	struct resource	* resource;
};

#define to_platform_device(x) container_of((x), struct platform_device, dev)

extern int platform_device_register(struct platform_device *);
extern void platform_device_unregister(struct platform_device *);

extern struct bus_type platform_bus_type;
extern struct device legacy_bus;

/* drivers/base/power.c */
extern int device_suspend(u32 state, u32 level);
extern void device_resume(u32 level);
extern void device_shutdown(void);


/* drivers/base/firmware.c */
extern int firmware_register(struct subsystem *);
extern void firmware_unregister(struct subsystem *);

/* debugging and troubleshooting/diagnostic helpers. */
#define dev_printk(level, dev, format, arg...)	\
	printk(level "%s %s: " format , (dev)->driver->name , (dev)->bus_id , ## arg)

#ifdef DEBUG
#define dev_dbg(dev, format, arg...)		\
	dev_printk(KERN_DEBUG , dev , format , ## arg)
#else
#define dev_dbg(dev, format, arg...) do {} while (0)
#endif

#define dev_err(dev, format, arg...)		\
	dev_printk(KERN_ERR , dev , format , ## arg)
#define dev_info(dev, format, arg...)		\
	dev_printk(KERN_INFO , dev , format , ## arg)
#define dev_warn(dev, format, arg...)		\
	dev_printk(KERN_WARNING , dev , format , ## arg)

#if 1 /* linux-pm */
extern int bus_scale(struct bus_type * bus, struct bus_op_point * op);
extern void assert_constraints(struct bus_type *bus, 
			       struct constraints *dev_constraints);
extern void deassert_constraints(struct bus_type *bus, 
				 struct constraints *drv_constraints);
extern int validate_constraints(struct bus_type *bus, 
				struct constraints *constraints);
extern void bus_reeval_constraints(struct bus_type * bus);
extern int driver_powerup(struct device_driver *drv, struct device *dev,
			  u32 level);
extern int driver_powerdown(struct device_driver *drv, struct device *dev,
			    u32 level);
extern int device_powerup(struct device *dev);
extern int device_powerdown(struct device *dev);
extern int driver_scale(struct device_driver * drv, void * data);
#endif /* linux-pm */

#endif /* _DEVICE_H_ */
