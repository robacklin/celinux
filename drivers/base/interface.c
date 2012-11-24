/*
 * drivers/base/interface.c - common driverfs interface that's exported to 
 * 	the world for all devices.
 *
 * Copyright (c) 2002-3 Patrick Mochel
 * Copyright (c) 2002-3 Open Source Development Labs
 * 
 * This file is released under the GPLv2
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/stat.h>
#include <linux/string.h>
#if 1 /* linux-pm */
#include <linux/mm.h>
#include <linux/slab.h>
#endif /* linux-pm */

static ssize_t device_read_name(struct device * dev, char * buf)
{
	return sprintf(buf,"%s\n",dev->name);
}

static DEVICE_ATTR(name,S_IRUGO,device_read_name,NULL);

static ssize_t
device_read_power(struct device * dev, char * page)
{
	return sprintf(page,"%d\n",dev->power_state);
}

static ssize_t
device_write_power(struct device * dev, const char * buf, size_t count)
{
	char	str_command[20];
	char	str_level[20];
	int	num_args;
	u32	state;
	u32	int_level;
	int	error = 0;

	if (!dev->driver)
		goto done;

	num_args = sscanf(buf,"%10s %10s %u",str_command,str_level,&state);

	error = -EINVAL;

	if (!num_args)
		goto done;

	if (!strnicmp(str_command,"suspend",7)) {
		if (num_args != 3)
			goto done;
		if (!strnicmp(str_level,"notify",6))
			int_level = SUSPEND_NOTIFY;
		else if (!strnicmp(str_level,"save",4))
			int_level = SUSPEND_SAVE_STATE;
		else if (!strnicmp(str_level,"disable",7))
			int_level = SUSPEND_DISABLE;
		else if (!strnicmp(str_level,"powerdown",8))
			int_level = SUSPEND_POWER_DOWN;
		else
			goto done;

		if (dev->driver->suspend)
			error = dev->driver->suspend(dev,state,int_level);
		else
			error = 0;
	} else if (!strnicmp(str_command,"resume",6)) {
		if (num_args != 2)
			goto done;

		if (!strnicmp(str_level,"poweron",7))
			int_level = RESUME_POWER_ON;
		else if (!strnicmp(str_level,"restore",7))
			int_level = RESUME_RESTORE_STATE;
		else if (!strnicmp(str_level,"enable",6))
			int_level = RESUME_ENABLE;
		else
			goto done;

		if (dev->driver->resume)
			error = dev->driver->resume(dev,int_level);
		else
			error = 0;
	}
 done:
	return error < 0 ? error : count;
}

static DEVICE_ATTR(power,S_IWUSR | S_IRUGO,
		   device_read_power,device_write_power);

#if 1 /* linux-pm */
static ssize_t 
device_read_constraints(struct device * dev, char * buf)
{
	int i, cnt = 0;
	
	if (dev->driver && dev->driver->constraints) {
		for (i = 0; i < DPM_PARAM_MAX; i++) {
			cnt += sprintf(buf + cnt,"%d %d %d\n",
				       dev->driver->constraints->param[i].id,
				       dev->driver->constraints->param[i].max,
				       dev->driver->constraints->param[i].min);
		}
	}

	return cnt;
}

static ssize_t
device_write_constraints(struct device * dev, const char * buf, size_t count)
{
	int num_args, id, max, min;

	if (! dev->driver)
		return -EINVAL;

	if(!dev->driver->constraints) {
		dev->driver->constraints = kmalloc(sizeof(struct constraints), GFP_KERNEL);

		if (! dev->driver->constraints)
			return -EINVAL;

		memset(dev->driver->constraints, 0, sizeof(struct constraints));
	}

	num_args = sscanf(buf,"%d %d %d",&id,&max, &min);

	if (num_args != 3 || id > DPM_PARAM_MAX)
		return -EINVAL;

	dev->driver->constraints->param[id].id = id; 
	dev->driver->constraints->param[id].max = max; 
	dev->driver->constraints->param[id].min = min; 

	return count;

}

static DEVICE_ATTR(constraints,S_IWUSR | S_IRUGO,
		   device_read_constraints,device_write_constraints);
#endif /* linux-pm */

struct attribute * dev_default_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_power.attr,
#if 1 /* linux-pm */
	&dev_attr_constraints.attr,
#endif /* linux-pm */
	NULL,
};

