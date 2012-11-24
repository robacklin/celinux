/*
 * au1000_gpio.h
 *
 * BRIEF MODULE DESCRIPTION
 *	API to Alchemy Au1000 GPIO device.
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2001 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __AU1000_GPIO_H
#define __AU1000_GPIO_H

#include <linux/ioctl.h>

#define AU1000GPIO_IOC_MAGIC 'A'

#define AU1000GPIO_IN		_IOR (AU1000GPIO_IOC_MAGIC, 0, int)
#define AU1000GPIO_SET		_IOW (AU1000GPIO_IOC_MAGIC, 1, int)
#define AU1000GPIO_CLEAR	_IOW (AU1000GPIO_IOC_MAGIC, 2, int)
#define AU1000GPIO_OUT		_IOW (AU1000GPIO_IOC_MAGIC, 3, int)
#define AU1000GPIO_TRISTATE	_IOW (AU1000GPIO_IOC_MAGIC, 4, int)
#define AU1000GPIO_AVAIL_MASK	_IOR (AU1000GPIO_IOC_MAGIC, 5, int)

#ifdef __KERNEL__
extern u32 get_au1000_avail_gpio_mask(void);
extern int au1000gpio_tristate(u32 data);
extern int au1000gpio_in(u32 *data);
extern int au1000gpio_set(u32 data);
extern int au1000gpio_clear(u32 data);
extern int au1000gpio_out(u32 data);
#endif

#endif
