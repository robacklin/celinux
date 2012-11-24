/*
 * linux/arch/arm/mach-s3c2440/led.c
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

#include <linux/init.h>

#include <asm/leds.h>
#include <asm/mach-types.h>

#include "leds.h"
static int __init
s3c2440_leds_init(void)
{
	if (machine_is_smdk2440())
		leds_event = smdk2440_leds_event;

	leds_event(led_start);
	return 0;
}

__initcall(s3c2440_leds_init);
