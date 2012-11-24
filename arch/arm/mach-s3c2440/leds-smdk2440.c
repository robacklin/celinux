/*
 * linux/arch/arm/mach-s3c2440/led-smdk2440.c
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

#include <linux/config.h>
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/system.h>

#include "leds.h"

#define LED_STATE_ENABLED	1
#define LED_STATE_CLAIMED	2

static unsigned int led_state;
static unsigned int hw_led_state;

#define _LED1		4
#define _LED2		5
#define _LED3		6
#define _LED4		7
#define LED1		(1 << _LED1)
#define LED2		(1 << _LED2)
#define LED3		(1 << _LED3)
#define LED4		(1 << _LED4)

#define LED_MASK	LED1 | LED2 | LED3 | LED4

void smdk2440_leds_event(led_event_t evt)
{
        unsigned long flags;

	local_irq_save(flags);

        switch (evt) {
        case led_start:
                hw_led_state = LED_MASK;
                led_state = LED_STATE_ENABLED;
                GPFCON &= ~((0x3<<(_LED1*2)) | (0x3<<(_LED2*2)) 
				| (0x3<<(_LED3*2)) | (0x3<<(_LED3*2)));
                GPFCON |= (0x1<<(_LED1*2)) | (0x1<<(_LED2*2)) 
				| (0x1<<(_LED3*2)) | (0x1<<(_LED3*2));
                GPFUP  &= ~(LED_MASK);
                break;

        case led_stop:
                led_state &= ~LED_STATE_ENABLED;
                break;

        case led_claim:
                led_state |= LED_STATE_CLAIMED;
                hw_led_state = LED_MASK;
                break;
        case led_release:
                led_state &= ~LED_STATE_CLAIMED;
                hw_led_state = LED_MASK;
                break;

#ifdef CONFIG_LEDS_TIMER
        case led_timer:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state ^= LED1;
                break;
#endif

#ifdef CONFIG_LEDS_CPU
        case led_idle_start:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state |= LED2;
                break;

        case led_idle_end:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state &= ~LED2;
                break;
#endif
        case led_green_on:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state &= ~LED3;
                break;

        case led_green_off:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state |= LED3;
                break;

        case led_amber_on:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state &= ~LED3;
                break;

        case led_amber_off:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state |= LED3;
                break;

        case led_red_on:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state &= ~LED4;
                break;

        case led_red_off:
                if (!(led_state & LED_STATE_CLAIMED))
                        hw_led_state |= LED4;
                break;

        default:
                break;
        }

        if  (led_state & LED_STATE_ENABLED) {
		GPFDAT &= ~(LED_MASK);
		GPFDAT |= hw_led_state;
        }

	local_irq_restore(flags);
}
