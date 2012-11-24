/*
 * linux/include/asm-arm/arch-s3c2440/uncompress.h
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

#define UART_UTRSTAT		(*(volatile unsigned long *)0x50000010)
#define UART_UTXH		(*(volatile unsigned long *)0x50000020)	/* littel endian */
#define UTRSTAT_TX_EMPTY	(1 << 2)

static void puts(const char *s)
{
	while (*s) {
		while (!(UART_UTRSTAT & UTRSTAT_TX_EMPTY));

		UART_UTXH = *s;

		if (*s == '\n') {
			while (!(UART_UTRSTAT & UTRSTAT_TX_EMPTY));

			UART_UTXH = '\r';
		}
		s++;
	}
	while (!(UART_UTRSTAT & UTRSTAT_TX_EMPTY));
}
#define arch_decomp_setup()
#define arch_decomp_wdog()
