/*
 * linux/arch/arm/mach-s3c2440/cpu.c
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include <asm/errno.h>
#include <asm/arch/cpu_s3c2440.h>

#ifdef CONFIG_BOOT_FREQ
#include <asm/arch/ck.h>
#define CLKDIVN_111	0x0
#define CLKDIVN_112	0x1
#define CLKDIVN_122	0x2
#define CLKDIVN_124	0x3
#define CLKDIVN_133	0x6
#define CLKDIVN_136	0x7
#define CLKDIVN_144	0x4
#define CLKDIVN_148	0x5
#endif /* CONFIG_BOOT_FREQ */

unsigned long s3c2440_get_fclk (void)
{
        unsigned long val = MPLLCON;
        return (((GET_MDIV(val) + 8) * FIN) / \
                        ((GET_PDIV(val) + 2) * (1 << GET_SDIV(val))));
}

EXPORT_SYMBOL(s3c2440_get_fclk);

unsigned long s3c2440_get_hclk (void)
{
	int	clkdivn;
        unsigned long hclk = s3c2440_get_fclk();

	clkdivn = CLKDIVN & 0x6;
	switch (clkdivn)
	{
		case 0:
			hclk /= 1;
			break;
		case 2:
			hclk /= 2;
			break;
		case 4:
			hclk /= 4;
			break;
		case 6:
			hclk /= 3;
			break;
		default:
			hclk /= 1;
	}
        return hclk;
}

EXPORT_SYMBOL(s3c2440_get_hclk);

unsigned long s3c2440_get_pclk (void)
{
	int	clkdivn;
        unsigned long pclk = s3c2440_get_fclk();

	clkdivn = CLKDIVN & 0x7;
	switch (clkdivn)
        {
                case 0:
			pclk /= 1;
			break;
                case 1:
			pclk /= 2;
			break;
                case 2:
                        pclk /= 2;
                        break;
                case 3:
                        pclk /= 4;
                        break;
                case 4:
                        pclk /= 4;
                        break;
                case 5:
                        pclk /= 8;
                        break;
                case 6:
                        pclk /= 3;
                        break;
                case 7:
                        pclk /= 6;
                        break;
                default:
                        pclk /= 1;
        }
        return pclk;
}
EXPORT_SYMBOL(s3c2440_get_pclk);

unsigned long s3c2440_get_uclk(void)
{
	unsigned long val = UPLLCON;

	return (((GET_MDIV(val) + 8) * FIN) / \
			((GET_PDIV(val) + 2) * (1 << GET_SDIV(val))));
}

EXPORT_SYMBOL(s3c2440_get_uclk);

#ifdef CONFIG_BOOT_FREQ
void s3c2440_ck_init_boot(void)
	{
        unsigned int fclk;
        unsigned int hclk;
        unsigned int pclk;
	unsigned int clkdivn = CLKDIVN & 0x8; 
	
	#if defined(CONFIG_BOOT_FREQ_400MHZ)
		CLKDIVN = clkdivn | CLKDIVN_136;
		MPLLCON = mpll_value[MPLL_400MHZ];
	#elif defined(CONFIG_BOOT_FREQ_135MHZ)
		MPLLCON = mpll_value[MPLL_135MHZ];
		CLKDIVN = clkdivn | CLKDIVN_112;
	#endif
	
        fclk = (unsigned int)s3c2440_get_fclk();
        hclk = (unsigned int)s3c2440_get_hclk();
        pclk = (unsigned int)s3c2440_get_pclk();

       printk("S3C2440 clock setting. (FCLK:%dMHz HCLK:%dMHz PCLK:%dMHz)\n",
                       fclk/1000000, hclk/1000000, pclk/1000000);
}
#endif /* CONFIG_BOOT_FRRQ */
