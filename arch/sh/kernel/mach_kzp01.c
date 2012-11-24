/*
 * linux/arch/sh/kernel/mach_kzp01.c
 *
 * Copyright (C) 2003 Mitsubishi Electric Corporation
 * Copyright (C) 2001 Nobuhiro Sakawa
 *
 * Machine vector for the kmc Solution Platfoem
 *
 * from
 *
 * linux/arch/sh/kernel/mach_se.c
 *
 * Copyright (C) 2000 Stuart Menefy (stuart.menefy@st.com)
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * Machine vector for the Hitachi SolutionEngine
 */

#include <linux/config.h>
#include <linux/init.h>

#include <asm/machvec.h>
#include <asm/rtc.h>
#include <asm/machvec_init.h>

#include <asm/io_kzp.h>

void heartbeat_kzp(void);
void setup_kzp(void);
void init_kzp_IRQ(void);

/*
 * The Machine Vector
 */

struct sh_machine_vector mv_kzp __initmv = {
	mv_name:		"Solution Platform",
	mv_nr_irqs:		48,

	mv_inb:			kzp_inb,
	mv_inw:			kzp_inw,
	mv_inl:			kzp_inl,
	mv_outb:		kzp_outb,
	mv_outw:		kzp_outw,
	mv_outl:		kzp_outl,

	mv_inb_p:		kzp_inb_p,
	mv_inw_p:		kzp_inw,
	mv_inl_p:		kzp_inl,
	mv_outb_p:		kzp_outb_p,
	mv_outw_p:		kzp_outw,
	mv_outl_p:		kzp_outl,

	mv_insb:		kzp_insb,
	mv_insw:		kzp_insw,
	mv_insl:		kzp_insl,
	mv_outsb:		kzp_outsb,
	mv_outsw:		kzp_outsw,
	mv_outsl:		kzp_outsl,

	mv_readb:		kzp_readb,
	mv_readw:		kzp_readw,
	mv_readl:		kzp_readl,
	mv_writeb:		kzp_writeb,
	mv_writew:		kzp_writew,
	mv_writel:		kzp_writel,

	mv_ioremap:		generic_ioremap,
	mv_iounmap:		generic_iounmap,

	mv_isa_port2addr:	kzp_isa_port2addr,

	mv_init_arch:		setup_kzp,
	mv_init_irq:		init_kzp_IRQ,
#ifdef CONFIG_HEARTBEAT
	mv_heartbeat:		heartbeat_kzp,
#endif

	mv_rtc_gettimeofday:	sh_rtc_gettimeofday,
	mv_rtc_settimeofday:	sh_rtc_settimeofday,

	mv_hw_kzp:		1,
};
ALIAS_MV(kzp)
