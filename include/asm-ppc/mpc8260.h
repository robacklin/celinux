/* This is the single file included by all MPC8260 build options.
 * Since there are many different boards and no standard configuration,
 * we have a unique include file for each.  Rather than change every
 * file that has to include MPC8260 configuration, they all include
 * this one and the configuration switching is done here.
 */
#ifdef __KERNEL__
#ifndef __CONFIG_8260_DEFS
#define __CONFIG_8260_DEFS

#include <linux/config.h>

#ifdef CONFIG_8260

#ifdef CONFIG_EST8260
#include <platforms/est8260.h>
#endif

#ifdef CONFIG_SBS8260
#include <platforms/sbs8260.h>
#endif

#ifdef CONFIG_RPX6
#include <platforms/rpxsuper.h>
#endif

#ifdef CONFIG_ADS8260
#include <platforms/ads8260.h>
#endif

#ifdef CONFIG_WILLOW
#include <platforms/willow.h>
#endif

#ifdef CONFIG_TQM8260
#include <platforms/tqm8260.h>
#endif

/* The board-specific header files included above must define 
 * _IO_BASE, _ISA_MEM_BASE, and PCI_DRAM_OFFSET if non-zero.  If not defined,
 * a default value of zero is supplied here.
 */
#ifndef _IO_BASE
#define _IO_BASE        0
#endif
#ifndef _ISA_MEM_BASE
#define _ISA_MEM_BASE   0
#endif
#ifndef PCI_DRAM_OFFSET
#define PCI_DRAM_OFFSET 0
#endif

#ifndef __ASSEMBLY__
/* The "residual" data board information structure the boot loader
 * hands to us.
 */
extern unsigned char __res[];
#endif /* __ASSEMBLY__ */

#define request_8xxirq request_irq

#endif /* CONFIG_8260 */
#endif /* !__CONFIG_8260_DEFS */
#endif /* __KERNEL__ */
