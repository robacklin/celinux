/*
 * xtsonic.c
 *
 * (C) 2001 - 2003 Tensilica Inc.
 *	by Kevin Chea <kchea@yahoo.com> (2001)
 *	by Marc Gauthier (marc@tensilica.com, marc@alumni.uwaterloo.ca) (2003)
 *
 * (C) 1996,1998 by Thomas Bogendoerfer (tsbogend@alpha.franken.de)
 * 
 * This driver is based on work from Andreas Busse, but most of
 * the code is rewritten.
 * 
 * (C) 1995 by Andreas Busse (andy@waldorf-gmbh.de)
 *
 * A driver for the onboard Sonic ethernet controller on the XT2000.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <asm/io.h>
#include <asm/pci.h>
#include <xtensa/xtboard.h>

#define SONIC_MODE32

extern unsigned xtboard_nvram_valid(void);
extern void xtboard_get_ether_addr(unsigned char *buf);

#include "sonic.h"

/*
 * Macros to access SONIC registers
 */
#define SONIC_READ(reg) \
	(0xffff & *((volatile unsigned int *)base_addr+reg))

#define SONIC_WRITE(reg,val) \
	*((volatile unsigned int *)base_addr+reg) = val


/* use 0 for production, 1 for verification, >2 for debug */
#ifdef SONIC_DEBUG
static unsigned int sonic_debug = SONIC_DEBUG;
#else 
static unsigned int sonic_debug = 1;
#endif

/*
 * We cannot use station (ethernet) address prefixes to detect the
 * sonic controller since these are board manufacturer depended.
 * So we check for known Silicon Revision IDs instead. 
 */
static unsigned short known_revisions[] =
{
  0x101, 			/* SONIC 83934 */
  0xffff			/* end of list */
};

/* Index to functions, as function prototypes. */

extern int sonic_probe(struct net_device *dev);
static int sonic_probe1(struct net_device *dev, unsigned int base_addr, unsigned int irq);

static void delay_10us(unsigned unused)
{
	udelay(10);
}

/*
 * Probe for a SONIC ethernet controller on an XT2000 board.
 * Actually probing is superfluous but we're paranoid.
 */
int __init sonic_probe(struct net_device *dev)
{
    unsigned int base_addr = dev ? dev->base_addr : 0;

    if (base_addr == 0xFFE0)
	return -ENODEV;  
    if (base_addr != 0)	/* Check a single specified location. */
	return sonic_probe1(dev, base_addr, dev->irq);
    
    base_addr = SONIC83934_VADDR;
    if (!check_region(base_addr, 0x100)) {
	if (sonic_probe1(dev, base_addr, SONIC83934_INTNUM) == 0)
	    return 0;
    }
    return -ENODEV;
}

static int __init sonic_probe1(struct net_device *dev,
                               unsigned int base_addr, unsigned int irq)
{
    static unsigned version_printed = 0;
    unsigned int silicon_revision;
    struct sonic_local *lp;
    unsigned char *priv;
    dma_addr_t dma;
    int i;
    
    /*
     * get the Silicon Revision ID. If this is one of the known
     * one assume that we found a SONIC ethernet controller at
     * the expected location.
     */
    silicon_revision = SONIC_READ(SONIC_SR);
    if (sonic_debug > 1)
      printk("SONIC Silicon Revision = 0x%04x\n",silicon_revision);

    i = 0;
    while ((known_revisions[i] != 0xffff) &&
	   (known_revisions[i] != silicon_revision))
      i++;
	
    if (known_revisions[i] == 0xffff) {
	printk("SONIC ethernet controller not found (0x%4x)\n",
	       silicon_revision);
	return -ENODEV;
    }
    
    request_region(base_addr, 0x100, "SONIC");
    
    /* Allocate a new 'dev' if needed. */
    if (dev == NULL)
      dev = init_etherdev(0, 0);

    if (sonic_debug  &&  version_printed++ == 0)
      printk(version);

    printk("%s: %s found at 0x%08x, ",
	   dev->name, "SONIC ethernet", base_addr);

    /* Fill in the 'dev' fields. */
    dev->base_addr = base_addr;
    dev->irq = irq;

    /*
     * Put the sonic into software reset, then
     * retrieve and print the ethernet address.
     */
    SONIC_WRITE(SONIC_CMD,SONIC_CR_RST);
    SONIC_WRITE(SONIC_DCR,SONIC_DCR_WC0|SONIC_DCR_DW|SONIC_DCR_LBR);
    SONIC_WRITE(SONIC_CEP,0);
    SONIC_WRITE(SONIC_IMR,0);

    xtboard_set_nsdelay_func(&delay_10us);

    if (xtboard_nvram_valid()) {
	xtboard_get_ether_addr(dev->dev_addr);
    } else {
	/* Bogus ethernet address */
	dev->dev_addr[0]=0x0;
	dev->dev_addr[1]=0x1;
	dev->dev_addr[2]=0x2;
	dev->dev_addr[3]=0x3;
	dev->dev_addr[4]=0x4;
	dev->dev_addr[5]=0x5;
	printk("WARNING: Bogus ");
    }

    printk("HW Address ");
    for (i = 0; i < 6; i++) {
	printk("%2.2x", dev->dev_addr[i]);
	if (i<5)
	  printk(":");
    }
    
    printk(" IRQ %d\n", irq);
    
    /* Initialize the device structure. */
    if (dev->priv != NULL)
	printk("%s: preallocated priv field ignored\n", dev->name);

    /*
     *  Allocate local private descriptor areas in uncached space.
     *  The entire structure must be located within the same 64kb segment.
     *  A simple way to ensure this is to allocate twice the
     *  size of the structure -- given that the structure is
     *  much less than 64 kB, at least one of the halves of
     *  the allocated area will be contained entirely in 64 kB.
     *  We also allocate extra space for a pointer to allow freeing
     *  this structure later on (in xtsonic_cleanup_module()).
     */
    priv = pci_alloc_consistent(NULL, sizeof(struct sonic_local)*2
	    				+ sizeof(unsigned char*), &dma);
    priv += sizeof(unsigned char*);	/* skip pointer */
    dma  += sizeof(unsigned char*);	/* ditto */
    lp = (struct sonic_local*) \
	( ((unsigned)priv + sizeof(struct sonic_local) - 1) & 0xFFFF0000 );
    if( (unsigned char*)lp < priv )
	/*  Returned address is okay, fits in 64 kB:  */
	lp = (struct sonic_local*)priv;
    else
	/*  Aligned returned address to 64 kB, adjust 'dma' similarly:  */
	dma = (dma_addr_t)( (unsigned)dma + ((unsigned char*)lp - priv) );
    *((unsigned char**)lp - 1) = priv - sizeof(unsigned char*);

    /*  For debugging:  */
    printk("%s: SONIC priv area at 0x%X, dev at 0x%X\n",
	    dev->name, (unsigned) lp, (unsigned) dev);

    memset(lp, 0, sizeof(struct sonic_local));

    /* get the virtual dma address */
    lp->cda_laddr = dma;
    lp->tda_laddr = lp->cda_laddr + sizeof (lp->cda);
    lp->rra_laddr = lp->tda_laddr + sizeof (lp->tda);
    lp->rda_laddr = lp->rra_laddr + sizeof (lp->rra);

    /* allocate receive buffer area */
    /* FIXME, maybe we should use skbs */
    lp->rba = (char *)kmalloc(SONIC_NUM_RRS * SONIC_RBSIZE, GFP_KERNEL);
    if (lp->rba == NULL) {
	printk ("%s: couldn't allocate receive buffers\n",dev->name);
	return -ENOMEM;
    }
    lp->rba_laddr = virt_to_phys(lp->rba);

    flush_cache_all();

    dev->priv = lp;

    dev->open = sonic_open;
    dev->stop = sonic_close;
    dev->hard_start_xmit    = sonic_send_packet;
    dev->get_stats	    = sonic_get_stats;
    dev->set_multicast_list = &sonic_multicast_list;

    /*
     * clear tally counter
     */
    SONIC_WRITE(SONIC_CRCT,0xffff);
    SONIC_WRITE(SONIC_FAET,0xffff);
    SONIC_WRITE(SONIC_MPT,0xffff);

    /* Fill in the fields of the device structure with ethernet values. */
    ether_setup(dev);
    return 0;
}

static struct net_device dev_xtsonic = {
    name:	"",
    init:	sonic_probe
};

static int
xtsonic_init_module(void)
{
    if (register_netdev(&dev_xtsonic) != 0) {
	printk(KERN_WARNING "xtsonic.c: No card found\n");
	return -ENODEV;
    }
    return 0;
}

static void
xtsonic_cleanup_module(void)
{
    if (dev_xtsonic.priv != NULL) {
	unregister_netdev(&dev_xtsonic);
	kfree(*((unsigned char**)dev_xtsonic.priv-1));
	dev_xtsonic.priv = NULL;
    }
}

module_init(xtsonic_init_module);
module_exit(xtsonic_cleanup_module);

/*
 *      SONIC uses a normal IRQ
 */
#define sonic_request_irq       request_irq
#define sonic_free_irq          free_irq

#define vdma_alloc(foo, bar)	((u32)foo)
#define vdma_free(baz)
#define sonic_chiptomem(x)      phys_to_virt(x);

#define SONIC_IRQ_FLAGS	0

#include "sonic.c"
