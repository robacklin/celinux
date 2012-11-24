/*
 *  linux/drivers/usb/usb-ohci-omap1510.c
 *
 *  The outline of this code was taken from Brad Parkers <brad@heeltoe.com>
 *  original OHCI driver modifications, and reworked into a cleaner form
 *  by Russell King <rmk@arm.linux.org.uk>.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pci.h>		/* for pci_pool_* prototypes */
#include <linux/usb.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "usb-ohci.h"

#define OMAP1510_LB_OFFSET (0x30000000UL)

extern int __devinit hc_add_ohci(struct pci_dev *dev, int irq, void *membase,
				 unsigned long flags, ohci_t ** ohci,
				 const char *name, const char *slot_name);
extern void __devexit hc_remove_ohci(ohci_t * ohci);

static ohci_t *omap1510_ohci;

/* bogus pci_dev structure */
static struct pci_dev bogus_pcidev;

static int __devinit omap1510_ohci_configure(void);
static void __devexit omap1510_ohci_release(void);

#if	defined(CONFIG_OMAP_INNOVATOR)	/* MVL-CEE */
#include <linux/device.h>

static int omap1510_ohci_suspend(struct device *dev, u32 state, u32 level);
static int omap1510_ohci_resume(struct device *dev, u32 level);
static int omap1510_ohci_scale(struct bus_op_point *op, u32 level);

static struct device_driver omap1510_ohci_driver_ldm = {
	.name		= "omap1510-ohci",
	.devclass	= NULL,
	.probe		= NULL,
	.suspend	= omap1510_ohci_suspend,
	.resume		= omap1510_ohci_resume,
	.scale		= omap1510_ohci_scale,
	.remove		= NULL,
	.constraints	= NULL,
};

static struct device omap1510_ohci_device_ldm = {
	.name		= "OMAP1510 OHCI",
	.bus_id		= "OHCI",
	.driver		= NULL,
	.power_state	= DPM_POWER_ON,
};

static void
omap1510_ohci_ldm_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);
	mpu_public_driver_register(&omap1510_ohci_driver_ldm);
}

static void
omap1510_ohci_ldm_device_register(void)
{
	extern void mpu_public_device_register(struct device *device);
	mpu_public_device_register(&omap1510_ohci_device_ldm);
}

static void
omap1510_ohci_ldm_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	mpu_public_driver_unregister(&omap1510_ohci_driver_ldm);
}

static void
omap1510_ohci_ldm_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *device);
	mpu_public_device_unregister(&omap1510_ohci_device_ldm);
}

static int
omap1510_ohci_scale(struct bus_op_point *op, u32 level)
{
	/* REVISIT */
	return 0;
}

static int
omap1510_ohci_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		omap1510_ohci_release();
		break;
	}

	return 0;
}

static int
omap1510_ohci_resume(struct device *dev, u32 level)
{
	int ret = 0;

	switch (level) {
	case RESUME_POWER_ON:
		ret = omap1510_ohci_configure();
		break;
	}

	return ret;
}
#endif				/* MVL-CEE */

static int __devinit
omap1510_ohci_configure(void)
{
	/* TO DO:  make a proper header file for all of these registers. */
#ifdef CONFIG_OMAP_INNOVATOR
	volatile unsigned char *fpga_usb_host_ctrl =
	    (unsigned char *) 0xe800020c;
#endif
	volatile unsigned short *apll_ctrl_reg = (unsigned short *) 0xfffe084c;
	volatile unsigned short *dpll_ctrl_reg = (unsigned short *) 0xfffe083c;
	volatile unsigned short *soft_req_reg = (unsigned short *) 0xfffe0834;
	volatile unsigned short *clock_ctrl_reg = (unsigned short *) 0xfffe0830;
	volatile unsigned long *mod_conf_ctrl_0 = (unsigned long *) 0xfffe1080;

	volatile unsigned long *lb_clock_div = (unsigned long *) 0xfffec10c;
	volatile unsigned short *lb_mmu_cntl_reg =
	    (unsigned short *) 0xfffec208;
	volatile unsigned short *lb_mmu_lock_reg =
	    (unsigned short *) 0xfffec224;
	volatile unsigned short *lb_mmu_ld_tlb_reg =
	    (unsigned short *) 0xfffec228;
	volatile unsigned short *lb_mmu_cam_h_reg =
	    (unsigned short *) 0xfffec22c;
	volatile unsigned short *lb_mmu_cam_l_reg =
	    (unsigned short *) 0xfffec230;
	volatile unsigned short *lb_mmu_ram_h_reg =
	    (unsigned short *) 0xfffec234;
	volatile unsigned short *lb_mmu_ram_l_reg =
	    (unsigned short *) 0xfffec238;
	int tlb;
	unsigned long lbaddr, physaddr;
	int ret;

	/*
	 * Request memory resources.
	 */
//	if (!request_mem_region(_USB_OHCI_OP_BASE, _USB_EXTENT, "usb-ohci"))
//		return -EBUSY;

#define APLL_CTRL_REG_APLL_NDPLL_SWITCH		0x0001
#define DPLL_CTRL_REG_PLL_ENABLE		0x0010
#define DPLL_CTRL_REG_LOCK			0x0001
#define SOFT_REQ_REG_DPLL_REQ			0x0001
#define CLOCK_CTRL_REG_USB_MCLK_EN		0x0010
#define MOD_CONF_CTRL_0_USB_HOST_HHC_UHOST_EN	0x00000200

	*apll_ctrl_reg &= ~APLL_CTRL_REG_APLL_NDPLL_SWITCH;
	*dpll_ctrl_reg |= DPLL_CTRL_REG_PLL_ENABLE;
	*soft_req_reg |= SOFT_REQ_REG_DPLL_REQ;
	while (!(*dpll_ctrl_reg & DPLL_CTRL_REG_LOCK)) ;
	*clock_ctrl_reg |= CLOCK_CTRL_REG_USB_MCLK_EN;
	*ARM_IDLECT2 |= (1 << EN_LBFREECK) | (1 << EN_LBCK);
	*mod_conf_ctrl_0 |= MOD_CONF_CTRL_0_USB_HOST_HHC_UHOST_EN;
#ifdef CONFIG_OMAP_INNOVATOR
	*fpga_usb_host_ctrl |= 0x20;
#endif

	*lb_clock_div = (*lb_clock_div & 0xfffffff8) | 0x4;
	*lb_mmu_cntl_reg = 0x3;
	udelay(200);
	for (tlb = 0; tlb < 32; tlb++) {
		lbaddr = tlb * 0x00100000 + OMAP1510_LB_OFFSET;
		physaddr = tlb * 0x00100000 + PHYS_OFFSET;
		*lb_mmu_cam_h_reg = (lbaddr & 0x0fffffff) >> 22;
		*lb_mmu_cam_l_reg = ((lbaddr & 0x003ffc00) >> 6) | 0xc;
		*lb_mmu_ram_h_reg = physaddr >> 16;
		*lb_mmu_ram_l_reg = (physaddr & 0x0000fc00) | 0x300;
		*lb_mmu_lock_reg = tlb << 4;
		*lb_mmu_ld_tlb_reg = 0x1;
	}
	*lb_mmu_cntl_reg = 0x7;
	udelay(200);

	/*
	 * Fill in some fields of the bogus pci_dev.
	 */
	memset(&bogus_pcidev, 0, sizeof(struct pci_dev));
	strcpy(bogus_pcidev.name, "OMAP1510 OHCI");
	strcpy(bogus_pcidev.slot_name, "builtin");
	bogus_pcidev.resource[0].name = "OHCI Operational Registers";
	bogus_pcidev.resource[0].start = 0xfffba000;
	bogus_pcidev.resource[0].end = 0xfffba000 + 4096; /* REVISIT */
	bogus_pcidev.resource[0].flags = 0;
	bogus_pcidev.irq = IH2_BASE + 6;

	/*
	 * Initialise the generic OHCI driver.
	 */
	ret = hc_add_ohci(&bogus_pcidev, bogus_pcidev.irq,
			  (void *) bogus_pcidev.resource[0].start, 0,
			  &omap1510_ohci, "usb-ohci", "omap1510");

//	if (ret)
//		release_mem_region(_USB_OHCI_OP_BASE, _USB_EXTENT);

	return ret;
}

static void __devexit
omap1510_ohci_release(void)
{
	/* REVISIT: Need to properly shut off clocks here. */
	volatile unsigned long *mod_conf_ctrl_0 = (unsigned long *) 0xfffe1080;

	hc_remove_ohci(omap1510_ohci);

	*mod_conf_ctrl_0 &= ~MOD_CONF_CTRL_0_USB_HOST_HHC_UHOST_EN;

	/*
	 * Release memory resources.
	 */
//	release_mem_region(_USB_OHCI_OP_BASE, _USB_EXTENT);
}

static int __init
omap1510_ohci_init(void)
{
	int ret = 0;

#if	defined(CONFIG_OMAP_INNOVATOR)	/* MVL-CEE */
	omap1510_ohci_ldm_driver_register();
	omap1510_ohci_ldm_device_register();
#endif				/* MVL-CEE */
	ret = omap1510_ohci_configure();

	return 0;
}

static void __exit
omap1510_ohci_exit(void)
{
	int ret;

#if	defined(CONFIG_OMAP_INNOVATOR)	/* MVL-CEE */
	omap1510_ohci_ldm_device_unregister();
	omap1510_ohci_ldm_driver_unregister();
#endif				/* MVL-CEE */

	omap1510_ohci_release();
}

MODULE_LICENSE("GPL");

module_init(omap1510_ohci_init);
module_exit(omap1510_ohci_exit);
