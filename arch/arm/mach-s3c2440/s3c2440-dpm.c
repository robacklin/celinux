/*
 * S3C2410 Specific DPM support
 * 
 * arch/arm/mach-s3c2440/s3c2440_dpm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002 MontaVista Software <source@mvista.com>.
 * Matthew Locke <mlocke@mvista.com>
 *
 * Adapted for S3C2410 by Samsung
 * Copyright (C) 2003 Samsung Electronics Inc.
 * 
 * HISTORY:
 * 2003-12-16:	Seongil Na <seongil@samsung.com>
 * 	- Based on OMAP1510 DPM code by MontaVista Software
 * 	- Implemented a specific code of S3C2410 CPU
 * 	- Comment a proc interface
 * 	- Merge with board support code
 * 	
 */

#include <linux/config.h>
#include <linux/dpm.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/string.h>

#include <linux/delay.h>
#include <asm/hardirq.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <asm/arch/hardware.h>
#include <asm/arch/ck.h>
#include <linux/device.h>

//#define _DEBUG_
#ifdef _DEBUG_
	#define DPRINTK(x, args...) printk("DEBUG>>%s:%d: "x, __FUNCTION__, __LINE__, ##args)
#else
	#define DPRINTK(x, args...) do {} while (0)
#endif

struct dpm_bd dpm_bd;
struct dpm_md dpm_md;

static void s3c2440_dpm_clocks(struct dpm_regs *regs)
{
	CLKCON = regs->clkcon;
}

#define CLKDIVN_111	0x0
#define CLKDIVN_112	0x1
#define CLKDIVN_122	0x2
#define CLKDIVN_124	0x3
#define CLKDIVN_133	0x6
#define CLKDIVN_136	0x7
#define CLKDIVN_144	0x4
#define CLKDIVN_148	0x5
/* CPU frequency scaling */
static void s3c2440_dpm_scale_mpll(struct dpm_regs *regs)
{
	int ret = 0;
	unsigned int clkdivn = CLKDIVN & 0x8; 

	//printk("1");
	//mdelay(10);
	if (mpll_value[MPLL_400MHZ] == regs->mpllcon)
	{
		CLKDIVN = clkdivn | CLKDIVN_136;
		MPLLCON = mpll_value[MPLL_400MHZ];
	}
	else if (mpll_value[MPLL_135MHZ] == regs->mpllcon)
	{
		MPLLCON = mpll_value[MPLL_135MHZ];
		CLKDIVN = clkdivn | CLKDIVN_112;
	}
	//mdelay(10);
#if 0
#if 1	// Up scaling
	CLKDIVN = regs->clkdivn;
	MPLLCON = regs->mpllcon;
#else	// Down scaling
	MPLLCON = regs->mpllcon;
	CLKDIVN = regs->clkdivn;
#endif
#endif
	// Reset a loops per jiffy
	loops_per_jiffy = regs->lpj;
}

unsigned int dpm_fscaler_flags;
#define DPM_FSCALER_NOP 		0
#define DPM_FSCALER_DIVISORS 	1
#define DPM_FSCALER_SLEEP 		2
#define DPM_FSCALER_WAKEUP 		4
#define DPM_FSCALER_DPLL	 	8
#define DPM_FSCALER_MPLL		16	// Add for S3C2410
#define DPM_FSCALER_CLOCKS		32	// Add for S3C2410
#define DPM_FSCALER_IDLE		64	// Add for S3C2410

static void s3c2440_dpm_fscaler(struct dpm_regs *regs)
{
	extern void s3c2440_pm_suspend(void);

	if (dpm_fscaler_flags & DPM_FSCALER_NOP)
		return;

	/* We want to scale after devices are powered off
	 * if going to sleep
	 */
	if (dpm_fscaler_flags & DPM_FSCALER_SLEEP)
		device_suspend(0, SUSPEND_POWER_DOWN);

	if (dpm_fscaler_flags & DPM_FSCALER_CLOCKS)
		s3c2440_dpm_clocks(regs);

	if (dpm_fscaler_flags & DPM_FSCALER_MPLL)
		s3c2440_dpm_scale_mpll(regs);
	else
		printk("2");

	/* Don't bring the devices back until frequencies restored */
	if (dpm_fscaler_flags & DPM_FSCALER_WAKEUP)
		device_resume(RESUME_POWER_ON);

	if (dpm_fscaler_flags & DPM_FSCALER_SLEEP)
	{
		s3c2440_pm_suspend();

	 	/* Here when we wake up.  Recursive call to switch back to
	 	* to task state.
	 	*/
		dpm_set_os(DPM_TASK_STATE);
	}
}

static dpm_fscaler s3c2440_dpm_fscalers[1] = {
	s3c2440_dpm_fscaler,
};

/* This routine computes the "forward" frequency scaler that moves the system
 * from the current operating point to the new operating point. The resulting
 * fscaler is applied to the registers of the new operating point. 
 */

dpm_fscaler compute_fscaler(struct dpm_md_opt *cur, struct dpm_md_opt *new)
{
	DPRINTK("cur-cpu:%d, new-cpu:%d\n", cur->cpu, new->cpu);
	//udelay(10);

	dpm_fscaler_flags = DPM_FSCALER_NOP;

	if (cur->cpu && ! new->cpu)
		dpm_fscaler_flags = DPM_FSCALER_SLEEP;
	else if (! cur->cpu && new->cpu)
		dpm_fscaler_flags = DPM_FSCALER_WAKEUP;
	
	if (new->cpu != cur->cpu)
		dpm_fscaler_flags |= DPM_FSCALER_MPLL;

	if (new->regs.clkcon != cur->regs.clkcon)
		dpm_fscaler_flags |= DPM_FSCALER_CLOCKS;
		
	return s3c2440_dpm_fscalers[0];
}

/* Initialize the machine-dependent operating point from a list of parameters,
   which has already been installed in the pp field of the operating point.
   Some of the parameters may be specified with a value of -1 to indicate a
   default value. */

int s3c2440_dpm_init_opt(struct dpm_opt *opt)
{
	int v		= opt->pp[DPM_MD_V];
	int mpll	= opt->pp[DPM_MD_MPLL];
	int cpu		= opt->pp[DPM_MD_CPU];
	int clkdivn	= opt->pp[DPM_MD_CLKDIVN];
	int lcd		= opt->pp[DPM_MD_LCD];
	int nand	= opt->pp[DPM_MD_NAND];
	int spi		= opt->pp[DPM_MD_SPI];
	int iis		= opt->pp[DPM_MD_IIS];
	int iic		= opt->pp[DPM_MD_IIC];
	int adc		= opt->pp[DPM_MD_ADC];
	int rtc		= opt->pp[DPM_MD_RTC];
	int gpio	= opt->pp[DPM_MD_GPIO];
	int uart0	= opt->pp[DPM_MD_UART0];
	int uart1	= opt->pp[DPM_MD_UART1];
	int uart2	= opt->pp[DPM_MD_UART2];
	int sdi		= opt->pp[DPM_MD_SDI];
	int pwm		= opt->pp[DPM_MD_PWM];
	int usbh	= opt->pp[DPM_MD_USBH];
	int usbd	= opt->pp[DPM_MD_USBD];

	struct dpm_md_opt *md_opt = &opt->md_opt;

	/* Let's do some upfront error checking.  If we fail any of these, then the
	 * whole operating point is suspect and therefore invalid.
	 */

	/* Check a MPLL value */
	if ((mpll < -1) || (mpll > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: "
				"%d out of range for opt named %s\n", mpll, opt->name);
		return -EINVAL;
	}
	else if (mpll != 1)
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: "
				"Not support a stop mode for opt named %s\n",
				opt->name);
		return -EINVAL;
	}
	
	/* Check a CPU freq. */
#ifdef CONFIG_DPM_IDLE
	if (!((cpu == MPLL_SUSPEND) || (cpu == -1) || (cpu == 1) || 
		  (cpu == MPLL_400MHZ) || (cpu == MPLL_135MHZ)))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: "
				"Support only 400MHz/135MHz/Suspend/IDLE of CPU now for opt named %s\n",
				opt->name);
		return -EINVAL;
	}
#else
	if (!((cpu == MPLL_SUSPEND) || (cpu == -1) ||
		  (cpu == MPLL_400MHZ) || (cpu == MPLL_135MHZ)))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: "
				"Support only 400MHz/135MHz/Suspend of CPU now for opt named %s\n",
				opt->name);
		return -EINVAL;
	}
#endif

	/* Check a divider for HCLK, PCLK */
	if ((clkdivn < -1) || (clkdivn > 7))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: Clock Divider is invalid:"
				"%d out of range for opt named %s\n", clkdivn, opt->name);
		return -EINVAL;
	}

	/* Check each peripheral device clocks */
	if ((lcd < -1) || (lcd > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: LCD clock is invalid:"
				"%d out of range for opt named %s\n", lcd, opt->name);
		return -EINVAL;
	}
	
	if ((nand < -1) || (nand > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: NAND clock is invalid:"
				"%d out of range for opt named %s\n", nand, opt->name);
		return -EINVAL;
	}
	if ((spi < -1) || (spi > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: SPI clock is invalid:"
				"%d out of range for opt named %s\n", spi, opt->name);
		return -EINVAL;
	}
	if ((iis < -1) || (iis > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: I2S clock is invalid:"
				"%d out of range for opt named %s\n", iis, opt->name);
		return -EINVAL;
	}
	if ((iic < -1) || (iic > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: I2C clock is invalid:"
				"%d out of range for opt named %s\n", iic, opt->name);
		return -EINVAL;
	}
	if ((adc < -1) || (adc > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: ADC clock is invalid:"
				"%d out of range for opt named %s\n", adc, opt->name);
		return -EINVAL;
	}
	if ((rtc < -1) || (rtc > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: RTC clock is invalid:"
				"%d out of range for opt named %s\n", rtc, opt->name);
		return -EINVAL;
	}
	if ((gpio < -1) || (gpio > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: GPIO clock is invalid:"
				"%d out of range for opt named %s\n", gpio, opt->name);
		return -EINVAL;
	}
	if ((uart0 < -1) || (uart0 > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: UART0 clock is invalid:"
				"%d out of range for opt named %s\n", uart0, opt->name);
		return -EINVAL;
	}
	if ((uart1 < -1) || (uart1 > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: UART1 clock is invalid:"
				"%d out of range for opt named %s\n", uart1, opt->name);
		return -EINVAL;
	}
	if ((uart2 < -1) || (uart2 > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: UART2 clock is invalid:"
				"%d out of range for opt named %s\n", uart2, opt->name);
		return -EINVAL;
	}
	if ((sdi < -1) || (sdi > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: SDI clock is invalid:"
				"%d out of range for opt named %s\n", sdi, opt->name);
		return -EINVAL;
	}
	if ((pwm < -1) || (pwm > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: PWM clock is invalid:"
				"%d out of range for opt named %s\n", pwm, opt->name);
		return -EINVAL;
	}
	if ((usbh < -1) || (usbh > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: USB Host clock is invalid:"
				"%d out of range for opt named %s\n", usbh, opt->name);
		return -EINVAL;
	}
	if ((usbd < -1) || (usbd > 1))
	{
		printk(KERN_WARNING "s3c2440_dpm_init_opt: USB Device clock is invalid:"
				"%d out of range for opt named %s\n", usbd, opt->name);
		return -EINVAL;
	}

	/* */
	md_opt->v = v;
	md_opt->mpll = -1;

	md_opt->cpu = cpu;
	md_opt->regs.cpu = cpu;
	md_opt->regs.mpllcon = mpll_value[cpu];

	md_opt->clkdivn = clkdivn;
	md_opt->regs.clkdivn = clkdivn;

	md_opt->regs.clkcon = 0xFFFF0;	// initial value
	
	md_opt->lcd = lcd;
	if (lcd == 0)
		md_opt->regs.clkcon &= ~(CLKCON_LCDC);

	md_opt->nand = nand;
	if (nand == 0)
		md_opt->regs.clkcon &= ~(CLKCON_NAND);
	
	md_opt->spi = spi;
	if (spi == 0)
		md_opt->regs.clkcon &= ~(CLKCON_SPI);
	
	md_opt->iis = iis;
	if (iis == 0)
		md_opt->regs.clkcon &= ~(CLKCON_IIS);
	
	md_opt->iic = iic;
	if (iic == 0)
		md_opt->regs.clkcon &= ~(CLKCON_IIC);
	
	md_opt->adc = adc;
	if (adc == 0)
		md_opt->regs.clkcon &= ~(CLKCON_ADC);
	
	md_opt->rtc = rtc;
	if (rtc == 0)
		md_opt->regs.clkcon &= ~(CLKCON_RTC);
	
	md_opt->gpio = gpio;
	if (gpio == 0)
		md_opt->regs.clkcon &= ~(CLKCON_GPIO);
	
	md_opt->uart0 = uart0;
	if (uart0 == 0)
		md_opt->regs.clkcon &= ~(CLKCON_UART0);
	
	md_opt->uart1 = uart1;
	if (uart1 == 0)
		md_opt->regs.clkcon &= ~(CLKCON_UART1);
	
	md_opt->uart2 = uart2;
	if (uart2 == 0)
		md_opt->regs.clkcon &= ~(CLKCON_UART2);
	
	md_opt->sdi = sdi;
	if (sdi == 0)
		md_opt->regs.clkcon &= ~(CLKCON_SDI);
	
	md_opt->pwm = pwm;
	if (pwm == 0)
		md_opt->regs.clkcon &= ~(CLKCON_PWM);
	
	md_opt->usbh = usbh;
	if (usbh == 0)
		md_opt->regs.clkcon &= ~(CLKCON_USBH);
	
	md_opt->usbd = usbd;
	if (usbd == 0)
		md_opt->regs.clkcon &= ~(CLKCON_USBD);
	
	md_opt->regs.lpj = lpj_value[cpu];
	md_opt->lpj = lpj_value[cpu];

	return 0;
}

/* Fully determine the current machine-dependent operating point, and fill in a
   structure presented by the caller. This should only be called when the
   dpm_sem is held. This call can return an error if the system is currently at
   an operating point that could not be constructed by dpm_md_init_opt(). */

#define CLKDIVN_MASK	0x7
int s3c2440_dpm_get_opt(struct dpm_opt *opt)
{
	int i = 0;
	unsigned int mpllvalue = 0;
	struct dpm_md_opt *md_opt = &opt->md_opt;

	md_opt->v = 5000; /* hardwired for now. */
	md_opt->mpll = -1;
	
	mpllvalue = MPLLCON;	// get current mpllcon register value
	for (i = 0; i < MPLL_SIZE; i++)
	{
		if (mpllvalue == mpll_value[i])
			md_opt->cpu = md_opt->regs.cpu = i;
	}

	md_opt->clkdivn = CLKDIVN & CLKDIVN_MASK;
	md_opt->lcd = ck_is_enabled(ck_lcdc);
	md_opt->nand = ck_is_enabled(ck_nandc);
	md_opt->spi = ck_is_enabled(ck_spi);
	md_opt->iis = ck_is_enabled(ck_iis);
	md_opt->iic = ck_is_enabled(ck_iic);
	md_opt->adc = ck_is_enabled(ck_adc);
	md_opt->rtc = ck_is_enabled(ck_rtc);
	md_opt->gpio = ck_is_enabled(ck_gpio);
	md_opt->uart0 = ck_is_enabled(ck_uart0);
	md_opt->uart1 = ck_is_enabled(ck_uart1);
	md_opt->uart2 = ck_is_enabled(ck_uart2);
	md_opt->sdi = ck_is_enabled(ck_sdi);
	md_opt->pwm = ck_is_enabled(ck_pwm);
	md_opt->usbh = ck_is_enabled(ck_usbh);;
	md_opt->usbd = ck_is_enabled(ck_usbd);
	md_opt->lpj = loops_per_jiffy;
	md_opt->regs.lpj = loops_per_jiffy;
	md_opt->regs.mpllcon = MPLLCON;
	md_opt->regs.clkdivn = CLKDIVN;
	md_opt->regs.upllcon = UPLLCON;
	md_opt->regs.clkcon = CLKCON;
	md_opt->regs.clkslow = CLKSLOW;

	return 0;
}

#ifdef CONFIG_DPM_IDLE
/****************************************************************************
 *  DPM Idle Handler
 ****************************************************************************/

/* Check for pending external interrupts.  If so, the entry to a low-power
   idle is preempted. */
int return_from_idle_immediate(void)
{
	// jyhwang
	if(INTPND) 
		return 1;
	// JYHWANG
	return 0;
}
#endif

/****************************************************************************
 * Machine-dependent /proc/driver/dpm/md entries
 ****************************************************************************/

static inline int 
p5d(char *buf, unsigned mhz)
{
	return sprintf(buf, "%5d", mhz ); /* Round */
}

static int
dpm_proc_print_opt(char *buf, struct dpm_opt *opt)
{
#if 0
        int len = 0;
        struct dpm_md_opt *md_opt = &opt->md_opt;

        len += sprintf(buf + len, "%12s %9llu", 
                       opt->name, opt->stats.count);
        len += p5d(buf + len, md_opt->dpll);
        len += p5d(buf + len, md_opt->cpu);
        len += p5d(buf + len, md_opt->tc);
        len += p5d(buf + len, md_opt->per);
        len += p5d(buf + len, md_opt->dsp);
        len += p5d(buf + len, md_opt->dspmmu);
        len += p5d(buf + len, md_opt->lcd);
        len += sprintf(buf + len, "\n");
        return len;
#endif
	return -EINVAL;
}

int
read_proc_dpm_md_opts(char *page, char **start, off_t offset,
		      int count, int *eof, void *data)
{
#if 0
	int len = 0;
	int limit = offset + count;
	struct dpm_opt *opt;
	struct list_head *opt_list;
	
	/* FIXME: For now we assume that the complete table,
	 * formatted, fits within one page */
	if (offset >= PAGE_SIZE)
		return 0;

	if (dpm_lock_interruptible())
		return -ERESTARTSYS;

	if (!dpm_initialized)
		len += sprintf(page + len, "DPM is not initialized\n");
	else if (!dpm_enabled)
		len += sprintf(page + len, "DPM is disabled\n");
	else {
		len += sprintf(page + len,
			       "The active DPM policy is \"%s\"\n",
			       dpm_active_policy->name);
		len += sprintf(page + len, 
			       "The current operating point is \"%s\"\n",
			       dpm_active_opt->name);
	}
#if 0 // TODO
	len += sprintf(page + len, "The system clock speed is");
	len += p61f(page + len, bip->sys_speed / 1000);
	len += sprintf(page + len, " MHz\n\n");
#endif

	if (dpm_initialized) {
		len += sprintf(page + len, 
			       "Table of all defined operating points, "
			       "frequencies in MHz:\n");

		len += sprintf(page + len, 
				" Name           Count  DPLL  CPU  TC  PER  DSP  DSPMMU   LCD\n");

		list_for_each(opt_list, &dpm_opts) {
			opt = list_entry(opt_list, struct dpm_opt, list);
			if (len >= PAGE_SIZE)
				BUG();
			if (len >= limit)
				break;
			len += dpm_proc_print_opt(page + len, opt);
		}
		len += sprintf(page +len,"\nCurrent values DPLL\tCPU\tTC\tPER\tDSP\tDSPMMU\tLCD\n");
		len += sprintf(page +len,"\n                %d\t%d\t%d\t%d\t%d\t%d\t%d\n",
				ck_get_rate(ck_gen1), ck_get_rate(arm_ck),
				ck_get_rate(tc_ck), ck_get_rate(mpuper_ck),
				ck_get_rate(dsp_ck), ck_get_rate(dspmmu_ck),
				ck_get_rate(lcd_ck));
		//calibrate_delay();
	}
	dpm_unlock();
	*eof = 1;
	if (offset >= len)
		return 0;
	*start = page + offset;
	return min(count, len - (int)offset);
#endif
	return count;
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * /proc/driver/dpm/md/cmd (Write-only)
 *
 *  This is a catch-all, simple command processor for the Innovator DPM
 *  implementation. These commands are for experimentation and development
 *  _only_, and may leave the system in an unstable state.
 *
 *  No commands defined now.
 *
 ****************************************************************************/

int 
write_proc_dpm_md_cmd (struct file *file, const char *buffer,
		       unsigned long count, void *data)
{
	char *buf, *tok, *s;
	char *whitespace = " \t\r\n";
	int ret = 0;

	if (current->uid != 0)
		return -EACCES;
	if (count == 0)
		return 0;
	if (!(buf = kmalloc(count + 1, GFP_KERNEL)))
		return -ENOMEM;
	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}
	buf[count] = '\0';
	s = buf + strspn(buf, whitespace);
	tok = strsep(&s, whitespace);
	
	if (strcmp(tok, "define-me") == 0) {
		;
	} else {
		ret = -EINVAL;
	}
		kfree(buf);
	if (ret == 0)
		return count;
	else 
		return ret;
}

/*
 * Support a SMDK2410 board
 */
static int s3c2440_dpm_bd_init(void)
{
	return 0;
}

static void s3c2440_dpm_bd_exit(void)
{
	return;
}

static void s3c2440_dpm_bd_setup(void)
{
	dpm_bd.init				= s3c2440_dpm_bd_init;
	dpm_bd.exit				= s3c2440_dpm_bd_exit;
	dpm_bd.check_v			= NULL;
	dpm_bd.set_v_pre 		= NULL;
	dpm_bd.set_v_post 		= NULL;
}

/****************************************************************************
 * Initialization/Exit
 ****************************************************************************/

void s3c2440_dpm_cleanup(void)
{
	dpm_bd.exit();
}

extern void (*pm_idle)(void);   

int __init s3c2440_dpm_init(void)
{
	dpm_md.init				= NULL;
	dpm_md.init_opt			= s3c2440_dpm_init_opt;
	dpm_md.set_opt			= dpm_default_set_opt;
	dpm_md.get_opt			= s3c2440_dpm_get_opt;
	dpm_md.idle_set_parms	= NULL;
	dpm_md.cleanup			= s3c2440_dpm_cleanup;

	s3c2440_dpm_bd_setup();
	dpm_bd.init();
	
#ifdef CONFIG_DPM_UTIMER
	init_utimer(&set_opt_utimer);
#endif

#ifdef CONFIG_DPM_IDLE
	pm_idle = dpm_idle;
#endif

	printk(KERN_INFO "S3C2410 Dynamic Power Management\n");
	return 0;
}
__initcall(s3c2440_dpm_init);
