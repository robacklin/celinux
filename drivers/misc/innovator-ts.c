/*
 * innovator_ts.c
 *
 * Touch screen driver for the TI Innovator (OMAP1510).
 *
 * The touchscreen hardware on the Innovator consists of an FPGA
 * register which is bit-banged to generate SPI-like transactions
 * to an ADS7846 touch screen controller.
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2002 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/ioport.h>       /* request_region */
#include <asm/uaccess.h>        /* get_user,copy_to_user */
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware.h>

#define TS_NAME "innovator-ts"
#define TS_MINOR 14
#define PFX TS_NAME

//#define INNOVATOR_TS_DEBUG

#ifdef INNOVATOR_TS_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG PFX ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) printk(KERN_ERR PFX ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format "\n" , ## arg)

/* Boot options */
static int samples = 1; /* 1-16 in even increments */
MODULE_PARM(samples, "i");
MODULE_PARM_DESC(samples, "number of samples to acquire in each dimension for averaging (1,2,4,6,...,16 default=1)");

// The Touch Screen Register on Innovator FPGA
#define FPGA_TS_BCLK     (1<<0)
#define FPGA_TS_BDIN     (1<<1)
#define FPGA_TS_BCS      (1<<2)
#define FPGA_TS_BBUSY    (1<<3)
#define FPGA_TS_BOUT     (1<<4)
#define FPGA_TS_BPENDOWN (1<<5)

// ADS7846 Control Byte bit defines
#define ADS7846_S         (1<<7)
#define ADS7846_ADDR_BIT  4
#define ADS7846_ADDR_MASK (0x7<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_X  (0x5<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_Y  (0x1<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_Z1 (0x3<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_Z2 (0x4<<ADS7846_ADDR_BIT)
#define ADS7846_8BITS     (1<<3)
#define ADS7846_12BITS    0
#define ADS7846_SER       (1<<2)
#define ADS7846_DFR       0
#define ADS7846_PWR_BIT   0
#define   ADS7846_PD      0
#define   ADS7846_ADC_ON  (0x1<<ADS7846_PWR_BIT)
#define   ADS7846_REF_ON  (0x2<<ADS7846_PWR_BIT)
#define   ADS7846_REF_ADC_ON (0x3<<ADS7846_PWR_BIT)

#define MEASURE_12BIT_X \
  (ADS7846_S | ADS7846_MEASURE_X | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)
#define MEASURE_12BIT_Y \
  (ADS7846_S | ADS7846_MEASURE_Y | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)
#define MEASURE_12BIT_Z1 \
  (ADS7846_S | ADS7846_MEASURE_Z1 | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)
#define MEASURE_12BIT_Z2 \
  (ADS7846_S | ADS7846_MEASURE_Z2 | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)


/*
 * Which pressure equation to use from ADS7846 datasheet.
 * The first equation requires knowing only the X plate
 * resistance, but needs 4 measurements (X, Y, Z1, Z2).
 * The second equation requires knowing both X and Y plate
 * resistance, but only needs 3 measurements (X, Y, Z1).
 */
enum {
	PRESSURE_EQN_1 = 0,
	PRESSURE_EQN_2
};


/*
 * The touch screen's X and Y plate resistances, used by
 * pressure equations.
 */
#define DEFAULT_X_PLATE_OHMS 419
#define DEFAULT_Y_PLATE_OHMS 486

/*
 * The definition of the following structure is copied from ucb1x00-ts.c
 * so the touchscreen will "just work" with tslib.
 */
struct ts_event {
        u16 pressure;
        u16 x;
        u16 y;
        u16 pad;
        struct timeval stamp;
};


#define EVENT_BUFSIZE 32 // must be power of two

struct innovator_ts_t {
	// The X and Y plate resistance, needed to calculate pressure
	int x_plate_ohms, y_plate_ohms;
	int pressure_eqn;      // eqn to use for pressure calc
	int pendown_irq;       // IRQ of pendown interrupt
	int pen_is_down;       // 1 = pen is down, 0 = pen is up
	int irq_enabled;
	struct ts_event event_buf[EVENT_BUFSIZE];// The event queue
	int nextIn, nextOut;
	int event_count;
	struct fasync_struct *fasync;     // asynch notification
	struct timer_list acq_timer;      // Timer for triggering acquisitions
	wait_queue_head_t wait;           // read wait queue
	spinlock_t lock;
};

static struct innovator_ts_t innovator_ts;

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */
#include <linux/device.h>

static int ts_suspend(struct device *dev, u32 state, u32 level);
static int ts_resume(struct device *dev, u32 level);

static struct device_driver ts_driver_ldm = {
       name:      "innovator-touch_screen",
       devclass:  NULL,
       probe:     NULL,
       suspend:   ts_suspend,
       resume:    ts_resume,
       remove:    NULL,
};

static struct device ts_device_ldm = {
       name: "Innovator Touch Screen",
       bus_id: "innovator-ts",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void ts_ldm_driver_register(void)
{
   extern void dsp_public_driver_register(struct device_driver *driver);

   dsp_public_driver_register(&ts_driver_ldm);
}

static void ts_ldm_device_register(void)
{
   extern void dsp_public_device_register(struct device *device);

   dsp_public_device_register(&ts_device_ldm);
}

static void ts_ldm_driver_unregister(void)
{
   extern void dsp_public_driver_unregister(struct device_driver *driver);

   dsp_public_driver_unregister(&ts_driver_ldm);
}

static void ts_ldm_device_unregister(void)
{
   extern void dsp_public_device_unregister(struct device *device);

   dsp_public_device_unregister(&ts_device_ldm);
}

#endif /* MVL-CEE */

static inline u8 fpga_ts_read(void)
{
	return inb(OMAP1510P1_FPGA_TOUCHSCREEN);
}

static inline void fpga_ts_write(u8 val)
{
	outb(val, OMAP1510P1_FPGA_TOUCHSCREEN);
}

static inline void fpga_ts_set_bits(u8 mask)
{
	fpga_ts_write(fpga_ts_read() | mask);
}

static inline void fpga_ts_clear_bits(u8 mask)
{
	fpga_ts_write(fpga_ts_read() & ~mask);
}

static inline void CS_H(void)
{
	// EPLD inverts active low signals.
	fpga_ts_clear_bits(FPGA_TS_BCS);
}

static inline void CS_L(void)
{
	fpga_ts_set_bits(FPGA_TS_BCS);
}

static inline void SCLK_L(void)
{
	fpga_ts_clear_bits(FPGA_TS_BCLK);
}

static inline void SCLK_H(void)
{
	fpga_ts_set_bits(FPGA_TS_BCLK);
}

static inline void SDI_L(void)
{
	fpga_ts_clear_bits(FPGA_TS_BDIN);
}

static inline void SDI_H(void)
{
	fpga_ts_set_bits(FPGA_TS_BDIN);
}

static inline int BUSY(void)
{
	return (((fpga_ts_read() & FPGA_TS_BBUSY) == 0) ? 1 : 0) ;
}

static inline u8 DOUT(void)
{	 
	return ((fpga_ts_read() & FPGA_TS_BOUT) ? 1 : 0) ;
}

static inline int PenIsDown(void)
{
	return ((fpga_ts_read() & FPGA_TS_BPENDOWN) ? 1 : 0) ;
}


static u16 ads7846_do(u8 cmd)
{  
	int i;
	u16 val=0;

	SCLK_L() ;
	SDI_L();
	CS_L() ;	// enable the chip select

	// send the command to the ADS7846
	for (i=0; i<8; i++ ) {
		if (cmd & 0x80)
			SDI_H();
		else
			SDI_L();   // prepare the data on line sdi OR din

		SCLK_H() ;      // clk in the data
		cmd <<= 1 ;
		SCLK_L() ;
	}

	SDI_L();
	while (BUSY())
		;

	// now read returned data
	for (i=0 ; i<16 ; i++ ) {
		SCLK_L() ;
		
		if (i < 12) {
			val <<= 1 ;
			val |= DOUT();
		}
		SCLK_H() ;
	}

	SCLK_L() ;
	CS_H() ;   // disable the chip select

	return val;
}

// hold the spinlock before calling.
static void event_add(struct innovator_ts_t* ts,
		      struct ts_event* event)
{
	// add this event to the event queue
	ts->event_buf[ts->nextIn] = *event;
	ts->nextIn = (ts->nextIn + 1) & (EVENT_BUFSIZE - 1);
	if (ts->event_count < EVENT_BUFSIZE) {
		ts->event_count++;
	} else {
		// throw out the oldest event
		ts->nextOut = (ts->nextOut + 1) & (EVENT_BUFSIZE - 1);
	}
	
	// async notify
	if (ts->fasync)
		kill_fasync(&ts->fasync, SIGIO, POLL_IN);
	// wake up any read call
	if (waitqueue_active(&ts->wait))
		wake_up_interruptible(&ts->wait);
}

static int event_pull(struct innovator_ts_t* ts,
		      struct ts_event* event)
{
	unsigned long flags;
	int ret;
	
	spin_lock_irqsave(&ts->lock, flags);
	ret = ts->event_count;
	if (ts->event_count) {
		*event = ts->event_buf[ts->nextOut];
		ts->nextOut = (ts->nextOut + 1) & (EVENT_BUFSIZE - 1);
		ts->event_count--;
	}
	spin_unlock_irqrestore(&ts->lock, flags);

	return ret;
}


static void pendown_interrupt(int irq, void * dev_id, struct pt_regs * regs)
{
	struct innovator_ts_t* ts = dev_id;

	dbg("pen down");

	spin_lock(&ts->lock);

	if (ts->irq_enabled) {
		ts->irq_enabled = 0;
		disable_irq(irq);
	}

	ts->pen_is_down = 1;
	// restart acquire
	ts->acq_timer.expires = jiffies + HZ / 100;
	add_timer(&ts->acq_timer);

	spin_unlock(&ts->lock);
}


/*
 * Acquire Raw pen coodinate data and compute touch screen
 * pressure resistance. Hold spinlock when calling.
 */
static void AcquireEvent(struct innovator_ts_t* ts,
			 struct ts_event* event)
{
	unsigned long x_raw=0, y_raw=0, z1_raw=0, z2_raw=0;
	unsigned long Rt = 0;
	int i;
	
	for (i=0; i<samples; i++)
		y_raw += ads7846_do(MEASURE_12BIT_X);
	for (i=0; i<samples; i++)
		x_raw += ads7846_do(MEASURE_12BIT_Y); 
	for (i=0; i<samples; i++)
		z1_raw += ads7846_do(MEASURE_12BIT_Z1); 

	if (samples > 1) {
		y_raw = (y_raw + (samples>>1)) / samples;
		x_raw = (x_raw + (samples>>1)) / samples;
		z1_raw = (z1_raw + (samples>>1)) / samples;
	}
	
	if (ts->pressure_eqn == PRESSURE_EQN_1) {
		for (i=0; i<samples; i++)
			z2_raw += ads7846_do(MEASURE_12BIT_Z2); 
		if (samples > 1)
			z2_raw = (z2_raw + (samples>>1)) / samples;
	}
	
	// Calculate touch pressure resistance
	if (z1_raw) {
		if (ts->pressure_eqn == PRESSURE_EQN_1) {
			Rt = (ts->x_plate_ohms * (u32)x_raw *
			      ((u32)z2_raw - (u32)z1_raw)) / (u32)z1_raw;
		} else {
			Rt = (ts->x_plate_ohms * (u32)x_raw *
			      (4096 - (u32)z1_raw)) / (u32)z1_raw;
			Rt -= (ts->y_plate_ohms * (u32)y_raw);
		}

		Rt = (Rt + 2048) >> 12; // round up to nearest ohm
	}

	event->x = x_raw;
	event->y = y_raw;
	event->pressure = (u16)Rt;
	// timestamp this new event.
	do_gettimeofday(&event->stamp);
}


/*
 * Raw X,Y,pressure acquisition timer function. It gets scheduled
 * only while pen is down. Its duration between calls is the polling
 * rate.
 */
static void
innovator_acq_timer(unsigned long data)
{
	struct innovator_ts_t* ts = (struct innovator_ts_t*)data;
	unsigned long flags;
	struct ts_event event;
	int pen_was_down = ts->pen_is_down;
	
	spin_lock_irqsave(&ts->lock, flags);

	if (PenIsDown()) {
		ts->pen_is_down = 1;
		AcquireEvent(ts, &event);
		event_add(ts, &event);

		// schedule next acquire
		ts->acq_timer.expires = jiffies + HZ / 100;
		add_timer(&ts->acq_timer);
	} else {
		if (!ts->irq_enabled) {
			ts->irq_enabled = 1;
			enable_irq(ts->pendown_irq);
		}
		ts->pen_is_down = 0;
		if (pen_was_down) {
			dbg("pen up");
			event.x = event.y = event.pressure = 0;
			do_gettimeofday(&event.stamp);
			event_add(ts, &event);
		}
	}
	
	spin_unlock_irqrestore(&ts->lock, flags);
}


/* +++++++++++++ File operations ++++++++++++++*/

static int
innovator_fasync(int fd, struct file *filp, int mode)
{
	struct innovator_ts_t* ts = (struct innovator_ts_t*)filp->private_data;
	return fasync_helper(fd, filp, mode, &ts->fasync);
}


static unsigned int
innovator_poll(struct file * filp, poll_table * wait)
{
	struct innovator_ts_t* ts = (struct innovator_ts_t*)filp->private_data;
	poll_wait(filp, &ts->wait, wait);
	if (ts->event_count)
		return POLLIN | POLLRDNORM;
	return 0;
}

static ssize_t
innovator_read(struct file * filp, char * buffer, size_t count, loff_t * ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct innovator_ts_t* ts = (struct innovator_ts_t*)filp->private_data;
	char *ptr = buffer;
	struct ts_event event;
	int err = 0;

	add_wait_queue(&ts->wait, &wait);
	while (count >= sizeof(struct ts_event)) {
		err = -ERESTARTSYS;
		if (signal_pending(current))
			break;

		if (event_pull(ts, &event)) {
			err = copy_to_user(ptr, &event,
					   sizeof(struct ts_event));
			if (err)
				break;
			ptr += sizeof(struct ts_event);
			count -= sizeof(struct ts_event);
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
			err = -EAGAIN;
			if (filp->f_flags & O_NONBLOCK)
				break; 
			schedule();
		}
	}
	
	current->state = TASK_RUNNING;
	remove_wait_queue(&ts->wait, &wait);
	
	return ptr == buffer ? err : ptr - buffer;
}


static int
innovator_open(struct inode * inode, struct file * filp)
{
	struct innovator_ts_t* ts;
	unsigned long flags;
	int retval;
	
	filp->private_data = ts = &innovator_ts;

	spin_lock_irqsave(&ts->lock, flags);

	ts->pen_is_down = 0; // start with pen up
	
	// flush event queue
	ts->nextIn = ts->nextOut = ts->event_count = 0;
	
	// Init acquisition timer function
	init_timer(&ts->acq_timer);
	ts->acq_timer.function = innovator_acq_timer;
	ts->acq_timer.data = (unsigned long)ts;

	ts->irq_enabled = 1;

	spin_unlock_irqrestore(&ts->lock, flags);

	/* Since ts interrupt can happen immediately after request_irq,
	 * we wait until we've completed init of all relevent driver
	 * state variables. Now we grab the PenDown IRQ
	 */
	retval = request_irq(ts->pendown_irq, &pendown_interrupt,
			     0, TS_NAME, ts);
	if (retval) {
		err("unable to get PenDown IRQ %d", ts->pendown_irq);
		return retval;
	}

	MOD_INC_USE_COUNT;
	return 0;
}

static int
innovator_release(struct inode * inode, struct file * filp)
{
	struct innovator_ts_t* ts = (struct innovator_ts_t*)filp->private_data;

	free_irq(ts->pendown_irq, ts);
	innovator_fasync(-1, filp, 0);
	del_timer_sync(&ts->acq_timer);

	MOD_DEC_USE_COUNT;
	return 0;
}


static struct file_operations ts_fops = {
	owner:          THIS_MODULE,
	read:           innovator_read,
	poll:           innovator_poll,
	fasync:         innovator_fasync,
	open:		innovator_open,
	release:	innovator_release,
};

/* +++++++++++++ End File operations ++++++++++++++*/

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */

/* DPM power management functions */

static int ts_suspend(struct device *dev, u32 state, u32 level)
{
  
  switch(level)
  {
     case SUSPEND_POWER_DOWN: 

       del_timer_sync(&innovator_ts.acq_timer);

       break;
  }

  return 0;

}

static int ts_resume(struct device *dev, u32 level)
{
 
  switch(level)
  {
     case RESUME_POWER_ON:
    
       innovator_ts.pen_is_down = 0;
       init_timer(&innovator_ts.acq_timer);
       innovator_ts.acq_timer.function = innovator_acq_timer;
       innovator_ts.acq_timer.data = (unsigned long) &innovator_ts;         

       break;
  }

  return 0;
}

#endif

static struct miscdevice innovator_ts_dev = {
	minor:  TS_MINOR,
	name:   TS_NAME,
	fops:   &ts_fops,
};

int __init
innovatorts_init_module(void)
{
	struct innovator_ts_t* ts = &innovator_ts;
	int ret;

	/* register our character device */
	if ((ret = misc_register(&innovator_ts_dev)) < 0) {
		err("can't register misc device");
		return ret;
	}
	info("registered");

	memset(ts, 0, sizeof(struct innovator_ts_t));
	init_waitqueue_head(&ts->wait);
	spin_lock_init(&ts->lock);

	if (samples < 1)
		samples = 1;
	if (samples > 16)
		samples = 16;
	if (samples > 1 && (samples & 1))
		samples++; /* make even */

	ts->pendown_irq = INT_FPGA_TS;
	ts->pressure_eqn = PRESSURE_EQN_1;
	// init X and Y plate resistances
	ts->x_plate_ohms = DEFAULT_X_PLATE_OHMS;
	ts->y_plate_ohms = DEFAULT_Y_PLATE_OHMS;

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
        ts_ldm_device_register();
        ts_ldm_driver_register();
#endif /* MVL-CCE */

	return 0;
}

void
innovatorts_cleanup_module(void)
{

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
        ts_ldm_device_unregister();
        ts_ldm_driver_unregister();
#endif /* MVL-CCE */

	misc_deregister(&innovator_ts_dev);
}

#ifndef MODULE
/*
 * To set number of samples to acquire in each dimension for averaging
 * using a statically built driver, use the kernel boot option:
 *     ts=samples:<N>
 */
static int __init ts_setup(char *options)
{
	char * this_opt;

	if (!options || !*options)
		return 0;
	
	for(this_opt=strtok(options, ",");
	    this_opt; this_opt=strtok(NULL, ",")) {
		if (!strncmp(this_opt, "samples:", 8)) {
			samples = simple_strtoul(this_opt+8, NULL, 0);
		}
	}
	
	return 1;
}
__setup("ts=", ts_setup);
#endif /* !MODULE */

/* Module information */
MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Innovator/ADS7846 Touch Screen Driver");
MODULE_LICENSE("GPL");

module_init(innovatorts_init_module);
module_exit(innovatorts_cleanup_module);
