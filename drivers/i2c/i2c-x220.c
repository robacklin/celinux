/*
 * i2c-x220.c: adapter and algoritms driver for the I2C peripheral on the
 * ATI Xilleon 220.
 * 
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2002 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/i2c.h>

/* I2c API lifted from lld_i2c.c */

#if 1
/* M14.0 I2c API */
extern void* lld_GetLldInfoPtr (u32 ChipIndex);

extern u32 lld_i2c_open  (void* pdevice, u32 PortNum, u32 ClockRate,
			  u32 TimeLimit);
extern u32 lld_i2c_close (u32 Handle);
extern u32 lld_i2c_read  (u32 Handle, u8 AddrInfo[4], u8 *DataBuf,
			  u32 BufLen, u32 BusCntl);
extern u32 lld_i2c_write (u32 Handle, u8 AddrInfo[4], u8 *DataBuf,
			  u32 BufLen, u32 BusCntl);

#define lld_I2cOpen(a,b,c,d)    lld_i2c_open(lld_GetLldInfoPtr(a),b,c,d)
#define lld_I2cClose            lld_i2c_close
#define lld_I2cRead             lld_i2c_read
#define lld_I2cWrite            lld_i2c_write

#else
/* M10.2 I2c API */
extern u32 lld_I2cOpen  (u32 ChipIndex, u32 PortNum, u32 ClockRate,
			 u32 TimeLimit);
extern u32 lld_I2cClose (u32 Handle);
extern u32 lld_I2cRead  (u32 Handle, u8 AddrInfo[4], u8 *DataBuf,
			 u32 BufLen, u32 BusCntl);
extern u32 lld_I2cWrite (u32 Handle, u8 AddrInfo[4], u8 *DataBuf,
			 u32 BufLen, u32 BusCntl);
#endif

#define CORE_OK 0
/*
 * I2C control flags lifted from
 * drivers/hdtv2/common/atidrivers/core/include/ati_core.h.
 */
#define I2C_CTL_NORMAL      0x00    /* Normal I2C operation         */
#define I2C_CTL_NO_START    0x01    /* Do not generate start signal */
#define I2C_CTL_NO_STOP     0x02    /* Do not generate stop signal  */

#define NUM_PORTS 2

/* Module parameters */
#define DEFAULT_CHIPINDEX       0
#define DEFAULT_CLOCK      100000
#define DEFAULT_TIMELIMIT     500

static int ChipIndex = DEFAULT_CHIPINDEX;
static int ClockRate = DEFAULT_CLOCK;
static int TimeLimit = DEFAULT_TIMELIMIT;
static int i2c_debug = 0;

#define X220_MODULE_NAME "i2c-x220"

MODULE_PARM(ChipIndex, "i");
MODULE_PARM(ClockRate, "i");
MODULE_PARM(TimeLimit, "i");
MODULE_PARM(i2c_debug, "i");

MODULE_PARM_DESC(ChipIndex, "Xilleon 220 Chip Index Number");
MODULE_PARM_DESC(ClockRate, "I2C bus clock in Hz");
MODULE_PARM_DESC(TimeLimit, "I2C bus timeout in msec");
MODULE_PARM_DESC(i2c_debug, "I2C debug level");

#define	dbg(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __func__, ## args)

#define DEB(fmt, args...) if (i2c_debug>=1) dbg(fmt, ## args)
#define DEB2(fmt, args...) if (i2c_debug>=2) dbg(fmt, ## args)
#define DEB3(fmt, args...) if (i2c_debug>=3) dbg(fmt, ## args)
#define DEBPROTO(fmt, args...) if (i2c_debug>=9) dbg(fmt, ## args)

#define PFX X220_MODULE_NAME
#define err(format, arg...) printk(KERN_ERR PFX ": " format , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format , ## arg)
#define emerg(format, arg...) printk(KERN_EMERG PFX ": " format , ## arg)


/* Our private adapter structure */
struct x220_adapter {
	int ChipIndex;
	int Handle;
	int PortNum;
	int ClockRate;
	int TimeLimit;
};

static struct x220_adapter x220_adapter[2];


static int x220_open(struct i2c_adapter *adap)
{
	struct x220_adapter *x220_adap = adap->data;
	int handle;
	
	/* Open X220 I2C bus adapter */
	if (!(handle = lld_I2cOpen(x220_adap->ChipIndex,
				   x220_adap->PortNum,
				   x220_adap->ClockRate,
				   x220_adap->TimeLimit))) {
		err("open failed\n");
		return -ENXIO;
	}

	x220_adap->Handle = handle;
	return 0;
}

static int x220_close(struct i2c_adapter *adap)
{
	struct x220_adapter *x220_adap = adap->data;
	int ret = 0;
	
	/* Close X220 I2C bus adapter */
	if (lld_I2cClose(x220_adap->Handle) != CORE_OK) {
		err("close failed\n");
		ret = -ENXIO;
	}

	x220_adap->Handle = -1;
	return ret;
}

/* Algorithm layer starts here. */

static int x220_xfer(struct i2c_adapter *i2c_adap,
		     struct i2c_msg msgs[], 
		     int num)
{
	struct x220_adapter *adap = i2c_adap->data;
	struct i2c_msg *pmsg;
	int i, ret;
	u32 BusCntl;
	union {
		u8  b[4];
		u32 w;
	} AddrInfo;
	
	if ((ret = x220_open(i2c_adap)) < 0)
		return ret;
		
	for (i = 0; i < num; i++) {
		
		pmsg = &msgs[i];
		
		DEB2("%s %d bytes to 0x%02x - %d of %d messages\n",
		     pmsg->flags & I2C_M_RD ? "reading" : "writing",
		     pmsg->len, pmsg->addr, i + 1, num);
    
		DEB3("Msg %d, addr=0x%x, flags=0x%x, len=%d\n",
		     i, pmsg->addr, pmsg->flags, pmsg->len);

		BusCntl = I2C_CTL_NORMAL;
		if (pmsg->flags & I2C_M_NOSTART)
			BusCntl = I2C_CTL_NO_START;
		if (i + 1 < num)
			BusCntl |= I2C_CTL_NO_STOP;
		
		AddrInfo.w = (u32)pmsg->addr << 1;
		if (pmsg->flags & I2C_M_RD)
			AddrInfo.w |= 1;
		if (pmsg->flags & I2C_M_REV_DIR_ADDR)
			 AddrInfo.w ^= 1;
		if (pmsg->flags & I2C_M_TEN)
			AddrInfo.b[3] = 2;
		else
			AddrInfo.b[3] = 1;
		
		if (pmsg->flags & I2C_M_RD)
			ret = lld_I2cRead (adap->Handle, AddrInfo.b,
					   pmsg->buf, pmsg->len, BusCntl);
		else
			ret = lld_I2cWrite (adap->Handle, AddrInfo.b,
					    pmsg->buf, pmsg->len, BusCntl);
		if (ret != CORE_OK) {
			err("xfer failed, ret=%d\n", ret);
			break;
		}
	}
	
	x220_close(i2c_adap);
	return (i);
}

static int algo_control(struct i2c_adapter *adapter, 
			unsigned int cmd, unsigned long arg)
{
	return 0;
}

static u32 x220_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR | 
	       I2C_FUNC_PROTOCOL_MANGLING; 
}

/* -----exported algorithm data: -------------------------------------	*/

static struct i2c_algorithm x220_algo = {
	"X220 algorithm",
	I2C_ALGO_X220,
	x220_xfer,
	NULL,
	NULL,				/* slave_xmit		*/
	NULL,				/* slave_recv		*/
	algo_control,			/* ioctl		*/
	x220_func,			/* functionality	*/
};

/* 
 * registering functions to load algorithms at runtime 
 */
int x220_add_bus(struct i2c_adapter *adap, int portnum)
{
	struct x220_adapter *x220_adap = adap->data;
	
	DEB2("hw routines for %s registered.\n", adap->name);

	/* register new adapter to i2c module... */

	adap->id |= x220_algo.id;
	adap->algo = &x220_algo;

	adap->timeout = 100;		/* default values, should */
	adap->retries = 3;		/* be replaced by defines */

	/* Fill in our private adapter info */
	x220_adap->ChipIndex = ChipIndex;
	x220_adap->Handle    = -1;
	x220_adap->PortNum   = portnum;
	x220_adap->ClockRate = ClockRate;
	x220_adap->TimeLimit = TimeLimit;
	
	i2c_add_adapter(adap);

	return 0;
}


int x220_del_bus(struct i2c_adapter *adap)
{
	int res;

	if ((res = i2c_del_adapter(adap)) < 0)
		return res;

	DEB2("adapter unregistered: %s\n", adap->name);

	return 0;
}


/* Adapter layer starts here. */

static int x220_client_reg(struct i2c_client *client)
{
	return 0;
}


static int x220_client_unreg(struct i2c_client *client)
{
	return 0;
}


static void x220_inc_use(struct i2c_adapter *i2c_adap)
{
#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif
}


static void x220_dec_use(struct i2c_adapter *i2c_adap)
{
#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
}


static struct i2c_adapter adapter[2] = {
	{
		"Xilleon 220 I2C adapter 0",
		I2C_HW_X_X220,
		NULL,
		NULL,
		x220_inc_use,
		x220_dec_use,
		x220_client_reg,
		x220_client_unreg,
		&x220_adapter[0],
	},
	{
		"Xilleon 220 I2C adapter 1",
		I2C_HW_X_X220,
		NULL,
		NULL,
		x220_inc_use,
		x220_dec_use,
		x220_client_reg,
		x220_client_unreg,
		&x220_adapter[1],
	}
};

/* Called when the module is loaded.  This function starts the
 * cascade of calls up through the heirarchy of i2c modules (i.e. up to the
 *  algorithm layer and into to the core layer)
 */
static int __init x220_init(void) 
{
	int i;
	
	info("Initializing Xilleon 220 I2C adapter\n");

	for (i=0; i<NUM_PORTS; i++) {
		if (x220_add_bus(&adapter[i], i) < 0)
			return -ENODEV;
	}
	
	return 0;
}


static void x220_exit(void)
{
	int i;

	for (i=0; i<NUM_PORTS; i++) {
		x220_del_bus(&adapter[i]);
	}
}

EXPORT_NO_SYMBOLS;

MODULE_AUTHOR("Steve Longerbeam");
MODULE_DESCRIPTION("I2C-Bus adapter driver for the ATI Xilleon 220");

module_init(x220_init);
module_exit(x220_exit); 
