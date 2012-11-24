/*
 * i2c-necravine.c: adapter and algoritms driver for the I2C peripheral on the
 *                  NEC RavinE
 * 
 * Copyright 2003 mycable GmbH
 * Author: mycable GmbH
 *         	 Joerg Ritter jr@mycable.de 
 *
 *  based on Montavista's i2c-x220.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
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
#include <linux/pci.h>
#include <asm/au1000.h>


extern u32 necravine_fb_initialised;
#if 0

/* I2c API lifted from lld_i2c.c */
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
#endif

#define CORE_OK 0
/*
 * I2C control flags lifted from
 * drivers/hdtv2/common/atidrivers/core/include/ati_core.h.
 */
#define I2C_CTL_NORMAL      0x00    /* Normal I2C operation         */
#define I2C_CTL_NO_START    0x01    /* Do not generate start signal */
#define I2C_CTL_NO_STOP     0x02    /* Do not generate stop signal  */

#define PCI_RAVINE_VENDORID             0x1033
#define PCI_RAVINE_DEVICEID             0x0111

/* Module parameters */
#define DEFAULT_CHIPINDEX       0
#define DEFAULT_CLOCK      100000
#define DEFAULT_TIMELIMIT     500

static int ChipIndex = DEFAULT_CHIPINDEX;
static int ClockRate = DEFAULT_CLOCK;
static int TimeLimit = DEFAULT_TIMELIMIT;
static int i2c_debug = 1;

#define NECRAVINE_MODULE_NAME "i2c-necravine"

MODULE_PARM(ChipIndex, "i");
MODULE_PARM(ClockRate, "i");
MODULE_PARM(TimeLimit, "i");
MODULE_PARM(i2c_debug, "i");

MODULE_PARM_DESC(ChipIndex, "Chip Index Number");
MODULE_PARM_DESC(ClockRate, "I2C bus clock in Hz");
MODULE_PARM_DESC(TimeLimit, "I2C bus timeout in msec");
MODULE_PARM_DESC(i2c_debug, "I2C debug level");

//#define	dbg(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __func__, ## args)
#define	dbg(fmt, args...) printk(KERN_INFO "%s: " fmt, __func__, ## args)

#define DEB(fmt, args...) if (i2c_debug>=1) dbg(fmt, ## args)
#define DEB2(fmt, args...) if (i2c_debug>=2) dbg(fmt, ## args)
#define DEB3(fmt, args...) if (i2c_debug>=3) dbg(fmt, ## args)
#define DEBPROTO(fmt, args...) if (i2c_debug>=9) dbg(fmt, ## args)

#define PFX NECRAVINE_MODULE_NAME
#define err(format, arg...) printk(KERN_ERR PFX ": " format , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format , ## arg)
#define emerg(format, arg...) printk(KERN_EMERG PFX ": " format , ## arg)


/* Our private adapter structure */
struct necravine_adapter {
	int ChipIndex;
	int Handle;
	int PortNum;
	int ClockRate;
	int TimeLimit;
};

static struct necravine_adapter necravine_adapter[2];

/* ------------------------------------------------------------------------ */
/*    Low level I2C functions                                               */
/* ------------------------------------------------------------------------ */

static u32 ravine_regbase;

#define WRITE_I2C_REG(r, v) writel(v, ravine_regbase + r)
#define READ_I2C_REG(r)     readl(ravine_regbase + r)

#define RAVINE_I2CStatus               0x814
#define RAVINE_I2CCommand              0x818
#define RAVINE_I2CAddress              0x81C
#define RAVINE_I2CData                 0x820
#define RAVINE_I2CTiming               0x824

#define I2CSTS_BUSBUSY    0x00000001
#define I2CSTS_ACK        0x00000002
#define I2CSTS_SDA        0x00000008
#define I2CSTS_SCL        0x00000010
#define I2CCMD_START      0x00000001
#define I2CCMD_STOP       0x00000000
#define I2CADR_WRITE      0x0000004a
#define I2CADR_READ       0x0000004b
#define SIL164_WRITE      0x00000070
#define SIL164_READ       0x00000071

#define I2C_WAIT_BUSBUSY {volatile u32 dummy; while ((dummy = READ_I2C_REG(RAVINE_I2CStatus)) & I2CSTS_BUSBUSY);}
#define I2C_WAIT_ACK {volatile u32 dummy; while ((dummy = READ_I2C_REG(RAVINE_I2CStatus)) & I2CSTS_ACK);}


static void init_i2c (void)
{
	WRITE_I2C_REG(RAVINE_I2CTiming, 0xa);                  /* set I2C clock           */
        
	I2C_WAIT_BUSBUSY;
	DEB2("initialised\n");
    
}

#define I2C_DELAY 1000

static int write_i2c (unsigned char chipaddr, int num, unsigned char *data)
{
	int i, a;

	DEB2("chipaddr=%x num=%d", chipaddr, num);
	if (num <= 0) {
		DEB2(" skipping\n");
		return 0;
	}

	a = (chipaddr & 0x7f) << 1;
	I2C_WAIT_BUSBUSY;
	WRITE_I2C_REG(RAVINE_I2CAddress, a);
	WRITE_I2C_REG(RAVINE_I2CCommand, I2CCMD_START);
	
	udelay(I2C_DELAY);
	
	I2C_WAIT_BUSBUSY;

	udelay(I2C_DELAY);
	for (i = 0; i < num; ++i) {
		/* printk("%02x ", *data); */
		WRITE_I2C_REG(RAVINE_I2CData ,*data++);
		udelay(I2C_DELAY);
		I2C_WAIT_BUSBUSY;
		udelay(I2C_DELAY);
	}
	
	WRITE_I2C_REG(RAVINE_I2CCommand, I2CCMD_STOP);
	udelay(I2C_DELAY);
	if (i2c_debug >= 2) printk("\n");
	return 0;
}



static unsigned char read_i2c (unsigned char chipaddr, int num, unsigned char *data)
{

	unsigned char rc, a;
	int i;
	
	DEB2("chipaddr=%d num=%d data=0x", chipaddr, num);
	I2C_WAIT_BUSBUSY;
	
	a = (chipaddr & 0x7f) << 1;
	WRITE_I2C_REG(RAVINE_I2CAddress , a | 0x01);
	WRITE_I2C_REG(RAVINE_I2CCommand ,I2CCMD_START);
	
	for (i = 0; i < num; ++i) {
		I2C_WAIT_BUSBUSY;
		rc = READ_I2C_REG(RAVINE_I2CData);
		DEB("%02x ", rc);
		*data++ = rc;
	}
	
	WRITE_I2C_REG(RAVINE_I2CCommand, I2CCMD_STOP);
	I2C_WAIT_BUSBUSY;
	
	DEB("\n");
	
	return 0;
}

static unsigned char write_read_i2c (unsigned char chipaddr, int num, unsigned char *data)
{

	unsigned char rc, a;

		
	DEB2("chipaddr=0x%x num=%d data[0]=0x%x\n", chipaddr, num, *data);
	
	a = (chipaddr & 0x7f) << 1;
	
	// write
	I2C_WAIT_BUSBUSY;
	WRITE_I2C_REG(RAVINE_I2CAddress , a);
	WRITE_I2C_REG(RAVINE_I2CCommand ,I2CCMD_START);
	
	I2C_WAIT_BUSBUSY;
	I2C_WAIT_ACK;
	
	I2C_WAIT_BUSBUSY;
	WRITE_I2C_REG(RAVINE_I2CData ,*data);
	
	I2C_WAIT_BUSBUSY;
	I2C_WAIT_ACK;
	
	// read
	I2C_WAIT_BUSBUSY;
	WRITE_I2C_REG(RAVINE_I2CAddress , a | 0x01 );
	WRITE_I2C_REG(RAVINE_I2CCommand ,I2CCMD_START);
	
	I2C_WAIT_BUSBUSY;
	I2C_WAIT_ACK;
	
	I2C_WAIT_BUSBUSY;
	rc = READ_I2C_REG(RAVINE_I2CData);
	*data = rc;
	DEB2(" read=%02x \n", rc); 
	
	WRITE_I2C_REG(RAVINE_I2CCommand, I2CCMD_STOP);
	
	return(0);
}


static int combined_transaction_i2c (struct i2c_msg msgs[], int num)
{

	int i, j;
	int ret = 0;
	u8 chipaddr, rc, a;
	struct i2c_msg *pmsg;

	
	DEB2("num=%d \n", num);

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		chipaddr = (u8) pmsg->addr;
		a = (chipaddr & 0x7f) << 1;
		DEB2("  msg%d len=%d addr=0x%x flags=0x%x\n", i, pmsg->len, chipaddr, pmsg->flags);
		if (pmsg->flags & I2C_M_RD) {
			/* read */
			I2C_WAIT_BUSBUSY;
			WRITE_I2C_REG(RAVINE_I2CAddress , a | 0x01 );
			WRITE_I2C_REG(RAVINE_I2CCommand ,I2CCMD_START);
			
			for (j = 0; j < pmsg->len; ++j) {
				I2C_WAIT_BUSBUSY;
				I2C_WAIT_ACK;
				
				I2C_WAIT_BUSBUSY;
				rc = READ_I2C_REG(RAVINE_I2CData);
				pmsg->buf[j] = rc;
				DEB2("data=%02x \n", rc);
			}
		} else if (!(pmsg->flags & I2C_M_RD)) {
			/* write */
			I2C_WAIT_BUSBUSY;
			WRITE_I2C_REG(RAVINE_I2CAddress , a);
			WRITE_I2C_REG(RAVINE_I2CCommand ,I2CCMD_START);
			
			for (j = 0; j < pmsg->len; ++j) {
				I2C_WAIT_BUSBUSY;
				I2C_WAIT_ACK;
				
				I2C_WAIT_BUSBUSY;
				WRITE_I2C_REG(RAVINE_I2CData , pmsg->buf[j]);
				
				I2C_WAIT_BUSBUSY;
				I2C_WAIT_ACK;
			}
		}
	}
	
	WRITE_I2C_REG(RAVINE_I2CCommand, I2CCMD_STOP);
	
	return ret;
}

/* ------------------------------------------------------------------------ */


/* Algorithm layer starts here. */


static int necravine_xfer(struct i2c_adapter *i2c_adap,
		     struct i2c_msg msgs[], 
		     int num)
{
	struct i2c_msg *pmsg;
	int i = 0;
	int ret = 0;
	
	pmsg = &msgs[i];
	/*DEB2(" num=%d flags=0x%x\n", num, pmsg->flags);*/

	if (num > 1) {
		/* Combined transaction (read/write) */
		DEB2(" Call combined transaction\n");
		ret = combined_transaction_i2c(msgs, num);
	} else if ((num == 1) && (pmsg->flags & I2C_M_RD)) {
		 /* Read only */
		/* Tell device to begin reading data from the master data */
		DEB2(" Call adapter's read\n");
		ret = read_i2c(pmsg->addr, pmsg->len, pmsg->buf);
	} else if ((num == 1) && (!(pmsg->flags & I2C_M_RD))) { 	
		/* Write only */
		/*
		 * Write data to master data buffers and tell our device
		 * to begin transmitting
		 */
		/*DEB2( " Call adapter's write function\n");*/
		ret = write_i2c(pmsg->addr, pmsg->len, pmsg->buf);
	}

	return ret;
}

static int algo_control(struct i2c_adapter *adapter, 
			unsigned int cmd, unsigned long arg)
{
	printk("algo_control: cmd %x\n", cmd);
	return 0;
}

static u32 necravine_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR | 
	       I2C_FUNC_PROTOCOL_MANGLING; 
}

/* -----exported algorithm data: -------------------------------------	*/

static struct i2c_algorithm necravine_algo = {
	"NECRAVINE algorithm",
	I2C_ALGO_EXP,
	necravine_xfer,
	NULL,
	NULL,				/* slave_xmit		*/
	NULL,				/* slave_recv		*/
	algo_control,			/* ioctl		*/
	necravine_func,			/* functionality	*/
};

/* 
 * registering functions to load algorithms at runtime 
 */
int necravine_add_bus(struct i2c_adapter *adap, int portnum)
{
	struct necravine_adapter *necravine_adap = adap->data;
	
	DEB2("hw routines for %s registered.\n", adap->name);

	/* register new adapter to i2c module... */

	adap->id |= necravine_algo.id;
	adap->algo = &necravine_algo;

	adap->timeout = 100;		/* default values, should */
	adap->retries = 3;		/* be replaced by defines */

	/* Fill in our private adapter info */
	necravine_adap->ChipIndex = ChipIndex;
	necravine_adap->Handle    = -1;
	necravine_adap->PortNum   = portnum;
	necravine_adap->ClockRate = ClockRate;
	necravine_adap->TimeLimit = TimeLimit;
	
	init_i2c();

	i2c_add_adapter(adap);
    

	return 0;
}


int necravine_del_bus(struct i2c_adapter *adap)
{
	int res;

	if ((res = i2c_del_adapter(adap)) < 0)
		return res;

	DEB2("adapter unregistered: %s\n", adap->name);

	return 0;
}


/* Adapter layer starts here. */

static int necravine_client_reg(struct i2c_client *client)
{
	return 0;
}


static int necravine_client_unreg(struct i2c_client *client)
{
	return 0;
}


static void necravine_inc_use(struct i2c_adapter *i2c_adap)
{
#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif
}


static void necravine_dec_use(struct i2c_adapter *i2c_adap)
{
#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
}


static struct i2c_adapter adapter = {
	"NEC RavinE I2C adapter",
	0,                            // FIXME define I2C_HW_NECRAVINE,
	NULL,
	NULL,
	necravine_inc_use,
	necravine_dec_use,
	necravine_client_reg,
	necravine_client_unreg,
	&necravine_adapter[0],
};

/* Called when the module is loaded.  This function starts the
 * cascade of calls up through the heirarchy of i2c modules (i.e. up to the
 *  algorithm layer and into to the core layer)
 */
static int __init necravine_i2c_init(void) 
{
	u32 regbase_phys;
	int i;
	
	struct pci_dev *pdev = NULL;
	
	info("i2c-necravine.o: version %s\n", "0.1");

	if (!(pdev = pci_find_device(PCI_RAVINE_VENDORID,
				     PCI_RAVINE_DEVICEID, pdev))) {
		warn("  RavinE not found\n");
		return -ENODEV;
	} else {
		DEB2("  RavinE found\n");
		if (necravine_fb_initialised == 0) {
			warn("necravinefb not yet initialized, exiting\n");
			return -ENODEV;
		}
	}
	regbase_phys = pci_resource_start(pdev, 0) + 0x1fff000;
	ravine_regbase = (u32) ioremap_nocache(regbase_phys, 0x2000);
	DEB2("regbase_phys =0x%x, ravine_regbase=0x%x\n", regbase_phys, ravine_regbase);

	if (necravine_add_bus(&adapter, 0) < 0)
		return -ENODEV;

#if 0
	/* some test code for initializing and testing sii164 panel link transmitter */
	/* without chipdriver */
	u8 mydata[8];
	
	mydata[0] = 0x08;
	mydata[1] = 0x3f;
	write_i2c(0x38, 2, mydata);
	mydata[0] = 0x09;
	mydata[1] = 0x07;
	write_i2c(0x38, 2, mydata);
	mydata[0] = 0x0a;
	mydata[1] = 0x06;
	write_i2c(0x38, 2, mydata);
	mydata[0] = 0x0c;
	mydata[1] = 0x89;
	write_i2c(0x38, 2, mydata);
	mydata[0] = 0x0d;
	mydata[1] = 0x03;
	write_i2c(0x38, 2, mydata);
	mydata[0] = 0x0e;
	mydata[1] = 0x00;
	write_i2c(0x38, 2, mydata);
	mydata[0] = 0x0f;
	mydata[1] = 0x00;
	write_i2c(0x38, 2, mydata);
	
	for (i = 0; i < 15; ++i) {   
		mydata[0] = i;
		write_read_i2c(0x38, 1, mydata);
	}
#endif
	
	return 0;
}

static void necravine_i2c_exit(void)
{
	necravine_del_bus(&adapter);
}

EXPORT_NO_SYMBOLS;

MODULE_AUTHOR("Joerg Ritter, mycable GmbH <jr@mycable.de>");
MODULE_DESCRIPTION("I2C-Bus adapter driver for the NEC RavinE");
MODULE_LICENSE("GPL");

module_init(necravine_i2c_init);
module_exit(necravine_i2c_exit); 
