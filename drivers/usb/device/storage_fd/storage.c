/*
 * storage_fd/storage.c
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
 *
 * Copyright (C) 2002 Toshiba Corporation
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * This module implements USB Mass Storage Class
 * (SubClass: RBC, Transport: Bulk-Only)
 *
 * Usage:
 */

#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"


MODULE_AUTHOR("sl@lineo.com, tbr@lineo.com, TOSHIBA Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("USB Device Mass Storage Function");

#define STORAGE_MOD_NAME	"storage_fd"
USBD_MODULE_INFO(STORAGE_MOD_NAME " 0.1-beta");

#ifndef MODULE
#undef GET_USE_COUNT
#define GET_USE_COUNT(foo) 1
#endif

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/netdevice.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <scsi/scsi.h>

#define __KERNEL_SYSCALLS__
static int errno;
#include <asm/unistd.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-debug.h"
#include "../usbd-inline.h"
#include "../usbd-arch.h"

#define MAX_STORAGES	1
#define STORAGE_DEFAULT_FILENAME	"/dev/ram0"
#define STORAGE_BLOCK_SIZE	512
#define STORAGE_PROC_NAME	"driver/" STORAGE_MOD_NAME

#define USB_REQ_BO_MASS_STORAGE_RESET	0xFF
#define USB_REQ_GET_MAX_LUN		0xFE

#if 0
/* RBC 6.1 states "RBC devices are identified by a PERIPHERAL DEVICE
 * TYPE 0Eh" */
#define PERIPHERAL_DEVICE_TYPE	0x0e
#else
/* XXX: Linux sd,scsi stack seems to expect this value... */
#define PERIPHERAL_DEVICE_TYPE	0x00
#endif

#if !defined (CONFIG_USBD_VENDORID) && !defined(CONFIG_USBD_STORAGE_VENDORID)
        #error No Vendor ID
#endif
#if !defined (CONFIG_USBD_PRODUCTID) && !defined(CONFIG_USBD_STORAGE_PRODUCTID)
        #error No Product ID
#endif


#if CONFIG_USBD_STORAGE_VENDORID
        #undef CONFIG_USBD_VENDORID
        #define CONFIG_USBD_VENDORID CONFIG_USBD_STORAGE_VENDORID
#endif

#if CONFIG_USBD_STORAGE_PRODUCTID
        #undef CONFIG_USBD_PRODUCTID
        #define CONFIG_USBD_PRODUCTID CONFIG_USBD_STORAGE_PRODUCTID
#endif

#ifndef CONFIG_USBD_SERIAL_NUMBER_STR
        #define CONFIG_USBD_SERIAL_NUMBER_STR	"000000000000"
#endif


#ifdef CONFIG_USBD_SELFPOWERED
        #define BMATTRIBUTE BMATTRIBUTE_RESERVED | BMATTRIBUTE_SELF_POWERED
        #define BMAXPOWER 0
#else
        #define BMATTRIBUTE BMATTRIBUTE_RESERVED
        #define BMAXPOWER CONFIG_USBD_MAXPOWER
#endif


/*
 * setup some default values for pktsizes and endpoint addresses.
 */

#ifndef CONFIG_USBD_STORAGE_OUT_PKTSIZE
        #define CONFIG_USBD_STORAGE_OUT_PKTSIZE             64
#endif

#ifndef CONFIG_USBD_STORAGE_IN_PKTSIZE
        #define CONFIG_USBD_STORAGE_IN_PKTSIZE              64
#endif

#ifndef CONFIG_USBD_STORAGE_OUT_ENDPOINT
    #define CONFIG_USBD_STORAGE_OUT_ENDPOINT                1
#endif

#ifndef CONFIG_USBD_STORAGE_IN_ENDPOINT
    #define CONFIG_USBD_STORAGE_IN_ENDPOINT                 2
#endif


/*
 * check for architecture specific endpoint configurations
 */

#if     defined(ABS_OUT_ADDR) 
        #warning
        #warning USING ABS ENDPOINT OUT ADDRESS
        #undef CONFIG_USBD_STORAGE_OUT_ENDPOINT 

        #if     ABS_OUT_ADDR
                #define CONFIG_USBD_STORAGE_OUT_ENDPOINT        ABS_OUT_ADDR
        #endif

#if     defined(ABS_IN_ADDR) 
        #warning
        #warning USING ABS ENDPOINT IN ADDRESS
        #undef CONFIG_USBD_STORAGE_IN_ENDPOINT 

        #if     ABS_IN_ADDR
                #define CONFIG_USBD_STORAGE_IN_ENDPOINT         ABS_IN_ADDR
        #endif

#elif   defined(MAX_OUT_ADDR) && defined(CONFIG_USBD_STORAGE_OUT_ENDPOINT) && (CONFIG_USBD_STORAGE_OUT_ENDPOINT > MAX_OUT_ADDR)
        #warning
        #warning USING DEFAULT ENDPOINT OUT ADDRESS
        #undef CONFIG_USBD_STORAGE_OUT_ENDPOINT 
        #define CONFIG_USBD_STORAGE_OUT_ENDPOINT            DFL_OUT_ADDR
#endif

#elif   defined(MAX_IN_ADDR) && defined(CONFIG_USBD_STORAGE_IN_ENDPOINT) && (CONFIG_USBD_STORAGE_IN_ENDPOINT > MAX_IN_ADDR)
        #warning
        #warning USING DEFAULT ENDPOINT IN ADDRESS
        #undef CONFIG_USBD_STORAGE_IN_ENDPOINT 
        #define CONFIG_USBD_STORAGE_IN_ENDPOINT             DFL_IN_ADDR
#endif

#if     defined(MAX_OUT_PKTSIZE) && defined(CONFIG_USBD_STORAGE_OUT_PKTSIZE) && CONFIG_USBD_STORAGE_OUT_PKTSIZE > MAX_OUT_PKTSIZE
        #warning
        #warning OVERIDING ENDPOINT OUT PKTSIZE
        #undef CONFIG_USBD_STORAGE_OUT_PKTSIZE
        #define CONFIG_USBD_STORAGE_OUT_PKTSIZE             MAX_OUT_PKTSIZE
#endif

#if     defined(MAX_IN_PKTSIZE) && defined(CONFIG_USBD_STORAGE_IN_PKTSIZE) && CONFIG_USBD_STORAGE_IN_PKTSIZE > MAX_IN_PKTSIZE
        #warning
        #warning OVERIDING ENDPOINT IN PKTSIZE
        #undef CONFIG_USBD_STORAGE_IN_PKTSIZE
        #define CONFIG_USBD_STORAGE_IN_PKTSIZE              MAX_IN_PKTSIZE
#endif

enum usb_storage_device_state {
    STATE_INVALID,
    STATE_IDLE,	/* Device intends to receive command */
    STATE_DN,	/* Device intends to transfer no data */
    STATE_DI,	/* Device intends to send data to the host */
    STATE_DO,	/* Device intends to receive data from the host */
};

#define CSW_STAT_GOOD	0x00
#define CSW_STAT_FAILED	0x01
#define CSW_STAT_PERR	0x02	/* Phase Error */

/* Command Block Wrapper */
#define MAX_CBLENGTH	16
struct CBW {
    u32	dSignature;
    u32 dTag;
    u32 dDataTransferLength;
    u8	bmFlags;
    u8	bLUN;
    u8	bCBLength;
    u8	CB[MAX_CBLENGTH];
} __attribute__((packed));
#define CBW_SIGNATURE	0x43425355

/* Command Status Wrapper */
struct CSW {
    u32	dSignature;
    u32	dTag;
    u32	dDataResidue;
    u8	bStatus;
} __attribute__((packed));
#define CSW_SIGNATURE	0x53425355

struct usb_storage_threaddata {
    int busy;
    struct CBW cbw;	/* cpu endian */
    struct CSW csw;	/* cpu endian */
    u8 *data_buf;
    /* NOTE: data_len CAN exceed cbw.dDataTransferLength */
    unsigned int data_len;
    unsigned int data_alloclen;

    /* sense data */
    struct {
	u8 key;
	u32 info;
	u32 cmdinfo;
	u8 code;
    } sense;

    /* real storage */
    int real_fd;
    char filename[128];
    unsigned int num_blocks;

    /* statistics */
    struct {
	/* transmit statistics */
	unsigned int read_blocks;
	unsigned int write_blocks;
	/* command statictics */
	unsigned int inquiry;
	unsigned int mode_select;
	unsigned int mode_sense;
	unsigned int read_10;
	unsigned int read_capacity;
	unsigned int request_sense;
	unsigned int start_stop;
	unsigned int test_unit_ready;
	unsigned int verify;
	unsigned int write_10;
	unsigned int write_buffer;
	unsigned int unsupported;
    } stat;
};

struct usb_storage_private {
    struct usb_device_instance *device;
    enum usb_storage_device_state devstate;

    struct usb_storage_threaddata tdata;
};

static spinlock_t storage_lock;
static struct usb_storage_private storage_private;	// one and only storage

static DECLARE_MUTEX_LOCKED(storage_sem_start);
static DECLARE_MUTEX_LOCKED(storage_sem_work);
static int storage_thread_terminating;

static void storage_thread_poke(void)
{
    up(&storage_sem_work);
}

/* SCSI related definitions */
/*
 *	Sense codes
 */
 
#define SENCODE_NO_SENSE                        0x00
#define SENCODE_END_OF_DATA                     0x00
#define SENCODE_BECOMING_READY                  0x04
#define SENCODE_INIT_CMD_REQUIRED               0x04
#define SENCODE_PARAM_LIST_LENGTH_ERROR         0x1A
#define SENCODE_INVALID_COMMAND                 0x20
#define SENCODE_LBA_OUT_OF_RANGE                0x21
#define SENCODE_INVALID_CDB_FIELD               0x24
#define SENCODE_LUN_NOT_SUPPORTED               0x25
#define SENCODE_INVALID_PARAM_FIELD             0x26
#define SENCODE_PARAM_NOT_SUPPORTED             0x26
#define SENCODE_PARAM_VALUE_INVALID             0x26
#define SENCODE_RESET_OCCURRED                  0x29
#define SENCODE_LUN_NOT_SELF_CONFIGURED_YET     0x3E
#define SENCODE_INQUIRY_DATA_CHANGED            0x3F
#define SENCODE_SAVING_PARAMS_NOT_SUPPORTED     0x39
#define SENCODE_DIAGNOSTIC_FAILURE              0x40
#define SENCODE_INTERNAL_TARGET_FAILURE         0x44
#define SENCODE_INVALID_MESSAGE_ERROR           0x49
#define SENCODE_LUN_FAILED_SELF_CONFIG          0x4c
#define SENCODE_OVERLAPPED_COMMAND              0x4E

/* Module Parameters ************************************************************************* */

static char *dbg = NULL;
static u32 vendor_id;
static u32 product_id;
static char *serial_number = NULL;
static char *filename = NULL;

MODULE_PARM(dbg, "s");
MODULE_PARM(vendor_id, "i");
MODULE_PARM(product_id, "i");
MODULE_PARM(serial_number, "s");
MODULE_PARM(filename, "s");

MODULE_PARM_DESC(dbg, "USB Device Debug options");
MODULE_PARM_DESC(vendor_id, "USB Device Vendor ID");
MODULE_PARM_DESC(product_id, "USB Device Product ID");
MODULE_PARM_DESC(serial_number, "USB Device Serial Number");
MODULE_PARM_DESC(filename, "USB Device Storage Filename");



/* Debug switches (module parameter "dbg=...") *********************************************** */

extern int dbgflg_usbdfd_init;
int      dbgflg_usbdfd_ep0;
int      dbgflg_usbdfd_usbe;
int      dbgflg_usbdfd_tx;
int      dbgflg_usbdfd_rx;

static debug_option dbg_table[] = {
    {&dbgflg_usbdfd_init,NULL,"init","initialization and termination"},
    {&dbgflg_usbdfd_ep0,NULL,"ep0","End Point 0 (setup) packet handling"},
    {&dbgflg_usbdfd_usbe,NULL,"usbe","USB events"},
    {&dbgflg_usbdfd_tx,NULL,"tx","transmit (to host)"},
    {&dbgflg_usbdfd_rx,NULL,"rx","receive (from host)"},
    {NULL,NULL,NULL,NULL}
};

#define dbg_init(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_init,lvl,fmt,##args)
#define dbg_ep0(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_ep0,lvl,fmt,##args)
#define dbg_usbe(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_usbe,lvl,fmt,##args)
#define dbg_tx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_tx,lvl,fmt,##args)
#define dbg_rx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_rx,lvl,fmt,##args)

/* ******************************************************************************************* */

/* Mass Storage Class descriptions 
 */

#define STORAGE_TRANSFER_SIZE	2048	/* XXX max 0xfff. why? (see usbd-bi.c) */
static struct usb_endpoint_description storage_default[] = {
    { bEndpointAddress: CONFIG_USBD_STORAGE_OUT_ENDPOINT,
        bmAttributes: BULK,
        wMaxPacketSize: CONFIG_USBD_STORAGE_OUT_PKTSIZE,
        bInterval: 0,
        direction: OUT,
        transferSize: STORAGE_TRANSFER_SIZE, },

    { bEndpointAddress: CONFIG_USBD_STORAGE_IN_ENDPOINT,
        bmAttributes: BULK,
        wMaxPacketSize: CONFIG_USBD_STORAGE_IN_PKTSIZE,
        bInterval: 0,
        direction: IN,
        transferSize: STORAGE_TRANSFER_SIZE, },

};

/* Data Interface Alternate description(s)
 */
static __initdata struct usb_alternate_description storage_data_alternate_descriptions[] = {
    {   iInterface: "Simple Mass Storage Data Interface - Bulk-Only", 
        bAlternateSetting: 0,
	classes: 0,
	class_list: NULL,
        endpoints: sizeof(storage_default)/sizeof(struct usb_endpoint_description),
        endpoint_list: storage_default, },
};

/* Interface description(s)
 */
static __initdata struct usb_interface_description storage_interfaces[] = {
    {   iInterface: "Simple Mass Storage Data Interface", 
        bInterfaceClass: USB_CLASS_MASS_STORAGE,
        bInterfaceSubClass: 0x01, /* RBC */
        bInterfaceProtocol: 0x50, /* Bulk-Only Transport */
        alternates: sizeof(storage_data_alternate_descriptions)/sizeof(struct usb_alternate_description),
        alternate_list: storage_data_alternate_descriptions, },
};


/* Configuration description(s)
 */
struct __initdata usb_configuration_description storage_description[] = {
    {   iConfiguration: "USB Simple Mass Storage Configuration", 
        bmAttributes: BMATTRIBUTE,
        bMaxPower: BMAXPOWER,
        interfaces: sizeof(storage_interfaces)/sizeof(struct usb_interface_description),
        interface_list: storage_interfaces, },
};

/* Device Description
 */
struct __initdata usb_device_description storage_device_description = {
    bDeviceClass:       0, 
    bDeviceSubClass:    0,
    bDeviceProtocol:    0,
    idVendor:           CONFIG_USBD_VENDORID,
    idProduct:          CONFIG_USBD_PRODUCTID,
    iManufacturer:      CONFIG_USBD_MANUFACTURER,
    iProduct:           CONFIG_USBD_PRODUCT_NAME,
    iSerialNumber:      CONFIG_USBD_SERIAL_NUMBER_STR,
};

static int storage_receive_CBW(struct usb_storage_private *private,
			       u8 *buf, unsigned int len)
{
    struct CBW *cbwp = (struct CBW *)buf;
    struct usb_storage_threaddata *tdata = &private->tdata;

#if 0	/* bad bi driver */
    if (len < sizeof(struct CBW)) {
	printk(KERN_ERR STORAGE_MOD_NAME ": bad CBW length (%d)\n", len);
	return -1;
    }
#else
    if (len != sizeof(struct CBW)) {
	printk(KERN_ERR STORAGE_MOD_NAME ": bad CBW length (%d)\n", len);
	return -1;
    }
#endif
    if (le32_to_cpu(cbwp->dSignature) != CBW_SIGNATURE) {
	printk(KERN_ERR STORAGE_MOD_NAME ": bad CBW signature (%x)\n",
	       le32_to_cpu(cbwp->dSignature));
	return -1;
    }
    if (cbwp->bLUN != 0) {
	printk(KERN_ERR STORAGE_MOD_NAME ": bad CBW LUN (%x)\n", cbwp->bLUN);
	return -1;
    }

    tdata->cbw = *cbwp;
    tdata->cbw.dSignature = le32_to_cpu(tdata->cbw.dSignature);
    tdata->cbw.dTag = le32_to_cpu(tdata->cbw.dTag);
    tdata->cbw.dDataTransferLength = le32_to_cpu(tdata->cbw.dDataTransferLength);
    tdata->csw.dSignature = CSW_SIGNATURE;
    tdata->csw.dTag = tdata->cbw.dTag;
    tdata->csw.dDataResidue = tdata->cbw.dDataTransferLength;
    tdata->csw.bStatus = CSW_STAT_GOOD;

    return len - sizeof(struct CBW);
}

static int storage_send_CSW(struct usb_storage_private *private)
{
    struct urb *urb;
    int port = 0; // XXX compound device
    struct usb_storage_threaddata *tdata = &private->tdata;

    urb = usbd_alloc_urb(private->device,
			 private->device->function_instance_array+port,
			 CONFIG_USBD_STORAGE_IN_ENDPOINT | IN, sizeof (struct CSW));
    if (!urb) {
	printk(KERN_ERR STORAGE_MOD_NAME ": failed to alloc CSW urb\n");
	return -EINVAL;
    }


    tdata->csw.dSignature = cpu_to_le32(tdata->csw.dSignature);
    tdata->csw.dTag = cpu_to_le32(tdata->csw.dTag);
    tdata->csw.dDataResidue = cpu_to_le32(tdata->csw.dDataResidue);
    memcpy(urb->buffer, &tdata->csw, sizeof(struct CSW));
    urb->actual_length = sizeof(struct CSW);
    dbg_rx(3,"CSW (length %d)", urb->actual_length);
    dbgPRINTmem(dbgflg_usbdfd_tx,3,urb->buffer,min(urb->actual_length, 32u));
    if (usbd_send_urb(urb))
	return -EINVAL;
    return 0;
}

/* storage_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 */
int storage_urb_sent (struct urb *urb, int status)
{
    dbg_tx(2,"%s length: %d status : %x",urb->device->name, urb->actual_length, status);
    usbd_dealloc_urb(urb);

    return 0;
}

/* storage_recv_urb - called to indicate URB has been received
 * @urb - pointer to struct urb
 *
 * Return non-zero if we failed and urb is still valid (not disposed)
 * NOTE: This function is called in bottom half context.
 */
int storage_recv_urb (struct urb *urb)
{
    int port = 0; // XXX compound device
    struct usb_device_instance *device = urb->device;
    struct usb_storage_private *private = (device->function_instance_array+port)->privdata;
    struct usb_storage_threaddata *tdata = &private->tdata;
#if 0	/* bad bi driver */
    unsigned int datalen;
#endif

    if (urb->status != RECV_OK)
	return 1;
    dbg_rx(2,"length=%d",urb->actual_length);
    dbgPRINTmem(dbgflg_usbdfd_rx,3,urb->buffer,min(urb->actual_length, 32u));

    // push the data up
    spin_lock(&storage_lock);
    if (private->tdata.busy) {
	printk(KERN_ERR STORAGE_MOD_NAME ": receive in busy state (%d)\n",
	       private->devstate);
	private->devstate = STATE_INVALID;
	spin_unlock(&storage_lock);
	usbd_recycle_urb(urb);	// free urb
	return 0;
    }
    switch (private->devstate) {
    case STATE_IDLE:
	tdata->data_len = 0;
	if (storage_receive_CBW(private, urb->buffer, urb->actual_length) < 0) {
	    private->devstate = STATE_INVALID;
	    break;
	}
#if 0	/* bad bi driver */
	datalen = 0;
	if (urb->actual_length > sizeof(struct CBW)) {
	    datalen = urb->actual_length - sizeof(struct CBW);
	    /* buf contains CBW and (part of) DATA */
	    if (tdata->cbw.dDataTransferLength == 0 ||
		(tdata->cbw.bmFlags & 0x80) ||
		tdata->cbw.dDataTransferLength < datalen) {
		printk(KERN_ERR STORAGE_MOD_NAME ": bad DATA length (%d)\n",
		       datalen);
		private->devstate = STATE_INVALID;
		break;
	    }
	}
#endif

	if (tdata->cbw.dDataTransferLength == 0) {
	    private->devstate = STATE_DN;
	    private->tdata.busy = 1;
	    storage_thread_poke();
	    break;
	}
	if (tdata->data_alloclen < tdata->cbw.dDataTransferLength) {
	    /* expand data buffer */
	    if (tdata->data_buf)
		kfree(tdata->data_buf);
	    tdata->data_buf = kmalloc(tdata->cbw.dDataTransferLength, GFP_ATOMIC);
	    if (!tdata->data_buf) {
		printk(KERN_ERR STORAGE_MOD_NAME ": failed to expand buffer (%d)\n",
		       tdata->cbw.dDataTransferLength);
		tdata->data_alloclen = 0;
		private->devstate = STATE_INVALID;
		break;
	    }
	    tdata->data_alloclen = tdata->cbw.dDataTransferLength;
	}
	if (tdata->cbw.bmFlags & 0x80) {	/* IN */
	    private->devstate = STATE_DI;
	    private->tdata.busy = 1;
	    storage_thread_poke();
	} else {	/* OUT */
#if 0	/* bad bi driver */
	    if (datalen > 0) {
		memcpy(tdata->data_buf, urb->buffer + sizeof(struct CBW),
		       datalen);
		tdata->data_len = datalen;
		if (tdata->data_len >= tdata->cbw.dDataTransferLength) {
		    /* all data received */
		    private->devstate = STATE_DN;
		    private->tdata.busy = 1;
		    storage_thread_poke();
		    break;
		}
	    }
#endif
	    private->devstate = STATE_DO;
	}
	break;
    case STATE_DO:
	if (tdata->data_len + urb->actual_length > tdata->cbw.dDataTransferLength) {
	    printk(KERN_ERR STORAGE_MOD_NAME ": bad DATA length (%d + %d > %d)\n",
		   tdata->data_len, urb->actual_length,
		   tdata->cbw.dDataTransferLength);
	    private->devstate = STATE_INVALID;
	    break;
	}
	memcpy(tdata->data_buf + tdata->data_len,
	       urb->buffer, urb->actual_length);
	tdata->data_len += urb->actual_length;
	if (tdata->data_len >= tdata->cbw.dDataTransferLength) {
	    /* all data received */
	    private->devstate = STATE_DN;
	    private->tdata.busy = 1;
	    storage_thread_poke();
	}
	break;
    case STATE_DI:
    default:
	printk(KERN_ERR STORAGE_MOD_NAME ": receive in bad state (%d)\n",
	       private->devstate);
	private->devstate = STATE_INVALID;
    }
    spin_unlock(&storage_lock);

    usbd_recycle_urb(urb);	// free urb
    return(0);
}

/**
 * storage_recv_setup - called with a control URB 
 * @urb - pointer to struct urb
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int storage_recv_setup (struct urb *urb)
{
    struct usb_device_request *request;
    struct usb_device_instance *device = urb->device;
    int port = 0;
    struct usb_storage_private *private = (device->function_instance_array+port)->privdata;

    request = &urb->device_request;

    // handle Mass Storage Class-Specific Request
    // c.f. USB Mass Storage Class Bulk-Only Transport 3.1, 3.2
    if ((request->bmRequestType & (USB_REQ_TYPE_MASK|USB_REQ_RECIPIENT_MASK)) !=
	 (USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE)) {
        dbg_ep0(1, "not class/interface request: %x", request->bmRequestType);
        return 0; // XXX
    }

    if ((request->bmRequestType&USB_REQ_DIRECTION_MASK)) {
        dbg_ep0(1, "Device-to-Host");
        switch (request->bRequest) {
        case USB_REQ_GET_MAX_LUN:
	    urb->actual_length = 1;
	    urb->buffer[0] = 0;	// only one LUN
	    break;
	default:
	    dbg_ep0(1, "Unknown request: %x", request->bRequest);
	}
    }
    else {
        dbg_ep0(1, "Host-to-Device");
        switch (request->bRequest) {
        case USB_REQ_BO_MASS_STORAGE_RESET:
	    /* do job in event handler */
	    usbd_device_event(private->device, DEVICE_FUNCTION_PRIVATE, 0);
            break;
	default:
	    dbg_ep0(1, "Unknown request: %x", request->bRequest);
        }
    }
    return 0;
}

/* USB Device Functions ************************************************************************ */

/* proc interface */
static int storage_read_proc(char *page, char **start, off_t off,
			     int count, int *eof, void *data)
{
    char *p = page;
    struct usb_storage_private *private = &storage_private;

    spin_lock_bh(&storage_lock);	/* prevent event handler */
    if (private->device) {
	struct usb_storage_threaddata *tdata = &private->tdata;
	p += sprintf(p, "storage \"%s\" %d blocks\n",
		     tdata->filename, tdata->num_blocks);
	p += sprintf(p, "state: %d\n", private->devstate);
	p += sprintf(p, "transfer statistics:\n");
	p += sprintf(p, "read_blocks\t%u\n", tdata->stat.read_blocks);
	p += sprintf(p, "write_blocks\t%u\n", tdata->stat.write_blocks);
	p += sprintf(p, "command statistics:\n");
	p += sprintf(p, "inquiry\t%u\n", tdata->stat.inquiry);
	p += sprintf(p, "mode_select\t%u\n", tdata->stat.mode_select);
	p += sprintf(p, "mode_sense\t%u\n", tdata->stat.mode_sense);
	p += sprintf(p, "read_10\t%u\n", tdata->stat.read_10);
	p += sprintf(p, "read_capacity\t%u\n", tdata->stat.read_capacity);
	p += sprintf(p, "request_sense\t%u\n", tdata->stat.request_sense);
	p += sprintf(p, "start_stop\t%u\n", tdata->stat.start_stop);
	p += sprintf(p, "test_unit_ready\t%u\n", tdata->stat.test_unit_ready);
	p += sprintf(p, "verify\t%u\n", tdata->stat.verify);
	p += sprintf(p, "write_10\t%u\n", tdata->stat.write_10);
	p += sprintf(p, "write_buffer\t%u\n", tdata->stat.write_buffer);
	p += sprintf(p, "unsupported\t%u\n", tdata->stat.unsupported);
    }
    spin_unlock_bh(&storage_lock);

    return p - page;
}

/* storage_event - process a device event
 *
 * NOTE: This function is called from keventd kernel thread.
 */
void storage_event(struct usb_device_instance *device, usb_device_event_t event, int data)
{
    int port = 0; // XXX compound device
    struct usb_function_instance *function;

    dbg_usbe(5,"%d",event);

    if ((function = device->function_instance_array+port)==NULL){
        dbg_usbe(1,"no function");
        return;
    }

    dbg_usbe(3,"---> %s %d", device->name, event);
    switch (event) {

    case DEVICE_UNKNOWN:
    case DEVICE_INIT:
        dbg_usbe(1,"---> INIT %s %d", device->name, event);
        break;

    case DEVICE_CREATE:
        dbg_usbe(1,"---> CREATE %s %d", device->name, event);
        {
            struct usb_storage_private *private;

            // There is no way to indicate error, so make this unconditional
            // and undo it in the DESTROY event unconditionally as well.
            // It the responsibility of the USBD core and the bus interface
            // to see that there is a matching DESTROY for every CREATE.

	    spin_lock(&storage_lock);
	    if (storage_private.device) {
                dbg_usbe(1,"---> CREATE no free storage");
		spin_unlock(&storage_lock);
		return;
	    }
	    private = &storage_private;
	    private->device = device;
	    private->devstate = STATE_IDLE;
            function->privdata = private;
	    spin_unlock(&storage_lock);

            dbg_usbe(1,"---> START %s privdata assigned: %p", device->name, private); 
            return;
        }
        break;

    case DEVICE_HUB_CONFIGURED:
        break;
    case DEVICE_RESET:
        break;
    case DEVICE_ADDRESS_ASSIGNED:
        break;
    case DEVICE_CONFIGURED:
        dbg_usbe(1,"---> CONFIGURED %s %d", device->name, event);
        break;
    case DEVICE_SET_INTERFACE:
        break;
    case DEVICE_SET_FEATURE:
        break;
    case DEVICE_CLEAR_FEATURE:
        break;
    case DEVICE_DE_CONFIGURED:
        dbg_usbe(1,"---> DECONFIGURED %s %d", device->name, event);
        break;
    case DEVICE_BUS_INACTIVE:
        break;
    case DEVICE_BUS_ACTIVITY:
        break;
    case DEVICE_POWER_INTERRUPTION:
        break;
    case DEVICE_HUB_RESET:
        break;

    case DEVICE_DESTROY:
        dbg_usbe(1, "---> DESTROY %s %d", device->name, event);
        {
            struct usb_storage_private *private;
	    struct usb_storage_threaddata *tdata;

            if ((private = (device->function_instance_array+port)->privdata) == NULL) {
                dbg_usbe(1, "---> DESTROY %s private null", device->name);
                return;
            }
            dbg_usbe(1, "---> DESTROY %s private %p", device->name, private);
	    tdata = &private->tdata;

	    spin_lock(&storage_lock);
	    private->device = NULL;
	    private->devstate = STATE_INVALID;
	    if (tdata->busy) {
		/* free on DEVICE_FUNCTION_PRIVATE */
	    } else {
		if (tdata->data_buf)
		    kfree(tdata->data_buf);
		tdata->data_buf = NULL;
		tdata->data_len = tdata->data_alloclen = 0;
	    }
	    spin_unlock(&storage_lock);

            dbg_usbe(1,"---> STOP %s",device->name); 
            return;
        }
        break;

    case DEVICE_FUNCTION_PRIVATE:
        dbg_usbe(2,"---> FUNCTION_PRIVATE %s %d", device->name, event);
	{
            struct usb_storage_private *private;
	    struct usb_storage_threaddata *tdata;
            if ((private = (device->function_instance_array+port)->privdata) == NULL) {
                dbg_usbe(1, "---> PRIVATE %s private null", device->name);
                return;
            }
            dbg_usbe(2, "---> PRIVATE %s private %p", device->name, private);
	    tdata = &private->tdata;

	    spin_lock_bh(&storage_lock);
	    if (data == 0) {
		dbg_rx(1, "USB_REQ_BO_MASS_STORAGE_RESET received.");
		private->devstate = STATE_IDLE;
	    } else {
		/* kicked by storage_thread */
		struct urb *urb;

		if (!private->tdata.busy) {
		    /* command completed. send data and status */
		    switch (private->devstate) {
		    case STATE_DI:
			urb = usbd_alloc_urb(private->device,
					     private->device->function_instance_array+port,
					     CONFIG_USBD_STORAGE_IN_ENDPOINT | IN,
					     tdata->cbw.dDataTransferLength);
			if (!urb) {
			    printk(KERN_ERR STORAGE_MOD_NAME ": failed to alloc DATA urb\n");
			    private->devstate = STATE_INVALID;
			    break;
			}
#if 0
			if (tdata->data_len < tdata->cbw.dDataTransferLength) {
			    dbg_tx(2, "fill data to pad up (%d < %d)",
				   tdata->data_len,
				   tdata->cbw.dDataTransferLength);
			    memset(tdata->data_buf + tdata->data_len, 0,
				   tdata->cbw.dDataTransferLength - tdata->data_len);
			    tdata->data_len = tdata->cbw.dDataTransferLength;
			}
#endif
			memcpy(urb->buffer, tdata->data_buf, tdata->data_len);
			urb->actual_length = tdata->data_len;
			tdata->csw.dDataResidue -= urb->actual_length;
			dbg_rx(3,"DATA (length %d)", urb->actual_length);
			dbgPRINTmem(dbgflg_usbdfd_tx,3,urb->buffer,min(urb->actual_length, 32u));
			if (usbd_send_urb(urb)) {
			    private->devstate = STATE_INVALID;
			    break;
			}
			/* FALLTHRU */
		    case STATE_DN:
			if (storage_send_CSW(private) < 0) {
			    private->devstate = STATE_INVALID;
			    break;
			}
			private->devstate = STATE_IDLE;
			break;
		    case STATE_IDLE:
			/* USB_REQ_BO_MASS_STORAGE_RESET received during
			   command processing */
			break;
		    default:
			private->devstate = STATE_INVALID;
			if (tdata->data_buf)
			    kfree(tdata->data_buf);
			tdata->data_buf = NULL;
			tdata->data_len = tdata->data_alloclen = 0;
		    }
		}
	    }
	    spin_unlock_bh(&storage_lock);
	}
        break;

    }
}


struct usb_function_operations function_ops = {
    event: storage_event,
    recv_urb: storage_recv_urb,
    urb_sent: storage_urb_sent,
    recv_setup: storage_recv_setup
};

struct usb_function_driver function_driver = {
    name: "usbd storage",
    ops: &function_ops,
    device_description: &storage_device_description,
    configurations: sizeof(storage_description)/sizeof(struct usb_configuration_description),
    configuration_description: storage_description,
    this_module: THIS_MODULE,
};


/*
 * RBC functions
 */

static void add_in_data(struct usb_storage_threaddata *tdata,
			void *buf, unsigned int len)
{
    if (tdata->data_len < tdata->cbw.dDataTransferLength) {
	memcpy(tdata->data_buf + tdata->data_len, buf,
	       min(tdata->cbw.dDataTransferLength - tdata->data_len, len));
    }
    tdata->data_len += len;
}

static int do_vpd(struct usb_storage_threaddata *tdata,
		  int page, unsigned int alloclen)
{
    u8 data[256];
    unsigned int len = 4;

    switch (page) {
    case 0x00:	/* Supported vital product data pages */
	data[4+0] = 0x00;	/* page list */
	data[4+1] = 0x80;
	data[4+2] = 0x83;
	len += 3;
	break;
    case 0x80:	/* Unit Serial Number */
	strcpy((char *)&data[4], storage_device_description.iSerialNumber);
	len += strlen(storage_device_description.iSerialNumber);
	break;
    case 0x83:	/* Vital Product Data Device Identification */
	sprintf(&data[4+4], "%s %s %s",
		storage_device_description.iManufacturer,
		storage_device_description.iProduct,
		storage_device_description.iSerialNumber);
	data[4+0] = 0x02;	/* ASCII */
	data[4+1] = 0x01;
	data[4+3] = strlen((char *)&data[4+4]);
	len += 4 + data[3];
	break;
    default:
	return -EINVAL;
    }
    data[0] = PERIPHERAL_DEVICE_TYPE;
    data[1] = page;	/* page code */
    data[2] = 0;
    data[3] = len - 4;
    add_in_data(tdata, data, min(len, alloclen));
    return 0;
}

static int do_inquiry(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[4 + 4 + 8 + 16 + 4];
    unsigned int len = 4 + 4 + 8 + 16 + 4;

    if (CB[1] & 0x02)	/* CMDDT */
	goto invalid_cdb_field;
    if (CB[1] & 0x01) {	/* EVPD */
	if (do_vpd(tdata, CB[2], CB[4]) < 0)
	    goto invalid_cdb_field;
	return 0;
    }
    if (CB[2])
	goto invalid_cdb_field;

    data[0] = PERIPHERAL_DEVICE_TYPE;
    data[1] = 0;	/* not removal */
    data[2] = 0x04;	/* SPC-2 */
    data[3] = 0x02;
    data[4] = 4 + 8 + 16 + 4;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    memset(&data[8], ' ', 8 + 16 + 4);
    memcpy(&data[8], storage_device_description.iManufacturer,
	   min_t(int, 8, strlen(storage_device_description.iManufacturer)));
    memcpy(&data[16], storage_device_description.iProduct,
	   min_t(int, 16, strlen(storage_device_description.iProduct)));
    add_in_data(tdata, data, min(len, (unsigned int)CB[4]));
    return 0;

  invalid_cdb_field:
    printk(KERN_WARNING STORAGE_MOD_NAME ": Invalid CDB field\n");
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_CDB_FIELD;
    return -EINVAL;
}

static int do_mode_select(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    if (tdata->data_len < CB[4]) {
	printk(KERN_ERR STORAGE_MOD_NAME ": phase error on MODE_SELECT\n");
	tdata->csw.bStatus = CSW_STAT_PERR;
	return -EINVAL;
    }
    /* nothing changeable */
    tdata->csw.dDataResidue -= CB[4];
    return 0;
}

static int do_mode_sense(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[4 + 13];
    unsigned int len = 4 + 13;
    int page = CB[2] & 0x3f;

    if ((CB[2] & 0xc0) == 0x40)	/* PC=01b: changable values */
	goto invalid_cdb_field;
    if (page != 0x06 && page != 0x3f)	/* page 6 or all */
	goto invalid_cdb_field;
    /* mode parameter header (c.f. SPC-2 8.3, RBC 5.8.2) */
    data[0] = len - 1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    /* parameter page 6 (c.f. RBC 5.8.3) */
    data[4] = 0x80 | 0x06;
    data[5] = 13 - 2;
    data[6] = 0;
    put_unaligned(cpu_to_be16(STORAGE_BLOCK_SIZE), (u16 *)&data[7]);
    data[9] = 0;
    put_unaligned(cpu_to_be32(tdata->num_blocks), (u32 *)&data[10]);
    data[14] = 0;
    data[15] = 0x03;	/* FORMATD,LOCKD */
    data[16] = 0;
    add_in_data(tdata, data, min(len, (unsigned int)CB[4]));
    return 0;

  invalid_cdb_field:
    printk(KERN_WARNING STORAGE_MOD_NAME ": Invalid CDB field\n");
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_CDB_FIELD;
    return -EINVAL;
}

static int do_read(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    unsigned int lba = be32_to_cpu(get_unaligned((u32 *)&CB[2]));
    unsigned int llen = be16_to_cpu(get_unaligned((u16 *)&CB[7]));
    unsigned int len = llen * STORAGE_BLOCK_SIZE;

    if (tdata->cbw.dDataTransferLength < len)
	goto phase_error;
    if (lba + llen > tdata->num_blocks)
	goto lba_out_of_range;
    if (lseek(tdata->real_fd, (off_t)lba * STORAGE_BLOCK_SIZE,
	      0 /* SEEK_SET */) == (off_t)-1) {
	printk(KERN_ERR STORAGE_MOD_NAME ": %s: lseek error on LBA %u (errno %d)\n",
	       tdata->filename, lba, errno);
	goto medium_error;
    }
    if (read(tdata->real_fd, tdata->data_buf, len) != len) {
	printk(KERN_ERR STORAGE_MOD_NAME ": %s: read error on LBA %u (errno %d)\n",
	       tdata->filename, lba, errno);
	goto medium_error;
    }
    tdata->data_len = len;
    tdata->stat.read_blocks += llen;
    return 0;

  phase_error:
    printk(KERN_ERR STORAGE_MOD_NAME ": phase error on READ\n");
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
  lba_out_of_range:
    printk(KERN_WARNING STORAGE_MOD_NAME ": LBA out of range\n");
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LBA_OUT_OF_RANGE;
    return -EINVAL;
  medium_error:
    printk(KERN_WARNING STORAGE_MOD_NAME ": Medium error\n");
    tdata->sense.key = MEDIUM_ERROR;
    return -EINVAL;
}

static int do_readcapacity(struct usb_storage_threaddata *tdata)
{
    u32 data[2];
    data[0] = cpu_to_be32(tdata->num_blocks - 1);
    data[1] = cpu_to_be32(STORAGE_BLOCK_SIZE);
    add_in_data(tdata, &data, sizeof(data));
    return 0;
}

static int do_request_sense(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[13];
    unsigned int len = sizeof(data);
    data[0] = 0x70;	/* current errors */
    data[1] = 0;
    data[2] = tdata->sense.key;
    put_unaligned(cpu_to_be32(tdata->sense.info), (u32 *)&data[3]);
    data[7] = len - 7;;
    put_unaligned(cpu_to_be32(tdata->sense.cmdinfo), (u32 *)&data[8]);
    data[12] = tdata->sense.code;
    add_in_data(tdata, data, min(len, (unsigned int)CB[4]));
    return 0;
}

static int do_write(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    unsigned int lba = be32_to_cpu(get_unaligned((u32 *)&CB[2]));
    unsigned int llen = be16_to_cpu(get_unaligned((u16 *)&CB[7]));
    unsigned int len = llen * STORAGE_BLOCK_SIZE;

    if (tdata->data_len < len)
	goto phase_error;
    if (lba + llen > tdata->num_blocks)
	goto lba_out_of_range;
    if (lseek(tdata->real_fd, (off_t)lba * STORAGE_BLOCK_SIZE,
	      0 /* SEEK_SET */) == (off_t)-1) {
	printk(KERN_ERR STORAGE_MOD_NAME ": %s: lseek error on LBA %u (errno %d)\n",
	       tdata->filename, lba, errno);
	goto medium_error;
    }
    if (write(tdata->real_fd, tdata->data_buf, len) != len) {
	printk(KERN_ERR STORAGE_MOD_NAME ": %s: write error on LBA %u (errno %d)\n",
	       tdata->filename, lba, errno);
	goto medium_error;
    }
    tdata->csw.dDataResidue -= len;
    tdata->stat.write_blocks += llen;
    return 0;

  phase_error:
    printk(KERN_ERR STORAGE_MOD_NAME ": phase error on WRITE\n");
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
  lba_out_of_range:
    printk(KERN_WARNING STORAGE_MOD_NAME ": LBA out of range\n");
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LBA_OUT_OF_RANGE;
    return -EINVAL;
  medium_error:
    printk(KERN_WARNING STORAGE_MOD_NAME ": Medium error\n");
    tdata->sense.key = MEDIUM_ERROR;
    return -EINVAL;
}

/* process RBC Command Block */
static int storage_process_CB(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;

    if (CB[0] != REQUEST_SENSE) {
	/* initialize sense data */
	tdata->sense.key = NO_SENSE;
	tdata->sense.info = 0;
	tdata->sense.code = SENCODE_NO_SENSE;
	tdata->sense.cmdinfo = 0;
    }

    switch (CB[0]) {
    case FORMAT_UNIT:
	dbg_rx(2, "FORMAT_UNIT");
	goto unsupported;
    case INQUIRY:
	dbg_rx(2, "INQUIRY");
	tdata->stat.inquiry++;
	if (tdata->data_len)
	    goto phase_error;
	return do_inquiry(tdata);
    case MODE_SELECT:
	dbg_rx(2, "MODE_SELECT");
	tdata->stat.mode_select++;
	return do_mode_select(tdata);
    case MODE_SENSE:
	dbg_rx(2, "MODE_SENSE");
	tdata->stat.mode_sense++;
	if (tdata->data_len)
	    goto phase_error;
	return do_mode_sense(tdata);
    case PERSISTENT_RESERVE_IN:
	dbg_rx(2, "PERSISTENT_RESERVE_IN");
	goto unsupported;
    case PERSISTENT_RESERVE_OUT:
	dbg_rx(2, "PERSISTENT_RESERVE_OUT");
	goto unsupported;
    case ALLOW_MEDIUM_REMOVAL:
	dbg_rx(2, "ALLOW_MEDIUM_REMOVAL");
	goto unsupported;
    case READ_10:
	dbg_rx(2, "READ_10");
	tdata->stat.read_10++;
	if (tdata->data_len)
	    goto phase_error;
	return do_read(tdata);
    case READ_CAPACITY:
	dbg_rx(2, "READ_CAPACITY");
	tdata->stat.read_capacity++;
	if (tdata->data_len)
	    goto phase_error;
	return do_readcapacity(tdata);
    case RELEASE:
	dbg_rx(2, "RELEASE");
	goto unsupported;
    case REQUEST_SENSE:
	dbg_rx(2, "REQUEST_SENSE");
	tdata->stat.request_sense++;
	if (tdata->data_len)
	    goto phase_error;
	return do_request_sense(tdata);
    case RESERVE:
	dbg_rx(2, "RESERVE");
	goto unsupported;
    case START_STOP:
	dbg_rx(2, "START_STOP");
	tdata->stat.start_stop++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case SYNCHRONIZE_CACHE:
	dbg_rx(2, "SYNCHRONIZE_CACHE");
	goto unsupported;
    case TEST_UNIT_READY:
	dbg_rx(2, "TEST_UNIT_READY");
	tdata->stat.test_unit_ready++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case VERIFY:
	dbg_rx(2, "VERIFY");
	tdata->stat.verify++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case WRITE_10:
	dbg_rx(2, "WRITE_10");
	tdata->stat.write_10++;
	return do_write(tdata);
    case WRITE_BUFFER:
	dbg_rx(2, "WRITE_BUFFER");
	tdata->stat.write_buffer++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    default:
	printk(KERN_ERR STORAGE_MOD_NAME ": unknown RBC command (%02x)\n",
	       CB[0]);
	goto unsupported;
    }

    return 0;

  phase_error:
    printk(KERN_ERR STORAGE_MOD_NAME ": phase error on RBC command %02x\n", CB[0]);
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
  unsupported:
    tdata->stat.unsupported++;
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_COMMAND;
    return -EINVAL;
}

static int storage_thread(void *data) 
{
    struct usb_storage_private *private;
    off_t off;

    daemonize ();
    reparent_to_init();
    spin_lock_irq(&current->sigmask_lock);
    sigemptyset(&current->blocked);
    recalc_sigpending(current);
    spin_unlock_irq(&current->sigmask_lock);

    strcpy (current->comm, STORAGE_MOD_NAME);

    private = &storage_private;
    memset(private, 0, sizeof(*private));
    if (!filename)
	filename = STORAGE_DEFAULT_FILENAME;
    strcpy(private->tdata.filename, filename);
    private->tdata.real_fd = open(private->tdata.filename, O_RDWR, 0);
    if (private->tdata.real_fd < 0) {
	printk(KERN_ERR STORAGE_MOD_NAME ": %s open failed (errno %d)\n",
	       private->tdata.filename, errno);
	storage_thread_terminating = 1;
	goto end;
    }
    if ((off = lseek(private->tdata.real_fd, 0, 2 /* SEEK_END*/)) == (off_t)-1) {
	printk(KERN_ERR STORAGE_MOD_NAME ": %s lseek failed (errno %d)\n",
	       private->tdata.filename, errno);
	close(private->tdata.real_fd);
	private->tdata.real_fd = -1;
	storage_thread_terminating = 1;
	goto end;
    }
    private->tdata.num_blocks = off / STORAGE_BLOCK_SIZE;
    private->devstate = STATE_INVALID;
    printk(KERN_INFO STORAGE_MOD_NAME ": storage filename %s, %d blocks\n",
	   private->tdata.filename, private->tdata.num_blocks);

    // let startup continue
    up(&storage_sem_start);

    // process loop
    for (storage_thread_terminating = 0; !storage_thread_terminating;) {
        // wait for someone to tell us to do something
        down(&storage_sem_work);

	spin_lock_bh(&storage_lock);	/* prevent event handler */
	if (private->device && private->tdata.busy) {
	    spin_unlock_bh(&storage_lock);
	    /* this function CAN sleep */
	    if (storage_process_CB(&private->tdata) < 0) {
		if (private->tdata.csw.bStatus == CSW_STAT_GOOD)
		    private->tdata.csw.bStatus = CSW_STAT_FAILED;
	    }
	    spin_lock_bh(&storage_lock);
	    private->tdata.busy = 0;
	    /* kick event routine */
	    usbd_device_event(private->device, DEVICE_FUNCTION_PRIVATE, 1);
	}
	spin_unlock_bh(&storage_lock);
    }

    /* shutdown */
    if (private->tdata.data_buf)
	kfree(private->tdata.data_buf);
    close(private->tdata.real_fd);

    // let the process stopping us know we are done and return
 end:
    up(&storage_sem_start);
    complete_and_exit (NULL, 0);
    return 0;
}

/**
 * storage_thread_kickoff - start command processing thread
 */
static int storage_thread_kickoff(void)
{
    storage_thread_terminating = 0;
    kernel_thread(&storage_thread, NULL, CLONE_FS | CLONE_FILES);
    down(&storage_sem_start);
    if (storage_thread_terminating)
	return -1;
    return 0;
}

/**
 * storage_thread_killoff - stop command processing thread
 */
static void storage_thread_killoff(void)
{
    if (!storage_thread_terminating) {
        storage_thread_terminating = 1;
        up(&storage_sem_work);
        down(&storage_sem_start);
    }
}

#define STORAGE_SERIAL_CHAR_MIN	12
static int __init storage_check_serial(const char *s)
{
    // c.f. USB Mass Storage Class Bulk-Only Transport 4.1.1 Serial Number
    if (strlen(s) < STORAGE_SERIAL_CHAR_MIN)
	return 0;
    while (*s) {
	// '0'-'9', 'A'-'F'
	if (!isxdigit(*s) || islower(*s))
	    return 0;
	s++;
    }
    return 1;
}

/*
 * storage_modinit - module init
 *
 */
static int __init storage_modinit(void)
{
    static char serial_str[STORAGE_SERIAL_CHAR_MIN + 1];
    int i, len;

    printk(KERN_INFO "%s (dbg=\"%s\")\n", __usbd_module_info, dbg?dbg:"");

    if (vendor_id) {
        storage_device_description.idVendor = vendor_id;
    }
    if (product_id) {
        storage_device_description.idProduct = product_id;
    }
    if (!serial_number)
	serial_number = storage_device_description.iSerialNumber;
    len = strlen(serial_number);
    while (len++ < STORAGE_SERIAL_CHAR_MIN)
	strcat(serial_str, "0");
    strcat(serial_str, serial_number);
    for (i = 0; serial_str[i]; i++) {
	if (islower(serial_str[i]))
	    serial_str[i] = toupper(serial_str[i]);
    }
    storage_device_description.iSerialNumber = serial_str;

    printk(KERN_INFO "vendor_id: %04x product_id: %04x, serial_number: %s\n",
	   storage_device_description.idVendor,
	   storage_device_description.idProduct,
	   storage_device_description.iSerialNumber);

    if (0 != scan_debug_options(STORAGE_MOD_NAME, dbg_table,dbg)) {
        return(-EINVAL);
    }

    if (!storage_check_serial(storage_device_description.iSerialNumber)) {
	printk(KERN_ERR "bad serial number (%s)\n",
	       storage_device_description.iSerialNumber);
	return -EINVAL;
    }

    spin_lock_init(&storage_lock);
    create_proc_read_entry(STORAGE_PROC_NAME, 0, NULL, storage_read_proc, 0);

    if (storage_thread_kickoff()) {
	return -ENODEV;
    }

    // register us with the usb device support layer
    //
    if (usbd_register_function(&function_driver)) {
        return -EINVAL;
    }

    // return
    return 0;
}


/* storage_modexit - module cleanup
 */
static void __exit storage_modexit(void)
{
    // de-register us with the usb device support layer
    // 
    usbd_deregister_function(&function_driver);
    remove_proc_entry(STORAGE_PROC_NAME, NULL);
    storage_thread_killoff();
}

module_init(storage_modinit);
module_exit(storage_modexit);

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
