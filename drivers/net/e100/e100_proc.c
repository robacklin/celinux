/*******************************************************************************

  
  Copyright(c) 1999 - 2003 Intel Corporation. All rights reserved.
  
  This program is free software; you can redistribute it and/or modify it 
  under the terms of the GNU General Public License as published by the Free 
  Software Foundation; either version 2 of the License, or (at your option) 
  any later version.
  
  This program is distributed in the hope that it will be useful, but WITHOUT 
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for 
  more details.
  
  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 59 
  Temple Place - Suite 330, Boston, MA  02111-1307, USA.
  
  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
  
  Contact Information:
  Linux NICS <linux.nics@intel.com>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
*******************************************************************************/

/**********************************************************************
*                                                                       *
* INTEL CORPORATION                                                     *
*                                                                       *
* This software is supplied under the terms of the license included     *
* above.  All use of this driver must be in accordance with the terms   *
* of that license.                                                      *
*                                                                       *
* Module Name:  e100_proc.c                                             *
*                                                                       *
* Abstract:     Functions to handle the proc file system.               *
*               Create the proc directories and files and run read and  *
*               write requests from the user                            *
*                                                                       *
* Environment:  This file is intended to be specific to the Linux       *
*               operating system.                                       *
*                                                                       *
**********************************************************************/

#include <linux/config.h>

#ifdef CONFIG_PROC_FS
#include "e100.h"
#include "e100_phy.h"
/* MDI sleep time is at least 50 ms, in jiffies */
#define MDI_SLEEP_TIME ((HZ / 20) + 1)
/***************************************************************************/
/*       /proc File System Interaface Support Functions                    */
/***************************************************************************/

static struct proc_dir_entry *adapters_proc_dir = 0;

/* externs from e100_main.c */
extern char e100_short_driver_name[];
extern char e100_driver_version[];
extern struct net_device_stats *e100_get_stats(struct net_device *dev);
extern int e100_mdi_write(struct e100_private *, u32, u32, u16);

static void e100_proc_cleanup(void);
static unsigned char e100_init_proc_dir(void);

#define ADAPTERS_PROC_DIR "PRO_LAN_Adapters"
#define WRITE_BUF_MAX_LEN 20	
#define READ_BUF_MAX_LEN  256
#define E100_PE_LEN       25

#define bdp_net_off(off) (unsigned long)(offsetof(struct e100_private, drv_stats.net_stats.off))
#define bdp_drv_off(off) (unsigned long)(offsetof(struct e100_private, drv_stats.off))
#define bdp_prm_off(off) (unsigned long)(offsetof(struct e100_private, params.off))

typedef struct _e100_proc_entry {
	char *name;
	read_proc_t *read_proc;
	write_proc_t *write_proc;
	unsigned long offset;	/* offset into bdp. ~0 means no value, pass NULL. */
} e100_proc_entry;

static int
generic_read(char *page, char **start, off_t off, int count, int *eof, int len)
{
	if (len <= off + count)
		*eof = 1;

	*start = page + off;
	len -= off;
	if (len > count)
		len = count;

	if (len < 0)
		len = 0;

	return len;
}

static int
read_ulong(char *page, char **start, off_t off,
	   int count, int *eof, unsigned long l)
{
	int len;

	len = sprintf(page, "%lu\n", l);

	return generic_read(page, start, off, count, eof, len);
}

static int 
read_ulong_hex(char *page, char **start, off_t off,
	       int count, int *eof, unsigned long l)
{
	int len;

	len = sprintf(page, "0x%04lx\n", l);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_gen_ulong(char *page, char **start, off_t off,
	       int count, int *eof, void *data)
{
	unsigned long val = 0;

	if (data)
		val = *((unsigned long *) data);

	return read_ulong(page, start, off, count, eof, val);
}

static int
read_hwaddr(char *page, char **start, off_t off,
	    int count, int *eof, unsigned char *hwaddr)
{
	int len;

	len = sprintf(page, "%02X:%02X:%02X:%02X:%02X:%02X\n",
		      hwaddr[0], hwaddr[1], hwaddr[2],
		      hwaddr[3], hwaddr[4], hwaddr[5]);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_descr(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;

	len = sprintf(page, "%s\n", "Intel(R) PRO/100 Network Connection");

	return generic_read(page, start, off, count, eof, len);
}

static int
read_mdix_status(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	len = sprintf(page, "%s\n", bdp->mdix_status);
	
	return generic_read(page, start, off, count, eof, len);
}

static int
read_cable_status(char *page, char **start, off_t off,
		  int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	len = sprintf(page, "%s\n", bdp->cable_status);
	return generic_read(page, start, off, count, eof, len);
}

static int
read_drvr_name(char *page, char **start, off_t off,
	       int count, int *eof, void *data)
{
	int len;

	len = sprintf(page, "%s\n", e100_short_driver_name);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_drvr_ver(char *page, char **start, off_t off,
	      int count, int *eof, void *data)
{
	int len;

	len = sprintf(page, "%s\n", e100_driver_version);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_pci_vendor(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) bdp->pdev->vendor;
	return read_ulong_hex(page, start, off, count, eof, val);
}

static int
read_pci_device(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) bdp->pdev->device;
	return read_ulong_hex(page, start, off, count, eof, val);
}

static int
read_pci_sub_vendor(char *page, char **start, off_t off,
		    int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) bdp->pdev->subsystem_vendor;
	return read_ulong_hex(page, start, off, count, eof, val);
}

static int
read_pci_sub_device(char *page, char **start, off_t off,
		    int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) bdp->pdev->subsystem_device;
	return read_ulong_hex(page, start, off, count, eof, val);
}

static int
read_pci_revision(char *page, char **start, off_t off,
		  int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) bdp->rev_id;
	return read_ulong_hex(page, start, off, count, eof, val);
}

static int
read_pci_bus(char *page, char **start, off_t off,
	     int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) bdp->pdev->bus->number;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_pci_slot(char *page, char **start, off_t off,
	      int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) (PCI_SLOT(bdp->pdev->devfn));
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_irq(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	val = (unsigned long) (bdp->device->irq);
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_dev_name(char *page, char **start, off_t off,
	      int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	len = sprintf(page, "%s\n", bdp->ifname);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_current_hwaddr(char *page, char **start, off_t off,
		    int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	unsigned char *hwaddr = bdp->device->dev_addr;

	return read_hwaddr(page, start, off, count, eof, hwaddr);
}

static int
read_permanent_hwaddr(char *page, char **start, off_t off,
		      int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	unsigned char *hwaddr = bdp->perm_node_address;

	return read_hwaddr(page, start, off, count, eof, hwaddr);
}

static int
read_part_number(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	len = sprintf(page, "%06lx-%03x\n",
		      (unsigned long) (bdp->pwa_no >> 8),
		      (unsigned int) (bdp->pwa_no & 0xFF));

	return generic_read(page, start, off, count, eof, len);
}

static int
read_link_status(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	if (netif_running(bdp->device)) {
		if (netif_carrier_ok(bdp->device))
			len = sprintf(page, "up\n");
		else
			len = sprintf(page, "down\n");
	} else {
		len = sprintf(page, "N/A\n");
	}
	return generic_read(page, start, off, count, eof, len);
}

static int
read_speed(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	if (netif_carrier_ok(bdp->device))
		return read_ulong(page, start, off, count, eof,
				  (unsigned long) (bdp->cur_line_speed));

	len = sprintf(page, "N/A\n");
	return generic_read(page, start, off, count, eof, len);
}

static int
read_dplx_mode(char *page, char **start, off_t off,
	       int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	if (netif_carrier_ok(bdp->device)) {
		if (bdp->cur_dplx_mode == FULL_DUPLEX)
			len = sprintf(page, "full\n");
		else
			len = sprintf(page, "half\n");
	} else {
		len = sprintf(page, "N/A\n");
	}

	return generic_read(page, start, off, count, eof, len);
}

static int
read_state(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	int len;

	if (bdp->device->flags & IFF_UP)
		len = sprintf(page, "up\n");
	else
		len = sprintf(page, "down\n");

	return generic_read(page, start, off, count, eof, len);
}

static int
read_rx_errors(char *page, char **start, off_t off,
	       int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	e100_get_stats(bdp->device);
	val = (unsigned long) bdp->drv_stats.net_stats.rx_errors;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_tx_errors(char *page, char **start, off_t off,
	       int count, int *eof, void *data)
{
	unsigned long val;
	struct e100_private *bdp = data;

	e100_get_stats(bdp->device);
	val = (unsigned long) bdp->drv_stats.net_stats.tx_errors;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_rx_multicast_packets(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len;

	len = sprintf(page, "N/A\n");
	return generic_read(page, start, off, count, eof, len);
}

static void
set_led(struct e100_private *bdp, u16 led_mdi_op)
{
	e100_mdi_write(bdp, PHY_82555_LED_SWITCH_CONTROL,
		       bdp->phy_addr, led_mdi_op);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(MDI_SLEEP_TIME);

	/* turn led ownership to the chip */
	e100_mdi_write(bdp, PHY_82555_LED_SWITCH_CONTROL,
		       bdp->phy_addr, PHY_82555_LED_NORMAL_CONTROL);
}

static int
write_blink_led_timer(struct file *file, const char *buffer,
		      unsigned long count, void *data)
{
	struct e100_private *bdp = data;
	char s_blink_op[WRITE_BUF_MAX_LEN + 1];
	char *res;
	unsigned long i_blink_op;

	if (!buffer)
		return -EINVAL;

	if (count > WRITE_BUF_MAX_LEN) {
		count = WRITE_BUF_MAX_LEN;
	}
	if (copy_from_user(s_blink_op, buffer, count))
		return -EFAULT;
	s_blink_op[count] = '\0';
	i_blink_op = simple_strtoul(s_blink_op, &res, 0);
	if (res == s_blink_op) {
		return -EINVAL;
	}

	switch (i_blink_op) {

	case LED_OFF:
		set_led(bdp, PHY_82555_LED_OFF);
		break;
	case LED_ON:
		if (bdp->rev_id >= D101MA_REV_ID)
			set_led(bdp, PHY_82555_LED_ON_559);
		else
			set_led(bdp, PHY_82555_LED_ON_PRE_559);

		break;
	default:
		return -EINVAL;
	}

	return count;
}

static e100_proc_entry e100_proc_list[] = {
	{"Description",           read_descr,            0, 0},
	{"Driver_Name",           read_drvr_name,        0, 0},
	{"Driver_Version",        read_drvr_ver,         0, 0},
	{"PCI_Vendor",            read_pci_vendor,       0, 0},
	{"PCI_Device_ID",         read_pci_device,       0, 0},
	{"PCI_Subsystem_Vendor",  read_pci_sub_vendor,   0, 0},
	{"PCI_Subsystem_ID",      read_pci_sub_device,   0, 0},
	{"PCI_Revision_ID",       read_pci_revision,     0, 0},
	{"PCI_Bus",               read_pci_bus,          0, 0},
	{"PCI_Slot",              read_pci_slot,         0, 0},
	{"IRQ",                   read_irq,              0, 0},
	{"System_Device_Name",    read_dev_name,         0, 0},
	{"Current_HWaddr",        read_current_hwaddr,   0, 0},
	{"Permanent_HWaddr",      read_permanent_hwaddr, 0, 0},
	{"Part_Number",           read_part_number,      0, 0},
	{"\n",},
	{"Link",                  read_link_status,      0, 0},
	{"Speed",                 read_speed,            0, 0},
	{"Duplex",                read_dplx_mode,        0, 0},
	{"State",                 read_state,            0, 0},
	{"\n",},
	{"Rx_Packets",            read_gen_ulong, 0, bdp_net_off(rx_packets)},
	{"Tx_Packets",            read_gen_ulong, 0, bdp_net_off(tx_packets)},
	{"Rx_Bytes",              read_gen_ulong, 0, bdp_net_off(rx_bytes)},
	{"Tx_Bytes",              read_gen_ulong, 0, bdp_net_off(tx_bytes)},
	{"Rx_Errors",             read_rx_errors, 0, 0},
	{"Tx_Errors",             read_tx_errors, 0, 0},
	{"Rx_Dropped",            read_gen_ulong, 0, bdp_net_off(rx_dropped)},
	{"Tx_Dropped",            read_gen_ulong, 0, bdp_net_off(tx_dropped)},
	{"Multicast",             read_rx_multicast_packets, 0, 0},
	{"Collisions",            read_gen_ulong, 0, bdp_net_off(collisions)},
	{"Rx_Length_Errors",      read_gen_ulong, 0, bdp_net_off(rx_length_errors)},
	{"Rx_Over_Errors",        read_gen_ulong, 0, bdp_net_off(rx_over_errors)},
	{"Rx_CRC_Errors",         read_gen_ulong, 0, bdp_net_off(rx_crc_errors)},
	{"Rx_Frame_Errors",       read_gen_ulong, 0, bdp_net_off(rx_frame_errors)},
	{"Rx_FIFO_Errors",        read_gen_ulong, 0, bdp_net_off(rx_fifo_errors)},
	{"Rx_Missed_Errors",      read_gen_ulong, 0, bdp_net_off(rx_missed_errors)},
	{"Tx_Aborted_Errors",     read_gen_ulong, 0, bdp_net_off(tx_aborted_errors)},
	{"Tx_Carrier_Errors",     read_gen_ulong, 0, bdp_net_off(tx_carrier_errors)},
	{"Tx_FIFO_Errors",        read_gen_ulong, 0, bdp_net_off(tx_fifo_errors)},
	{"Tx_Heartbeat_Errors",   read_gen_ulong, 0, ~0},
	{"Tx_Window_Errors",      read_gen_ulong, 0, ~0},
	{"\n",},
	{"Rx_TCP_Checksum_Good",  read_gen_ulong, 0, ~0},
	{"Rx_TCP_Checksum_Bad",   read_gen_ulong, 0, ~0},
	{"Tx_TCP_Checksum_Good",  read_gen_ulong, 0, ~0},
	{"Tx_TCP_Checksum_Bad",   read_gen_ulong, 0, ~0},
	{"\n",},
	{"Tx_Abort_Late_Coll",    read_gen_ulong, 0, bdp_drv_off(tx_late_col)},
	{"Tx_Deferred_Ok",        read_gen_ulong, 0, bdp_drv_off(tx_ok_defrd)},
	{"Tx_Single_Coll_Ok",     read_gen_ulong, 0, bdp_drv_off(tx_one_retry)},
	{"Tx_Multi_Coll_Ok",      read_gen_ulong, 0, bdp_drv_off(tx_mt_one_retry)},
	{"Rx_Long_Length_Errors", read_gen_ulong, 0, ~0},
	{"Rx_Align_Errors",       read_gen_ulong, 0, bdp_net_off(rx_frame_errors)},
	{"\n",},
	{"Tx_Flow_Control_Pause", read_gen_ulong, 0, bdp_drv_off(xmt_fc_pkts)},
	{"Rx_Flow_Control_Pause", read_gen_ulong, 0, bdp_drv_off(rcv_fc_pkts)},
	{"Rx_Flow_Control_Unsup", read_gen_ulong, 0, bdp_drv_off(rcv_fc_unsupported)},
	{"\n",},
	{"Tx_TCO_Packets",        read_gen_ulong, 0, bdp_drv_off(xmt_tco_pkts)},
	{"Rx_TCO_Packets",        read_gen_ulong, 0, bdp_drv_off(rcv_tco_pkts)},
	{"\n",},
	{"Rx_Interrupt_Packets",  read_gen_ulong, 0, bdp_drv_off(rx_intr_pkts)},
	{"Identify_Adapter", 0, write_blink_led_timer, 0},
        {"\n",},	
        {"MDIX_Status",           read_mdix_status,      0, 0},
        {"Cable_Status",          read_cable_status,     0, 0},
	{"", 0, 0, 0}
};

static int
read_info(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	e100_proc_entry *pe;
	int tmp;
	void *val;
	int len = 0;

	for (pe = e100_proc_list; pe->name[0]; pe++) {
		if (pe->name[0] == '\n') {
			len += sprintf(page + len, "\n");
			continue;
		}

		if (pe->read_proc) {
			if ((len + READ_BUF_MAX_LEN + E100_PE_LEN + 1) >=
			    PAGE_SIZE)
				break;

			if (pe->offset != ~0)
				val = ((char *) bdp) + pe->offset;
			else
				val = NULL;

			len += sprintf(page + len, "%-"
				       __MODULE_STRING(E100_PE_LEN)
				       "s ", pe->name);
			len += pe->read_proc(page + len, start, 0,
					     READ_BUF_MAX_LEN + 1, &tmp, val);
		}
	}

	return generic_read(page, start, off, count, eof, len);
}

/**********************
 *  parameter entries
 **********************/
static int
read_int_param(char *page, char *name, char *desc, int def, int min, int max)
{
	int len;

	len = sprintf(page, "Name: %s\n", name);
	len += sprintf(page + len, "Description: %s\n", desc);
	len += sprintf(page + len, "Default_Value: %d\n", def);
	len += sprintf(page + len, "Type: Range\n");
	len += sprintf(page + len, "Min: %d\n", min);
	len += sprintf(page + len, "Max: %d\n", max);
	len += sprintf(page + len, "Step:1\n");
	len += sprintf(page + len, "Radix: dec\n");

	return len;
}

static int
read_bool_param(char *page, char *name, char *desc, int def)
{
	int len;

	len = sprintf(page, "Name: %s\n", name);
	len += sprintf(page + len, "Description: %s\n", desc);
	len += sprintf(page + len, "Default_Value: %d\n", def);
	len += sprintf(page + len, "Type: Enum\n");
	len += sprintf(page + len, "0: Off\n");
	len += sprintf(page + len, "1: On\n");

	return len;
}

static int
read_speed_duplex_def(char *page, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len;

	len = sprintf(page, "Name: Speed and Duplex\n");
	len += sprintf(page + len, "Description: Sets the adapter's "
		       "speed and duplex mode\n");
	len += sprintf(page + len, "Default_Value: 0\n");
	len += sprintf(page + len, "Type: Enum\n");
	len += sprintf(page + len, "0: Auto-Negotiate\n");
	len += sprintf(page + len, "1: 10 Mbps / Half Duplex\n");
	len += sprintf(page + len, "2: 10 Mbps / Full Duplex\n");
	len += sprintf(page + len, "3: 100 Mbps / Half Duplex\n");
	len += sprintf(page + len, "4: 100 Mbps / Full Duplex\n");

	return generic_read(page, start, off, count, eof, len);
}

static int
read_tx_desc_def(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	int len;

	len = read_int_param(page, "Transmit Descriptors",
			     "Sets the number of Tx descriptors "
			     "available for the adapter",
			     E100_DEFAULT_TCB, E100_MIN_TCB, E100_MAX_TCB);
	return generic_read(page, start, off, count, eof, len);
}

static int
read_rx_desc_def(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	int len;

	len = read_int_param(page, "Receive Descriptors",
			     "Sets the number of Rx descriptors "
			     "available for the adapter",
			     E100_DEFAULT_RFD, E100_MIN_RFD, E100_MAX_RFD);
	return generic_read(page, start, off, count, eof, len);
}

static int
read_ber_def(char *page, char **start, off_t off,
	     int count, int *eof, void *data)
{
	int len;

	len = read_int_param(page, "Bit Error Rate",
			     "Sets the value for the BER correction algorithm",
			     E100_DEFAULT_BER, 0, ZLOCK_MAX_ERRORS);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_xsum_rx_def(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	int len;

	len = read_bool_param(page, "RX Checksum",
			      "Setting this value to \"On\" enables "
			      "receive checksum", E100_DEFAULT_XSUM);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_ucode_def(char *page, char **start, off_t off,
	       int count, int *eof, void *data)
{
	int len;

	len = read_bool_param(page, "Microcode",
			      "Setting this value to \"On\" enables "
			      "the adapter's microcode", E100_DEFAULT_UCODE);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_bundle_small_def(char *page, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len;

	len = read_bool_param(page, "Bundle Small Frames",
			      "Setting this value to \"On\" enables "
			      "interrupt bundling of small frames",
			      E100_DEFAULT_BUNDLE_SMALL_FR);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_fc_def(char *page, char **start, off_t off,
	    int count, int *eof, void *data)
{
	int len;

	len = read_bool_param(page, "Flow Control",
			      "Setting this value to \"On\" enables processing "
			      "flow-control packets", E100_DEFAULT_FC);

	return generic_read(page, start, off, count, eof, len);
}


static int
read_int_delay_def(char *page, char **start, off_t off,
		   int count, int *eof, void *data)
{
	int len;

	len = read_int_param(page, "CPU Saver Interrupt Delay",
			     "Sets the value for CPU saver's interrupt delay",
			     E100_DEFAULT_CPUSAVER_INTERRUPT_DELAY, 0x0,
			     0xFFFF);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_bundle_max_def(char *page, char **start, off_t off,
		    int count, int *eof, void *data)
{
	int len;

	len = read_int_param(page, "CPU Saver Maximum Bundle",
			     "Sets CPU saver's maximum value",
			     E100_DEFAULT_CPUSAVER_BUNDLE_MAX, 0x1, 0xFFFF);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_ifs_def(char *page, char **start, off_t off,
	     int count, int *eof, void *data)
{
	int len;

	len = read_bool_param(page, "IFS",
			      "Setting this value to \"On\" enables "
			      "the adaptive IFS algorithm", E100_DEFAULT_IFS);

	return generic_read(page, start, off, count, eof, len);
}

static int
read_xsum_rx_val(char *page, char **start, off_t off,
		 int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	unsigned long val;

	val = (bdp->params.b_params & PRM_XSUMRX) ? 1 : 0;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_ucode_val(char *page, char **start, off_t off,
	       int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	unsigned long val;

	val = (bdp->params.b_params & PRM_UCODE) ? 1 : 0;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_fc_val(char *page, char **start, off_t off,
	    int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	unsigned long val;

	val = (bdp->params.b_params & PRM_FC) ? 1 : 0;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_ifs_val(char *page, char **start, off_t off,
	     int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	unsigned long val;

	val = (bdp->params.b_params & PRM_IFS) ? 1 : 0;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_bundle_small_val(char *page, char **start, off_t off,
		      int count, int *eof, void *data)
{
	struct e100_private *bdp = data;
	unsigned long val;

	val = (bdp->params.b_params & PRM_BUNDLE_SMALL) ? 1 : 0;
	return read_ulong(page, start, off, count, eof, val);
}

static int
read_gen_prm(char *page, char **start, off_t off,
	     int count, int *eof, void *data)
{
	int val = 0;

	if (data)
		val = *((int *) data);

	return read_ulong(page, start, off, count, eof, (unsigned long) val);
}

static e100_proc_entry e100_proc_params[] = { 
	/* definitions */
	{"e100_speed_duplex.def", read_speed_duplex_def, 0, 0},
	{"RxDescriptors.def",     read_rx_desc_def,      0, 0},
	{"TxDescriptors.def",     read_tx_desc_def,      0, 0},
	{"XsumRX.def",            read_xsum_rx_def,      0, 0},
	{"ucode.def",             read_ucode_def,        0, 0},
	{"BundleSmallFr.def",     read_bundle_small_def, 0, 0},
	{"IntDelay.def",          read_int_delay_def,    0, 0},
	{"BundleMax.def",         read_bundle_max_def,   0, 0},
	{"ber.def",               read_ber_def,          0, 0},
	{"flow_control.def",      read_fc_def,           0, 0},
	{"IFS.def",               read_ifs_def,          0, 0},
	/* values */
	{"e100_speed_duplex.val", read_gen_prm, 0, bdp_prm_off(e100_speed_duplex)},
	{"RxDescriptors.val",     read_gen_prm, 0, bdp_prm_off(RxDescriptors)},
	{"TxDescriptors.val",     read_gen_prm, 0, bdp_prm_off(TxDescriptors)},
	{"XsumRX.val",            read_xsum_rx_val,      0, 0},
	{"ucode.val",             read_ucode_val,        0, 0},
	{"BundleSmallFr.val",     read_bundle_small_val, 0, 0},
	{"IntDelay.val",          read_gen_prm, 0, bdp_prm_off(IntDelay)},
	{"BundleMax.val",         read_gen_prm, 0, bdp_prm_off(BundleMax)},
	{"ber.val",               read_gen_prm, 0, bdp_prm_off(ber)},
	{"flow_control.val",      read_fc_val,           0, 0},
	{"IFS.val",               read_ifs_val,          0, 0},
	{"", 0, 0, 0}
};

static struct proc_dir_entry *
create_proc_rw(char *name, void *data, struct proc_dir_entry *parent,
	       read_proc_t * read_proc, write_proc_t * write_proc)
{
	struct proc_dir_entry *pdep;
	mode_t mode = S_IFREG;

	if (write_proc) {
		mode |= S_IWUSR;
		if (read_proc) {
			mode |= S_IRUSR;
		}

	} else if (read_proc) {
		mode |= S_IRUGO;
	}

	if (!(pdep = create_proc_entry(name, mode, parent)))
		return NULL;

	pdep->read_proc = read_proc;
	pdep->write_proc = write_proc;
	pdep->data = data;
	return pdep;
}

#ifdef MODULE
static int
create_proc_param_subdir(struct e100_private *bdp,
			 struct proc_dir_entry *dev_dir)
{
	struct proc_dir_entry *param_dir;
	e100_proc_entry *pe;
	void *data;

	param_dir = create_proc_entry("LoadParameters", S_IFDIR, dev_dir);
	if (!param_dir)
		return -ENOMEM;

	for (pe = e100_proc_params; pe->name[0]; pe++) {

		data = ((char *) bdp) + pe->offset;

		if (!(create_proc_rw(pe->name, data, param_dir,
				     pe->read_proc, pe->write_proc))) {
			return -ENOMEM;
		}
	}

	return 0;
}

static void
remove_proc_param_subdir(struct proc_dir_entry *parent)
{
	struct proc_dir_entry *de;
	e100_proc_entry *pe;
	int len;

	len = strlen("LoadParameters");

	for (de = parent->subdir; de; de = de->next) {
		if ((de->namelen == len) &&
		    (!memcmp(de->name, "LoadParameters", len)))
			break;
	}

	if (!de)
		return;

	for (pe = e100_proc_params; pe->name[0]; pe++) {
		remove_proc_entry(pe->name, de);
	}

	remove_proc_entry("LoadParameters", parent);
}
#endif /* MODULE */

void
e100_remove_proc_subdir(struct e100_private *bdp, char *name)
{
	e100_proc_entry *pe;
	char info[256];
	int len;

	/* If our root /proc dir was not created, there is nothing to remove */
	if (adapters_proc_dir == NULL) {
		return;
	}

	len = strlen(name);
	strncpy(info, name, sizeof (info));
	strncat(info + len, ".info", sizeof (info) - len);

	if (bdp->proc_parent) {
		for (pe = e100_proc_list; pe->name[0]; pe++) {
			if (pe->name[0] == '\n')
				continue;

			remove_proc_entry(pe->name, bdp->proc_parent);
		}

#ifdef MODULE
		remove_proc_param_subdir(bdp->proc_parent);
#endif
		remove_proc_entry(name, adapters_proc_dir);
		bdp->proc_parent = NULL;
	}

	remove_proc_entry(info, adapters_proc_dir);

	/* try to remove the main /proc dir, if it's empty */
	e100_proc_cleanup();
}

int
e100_create_proc_subdir(struct e100_private *bdp, char *name)
{
	struct proc_dir_entry *dev_dir;
	e100_proc_entry *pe;
	char info[256];
	int len;
	void *data;

	/* create the main /proc dir if needed */
	if (!adapters_proc_dir) {
		if (!e100_init_proc_dir())
			return -ENOMEM;
	}

	strncpy(info, name, sizeof (info));
	len = strlen(info);
	strncat(info + len, ".info", sizeof (info) - len);

	/* info */
	if (!(create_proc_rw(info, bdp, adapters_proc_dir, read_info, 0))) {
		e100_proc_cleanup();
		return -ENOMEM;
	}

	dev_dir = create_proc_entry(name, S_IFDIR,
				    adapters_proc_dir);
	bdp->proc_parent = dev_dir;

	if (!dev_dir) {
		e100_remove_proc_subdir(bdp, name);
		return -ENOMEM;
	}

	for (pe = e100_proc_list; pe->name[0]; pe++) {
		if (pe->name[0] == '\n')
			continue;

		if (pe->offset != ~0)
			data = ((char *) bdp) + pe->offset;
		else
			data = NULL;

		if (!(create_proc_rw(pe->name, data, dev_dir,
				     pe->read_proc, pe->write_proc))) {
			e100_remove_proc_subdir(bdp, name);
			return -ENOMEM;
		}
	}

#ifdef MODULE
	if (create_proc_param_subdir(bdp, dev_dir)) {
		e100_remove_proc_subdir(bdp, name);
		return -ENOMEM;
	}
#endif

	return 0;
}

/****************************************************************************
 * Name:          e100_init_proc_dir
 *
 * Description:   This routine creates the top-level /proc directory for the
 *                driver in /proc/net
 *
 * Arguments:     none
 *
 * Returns:       true on success, false on fail
 *
 ***************************************************************************/
static unsigned char
e100_init_proc_dir(void)
{
	int len;

	/* first check if adapters_proc_dir already exists */
	len = strlen(ADAPTERS_PROC_DIR);
	for (adapters_proc_dir = proc_net->subdir;
	     adapters_proc_dir; adapters_proc_dir = adapters_proc_dir->next) {

		if ((adapters_proc_dir->namelen == len) &&
		    (!memcmp(adapters_proc_dir->name, ADAPTERS_PROC_DIR, len)))
			break;
	}

	if (!adapters_proc_dir)
		adapters_proc_dir =
			create_proc_entry(ADAPTERS_PROC_DIR, S_IFDIR, proc_net);

	if (!adapters_proc_dir)
		return false;

	return true;
}

/****************************************************************************
 * Name:          e100_proc_cleanup
 *
 * Description:   This routine clears the top-level /proc directory, if empty.
 *
 * Arguments:     none
 *
 * Returns:       none
 *
 ***************************************************************************/
static void
e100_proc_cleanup(void)
{
	struct proc_dir_entry *de;

	if (adapters_proc_dir == NULL) {
		return;
	}

	/* check if subdir list is empty before removing adapters_proc_dir */
	for (de = adapters_proc_dir->subdir; de; de = de->next) {
		/* ignore . and .. */
		if (*(de->name) != '.')
			break;
	}

	if (de)
		return;

	remove_proc_entry(ADAPTERS_PROC_DIR, proc_net);
	adapters_proc_dir = NULL;
}

#endif /* CONFIG_PROC_FS */
