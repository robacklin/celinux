/******************************************************************************
 *                  QLOGIC LINUX SOFTWARE
 *
 * QLogic ISP2x00 device driver for Linux 2.4.x
 * Copyright (C) 2003 Qlogic Corporation
 * (www.qlogic.com)
 *
 * Portions (C) Arjan van de Ven <arjanv@redhat.com> for Red Hat, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 ******************************************************************************/

/****************************************************************************
              Please see revision.notes for revision history.
*****************************************************************************/

/*
* String arrays
*/
#define LINESIZE    256
#define MAXARGS      26

/*
* Include files
*/
#include <linux/config.h>
#if defined(MODULE)
#include <linux/module.h>
#endif

#if !defined(LINUX_VERSION_CODE)
#include <linux/version.h>
#endif  /* LINUX_VERSION_CODE not defined */

/* Restrict compilation to 2.4.0 or greater */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0)
#error "This driver does not support kernel versions earlier than 2.4.0"
#endif

/* IP support not available on ISP2100 */
#if defined(ISP2100) && defined(FC_IP_SUPPORT)
#error "The ISP2100 does not support IP"
#endif

#include "qla_settings.h"

#if defined(QLA2XXX_CONFIG_BOOTIMG)
#include <linux/bootimg.h>
#endif

static int num_hosts = 0;       /* ioctl related  */
static int apiHBAInstance = 0;  /* ioctl related keeps track of API HBA Instance */

#if QL_TRACE_MEMORY
static unsigned long mem_trace[1000];
static unsigned long mem_id[1000];
#endif

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/segment.h>
#include <asm/byteorder.h>
#include <asm/pgtable.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/blk.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#include <linux/tqueue.h>
#endif
#include <linux/interrupt.h>
#include <linux/stat.h>
#include <linux/slab.h>

#define  APIDEV        1

#define __KERNEL_SYSCALLS__

#include <linux/unistd.h>
#include <linux/smp_lock.h>

#include <asm/system.h>
/*
* We must always allow SHUTDOWN_SIGS.  Even if we are not a module,
* the host drivers that we are using may be loaded as modules, and
* when we unload these,  we need to ensure that the error handler thread
* can be shut down.
*
* Note - when we unload a module, we send a SIGHUP.  We mustn't
* enable SIGTERM, as this is how the init shuts things down when you
* go to single-user mode.  For that matter, init also sends SIGKILL,
* so we mustn't enable that one either.  We use SIGHUP instead.  Other
* options would be SIGPWR, I suppose.
*/
#define SHUTDOWN_SIGS	(sigmask(SIGHUP))
#include "sd.h"
#include "scsi.h"
#include "hosts.h"
#ifdef __VMWARE__
#include "vmklinux_dist.h"
#endif

#if defined(FC_IP_SUPPORT)
#include <linux/ip.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include "qla_ip.h"
#endif

#if defined(FC_SCTP_SUPPORT)
#endif

#include "exioct.h"
#include "qla2x00.h"


#define UNIQUE_FW_NAME                 /* unique F/W array names */
#if defined(ISP2100)
#include "ql2100_fw.h"                     /* ISP RISC 2100 TP code */
#endif
#if defined(ISP2200)
#if defined(FC_IP_SUPPORT)
#include "ql2200ip_fw.h"                   /* ISP RISC 2200 IP code */
#else
#include "ql2200_fw.h"                     /* ISP RISC 2200 TP code */
#endif
#endif
#if defined(ISP2300)
#if defined(FC_IP_SUPPORT)
#include "ql2300ip_fw.h"                   /* ISP RISC 2300 IP code */
#else
#include "ql2300_fw.h"                     /* ISP RISC 2300 TP code */
#endif
#endif

#include "qla_cfg.h"
#include "qla_gbl.h"

#if NO_LONG_DELAYS
#define  SYS_DELAY(x)		qla2x00_sleep(x)
#define  QLA2100_DELAY(sec)  qla2x00_sleep(sec * HZ)
#define NVRAM_DELAY() qla2x00_sleep(10) /* 10 microsecond delay */
#define  UDELAY(x)		qla2x00_sleep(x)
#else
#define  SYS_DELAY(x)		udelay(x);barrier()
#define  QLA2100_DELAY(sec)  mdelay(sec * HZ)
#define NVRAM_DELAY() udelay(10) /* 10 microsecond delay */
#define  UDELAY(x)		udelay(x)
#endif

/* 
 * We only use these macros in 64bit_start and not 32bit_start, so
 * we can assume a 8-byte address (a).
 */
#define pci_dma_hi32(a) ((u32) (0xffffffff & (((u64)(a))>>32)))
#define pci_dma_lo32(a) ((u32) (0xffffffff & (((u64)(a)))))

#define  CACHE_FLUSH(a) (RD_REG_WORD(a))
#define  INVALID_HANDLE    (MAX_OUTSTANDING_COMMANDS+1)

#define  ABORTS_ACTIVE  ((test_bit(LOOP_RESET_NEEDED, &ha->dpc_flags)) || \
			(test_bit(DEVICE_RESET_NEEDED, &ha->dpc_flags)) || \
			(test_bit(DEVICE_ABORT_NEEDED, &ha->dpc_flags)) || \
			(test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags)))

#define  STATIC static

#define  OFFSET(w)   (((u_long) &w) & 0xFFFF)  /* 256 byte offsets */

/*
 * LOCK MACROS
 */

#define QLA_MBX_REG_LOCK(ha)	\
    spin_lock_irqsave(&(ha)->mbx_reg_lock, mbx_flags);
#define QLA_MBX_REG_UNLOCK(ha)	\
    spin_unlock_irqrestore(&(ha)->mbx_reg_lock, mbx_flags);

#define	WATCH_INTERVAL		1       /* number of seconds */
#define	START_TIMER(f, h, w)	\
{ \
init_timer(&(h)->timer); \
(h)->timer.expires = jiffies + w * HZ;\
(h)->timer.data = (unsigned long) h; \
(h)->timer.function = (void (*)(unsigned long))f; \
add_timer(&(h)->timer); \
(h)->timer_active = 1;\
}

#define	RESTART_TIMER(f, h, w)	\
{ \
mod_timer(&(h)->timer,jiffies + w * HZ); \
}

#define	STOP_TIMER(f, h)	\
{ \
del_timer_sync(&(h)->timer); \
(h)->timer_active = 0;\
}

#define COMPILE 0

#if defined(ISP2100)
#define DRIVER_NAME "qla2100"
#endif
#if defined(ISP2200)
#define DRIVER_NAME "qla2200"
#endif
#if defined(ISP2300)
#define DRIVER_NAME "qla2300"
#endif

static char qla2x00_version_str[40];
typedef unsigned long paddr32_t;

/* proc info string processing */
struct info_str {
	char	*buffer;
	int	length;
	off_t	offset;
	int	pos;
};


/*
*  Qlogic Driver support Function Prototypes.
*/
STATIC void copy_mem_info(struct info_str *, char *, int);
STATIC int copy_info(struct info_str *, char *, ...);

STATIC uint8_t qla2x00_register_with_Linux(scsi_qla_host_t *ha,
			uint8_t maxchannels);
STATIC int qla2x00_done(scsi_qla_host_t *);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
STATIC void qla2x00_select_queue_depth(struct Scsi_Host *, Scsi_Device *);
#endif

#if defined (CONFIG_SCSIFCHOTSWAP) || defined(CONFIG_GAMAP)
int qla2x00_get_scsi_info_from_wwn (int mode, unsigned long long wwn, int *host, int *channel, int *lun, int *id);
int qla2x00_get_wwn_from_scsi_info (int host, int id, unsigned long long *wwn);
#endif /* CONFIG_SCSIFCHOTSWAP || CONFIG_GAMAP */

STATIC void qla2x00_timer(scsi_qla_host_t *);

STATIC uint8_t qla2x00_mem_alloc(scsi_qla_host_t *);

static void qla2x00_dump_regs(struct Scsi_Host *host);
#if STOP_ON_ERROR
static void qla2x00_panic(char *, struct Scsi_Host *host);
#endif
void qla2x00_print_scsi_cmd(Scsi_Cmnd *cmd);

#if 0
STATIC void qla2x00_abort_pending_queue(scsi_qla_host_t *ha, uint32_t stat);
#endif

STATIC void qla2x00_mem_free(scsi_qla_host_t *ha);
void qla2x00_do_dpc(void *p);

static inline void qla2x00_callback(scsi_qla_host_t *ha, Scsi_Cmnd *cmd);

static inline void qla2x00_enable_intrs(scsi_qla_host_t *);
static inline void qla2x00_disable_intrs(scsi_qla_host_t *);

static void qla2x00_extend_timeout(Scsi_Cmnd *cmd, int timeout);

static int  qla2x00_get_tokens(char *line, char **argv, int maxargs );

/*
*  QLogic ISP2x00 Hardware Support Function Prototypes.
*/
STATIC void qla2x00_cfg_persistent_binding(scsi_qla_host_t *ha);
STATIC uint8_t qla2x00_initialize_adapter(scsi_qla_host_t *);
STATIC uint8_t qla2x00_isp_firmware(scsi_qla_host_t *);
STATIC uint8_t qla2x00_pci_config(scsi_qla_host_t *);
STATIC uint8_t qla2x00_set_cache_line(scsi_qla_host_t *);
STATIC uint8_t qla2x00_chip_diag(scsi_qla_host_t *);
STATIC uint8_t qla2x00_setup_chip(scsi_qla_host_t *ha);
STATIC uint8_t qla2x00_init_rings(scsi_qla_host_t *ha);
STATIC uint8_t qla2x00_fw_ready(scsi_qla_host_t *ha);
#if defined(ISP2100)
STATIC uint8_t qla2100_nvram_config(scsi_qla_host_t *);
#else
STATIC uint8_t qla2x00_nvram_config(scsi_qla_host_t *);
#endif
STATIC uint8_t qla2x00_get_link_status(scsi_qla_host_t *,
		uint8_t, void *, uint16_t *);

STATIC uint8_t qla2x00_loop_reset(scsi_qla_host_t *ha);
STATIC uint8_t qla2x00_abort_isp(scsi_qla_host_t *, uint8_t);
STATIC uint8_t qla2x00_loop_resync(scsi_qla_host_t *);

STATIC void qla2x00_nv_write(scsi_qla_host_t *, uint16_t);
STATIC void qla2x00_nv_deselect(scsi_qla_host_t *ha);
STATIC void qla2x00_poll(scsi_qla_host_t *);
STATIC void qla2x00_init_fc_db(scsi_qla_host_t *);
STATIC void qla2x00_init_tgt_map(scsi_qla_host_t *);
STATIC void qla2x00_reset_adapter(scsi_qla_host_t *);
STATIC void qla2x00_enable_lun(scsi_qla_host_t *);
STATIC void qla2x00_isp_cmd(scsi_qla_host_t *);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,7)
STATIC void qla2x00_process_risc_intrs(scsi_qla_host_t *);
#endif
STATIC void qla2x00_isr(scsi_qla_host_t *, uint16_t,  uint8_t *);
STATIC void qla2x00_rst_aen(scsi_qla_host_t *);

STATIC void qla2x00_response_pkt(scsi_qla_host_t *, uint16_t);
STATIC void qla2x00_status_entry(scsi_qla_host_t *, sts_entry_t *);
STATIC void qla2x00_status_cont_entry(scsi_qla_host_t *, sts_cont_entry_t *);
STATIC void qla2x00_error_entry(scsi_qla_host_t *, response_t *);
STATIC void qla2x00_ms_entry(scsi_qla_host_t *, ms_iocb_entry_t *);

STATIC void qla2x00_restart_queues(scsi_qla_host_t *, uint8_t);
STATIC void qla2x00_abort_queues(scsi_qla_host_t *, uint8_t);

STATIC uint16_t qla2x00_get_nvram_word(scsi_qla_host_t *, uint32_t);
STATIC uint16_t qla2x00_nvram_request(scsi_qla_host_t *, uint32_t);
STATIC uint16_t qla2x00_debounce_register(volatile uint16_t *);

STATIC request_t *qla2x00_req_pkt(scsi_qla_host_t *);
STATIC request_t *qla2x00_ms_req_pkt(scsi_qla_host_t *, srb_t *);
STATIC uint8_t qla2x00_configure_hba(scsi_qla_host_t *ha);
STATIC void qla2x00_reset_chip(scsi_qla_host_t *ha);

STATIC void qla2x00_display_fc_names(scsi_qla_host_t *ha);
void qla2x00_dump_requests(scsi_qla_host_t *ha);
static void qla2x00_get_properties(scsi_qla_host_t *ha, char *string);
STATIC uint8_t qla2x00_find_propname(scsi_qla_host_t *ha,
		char *propname, char *propstr, char *db, int siz);
static int qla2x00_get_prop_16chars(scsi_qla_host_t *ha,
		char *propname, char *propval, char *cmdline);
static char *qla2x00_get_line(char *str, char *line);
void qla2x00_check_fabric_devices(scsi_qla_host_t *ha);

#if defined(FC_IP_SUPPORT)
/* General support routines */
static int qla2x00_ip_initialize(scsi_qla_host_t *ha);
static void qla2x00_ip_send_complete(scsi_qla_host_t *ha,
		uint32_t handle, uint16_t comp_status);
static void qla2x00_ip_receive(scsi_qla_host_t *ha, response_t *pkt);
static void qla2x00_ip_receive_fastpost(scsi_qla_host_t *ha, uint16_t type);

/* IP device list manipulation routines */
static int qla2x00_convert_to_arp(scsi_qla_host_t *ha, struct send_cb *scb);
static int qla2x00_get_ip_loopid(scsi_qla_host_t *ha,
		struct packet_header *packethdr, uint8_t *loop_id);
static int qla2x00_reserve_loopid(scsi_qla_host_t *ha, uint16_t *loop_id);
static void qla2x00_free_loopid(scsi_qla_host_t *ha, uint16_t loop_id);

static int qla2x00_add_new_ip_device(scsi_qla_host_t *ha,
		uint16_t loop_id, uint8_t *port_id,
		uint8_t *port_name, int force_add, uint32_t ha_locked);
static void qla2x00_free_ip_block(scsi_qla_host_t *ha, struct ip_device *ipdev);
static int qla2x00_reserve_ip_block(scsi_qla_host_t *ha,
		struct ip_device **ipdevblk);
static int qla2x00_update_ip_device_data(scsi_qla_host_t *ha, fcdev_t *fcdev);
static int qla2x00_ip_send_login_port_iocb(scsi_qla_host_t *ha,
		struct ip_device *ipdev, uint32_t ha_locked);
static int qla2x00_ip_send_logout_port_iocb(scsi_qla_host_t *ha, 
		struct ip_device *ipdev, uint32_t ha_locked);
static void qla2x00_ip_mailbox_iocb_done(scsi_qla_host_t *ha,
		struct mbx_entry *mbxentry);

/* Entry point network driver */
#if defined(ISP2200)
int  qla2200_ip_inquiry(uint16_t adapter_num, struct bd_inquiry *inq_data);
EXPORT_SYMBOL(qla2200_ip_inquiry);
#elif defined(ISP2300)
int  qla2300_ip_inquiry(uint16_t adapter_num, struct bd_inquiry *inq_data);
EXPORT_SYMBOL(qla2300_ip_inquiry);
#endif

/* Network driver callback routines */
static int  qla2x00_ip_enable(scsi_qla_host_t *ha,
		struct bd_enable *enable_data);
static void qla2x00_ip_disable(scsi_qla_host_t *ha);
static void qla2x00_add_buffers(scsi_qla_host_t *ha,
		uint16_t rec_count, int ha_locked);
static int  qla2x00_send_packet(scsi_qla_host_t *ha, struct send_cb *scb);
static int  qla2x00_tx_timeout(scsi_qla_host_t *ha);
#endif	/* if defined(FC_IP_SUPPORT) */

static void qla2x00_device_resync(scsi_qla_host_t *);
STATIC uint8_t qla2x00_update_fc_database(scsi_qla_host_t *, fcdev_t *,
		uint8_t);

STATIC uint8_t qla2x00_configure_fabric(scsi_qla_host_t *, uint8_t );
static uint8_t qla2x00_find_all_fabric_devs(scsi_qla_host_t *,
		sns_cmd_rsp_t *, dma_addr_t, struct new_dev *,
		uint16_t *, uint8_t *);
#if REG_FC4_ENABLED
static uint8_t qla2x00_register_fc4(scsi_qla_host_t *, sns_cmd_rsp_t *, dma_addr_t);
static uint8_t qla2x00_register_fc4_feature(scsi_qla_host_t *, sns_cmd_rsp_t *, dma_addr_t);
#endif
static uint8_t qla2x00_gan(scsi_qla_host_t *, sns_cmd_rsp_t *, dma_addr_t,
		fcdev_t *);
static uint8_t qla2x00_fabric_login(scsi_qla_host_t *, fcdev_t *);
static uint8_t qla2x00_local_device_login(scsi_qla_host_t *, uint16_t);

STATIC uint8_t qla2x00_configure_loop(scsi_qla_host_t *);
static uint8_t qla2x00_configure_local_loop(scsi_qla_host_t *, uint8_t );

STATIC uint8_t qla2x00_32bit_start_scsi(srb_t *sp);

STATIC uint8_t qla2x00_64bit_start_scsi(srb_t *sp);

/* Routines for Failover */
os_tgt_t *qla2x00_tgt_alloc(scsi_qla_host_t *ha, uint16_t t);
#if APIDEV
static int apidev_init(struct Scsi_Host*);
static int apidev_cleanup(void);
#endif
void qla2x00_tgt_free(scsi_qla_host_t *ha, uint16_t t);
os_lun_t *qla2x00_lun_alloc(scsi_qla_host_t *ha, uint16_t t, uint16_t l);

static void qla2x00_lun_free(scsi_qla_host_t *ha, uint16_t t, uint16_t l);
void qla2x00_next(scsi_qla_host_t *vis_ha);
static int qla2x00_build_fcport_list(scsi_qla_host_t *ha);
static void qla2x00_config_os(scsi_qla_host_t *ha);
static uint16_t qla2x00_fcport_bind(scsi_qla_host_t *ha, fc_port_t *fcport);
static int qla2x00_update_fcport(scsi_qla_host_t *ha, fc_port_t *fcport, int);
static int qla2x00_lun_discovery(scsi_qla_host_t *ha, fc_port_t *fcport, int);
static int qla2x00_rpt_lun_discovery(scsi_qla_host_t *ha, fc_port_t *fcport);
static void qla2x00_cfg_lun(fc_port_t *fcport, uint16_t lun);

STATIC void qla2x00_process_failover(scsi_qla_host_t *ha) ;

STATIC int qla2x00_device_reset(scsi_qla_host_t *, uint16_t, uint16_t);

static inline int qla2x00_is_wwn_zero(uint8_t *wwn);
void qla2x00_get_lun_mask_from_config(scsi_qla_host_t *ha, fc_port_t *port,
                                      uint16_t tgt, uint16_t dev_no);
void 
qla2x00_print_q_info(os_lun_t *q);

#if QLA2X_PERFORMANCE
void qla2x00_done_tasklet(long p);
#endif

STATIC void qla2x00_failover_cleanup(srb_t *);
void qla2x00_flush_failover_q(scsi_qla_host_t *, os_lun_t *);

void qla2x00_chg_endian(uint8_t buf[], size_t size);
STATIC uint8_t qla2x00_check_sense(Scsi_Cmnd *cp, os_lun_t *);

STATIC uint8_t 
__qla2x00_suspend_lun(scsi_qla_host_t *, os_lun_t *, int, int, int);
STATIC uint8_t 
qla2x00_suspend_lun(scsi_qla_host_t *, os_lun_t *, int, int);
STATIC uint8_t
qla2x00_delay_lun(scsi_qla_host_t *, os_lun_t *, int);

STATIC uint8_t
qla2x00_check_for_devices_online(scsi_qla_host_t *ha);


#if DEBUG_QLA2100
#if !defined(QL_DEBUG_ROUTINES)
#define QL_DEBUG_ROUTINES
#endif
#endif

#if defined(QL_DEBUG_ROUTINES)
/*
*  Driver Debug Function Prototypes.
*/
static void qla2x00_dump_buffer(uint8_t *, uint32_t);
STATIC uint8_t ql2x_debug_print = 1;
#endif

/* ra 01/03/02 */
#if QLA2100_LIPTEST
STATIC int  mbxtimeout = 0;
#endif

#if DEBUG_GET_FW_DUMP
STATIC void qla2300_dump_isp(scsi_qla_host_t *ha),
qla2x00_dump_word(uint8_t *, uint32_t, uint32_t);
#endif
#if  NO_LONG_DELAYS
STATIC void qla2x00_sleep_done (struct semaphore * sem);
#endif

uint8_t qla2x00_allocate_sp_pool( scsi_qla_host_t *ha);
void qla2x00_free_sp_pool(scsi_qla_host_t *ha );
STATIC srb_t * qla2x00_get_new_sp (scsi_qla_host_t *ha);
STATIC uint8_t qla2x00_check_tgt_status(scsi_qla_host_t *ha, Scsi_Cmnd *cmd);
STATIC uint8_t qla2x00_check_port_status(scsi_qla_host_t *ha,
		fc_port_t *fcport);
STATIC void qla2x00_mark_device_lost(scsi_qla_host_t *ha, fc_port_t *fcport);
STATIC void qla2x00_mark_all_devices_lost( scsi_qla_host_t *ha );
STATIC inline void qla2x00_delete_from_done_queue(scsi_qla_host_t *, srb_t *); 

static inline int qla2x00_marker(scsi_qla_host_t *,
		uint16_t, uint16_t, uint8_t);
STATIC int __qla2x00_marker(scsi_qla_host_t *, uint16_t, uint16_t, uint8_t);
static inline int 
qla2x00_marker(scsi_qla_host_t *ha,
		uint16_t loop_id,
		uint16_t lun,
		uint8_t type)
{
	int ret;
	unsigned long flags = 0;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	ret = __qla2x00_marker(ha, loop_id, lun, type);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	return (ret);
}

/* Flash support routines */
#define FLASH_IMAGE_SIZE	131072

STATIC void qla2x00_flash_enable(scsi_qla_host_t *);
STATIC void qla2x00_flash_disable(scsi_qla_host_t *);
STATIC uint8_t qla2x00_read_flash_byte(scsi_qla_host_t *, uint32_t);
STATIC void qla2x00_write_flash_byte(scsi_qla_host_t *, uint32_t, uint8_t);
STATIC uint8_t qla2x00_poll_flash(scsi_qla_host_t *ha,
		uint32_t addr, uint8_t poll_data, uint8_t mid);
STATIC uint8_t qla2x00_program_flash_address(scsi_qla_host_t *ha,
		uint32_t addr, uint8_t data, uint8_t mid);
STATIC uint8_t qla2x00_erase_flash_sector(scsi_qla_host_t *ha,
		uint32_t addr, uint32_t sec_mask, uint8_t mid);
STATIC uint8_t qla2x00_get_flash_manufacturer(scsi_qla_host_t *ha);
STATIC uint16_t qla2x00_get_flash_version(scsi_qla_host_t *);
#if defined(NOT_USED_FUNCTION)
STATIC uint16_t qla2x00_get_flash_image(scsi_qla_host_t *ha, uint8_t *image);
#endif
STATIC uint16_t qla2x00_set_flash_image(scsi_qla_host_t *ha, uint8_t *image);

#if USE_FLASH_DATABASE
STATIC void qla2x00_flash_enable_database(scsi_qla_host_t *);
STATIC void qla2x00_flash_disable_database(scsi_qla_host_t *);
STATIC uint8_t qla2x00_get_database(scsi_qla_host_t *);
STATIC uint8_t qla2x00_save_database(scsi_qla_host_t *);
#endif

/* Some helper functions */
static inline uint32_t qla2x00_normalize_dma_addr(
		dma_addr_t *e_addr,  uint32_t *e_len,
		dma_addr_t *ne_addr, uint32_t *ne_len);

static inline uint16_t qla2x00_check_request_ring(
		scsi_qla_host_t *ha, uint16_t tot_iocbs,
		uint16_t req_ring_index, uint16_t *req_q_cnt);

static inline cont_entry_t *qla2x00_prep_cont_packet(
		scsi_qla_host_t *ha,
		uint16_t *req_ring_index, request_t **request_ring_ptr);

static inline cont_a64_entry_t *qla2x00_prep_a64_cont_packet(
		scsi_qla_host_t *ha,
		uint16_t *req_ring_index, request_t **request_ring_ptr);

/**
 * qla2x00_normalize_dma_addr() - Normalize an DMA address.
 * @e_addr: Raw DMA address
 * @e_len: Raw DMA length
 * @ne_addr: Normalized second DMA address
 * @ne_len: Normalized second DMA length
 *
 * If the address does not span a 4GB page boundary, the contents of @ne_addr
 * and @ne_len are undefined.  @e_len is updated to reflect a normalization.
 *
 * Example:
 *
 * 	ffffabc0ffffeeee	(e_addr) start of DMA address
 * 	0000000020000000	(e_len)  length of DMA transfer
 *	ffffabc11fffeeed	end of DMA transfer
 *
 * Is the 4GB boundary crossed?
 *
 * 	ffffabc0ffffeeee	(e_addr)
 *	ffffabc11fffeeed	(e_addr + e_len - 1)
 *	00000001e0000003	((e_addr ^ (e_addr + e_len - 1))
 *	0000000100000000	((e_addr ^ (e_addr + e_len - 1)) & ~(0xffffffff)
 *
 * Compute start of second DMA segment:
 *
 * 	ffffabc0ffffeeee	(e_addr)
 *	ffffabc1ffffeeee	(0x100000000 + e_addr)
 *	ffffabc100000000	(0x100000000 + e_addr) & ~(0xffffffff)
 *	ffffabc100000000	(ne_addr)
 *	
 * Compute length of second DMA segment:
 *
 *	00000000ffffeeee	(e_addr & 0xffffffff)
 *	0000000000001112	(0x100000000 - (e_addr & 0xffffffff))
 *	000000001fffeeee	(e_len - (0x100000000 - (e_addr & 0xffffffff))
 *	000000001fffeeee	(ne_len)
 *
 * Adjust length of first DMA segment
 *
 * 	0000000020000000	(e_len)
 *	0000000000001112	(e_len - ne_len)
 *	0000000000001112	(e_len)
 *
 * Returns non-zero if the specified address was normalized, else zero.
 */
static inline uint32_t
qla2x00_normalize_dma_addr(
		dma_addr_t *e_addr,  uint32_t *e_len,
		dma_addr_t *ne_addr, uint32_t *ne_len)
{
	uint32_t normalized;

	normalized = 0;
	if ((*e_addr ^ (*e_addr + *e_len - 1)) & ~(0xFFFFFFFFULL)) {
		/* Compute normalized crossed address and len */
		*ne_addr = (0x100000000ULL + *e_addr) & ~(0xFFFFFFFFULL);
		*ne_len = *e_len - (0x100000000ULL - (*e_addr & 0xFFFFFFFFULL));
		*e_len -= *ne_len;

		normalized++;
	}
	return (normalized);
}

static int
qla2x00_add_initiator_device(scsi_qla_host_t *ha, fcdev_t *device);


/* Debug print buffer */
char          debug_buff[LINESIZE*3];

/*
* insmod needs to find the variable and make it point to something
*/
static char *ql2xdevconf = NULL;
#if MPIO_SUPPORT
static int ql2xretrycount = 30;
#else
static int ql2xretrycount = 20;
#endif
static int qla2xenbinq = 1;
static int max_srbs = MAX_SRBS;
#if defined(ISP2200) || defined(ISP2300)
static int ql2xlogintimeout = 20;
static int qlport_down_retry = 0;
#endif
static int ql2xmaxqdepth = 0;
static int displayConfig = 0;

/* Enable for failover */
#if MPIO_SUPPORT
static int ql2xfailover = 1;
#else
static int ql2xfailover = 0;
#endif

static int ConfigRequired = 0;
static int recoveryTime = MAX_RECOVERYTIME;
static int failbackTime = MAX_FAILBACKTIME;

/* Persistent binding type */
static int Bind = BIND_BY_PORT_NAME;

static int ql2xsuspendcount = SUSPEND_COUNT;

#if defined(MODULE)
static char *ql2xopts = NULL;

/* insmod qla2100 ql2xopts=verbose" */
MODULE_PARM(ql2xopts, "s");
MODULE_PARM_DESC(ql2xopts,
		"Additional driver options.");

MODULE_PARM(ql2xfailover, "i");
MODULE_PARM_DESC(ql2xfailover,
		"Driver failover support: 0 to disable; 1 to enable. "
		"Default behaviour based on compile-time option "
		"MPIO_SUPPORT.");

MODULE_PARM(ql2xmaxqdepth, "i");
MODULE_PARM_DESC(ql2xmaxqdepth,
		"Maximum queue depth to report for target devices.");

#if defined(ISP2200) || defined(ISP2300)
MODULE_PARM(ql2xlogintimeout,"i");
MODULE_PARM_DESC(ql2xlogintimeout,
		"Login timeout value in seconds.");

MODULE_PARM(qlport_down_retry,"i");
MODULE_PARM_DESC(qlport_down_retry,
		"Maximum number of command retries to a port that returns"
		"a PORT-DOWN status.");
#endif

MODULE_PARM(ql2xretrycount,"i");
MODULE_PARM_DESC(ql2xretrycount,
		"Maximum number of mid-layer retries allowed for a command.  "
		"Default value in non-failover mode is 20, "
		"in failover mode, 30.");

MODULE_PARM(max_srbs,"i");
MODULE_PARM_DESC(max_srbs,
		"Maximum number of simultaneous commands allowed for an HBA.");

MODULE_PARM(displayConfig, "i");
MODULE_PARM_DESC(displayConfig,
		"If 1 then display the configuration used in "
		"/etc/modules.conf.");

MODULE_PARM(ConfigRequired, "i");
MODULE_PARM_DESC(ConfigRequired,
		"If 1, then only configured devices passed in through the"
		"ql2xopts parameter will be presented to the OS");

MODULE_PARM(recoveryTime, "i");
MODULE_PARM_DESC(recoveryTime,
		"Recovery time in seconds before a target device is sent I/O "
		"after a failback is performed.");

MODULE_PARM(failbackTime, "i");
MODULE_PARM_DESC(failbackTime,
		"Delay in seconds before a failback is performed.");

MODULE_PARM(Bind, "i");
MODULE_PARM_DESC(Bind,
		"Target persistent binding method: "
		"0 by Portname (default); 1 by PortID; 2 by Nodename. ");

MODULE_PARM(ql2xsuspendcount,"i");
MODULE_PARM_DESC(ql2xsuspendcount,
		"Number of 6-second suspend iterations to perform while a "
		"target returns a <NOT READY> status.  Default is 10 "
		"iterations.");

MODULE_AUTHOR("QLogic Corporation");
#if defined(MODULE_LICENSE)
	 MODULE_LICENSE("GPL");
#endif

/*
* Just in case someone uses commas to separate items on the insmod
* command line, we define a dummy buffer here to avoid having insmod
* write wild stuff into our code segment
*/
static char dummy_buffer[60] =
		"Please don't add commas in your insmod command!!\n";

#endif

#include "listops.h"
#include "qla_fo.cfg"


#if QLA2100_LIPTEST
static int qla2x00_lip = 0;
#endif

#include <linux/ioctl.h>
#include <scsi/scsi_ioctl.h>

/* multi-OS QLOGIC IOCTL definition file */
#include "exioct.h"


#if QLA_SCSI_VENDOR_DIR
/* Include routine to set direction for vendor specific commands */
#include "qla_vendor.c"
#endif
/***********************************************************************
* We use the Scsi_Pointer structure that's included with each command
* SCSI_Cmnd as a scratchpad. 
*
* SCp is defined as follows:
*  - SCp.ptr  -- > pointer to the SRB
*  - SCp.this_residual  -- > HBA completion status for ioctl code. 
*
* Cmnd->host_scribble --> Used to hold the hba actived handle (1..255).
***********************************************************************/
#define	CMD_SP(Cmnd)		((Cmnd)->SCp.ptr)
#define CMD_COMPL_STATUS(Cmnd)  ((Cmnd)->SCp.this_residual)
#define	CMD_HANDLE(Cmnd)	((Cmnd)->host_scribble)
/* Additional fields used by ioctl passthru */
#define CMD_RESID_LEN(Cmnd)     ((Cmnd)->SCp.buffers_residual)
#define CMD_SCSI_STATUS(Cmnd)   ((Cmnd)->SCp.Status)
#define CMD_ACTUAL_SNSLEN(Cmnd) ((Cmnd)->SCp.Message)
#define CMD_ENTRY_STATUS(Cmnd)  ((Cmnd)->SCp.have_data_in)

/*
 * Other SCS__Cmnd members we only reference
 */
#define	CMD_XFRLEN(Cmnd)	(Cmnd)->request_bufflen
#define	CMD_CDBLEN(Cmnd)	(Cmnd)->cmd_len
#define	CMD_CDBP(Cmnd)		(Cmnd)->cmnd
#define	CMD_SNSP(Cmnd)		(Cmnd)->sense_buffer
#define	CMD_SNSLEN(Cmnd)	(sizeof (Cmnd)->sense_buffer)
#define	CMD_RESULT(Cmnd)	((Cmnd)->result)
#define	CMD_TIMEOUT(Cmnd)	((Cmnd)->timeout_per_command)

#include "qla_debug.h"

uint8_t copyright[48] = "Copyright 1999-2003, QLogic Corporation";

/****************************************************************************/
/*  LINUX -  Loadable Module Functions.                                     */
/****************************************************************************/

/*****************************************/
/*   ISP Boards supported by this driver */
/*****************************************/
#define QLA2X00_VENDOR_ID   0x1077
#define QLA2100_DEVICE_ID   0x2100
#define QLA2200_DEVICE_ID   0x2200
#define QLA2200A_DEVICE_ID  0x2200A
#define QLA2300_DEVICE_ID   0x2300
#define QLA2312_DEVICE_ID   0x2312
#define QLA2200A_RISC_ROM_VER  4
#define FPM_2300            6
#define FPM_2310            7

#if defined(ISP2100)
#define NUM_OF_ISP_DEVICES  2
static struct pci_device_id qla2100_pci_tbl[] =
{
	{QLA2X00_VENDOR_ID, QLA2100_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{0,}
};
MODULE_DEVICE_TABLE(pci, qla2100_pci_tbl);
#endif
#if defined(ISP2200)
#define NUM_OF_ISP_DEVICES  2
static struct pci_device_id qla2200_pci_tbl[] =
{
	{QLA2X00_VENDOR_ID, QLA2200_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{0,}
};
MODULE_DEVICE_TABLE(pci, qla2200_pci_tbl);
#endif
#if defined(ISP2300)
#define NUM_OF_ISP_DEVICES  3
static struct pci_device_id qla2300_pci_tbl[] =
{
	{QLA2X00_VENDOR_ID, QLA2300_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{QLA2X00_VENDOR_ID, QLA2312_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{0,}
};
MODULE_DEVICE_TABLE(pci, qla2300_pci_tbl);
#endif

typedef struct _qlaboards
{
        unsigned char   bdName[9];       /* Board ID String             */
        unsigned long   device_id;       /* Device ID                   */
        int   numPorts;                  /* number of loops on adapter  */
        unsigned short   *fwcode;        /* pointer to FW array         */
        unsigned short   *fwlen;         /* number of words in array    */
        unsigned short   *fwstart;       /* start address for F/W       */
        unsigned char   *fwver;          /* Ptr to F/W version array    */
}
qla_boards_t;

/*
 * NOTE: Check the Product ID of the Chip during chip diagnostics
 *       whenever support for new ISP is added. 
 */
static struct _qlaboards   QLBoardTbl_fc[NUM_OF_ISP_DEVICES] =
{
	/* Name ,  Board PCI Device ID,         Number of ports */
#if defined(ISP2300)
	{"QLA2312 ", QLA2312_DEVICE_ID,           MAX_BUSES,
#if defined(FC_IP_SUPPORT)
		&fw2300ip_code01[0], &fw2300ip_length01,
		&fw2300ip_addr01, &fw2300ip_version_str[0]
	},
#else
		&fw2300tp_code01[0], &fw2300tp_length01,
		&fw2300tp_addr01, &fw2300tp_version_str[0]
	},
#endif
	{"QLA2300 ", QLA2300_DEVICE_ID,           MAX_BUSES,
#if defined(FC_IP_SUPPORT)
		&fw2300ip_code01[0], &fw2300ip_length01,
		&fw2300ip_addr01, &fw2300ip_version_str[0]
	},
#else
		&fw2300tp_code01[0], &fw2300tp_length01,
		&fw2300tp_addr01, &fw2300tp_version_str[0]
	},
#endif
#endif

#if defined(ISP2200)
	{"QLA2200 ", QLA2200_DEVICE_ID,           MAX_BUSES,
#if defined(FC_IP_SUPPORT)
		&fw2200ip_code01[0], &fw2200ip_length01,
		&fw2200ip_addr01, &fw2200ip_version_str[0]
	},
#else
		&fw2200tp_code01[0], &fw2200tp_length01,
		&fw2200tp_addr01, &fw2200tp_version_str[0]
	},
#endif
#endif

#if defined(ISP2100)
	{"QLA2100 ", QLA2100_DEVICE_ID,           MAX_BUSES,
		&fw2100tp_code01[0], &fw2100tp_length01,
		&fw2100tp_addr01, &fw2100tp_version_str[0]
	},
#endif

	{"        ",                 0,           0}
};

/*
* Stat info for all adpaters
*/
static struct _qla2100stats  {
        unsigned long   mboxtout;            /* mailbox timeouts */
        unsigned long   mboxerr;             /* mailbox errors */
        unsigned long   ispAbort;            /* ISP aborts */
        unsigned long   debugNo;
        unsigned long   loop_resync;
        unsigned long   outarray_full;
        unsigned long   retry_q_cnt;
#ifdef PERF_MONITORING
        unsigned long   highmem_io;
#endif
        scsi_qla_host_t *irqhba;
}
qla2x00_stats;

/*
 * Declare our global semaphores
 */
#if defined(ISP2100)
DECLARE_MUTEX_LOCKED(qla2100_detect_sem);
#endif
#if defined(ISP2200)
DECLARE_MUTEX_LOCKED(qla2200_detect_sem);
#endif
#if defined(ISP2300)
DECLARE_MUTEX_LOCKED(qla2300_detect_sem);
#endif


/*
* Command line options
*/
static unsigned long qla2x00_verbose = 1L;
static unsigned long qla2x00_quiet   = 0L;
static unsigned long qla2x00_reinit = 1L;
static unsigned long qla2x00_req_dmp = 0L;

#if QL_TRACE_MEMORY
extern unsigned long mem_trace[1000];
extern unsigned long mem_id[1000];
int	mem_trace_ptr = 0;
#endif

/*
 * List of host adapters
 */
static scsi_qla_host_t *qla2x00_hostlist = NULL;


STATIC int qla2x00_retryq_dmp = 0;              /* dump retry queue */

#include <linux/ioctl.h>
#include <scsi/scsi_ioctl.h>
#include <asm/uaccess.h>


#define MAX_LOCAL_LOOP_IDS	127
static uint8_t alpa_table[MAX_LOCAL_LOOP_IDS] = {
	0xEF, 0xE8, 0xE4, 0xE2, 0xE1, 0xE0, 0xDC, 0xDA,
	0xD9, 0xD6, 0xD5, 0xD4, 0xD3, 0xD2, 0xD1, 0xCE,
	0xCD, 0xCC, 0xCB, 0xCA, 0xC9, 0xC7, 0xC6, 0xC5,
	0xC3, 0xBC, 0xBA, 0xB9, 0xB6, 0xB5, 0xB4, 0xB3,
	0xB2, 0xB1, 0xAE, 0xAD, 0xAC, 0xAB, 0xAA, 0xA9,
	0xA7, 0xA6, 0xA5, 0xA3, 0x9F, 0x9E, 0x9D, 0x9B,
	0x98, 0x97, 0x90, 0x8F, 0x88, 0x84, 0x82, 0x81,
	0x80, 0x7C, 0x7A, 0x79, 0x76, 0x75, 0x74, 0x73,
	0x72, 0x71, 0x6E, 0x6D, 0x6C, 0x6B, 0x6A, 0x69,
	0x67, 0x66, 0x65, 0x63, 0x5C, 0x5A, 0x59, 0x56,
	0x55, 0x54, 0x53, 0x52, 0x51, 0x4E, 0x4D, 0x4C,
	0x4B, 0x4A, 0x49, 0x47, 0x46, 0x45, 0x43, 0x3C,
	0x3A, 0x39, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31,
	0x2E, 0x2D, 0x2C, 0x2B, 0x2A, 0x29, 0x27, 0x26,
	0x25, 0x23, 0x1F, 0x1E, 0x1D, 0x1B, 0x18, 0x17,
	0x10, 0x0F, 0x08, 0x04, 0x02, 0x01, 0x00
};

/*************************************************************************
*   qla2x00_set_info
*
* Description:
*   Set parameters for the driver from the /proc filesystem.
*
* Returns:
*************************************************************************/
int
qla2x00_set_info(char *buffer, int length, struct Scsi_Host *HBAptr)
{
	return (-ENOSYS);  /* Currently this is a no-op */
}

#include "qla_mbx.c"
#include "qla2x00_ioctl.c"
#if defined(INTAPI)
#include "qla_inioct.c"
#endif


/*
 * The following support functions are adopted to handle
 * the re-entrant qla2x00_proc_info correctly.
 */
STATIC void
copy_mem_info(struct info_str *info, char *data, int len)
{
	if (info->pos + len > info->offset + info->length)
		len = info->offset + info->length - info->pos;

	if (info->pos + len < info->offset) {
		info->pos += len;
		return;
	}
 
	if (info->pos < info->offset) {
		off_t partial;
 
		partial = info->offset - info->pos;
		data += partial;
		info->pos += partial;
		len  -= partial;
	}
 
	if (len > 0) {
		memcpy(info->buffer, data, len);
		info->pos += len;
		info->buffer += len;
	}
}

STATIC int
copy_info(struct info_str *info, char *fmt, ...)
{
	va_list args;
	char buf[256];
	int len;
 
	va_start(args, fmt);
	len = vsprintf(buf, fmt, args);
	va_end(args);
 
	copy_mem_info(info, buf, len);

	return (len);
}

/*************************************************************************
* qla2x00_proc_info
*
* Description:
*   Return information to handle /proc support for the driver.
*
* inout : decides the direction of the dataflow and the meaning of the
*         variables
* buffer: If inout==FALSE data is being written to it else read from it
*         (ptr to a page buffer)
* *start: If inout==FALSE start of the valid data in the buffer
* offset: If inout==FALSE starting offset from the beginning of all
*         possible data to return.
* length: If inout==FALSE max number of bytes to be written into the buffer
*         else number of bytes in "buffer"
* Returns:
*         < 0:  error. errno value.
*         >= 0: sizeof data returned.
*************************************************************************/
int
qla2x00_proc_info(char *buffer, char **start, off_t offset,
	          int length, int hostno, int inout)
{
	struct Scsi_Host *host;
	struct info_str	info;
	int             i;
	int             retval = -EINVAL;
	os_lun_t	*up;
	qla_boards_t    *bdp;
	scsi_qla_host_t *ha;
	uint32_t        t, l;
	uint32_t        tmp_sn;
	unsigned long   *flags;
	struct list_head *list, *temp;
	unsigned long	cpu_flags;
	uint8_t		*loop_state;

#if REQ_TRACE

	Scsi_Cmnd       *cp;
	srb_t           *sp;
#endif

	DEBUG3(printk(KERN_INFO
	    "Entering proc_info buff_in=%p, offset=0x%lx, length=0x%x, "
	    "hostno=%d\n", buffer, offset, length, hostno);)

	host = NULL;

	/* Find the host that was specified */
	for (ha=qla2x00_hostlist; (ha != NULL) && ha->host->host_no != hostno;
	    ha=ha->next) {
		continue;
	}

	/* if host wasn't found then exit */
	if (!ha) {
		DEBUG2_3(printk(KERN_WARNING
		    "%s: Can't find adapter for host number %d\n", 
		    __func__, hostno);)

		return (retval);
	}

	host = ha->host;

	if (inout == TRUE) {
		/* Has data been written to the file? */
		DEBUG3(printk(
		    "%s: has data been written to the file. \n",
		    __func__);)
		return (qla2x00_set_info(buffer, length, host));
	}

	if (start) {
		*start = buffer;
	}

	info.buffer = buffer;
	info.length = length;
	info.offset = offset;
	info.pos    = 0;


	/* start building the print buffer */
	bdp = &QLBoardTbl_fc[ha->devnum];
	copy_info(&info,
	    "QLogic PCI to Fibre Channel Host Adapter for "
#if defined(ISP2100)
	    "ISP2100:\n"
#endif
#if defined(ISP2200)
	    "ISP22xx:\n"
#endif
#if defined(ISP2300)
	    "ISP23xx:\n"
#endif
	    "        Firmware version: %2d.%02d.%02d, "
	    "Driver version %s\n",
	    bdp->fwver[0], bdp->fwver[1], bdp->fwver[2], 
	    qla2x00_version_str);


	copy_info(&info, "Entry address = %p\n",qla2x00_set_info);

	tmp_sn = ((ha->serial0 & 0x1f) << 16) | (ha->serial2 << 8) | 
	    ha->serial1;
	copy_info(&info, "HBA: %s, Serial# %c%05d\n",
	    bdp->bdName, ('A' + tmp_sn/100000), (tmp_sn%100000));

	copy_info(&info,
	    "Request Queue = 0x%lx, Response Queue = 0x%lx\n",
	    (long unsigned int)ha->request_dma,
	    (long unsigned int)ha->response_dma);

	copy_info(&info,
	    "Request Queue count= %ld, Response Queue count= %ld\n",
	    (long)REQUEST_ENTRY_CNT, (long)RESPONSE_ENTRY_CNT);

	copy_info(&info,
	    "Total number of active commands = %ld\n",
	    ha->actthreads);

	copy_info(&info,
	    "Total number of interrupts = %ld\n",
	    (long)ha->total_isr_cnt);

#if defined(FC_IP_SUPPORT)
	copy_info(&info,
	    "Total number of active IP commands = %ld\n",
	    ha->ipreq_cnt);
#endif

#if defined(IOCB_HIT_RATE)
	copy_info(&info,
	    "Total number of IOCBs (used/max/#hit) "
	    "= (%d/%d/%d)\n",
	    (int)ha->iocb_cnt,
	    (int)ha->iocb_hiwat,
	    (int)ha->iocb_overflow_cnt);
#else
	copy_info(&info,
	    "Total number of IOCBs (used/max) "
	    "= (%d/%d)\n",
	    (int)ha->iocb_cnt, (int)ha->iocb_hiwat);
#endif


	copy_info(&info,
	    "Total number of queued commands = %d\n",
	    (max_srbs - ha->srb_cnt));

	copy_info(&info,
	    "    Device queue depth = 0x%x\n",
	    (ql2xmaxqdepth == 0) ? 32 : ql2xmaxqdepth);

	copy_info(&info,
	    "Number of free request entries = %d\n", ha->req_q_cnt);

	copy_info(&info,
	    "Number of mailbox timeouts = %ld\n",
	    qla2x00_stats.mboxtout);

	copy_info(&info,
	    "Number of ISP aborts = %ld\n",qla2x00_stats.ispAbort);

	copy_info(&info,
	    "Number of loop resyncs = %ld\n",
	    qla2x00_stats.loop_resync);

	copy_info(&info,
	    "Number of retries for empty slots = %ld\n",
	    qla2x00_stats.outarray_full);

	copy_info(&info,
	    "Number of reqs in pending_q= %ld, retry_q= %d, "
	    "done_q= %ld, scsi_retry_q= %d\n",
	    ha->qthreads, ha->retry_q_cnt,
	    ha->done_q_cnt, ha->scsi_retry_q_cnt);

#ifdef PERF_MONITORING
	copy_info(&info,
	    "Number of highmem_io = %ld\n",
	    qla2x00_stats.highmem_io);
#endif

	if (ha->flags.failover_enabled) {
		copy_info(&info,
		    "Number of reqs in failover_q= %d\n",
		    ha->failover_cnt);
	}

	flags = (unsigned long *) &ha->flags;

	if (ha->loop_state == LOOP_DOWN) {
		loop_state = "DOWN";
	} else if (ha->loop_state ==LOOP_UP) {
		loop_state = "UP";
	} else if (ha->loop_state ==LOOP_READY) {
		loop_state = "READY";
	} else if (ha->loop_state ==LOOP_TIMEOUT) {
		loop_state = "TIMEOUT";
	} else if (ha->loop_state ==LOOP_UPDATE) {
		loop_state = "UPDATE";
	} else {
		loop_state = "UNKNOWN";
	}

	copy_info(&info, 
	    "Host adapter:loop state= <%s>, flags= 0x%lx\n",
	    loop_state , *flags);

	copy_info(&info, "Dpc flags = 0x%lx\n", ha->dpc_flags);

	copy_info(&info, "MBX flags = 0x%x\n", ha->mbx_flags);

	copy_info(&info, "SRB Free Count = %d\n", ha->srb_cnt);

	copy_info(&info, "Port down retry = %3.3d\n",
	    ha->port_down_retry_count);

	copy_info(&info, "Login retry count = %3.3d\n",
	    ha->login_retry_count);

	copy_info(&info,
	    "Commands retried with dropped frame(s) = %d\n",
	    ha->dropped_frame_error_cnt);

#if defined(ISP2300)
	copy_info(&info, "Firmware memory parity checking: %s\n",
#if defined(CONFIG_SCSI_QLOGIC_23XX_PARITY)
			"enabled"
#else
			"disabled"
#endif
	      );

	{
		struct qla2x00_special_options special_options;

		*((uint16_t *) &special_options) = le16_to_cpu(
			*((uint16_t *) &ha->init_cb->special_options));

		copy_info(&info,
			"Configured characteristic impedence: %d ohms\n",
			special_options.enable_50_ohm_termination ? 50 : 75);

		switch ( special_options.data_rate ) {
		case 0:
			loop_state = "1 Gb/sec";
			break;

		case 1:
			loop_state = "2 Gb/sec";
			break;

		case 2:
			loop_state = "1-2 Gb/sec auto-negotiate";
			break;

		default:
			loop_state = "unknown";
			break;
		}

		copy_info(&info, "Configured data rate: %s\n", loop_state);
	}
#endif

	copy_info(&info, "\n");

#if REQ_TRACE
	if (qla2x00_req_dmp) {
		copy_info(&info,
		    "Outstanding Commands on controller:\n");

		for (i = 1; i < MAX_OUTSTANDING_COMMANDS; i++) {
			if ((sp = ha->outstanding_cmds[i]) == NULL) {
				continue;
			}

			if ((cp = sp->cmd) == NULL) {
				continue;
			}

			copy_info(&info, "(%d): Pid=%d, sp flags=0x%lx"
			    ", cmd=0x%p, state=%d\n", 
			    i, 
			    (int)sp->cmd->serial_number, 
			    (long)sp->flags,
			    CMD_SP(sp->cmd),
			    (int)sp->state);

			if (info.pos >= info.offset + info.length) {
				/* No need to continue */
				goto profile_stop;
			}
		}
	}
#endif /* REQ_TRACE */

	if (qla2x00_retryq_dmp) {
		if (!list_empty(&ha->retry_queue)) {
			copy_info(&info,
			    "qla%ld: Retry queue requests:\n",
			    ha->host_no);

			spin_lock_irqsave(&ha->list_lock, cpu_flags);

			i = 0;
			list_for_each_safe(list, temp, &ha->retry_queue) {
				sp = list_entry(list, srb_t, list);
				t = SCSI_TCN_32(sp->cmd);
				l = SCSI_LUN_32(sp->cmd);

				copy_info(&info,
				    "%d: target=%d, lun=%d, "
				    "pid=%ld sp=%p, sp->flags=0x%x,"
				    "sp->state= %d\n", 
				    i, t, l, 
				    sp->cmd->serial_number, sp, 
				    sp->flags, sp->state );

				i++;

				if (info.pos >= info.offset + info.length) {
					/* No need to continue */
					goto profile_stop;
				}
			}

			spin_unlock_irqrestore(&ha->list_lock, cpu_flags);

		} /* if (!list_empty(&ha->retry_queue))*/
	} /* if ( qla2x00_retryq_dmp )  */

	/* 2.25 node/port display to proc */
	/* Display the node name for adapter */
	copy_info(&info, "\nSCSI Device Information:\n");
	copy_info(&info,
	    "scsi-qla%d-adapter-node="
	    "%02x%02x%02x%02x%02x%02x%02x%02x;\n",
	    (int)ha->instance,
	    ha->init_cb->node_name[0],
	    ha->init_cb->node_name[1],
	    ha->init_cb->node_name[2],
	    ha->init_cb->node_name[3],
	    ha->init_cb->node_name[4],
	    ha->init_cb->node_name[5],
	    ha->init_cb->node_name[6],
	    ha->init_cb->node_name[7]);

	/* display the port name for adapter */
	copy_info(&info,
	    "scsi-qla%d-adapter-port="
	    "%02x%02x%02x%02x%02x%02x%02x%02x;\n",
	    (int)ha->instance,
	    ha->init_cb->port_name[0],
	    ha->init_cb->port_name[1],
	    ha->init_cb->port_name[2],
	    ha->init_cb->port_name[3],
	    ha->init_cb->port_name[4],
	    ha->init_cb->port_name[5],
	    ha->init_cb->port_name[6],
	    ha->init_cb->port_name[7]);

	/* Print out device port names */
	for (i = 0; i < MAX_FIBRE_DEVICES; i++) {
		if (ha->fc_db[i].loop_id == PORT_UNUSED) {
			continue;
		}

		if (ha->flags.failover_enabled) {
			copy_info(&info,
			    "scsi-qla%d-port-%d="
			    "%02x%02x%02x%02x%02x%02x%02x%02x:"
			    "%02x%02x%02x%02x%02x%02x%02x%02x;\n",
			    (int)ha->instance, i,
			    ha->fc_db[i].name[0],
			    ha->fc_db[i].name[1],
			    ha->fc_db[i].name[2],
			    ha->fc_db[i].name[3],
			    ha->fc_db[i].name[4],
			    ha->fc_db[i].name[5],
			    ha->fc_db[i].name[6],
			    ha->fc_db[i].name[7],
			    ha->fc_db[i].wwn[0],
			    ha->fc_db[i].wwn[1],
			    ha->fc_db[i].wwn[2],
			    ha->fc_db[i].wwn[3],
			    ha->fc_db[i].wwn[4],
			    ha->fc_db[i].wwn[5],
			    ha->fc_db[i].wwn[6],
			    ha->fc_db[i].wwn[7]);
		} else {
			copy_info(&info,
			    "scsi-qla%d-target-%d="
			    "%02x%02x%02x%02x%02x%02x%02x%02x;\n",
			    (int)ha->instance, i,
			    ha->fc_db[i].wwn[0],
			    ha->fc_db[i].wwn[1],
			    ha->fc_db[i].wwn[2],
			    ha->fc_db[i].wwn[3],
			    ha->fc_db[i].wwn[4],
			    ha->fc_db[i].wwn[5],
			    ha->fc_db[i].wwn[6],
			    ha->fc_db[i].wwn[7]);
		}

	} /* 2.25 node/port display to proc */

	copy_info(&info, "\nSCSI LUN Information:\n");

	copy_info(&info, "(Id:Lun)\n");

	/* scan for all equipment stats */
	for (t = 0; t < MAX_FIBRE_DEVICES; t++) {
		/* scan all luns */
		for (l = 0; l < ha->max_luns; l++) {
			up = (os_lun_t *) GET_LU_Q(ha, t, l);

			if (up == NULL) {
				continue;
			}
			if (up->fclun == NULL) {
				continue;
			}
			if (up->fclun->flags & FC_DISCON_LUN) {
				continue;
			}

			copy_info(&info,
			    "(%2d:%2d): Total reqs %ld,",
			    t,l,up->io_cnt);

			copy_info(&info,
			    " Pending reqs %ld,",
			    up->out_cnt);

			if (up->io_cnt < 3) {
				copy_info(&info,
				    " flags 0x%x*,",
				    (int)up->q_flag);
			} else {
				copy_info(&info,
				    " flags 0x%x,",
				    (int)up->q_flag);
			}

#ifdef PERF_MONITORING
			copy_info(&info, 
			  " %lx:%lx (act,resp),",
			  up->act_time/up->io_cnt,
			  up->resp_time/up->io_cnt);
#endif

			copy_info(&info, 
			    " %ld:%d:%02x,",
			    up->fclun->fcport->ha->instance,
			    up->fclun->fcport->cur_path,
			    up->fclun->fcport->loop_id);

			copy_info(&info, "\n");

			if (info.pos >= info.offset + info.length) {
				/* No need to continue */
				goto profile_stop;
			}
		}

		if (info.pos >= info.offset + info.length) {
			/* No need to continue */
			break;
		}
	}

profile_stop:

	retval = info.pos > info.offset ? info.pos - info.offset : 0;

	DEBUG3(printk(KERN_INFO 
	    "Exiting proc_info: info.pos=%d, offset=0x%lx, "
	    "length=0x%x\n", info.pos, offset, length);)

#if QLA2100_LIPTEST
	qla2x00_lip = 1;
#endif

	return (retval);

}
 
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,3)
inline int pci_set_dma_mask(struct pci_dev *dev, u64 mask);

inline int
pci_set_dma_mask(struct pci_dev *dev, u64 mask)
{
	if (!pci_dma_supported(dev, mask))
		return -EIO;

	dev->dma_mask = mask;

	return 0;
}	 
#endif


/**************************************************************************
* sp_put
*
* Description:
*   Decrement reference count and call the callback if we're the last
*   owner of the specified sp. Will get io_request_lock before calling
*   the callback.
*
* Input:
*   ha - pointer to the scsi_qla_host_t where the callback is to occur.
*   sp - pointer to srb_t structure to use.
*
* Returns:
*
**************************************************************************/
static inline void
sp_put(struct scsi_qla_host * ha, srb_t *sp)
{
        unsigned long flags;

        if (atomic_read(&sp->ref_count) == 0) {
		printk(KERN_INFO
			"%s(): **** SP->ref_count not zero\n",
			__func__);
                DEBUG2(BUG();)

                return;
	}

        if (!atomic_dec_and_test(&sp->ref_count))
        {
                return;
        }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
        spin_lock_irqsave(&io_request_lock, flags);
#else
        spin_lock_irqsave(ha->host->host_lock, flags);
#endif

        qla2x00_callback(ha, sp->cmd);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
        spin_unlock_irqrestore(&io_request_lock, flags);
#else
        spin_unlock_irqrestore(ha->host->host_lock, flags);
#endif
}

/**************************************************************************
* sp_get
*
* Description:
*   Increment reference count of the specified sp.
*
* Input:
*   sp - pointer to srb_t structure to use.
*
* Returns:
*
**************************************************************************/
static inline void
sp_get(struct scsi_qla_host * ha, srb_t *sp)
{
        atomic_inc(&sp->ref_count);

        if (atomic_read(&sp->ref_count) > 2) {
		printk(KERN_INFO
			"%s(): **** SP->ref_count greater than two\n",
			__func__);
                DEBUG2(BUG();)

		return;
	}
}

/**************************************************************************
* __sp_put
*
* Description:
*   Decrement reference count and call the callback if we're the last
*   owner of the specified sp. Will NOT get io_request_lock before calling
*   the callback.
*
* Input:
*   ha - pointer to the scsi_qla_host_t where the callback is to occur.
*   sp - pointer to srb_t structure to use.
*
* Returns:
*
**************************************************************************/
static inline void
__sp_put(struct scsi_qla_host * ha, srb_t *sp)
{
        if (atomic_read(&sp->ref_count) == 0) {
		printk(KERN_INFO
			"%s(): **** SP->ref_count not zero\n",
			__func__);
                DEBUG2(BUG();)

		return;
	}

        if (!atomic_dec_and_test(&sp->ref_count))
        {
                return;
        }

        qla2x00_callback(ha, sp->cmd);
}

/**************************************************************************
*   qla2x00_cmd_timeout
*
* Description:
*       Handles the command if it times out in any state.
*
* Input:
*     sp - pointer to validate
*
* Returns:
* None.
* Note:Need to add the support for if( sp->state == SRB_FAILOVER_STATE).
**************************************************************************/
void
qla2x00_cmd_timeout(srb_t *sp)
{
	int t, l;
	int processed;
	scsi_qla_host_t *vis_ha, *dest_ha;
	Scsi_Cmnd *cmd;
	ulong      flags;
#if defined(QL_DEBUG_LEVEL_3)
	ulong      cpu_flags;
#endif
	fc_port_t	*fcport;

	cmd = sp->cmd;
	vis_ha = (scsi_qla_host_t *) cmd->host->hostdata;

	DEBUG3(printk("cmd_timeout: Entering sp->state = %x\n", sp->state);)

	t = SCSI_TCN_32(cmd);
	l = SCSI_LUN_32(cmd);
	fcport = sp->fclun->fcport;
	dest_ha = sp->ha;

	/*
	 * If IO is found either in retry Queue 
	 *    OR in Lun Queue
	 * Return this IO back to host
	 */
	spin_lock_irqsave(&vis_ha->list_lock, flags);
	processed = 0;
	if (sp->state == SRB_PENDING_STATE) {
		__del_from_pending_queue(vis_ha, sp);
		DEBUG2(printk("qla2100%ld: Found in Pending queue "
				"pid %ld, State = %x., "
			 	 "fcport state=%d jiffies=%lx\n",
				vis_ha->host_no,
				sp->cmd->serial_number, sp->state,
				atomic_read(&fcport->state),
				jiffies);)

		/*
		 * If FC_DEVICE is marked as dead return the cmd with
		 * DID_NO_CONNECT status.  Otherwise set the host_byte to
		 * DID_BUS_BUSY to let the OS  retry this cmd.
		 */
		if (atomic_read(&fcport->state) == FC_DEVICE_DEAD) {
			cmd->result = DID_NO_CONNECT << 16;
		} else {
			cmd->result = DID_BUS_BUSY << 16;
		}
		__add_to_done_queue(vis_ha, sp);
		processed++;
	} 
	spin_unlock_irqrestore(&vis_ha->list_lock, flags);
	if (processed) {
#if QLA2X_PERFORMANCE
		 tasklet_schedule(&vis_ha->run_qla_task);
#else
		 if (vis_ha->dpc_wait && !vis_ha->dpc_active) 
		 	 up(vis_ha->dpc_wait);
#endif
		 return;
	}

	spin_lock_irqsave(&dest_ha->list_lock, flags);
	if ((sp->state == SRB_RETRY_STATE)  ||
		 (sp->state == SRB_SCSI_RETRY_STATE)  ||
		 (sp->state == SRB_FAILOVER_STATE)) {

		DEBUG2(printk("qla2100%ld: Found in (Scsi) Retry queue or "
				"failover Q pid %ld, State = %x., "
				"fcport state=%d jiffies=%lx retried=%d\n",
				dest_ha->host_no,
				sp->cmd->serial_number, sp->state,
				atomic_read(&fcport->state),
				jiffies, sp->cmd->retries);)

		if ((sp->state == SRB_RETRY_STATE)) {
			__del_from_retry_queue(dest_ha, sp);
		} else if ((sp->state == SRB_SCSI_RETRY_STATE)) {
			__del_from_scsi_retry_queue(dest_ha, sp);
		} else if ((sp->state == SRB_FAILOVER_STATE)) {
			__del_from_failover_queue(dest_ha, sp);
		}

		/*
		 * If FC_DEVICE is marked as dead return the cmd with
		 * DID_NO_CONNECT status.  Otherwise set the host_byte to
		 * DID_BUS_BUSY to let the OS  retry this cmd.
		 */
		if (dest_ha->flags.failover_enabled) {
			cmd->result = DID_BUS_BUSY << 16;
		} else {
			if (atomic_read(&fcport->state) == FC_DEVICE_DEAD) {
				cmd->result = DID_NO_CONNECT << 16;
				qla2x00_extend_timeout(cmd, EXTEND_CMD_TIMEOUT);
			} else {
				cmd->result = DID_BUS_BUSY << 16;
			}
		}

		__add_to_done_queue(dest_ha, sp);
		processed++;
	} 
	spin_unlock_irqrestore(&dest_ha->list_lock, flags);
	if (processed) {
#if QLA2X_PERFORMANCE
		 tasklet_schedule(&dest_ha->run_qla_task);
#else
		 if (dest_ha->dpc_wait && !dest_ha->dpc_active) 
		 	 up(dest_ha->dpc_wait);
#endif
		 return;
	}

#if defined(QL_DEBUG_LEVEL_3)
	spin_lock_irqsave(&dest_ha->list_lock, cpu_flags);
	if (sp->state == SRB_DONE_STATE) {
		/* IO in done_q  -- leave it */
		DEBUG(printk("qla2100%ld: Found in Done queue pid %ld sp=%p.\n",
				dest_ha->host_no, sp->cmd->serial_number, sp);)
	} else if (sp->state == SRB_SUSPENDED_STATE) {
		DEBUG(printk("qla2100%ld: Found SP %p in suspended state  "
				"- pid %d:\n",
				dest_ha->host_no,sp,
				(int)sp->cmd->serial_number);)
		DEBUG(qla2x00_dump_buffer((uint8_t *)sp, sizeof(srb_t));)
	} else if (sp->state == SRB_ACTIVE_STATE) {
		/*
		 * IO is with ISP find the command in our active list.
		 */
		spin_unlock_irqrestore(&dest_ha->list_lock, cpu_flags); /* 01/03 */
		spin_lock_irqsave(&dest_ha->hardware_lock, flags);
		if (sp == dest_ha->outstanding_cmds
				[(u_long)CMD_HANDLE(sp->cmd)]) {

			DEBUG(printk("cmd_timeout: Found in ISP \n");)

			sp->state = SRB_ACTIVE_TIMEOUT_STATE;
			spin_unlock_irqrestore(&dest_ha->hardware_lock, flags);
		} else {
			spin_unlock_irqrestore(&dest_ha->hardware_lock, flags);
			printk(KERN_INFO 
				"qla_cmd_timeout: State indicates it is with "
				"ISP, But not in active array\n");
		}
		spin_lock_irqsave(&dest_ha->list_lock, cpu_flags); 	/* 01/03 */
	} else if (sp->state == SRB_ACTIVE_TIMEOUT_STATE) {
		DEBUG(printk("qla2100%ld: Found in Active timeout state"
				"pid %ld, State = %x., \n",
				dest_ha->host_no,
				sp->cmd->serial_number, sp->state);)
	} else {
		/* EMPTY */
		DEBUG2(printk("cmd_timeout%ld: LOST command state = "
				"0x%x, sp=%p\n",
				vis_ha->host_no, sp->state,sp);)

		printk(KERN_INFO
			"cmd_timeout: LOST command state = 0x%x\n", sp->state);
	}
	spin_unlock_irqrestore(&dest_ha->list_lock, cpu_flags);
#endif
	
	DEBUG3(printk("cmd_timeout: Leaving\n");)
}


/**************************************************************************
*   qla2x00_add_timer_to_cmd
*
* Description:
*       Creates a timer for the specified command. The timeout is usually
*       the command time from kernel minus 2 secs.
*
* Input:
*     sp - pointer to validate
*
* Returns:
*     None.
**************************************************************************/
static inline void
qla2x00_add_timer_to_cmd(srb_t *sp, int timeout)
{
	init_timer(&sp->timer);
	sp->timer.expires = jiffies + timeout * HZ;
	sp->timer.data = (unsigned long) sp;
	sp->timer.function = (void (*) (unsigned long))qla2x00_cmd_timeout;
#ifndef __VMWARE__
	add_timer(&sp->timer);
#else
        if (timeout) {
           add_timer(&sp->timer);
        }
        else {
           sp->timer.function = NULL;
        }
#endif
}

/**************************************************************************
*   qla2x00_delete_timer_from_cmd
*
* Description:
*       Delete the timer for the specified command.
*
* Input:
*     sp - pointer to validate
*
* Returns:
*     None.
**************************************************************************/
static inline void 
qla2x00_delete_timer_from_cmd(srb_t *sp )
{
	if (sp->timer.function != NULL) {
		del_timer(&sp->timer);
		sp->timer.function =  NULL;
		sp->timer.data = (unsigned long) NULL;
	}
}

#if defined(QLA2XXX_CONFIG_BOOTIMG)
/*
 * Called at shutdown time to shut down DMA on the device.  This
 * should eventually be converted to a pci_driver shutdown call (or
 * whatever the driver model comes up with).
 */

STATIC void __qla2x00_reset_chip(scsi_qla_host_t *ha);

static int
qla2x00_shutdown(struct notifier_block *self, unsigned long a, void *b)
{
	scsi_qla_host_t *ha = (scsi_qla_host_t *) self;

	if (qla2x00_verbose) {
		printk("scsi(%ld): Shutting down the device\n", ha->host_no);
	}

	__qla2x00_reset_chip(ha);

	return (NOTIFY_OK);
}
#endif

/**************************************************************************
* qla2x00_detect
*
* Description:
*    This routine will probe for Qlogic FC SCSI host adapters.
*    It returns the number of host adapters of a particular
*    type that were found.	 It also initialize all data necessary for
*    the driver.  It is passed-in the host number, so that it
*    knows where its first entry is in the scsi_hosts[] array.
*
* Input:
*     template - pointer to SCSI template
*
* Returns:
*  num - number of host adapters found.
**************************************************************************/
int
qla2x00_detect(Scsi_Host_Template *template)
{
	device_reg_t	*reg;
	int		i;
	uint16_t        subsystem_vendor, subsystem_device;
	struct Scsi_Host *host;
	scsi_qla_host_t *ha = NULL, *cur_ha;
	struct _qlaboards  *bdp;
	unsigned long		flags = 0;
	unsigned long		wait_switch = 0;
	struct pci_dev *pdev = NULL;

	ENTER("qla2x00_detect");

#ifdef __VMWARE__
        if (vmk_check_version(VMKDRIVER_VERSION) != VMK_VERSION_OK) {
           return 0;
        }
        /* In the vmkernel, we do not hold the io_request lock during init,
         * so leave it unlocked and do not lock it before returning. */
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	spin_unlock_irq(&io_request_lock);
#endif
#endif //__VMWARE__

#if defined(MODULE)
	DEBUG2(printk("DEBUG: qla2x00_set_info starts at address = %p\n",
			qla2x00_set_info);)
	printk(KERN_INFO
		"qla2x00_set_info starts at address = %p\n", qla2x00_set_info);

	/*
	 * If we are called as a module, the qla2100 pointer may not be null
	 * and it would point to our bootup string, just like on the lilo
	 * command line.  IF not NULL, then process this config string with
	 * qla2x00_setup
	 *
	 * Boot time Options To add options at boot time add a line to your
	 * lilo.conf file like:
	 * append="qla2100=verbose,tag_info:{{32,32,32,32},{32,32,32,32}}"
	 * which will result in the first four devices on the first two
	 * controllers being set to a tagged queue depth of 32.
	 */
	if (ql2xopts)
		qla2x00_setup(ql2xopts);
	if (dummy_buffer[0] != 'P')
		printk(KERN_WARNING
			"qla2x00: Please read the file "
			"/usr/src/linux/drivers/scsi/README.qla2x00\n"
			"qla2x00: to see the proper way to specify options to "
			"the qla2x00 module\n"
			"qla2x00: Specifically, don't use any commas when "
			"passing arguments to\n"
			"qla2x00: insmod or else it might trash certain memory "
			"areas.\n");
#endif

	if (!pci_present()) {
		printk("scsi: PCI not present\n");
#ifndef __VMWARE__
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#endif
#endif //__VMWARE__
		return 0;
	} /* end of !pci_present() */

	bdp = &QLBoardTbl_fc[0];
	qla2x00_hostlist = NULL;
	template->proc_name = DRIVER_NAME;
#if defined(SCSI_HOST_VARYIO)
	SCSI_HOST_VARYIO(template) = 1;
#endif
	if (ql2xfailover) {
		sprintf(qla2x00_version_str, "%s-fo", QLA2100_VERSION);
	} else {
		sprintf(qla2x00_version_str, "%s", QLA2100_VERSION);
	}


	/* Try and find each different type of adapter we support */
	for (i = 0; bdp->device_id != 0 && i < NUM_OF_ISP_DEVICES;
		i++, bdp++) {

		/* PCI_SUBSYSTEM_IDS supported */
		while ((pdev = pci_find_subsys(QLA2X00_VENDOR_ID,
						bdp->device_id,
						PCI_ANY_ID, PCI_ANY_ID, 
						pdev))) {

			if (pci_enable_device(pdev))
				continue;

			/* found a adapter */
			printk(KERN_INFO
				"qla2x00: Found  VID=%x DID=%x "
				"SSVID=%x SSDID=%x\n",
				pdev->vendor, 
				pdev->device,
				pdev->subsystem_vendor, 
				pdev->subsystem_device);

			subsystem_vendor = pdev->subsystem_vendor;
			subsystem_device = pdev->subsystem_device;

			/* If it's an XXX SubSys Vendor ID adapter, skip it. */
			/*
			   if (pdev->subsystem_vendor == PCI_VENDOR_ID_XXX) {
			   printk(KERN_WARNING
			   "qla2x00: Skip XXX SubSys Vendor ID "
			   "Controller\n");
			   continue;
			   }
			 */

#ifdef __VMWARE__
         /* We do not need to hold any lock when calling the
          * functions below in the vmkernel. */
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
			spin_lock_irq(&io_request_lock);
#endif
#endif //__VMWARE__

#if defined(ISP2100)
			template->name = "QLogic Fibre Channel 2100";
#endif
#if defined(ISP2200)
			template->name = "QLogic Fibre Channel 2200";
#endif
#if defined(ISP2300)
			template->name = "QLogic Fibre Channel 2300";
#endif
			if ((host = 
				scsi_register(
					template,
					sizeof(scsi_qla_host_t))) == NULL) {

				printk(KERN_WARNING
					"qla2x00: couldn't register "
					"with scsi layer\n");
				return 0;
			}

			ha = (scsi_qla_host_t *)host->hostdata;

#if defined(CONFIG_VMNIX) && !defined(__VMWARE__)
			host->bus = pdev->bus->number;
			host->function = pdev->devfn;
			host->devid = ha; 
#endif
			/* Clear our data area */
			memset(ha, 0, sizeof(scsi_qla_host_t));
#ifdef __VMWARE__
			scsi_register_uinfo(host, pdev->bus->number, pdev->devfn, ha);

			/* Now get and save the adapter pointer... */
			ha->vmk_adapter = host->adapter;
			if (ha->vmk_adapter == NULL) {
			  panic("qla : vmkernel adapter structure is NULL\n");
			}
#endif

#if defined(QLA2XXX_CONFIG_BOOTIMG)
			/* Set the bootimg notifier. */
			ha->bootimg_notifier.notifier_call = qla2x00_shutdown;
			ha->bootimg_notifier.priority = 0;
			notifier_chain_register(&bootimg_notifiers,
						&(ha->bootimg_notifier));
#endif

			ha->host_no = host->host_no;
			ha->host = host;

			/* Sanitize the information from PCI BIOS. */
			host->irq = pdev->irq;
			host->io_port = pci_resource_start(pdev, 0);
			ha->subsystem_vendor = subsystem_vendor;
			ha->subsystem_device = subsystem_device;
			ha->pdev = pdev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,4)
			scsi_set_pci_device(host, pdev);
#endif

			ha->device_id = bdp->device_id;
			ha->devnum = i;
			if (qla2x00_verbose) {
				printk(KERN_INFO
					"scsi(%d): Found a %s @ bus %d, "
					"device 0x%x, irq %d, iobase 0x%lx\n",
					host->host_no,
					bdp->bdName, 
					ha->pdev->bus->number,
					PCI_SLOT(ha->pdev->devfn),
					host->irq, 
					(unsigned long)host->io_port);
			}

			ha->iobase = (device_reg_t *) host->io_port;
			spin_lock_init(&ha->hardware_lock);

			/* 4.23 Initialize /proc/scsi/qla2x00 counters */
			ha->actthreads = 0;
			ha->qthreads   = 0;
			ha->dump_done  = 0;
			ha->total_isr_cnt = 0;
			ha->total_isp_aborts = 0;
			ha->total_lip_cnt = 0;
			ha->total_dev_errs = 0;
			ha->total_ios = 0;
			ha->total_bytes = 0;

#ifdef __VMWARE__
			/* It seems plain wrong that the driver would call
			 * qla2x00_mem_alloc with the io_request_lock held,
			 * since that function will call schedule_timeout
			 * if any allocation fails...
			 */
#endif
			if (qla2x00_mem_alloc(ha)) {
				printk(KERN_WARNING
					"scsi(%d): [ERROR] Failed to allocate "
					"memory for adapter\n",
					host->host_no);
				qla2x00_mem_free(ha);
#ifndef __VMWARE__
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
				spin_unlock_irq(&io_request_lock);
#endif
#endif //__VMWARE__
				continue;
			}

			ha->prev_topology = 0;
			ha->ports = bdp->numPorts;

#if defined(ISP2100)
			ha->max_targets = MAX_TARGETS_2100;
#else
			ha->max_targets = MAX_TARGETS_2200;
#endif

			/* load the F/W, read paramaters, and init the H/W */
			ha->instance = num_hosts;

			init_MUTEX_LOCKED(&ha->mbx_intr_sem);

			INIT_LIST_HEAD(&ha->fcinitiators);

			INIT_LIST_HEAD(&ha->done_queue);
			INIT_LIST_HEAD(&ha->retry_queue);
			INIT_LIST_HEAD(&ha->scsi_retry_queue);
			INIT_LIST_HEAD(&ha->failover_queue);

			INIT_LIST_HEAD(&ha->pending_queue);

			if (ql2xfailover)
				ha->flags.failover_enabled = 1;
			else
				ha->flags.failover_enabled = 0;

#if QLA2X_PERFORMANCE
			tasklet_init(&ha->run_qla_task,
					(void *)qla2x00_done_tasklet,
					(unsigned long) ha);
#endif

			/*
			 * These locks are used to prevent more than one CPU
			 * from modifying the queue at the same time. The
			 * higher level "io_request_lock" will reduce most
			 * contention for these locks.
			 */

			spin_lock_init(&ha->mbx_bits_lock);
			spin_lock_init(&ha->mbx_reg_lock);
			spin_lock_init(&ha->mbx_q_lock);
			spin_lock_init(&ha->list_lock);

#ifndef __VMWARE__
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
			spin_unlock_irq(&io_request_lock);
#endif
#endif //__VMWARE__

			if (qla2x00_initialize_adapter(ha) &&
				!(ha->device_flags & DFLG_NO_CABLE)) {

				printk(KERN_WARNING
					"qla2x00: Failed to "
					"initialize adapter\n");

				DEBUG2(printk("scsi%ld: Failed to initialize "
						"adapter - Adapter flags %x.\n",
						ha->host_no, ha->device_flags);)

				qla2x00_mem_free(ha);
				scsi_unregister(host);

				continue;
			}

			/*
			 * Startup the kernel thread for this host adapter
			 */
#ifdef __VMWARE__
			/*
			 * Initialize the extensions defined in ha to
			 * communicate with the DPC kernel thread.
			 */
			ha->should_die = FALSE;
                        
			ha->notify_sema = (struct semaphore)__SEMAPHORE_INITIALIZER(ha->notify_sema, 0);
			ha->dpc_notify = &ha->notify_sema;
#else
#if defined(ISP2100)
			ha->dpc_notify = &qla2100_detect_sem;
#endif
#if defined(ISP2200)
			ha->dpc_notify = &qla2200_detect_sem;
#endif
#if defined(ISP2300)
			ha->dpc_notify = &qla2300_detect_sem;
#endif
#endif //__VMWARE__

			kernel_thread((int (*)(void *))qla2x00_do_dpc,
					(void *) ha, 0);

			/*
			 * Now wait for the kernel dpc thread to initialize
			 * and go to sleep.
			 */
#ifdef __VMWARE__
			printk("qla: waiting for kernel_thread\n");
			down(ha->dpc_notify);
			printk("qla: kernel_thread back\n");
#else
#if defined(ISP2100)
			down(&qla2100_detect_sem);
#endif
#if defined(ISP2200)
			down(&qla2200_detect_sem);
#endif
#if defined(ISP2300)
			down(&qla2300_detect_sem);
#endif
#endif //__VMWARE__

			ha->dpc_notify = NULL;
			ha->next = NULL;
			/*  Mark preallocated Loop IDs in use. */
			ha->fabricid[SNS_FL_PORT].in_use = TRUE;
			ha->fabricid[FABRIC_CONTROLLER].in_use = TRUE;
			ha->fabricid[SIMPLE_NAME_SERVER].in_use = TRUE;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
			spin_lock_irq(&io_request_lock);
#endif

			/* Register our resources with Linux */
			if (qla2x00_register_with_Linux(ha, bdp->numPorts-1)) {
				printk(KERN_WARNING
					"scsi%ld: Failed to "
					"register resources.\n",
					ha->host_no);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
				spin_unlock_irq(&io_request_lock);
#endif

				qla2x00_mem_free(ha);

#ifdef __VMWARE__
				 /* No need to grab the lock just to call
				  * scsi_unregister in the vmkernel.
				  */
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
				spin_lock_irq(&io_request_lock);
#endif
#endif //__VMWARE__

				scsi_unregister(host);

#ifndef __VMWARE__
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
				spin_unlock_irq(&io_request_lock);
#endif
#endif //__VMWARE__
				continue;
			}

			DEBUG2(printk("DEBUG: detect hba %ld at "
					"address = %p\n",
					ha->host_no, ha);)

			reg = ha->iobase;

			/* Disable ISP interrupts. */
			qla2x00_disable_intrs(ha);

			/* Ensure mailbox registers are free. */
			spin_lock_irqsave(&ha->hardware_lock, flags);
			WRT_REG_WORD(&reg->semaphore, 0);
			WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
			WRT_REG_WORD(&reg->host_cmd, HC_CLR_HOST_INT);

			/* Enable proper parity */
#if defined(ISP2300) && defined(CONFIG_SCSI_QLOGIC_23XX_PARITY)
			if (ha->device_id == QLA2312_DEVICE_ID)
				/* SRAM, Instruction RAM and GP RAM parity */
				WRT_REG_WORD(&reg->host_cmd,
				    (HC_ENABLE_PARITY + 0x7));
			else
				/* SRAM parity */
				WRT_REG_WORD(&reg->host_cmd,
				    (HC_ENABLE_PARITY + 0x1));
#endif

			spin_unlock_irqrestore(&ha->hardware_lock, flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
			spin_unlock_irq(&io_request_lock);
#endif

			/*
			 * if failover is enabled read the user configuration
			 */
			if (ha->flags.failover_enabled) {
				if (ConfigRequired > 0)
					mp_config_required = 1;
				else
					mp_config_required = 0;

				DEBUG(printk("qla2x00_detect: qla2x00_cfg_init "
						"for hba %ld\n",
						ha->instance);)

				qla2x00_cfg_init(ha);
			}

			/* Enable chip interrupts. */
			qla2x00_enable_intrs(ha);

			/* Insert new entry into the list of adapters */
			ha->next = NULL;

			if( qla2x00_hostlist == NULL ) {
				qla2x00_hostlist = ha;
			} else {
				cur_ha = qla2x00_hostlist;

				while( cur_ha->next != NULL )
					cur_ha = cur_ha->next;

				cur_ha->next = ha;
			}

			/* v2.19.5b6 */
			/*
			 * Wait around max loop_reset_delay secs for the
			 * devices to come on-line. We don't want Linux
			 * scanning before we are ready.
			 */
			for (wait_switch = jiffies + 
				(ha->loop_reset_delay * HZ);
				/* jiffies < wait_switch */
				time_before(jiffies,wait_switch)  &&
				!(ha->device_flags &
					(DFLG_NO_CABLE | DFLG_FABRIC_DEVICES))
				&& (ha->device_flags & SWITCH_FOUND) ;) {

				qla2x00_check_fabric_devices(ha);

				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(5);
			}

			/* List the target we have found */
			if (displayConfig && (!ha->flags.failover_enabled))
				qla2x00_display_fc_names(ha);

			ha->init_done = 1;
			num_hosts++;
		}
	} /* end of FOR */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	spin_lock_irq(&io_request_lock);
#endif

 	if (displayConfig && ha->flags.failover_enabled)
		qla2x00_cfg_display_devices();
#ifdef __VMWARE__
        /* We do not hold the io_request lock when calling init and we
         * should not hold it when returning.
         */
        spin_unlock_irq(&io_request_lock);
#endif
	LEAVE("qla2x00_detect");

	return num_hosts;
}

/**************************************************************************
*   qla2x00_register_with_Linux
*
* Description:
*   Free the passed in Scsi_Host memory structures prior to unloading the
*   module.
*
* Input:
*     ha - pointer to host adapter structure
*     maxchannels - MAX number of channels.
*
* Returns:
*  0 - Sucessfully reserved resources.
*  1 - Failed to reserved a resource.
**************************************************************************/
STATIC uint8_t
qla2x00_register_with_Linux(scsi_qla_host_t *ha, uint8_t maxchannels)
{
	struct Scsi_Host *host = ha->host;

	host->can_queue = max_srbs;  /* default value:-MAX_SRBS(4096)  */
	host->cmd_per_lun = 1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	host->select_queue_depths = qla2x00_select_queue_depth;
#endif
#if defined (CONFIG_SCSIFCHOTSWAP) || defined(CONFIG_GAMAP)
	host->hostt->get_scsi_info_from_wwn = qla2x00_get_scsi_info_from_wwn;
	host->hostt->get_wwn_from_scsi_info = qla2x00_get_wwn_from_scsi_info;
#endif /* CONFIG_SCSIFCHOTSWAP || CONFIG_GAMAP */

	host->n_io_port = 0xFF;

#if MEMORY_MAPPED_IO
	host->base = (unsigned long) ha->mmpbase;
#else
	host->base = 0;
#endif

	host->max_channel = maxchannels;
	/* fix: 07/31 host->max_lun = MAX_LUNS-1; */
	host->max_lun = ha->max_luns;
	host->unique_id = ha->instance;
	host->max_id = ha->max_targets;

	/* set our host ID  (need to do something about our two IDs) */
	host->this_id = 255;

#if defined(CONFIG_MD_MULTIHOST_FC)
	{
		unsigned long   i;
		unsigned long   j;

		union {
			__u64   identifier;
			char    wwn[WWN_SIZE];
		} foo;

		for (i = 0, j = WWN_SIZE-1; i < WWN_SIZE; i++, j--) {
			foo.wwn[i] = ha->init_cb->port_name[j];
		}

		host->fc_wwn = foo.identifier;
	}
#endif /* CONFIG_MD_MULTIHOST_FC */

	/* Register the I/O space with Linux */
	if (check_region(host->io_port, 0xff)) {
		printk(KERN_WARNING
			"qla2x00: Failed to reserved i/o base region "
			"0x%04lx-0x%04lx already in use\n",
			host->io_port, host->io_port + 0xff);
		/* 6/15/01 - free_irq(host->irq, NULL); */
		return 1;
	}
	request_region(host->io_port, 0xff, DRIVER_NAME);

	/* Register the IRQ with Linux (sharable) */
	if (request_irq(host->irq, qla2x00_intr_handler,
			SA_INTERRUPT|SA_SHIRQ, DRIVER_NAME, ha)) {
		printk(KERN_WARNING
			"qla2x00 : Failed to reserve interrupt %d "
			"already in use\n",
			host->irq);
		release_region(host->io_port, 0xff);
		return 1;
	}

	/* Initialized the timer */
	START_TIMER(qla2x00_timer, ha, WATCH_INTERVAL);

	return 0;
}


/**************************************************************************
*   qla2x00_release
*
* Description:
*   Free the passed in Scsi_Host memory structures prior to unloading the
*   module.
*
* Input:
*     ha - pointer to host adapter structure
*
* Returns:
*  0 - Always returns good status
**************************************************************************/
int
qla2x00_release(struct Scsi_Host *host)
{
	scsi_qla_host_t *ha = (scsi_qla_host_t *) host->hostdata;
#if  QL_TRACE_MEMORY
	int t;
#endif

	ENTER("qla2x00_release");

	/* turn-off interrupts on the card */
	if (ha->interrupts_on)
		qla2x00_disable_intrs(ha);

	/* Detach interrupts */
	if (host->irq)
		free_irq(host->irq, ha);

	/* release io space registers  */
	if (host->io_port)
		release_region(host->io_port, 0xff);

	/* Disable timer */
	if (ha->timer_active)
		STOP_TIMER(qla2x00_timer,ha)

	/* Kill the kernel thread for this host */
	if (ha->dpc_handler != NULL ) {
#ifdef __VMWARE__
		extern int vmk_shutting_down(void);
		if (vmk_shutting_down()) {
			printk("qla: vmkernel shutting down\n");
		} else {
			printk("qla: killing thread and waiting\n");
			ha->should_die = 1;
			ha->notify_sema = (struct semaphore)__SEMAPHORE_INITIALIZER(ha->notify_sema, 0);
			ha->dpc_notify = &ha->notify_sema;
			up(&ha->wait_sema);
			down(ha->dpc_notify);
			printk("qla: back from killing thread\n");
		}
#else

#if defined(ISP2100)
		ha->dpc_notify = &qla2100_detect_sem;
#endif
#if defined(ISP2200)
		ha->dpc_notify = &qla2200_detect_sem;
#endif
#if defined(ISP2300)
		ha->dpc_notify = &qla2300_detect_sem;
#endif

		send_sig(SIGHUP, ha->dpc_handler, 1);

#if defined(ISP2100)
		down(&qla2100_detect_sem);
#endif
#if defined(ISP2200)
		down(&qla2200_detect_sem);
#endif
#if defined(ISP2300)
		down(&qla2300_detect_sem);
#endif

#endif //__VMWARE__
		ha->dpc_notify = NULL;
	}

#if USE_FLASH_DATABASE
	/* Move driver database to flash, if enabled. */
	if (ha->flags.enable_flash_db_update &&
		ha->flags.updated_fc_db) {

		ha->flags.updated_fc_db = FALSE;
		qla2x00_save_database(ha);
	}
#endif

#if MEMORY_MAPPED_IO
	if (ha->mmpbase) {
		iounmap((void *) (((unsigned long) ha->mmpbase) & PAGE_MASK));
	}
#endif

#if APIDEV
	apidev_cleanup();
#endif

#ifdef __VMWARE__
	spin_lock_destroy(&ha->hardware_lock);
	spin_lock_destroy(&ha->mbx_bits_lock);
	spin_lock_destroy(&ha->mbx_reg_lock);
	spin_lock_destroy(&ha->mbx_q_lock);
	spin_lock_destroy(&ha->list_lock);
#endif
	qla2x00_mem_free(ha);

	if (ha->flags.failover_enabled)
		qla2x00_cfg_mem_free(ha);

#if QL_TRACE_MEMORY
	for (t = 0; t < 1000; t++) {
		if (mem_trace[t] == 0L)
			continue;
		printk("mem_trace[%d]=%lx, %lx\n",
			t, mem_trace[t],mem_id[t]);
	}
#endif

	ha->flags.online = FALSE;

	LEAVE("qla2x00_release");

	return 0;
}

/**************************************************************************
*   qla2x00_info
*
* Description:
*
* Input:
*     host - pointer to Scsi host adapter structure
*
* Returns:
*     Return a text string describing the driver.
**************************************************************************/
const char *
qla2x00_info(struct Scsi_Host *host)
{
	static char qla2x00_buffer[255];
	char *bp;
	scsi_qla_host_t *ha;
	qla_boards_t   *bdp;

#if  APIDEV
	/* We must create the api node here instead of qla2x00_detect since we
	 * want the api node to be subdirectory of /proc/scsi/qla2x00 which
	 * will not have been created when qla2x00_detect exits, but which will
	 * have been created by this point.
	 */
	apidev_init(host);
#endif

	bp = &qla2x00_buffer[0];
	ha = (scsi_qla_host_t *)host->hostdata;
	bdp = &QLBoardTbl_fc[ha->devnum];
	memset(bp, 0, sizeof(qla2x00_buffer));
	sprintf(bp,
			"QLogic %sPCI to Fibre Channel Host Adapter: "
			"bus %d device %d irq %d\n"
			"        Firmware version: %2d.%02d.%02d, "
			"Driver version %s\n",
			(char *)&bdp->bdName[0], ha->pdev->bus->number,
			PCI_SLOT(ha->pdev->devfn),
			host->irq,
			bdp->fwver[0], bdp->fwver[1], bdp->fwver[2],
			qla2x00_version_str);

	return bp;
}

/*
 * This routine will alloacte SP from the free queue
 * input:
 *        scsi_qla_host_t *
 * output:
 *        srb_t * or NULL
 */
STATIC srb_t *
qla2x00_get_new_sp(scsi_qla_host_t *ha)
{
	srb_t * sp = NULL;
	ulong  flags;

	spin_lock_irqsave(&ha->list_lock, flags);
	if (!list_empty(&ha->free_queue)) {
		sp = list_entry(ha->free_queue.next, srb_t, list);
		__del_from_free_queue(ha, sp);
	}
	spin_unlock_irqrestore(&ha->list_lock, flags);

	if (sp) {
		DEBUG4(
		if ((int)atomic_read(&sp->ref_count) != 0) {
			/* error */
			printk("qla2x00_get_new_sp: WARNING "
				"ref_count not zero.\n");
		})

		sp_get(ha, sp);
	}

	return (sp);
}

/**************************************************************************
*   qla2x00_check_tgt_status
*
* Description:
*     Checks to see if the target or loop is down.
*
* Input:
*     cmd - pointer to Scsi cmd structure
*
* Returns:
*   1 - if target is present
*   0 - if target is not present
*
**************************************************************************/
STATIC uint8_t
qla2x00_check_tgt_status(scsi_qla_host_t *ha, Scsi_Cmnd *cmd)
{
	os_lun_t        *lq;
	uint32_t         b, t, l;
	fc_port_t	*fcport;

	/* Generate LU queue on bus, target, LUN */
	b = SCSI_BUS_32(cmd);
	t = SCSI_TCN_32(cmd);
	l = SCSI_LUN_32(cmd);

	if ((lq = GET_LU_Q(ha,t,l)) == NULL) {
		return(QL_STATUS_ERROR);
	}

	fcport = lq->fclun->fcport;

	if (TGT_Q(ha, t) == NULL || 
		l >= ha->max_luns ||
		(atomic_read(&fcport->state) == FC_DEVICE_DEAD) ||
		(!atomic_read(&ha->loop_down_timer) && 
		ha->loop_state == LOOP_DOWN)||
		(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	 	ABORTS_ACTIVE  || 
		ha->loop_state != LOOP_READY) {

		DEBUG(printk(KERN_INFO
				"scsi(%ld:%2d:%2d:%2d): %s connection is "
				"down\n",
				ha->host_no,
				b,t,l,
				__func__);)

		CMD_RESULT(cmd) = DID_NO_CONNECT << 16;
		return(QL_STATUS_ERROR);
	}
	return (QL_STATUS_SUCCESS);
}

/**************************************************************************
*   qla2x00_check_port_status
*
* Description:
*     Checks to see if the port or loop is down.
*
* Input:
*     fcport - pointer to fc_port_t structure.
*
* Returns:
*   1 - if port is present
*   0 - if port is not present
*
**************************************************************************/
STATIC uint8_t
qla2x00_check_port_status(scsi_qla_host_t *ha, fc_port_t *fcport)
{
	uint32_t	port_state;

	if (fcport == NULL) {
		return(QL_STATUS_ERROR);
	}

	port_state = (uint32_t)atomic_read(&fcport->state);
	if ((port_state != FC_ONLINE) || 
		(port_state == FC_DEVICE_DEAD) ||
		(!atomic_read(&ha->loop_down_timer) && 
		ha->loop_state == LOOP_DOWN) ||
		(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
		test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
		ABORTS_ACTIVE || 
		ha->loop_state != LOOP_READY) {

		DEBUG(printk(KERN_INFO
				"%s(%ld): connection is down. fcport=%p.\n",
				__func__,
				ha->host_no,
				fcport);)

		return(QL_STATUS_ERROR);
	}
	return (QL_STATUS_SUCCESS);
}


/**************************************************************************
* qla2x00_queuecommand
*
* Description:
*     Queue a command to the controller.
*
* Input:
*     cmd - pointer to Scsi cmd structure
*     fn - pointer to Scsi done function
*
* Returns:
*   0 - Always
*
* Note:
* The mid-level driver tries to ensures that queuecommand never gets invoked
* concurrently with itself or the interrupt handler (although the
* interrupt handler may call this routine as part of request-completion
* handling).
**************************************************************************/
int
qla2x00_queuecommand(Scsi_Cmnd *cmd, void (*fn)(Scsi_Cmnd *))
{
	fc_port_t	*fcport;
	os_lun_t	*lq;
	os_tgt_t	*tq;
	scsi_qla_host_t	*ha, *ha2;
	srb_t		*sp;
	struct Scsi_Host	*host;

	uint32_t	b, t, l;
#if  BITS_PER_LONG <= 32
	uint32_t	handle;
#else
	u_long		handle;
#endif

	ENTER(__func__);

	host = cmd->host;
	ha = (scsi_qla_host_t *) host->hostdata;

	cmd->scsi_done = fn;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	spin_unlock(&io_request_lock);
#else
	spin_unlock(ha->host->host_lock);
#endif

	/*
	 * Allocate a command packet from the "sp" pool.  If we cant get back
	 * one then let scsi layer come back later.
	 */
	if ((sp = qla2x00_get_new_sp(ha)) == NULL) {
		printk(KERN_WARNING
			"%s(): Couldn't allocate memory for sp - retried.\n",
			__func__);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif

		LEAVE(__func__);
		return (1);
	}

	sp->cmd = cmd;
	CMD_SP(cmd) = (void *)sp;

	sp->flags = 0;
	if (CMD_RESID_LEN(cmd) & SRB_IOCTL) {
		/* Need to set sp->flags */
		sp->flags |= SRB_IOCTL;
		CMD_RESID_LEN(cmd) = 0; /* Clear it since no more use. */
	}

	sp->fo_retry_cnt = 0;
	sp->iocb_cnt = 0;

	if (cmd->allowed < ql2xretrycount) {
		cmd->allowed = ql2xretrycount;
	}

	/* Generate LU queue on bus, target, LUN */
	b = SCSI_BUS_32(cmd);
	t = SCSI_TCN_32(cmd);
	l = SCSI_LUN_32(cmd);

	/*
	 * Start Command Timer. Typically it will be 2 seconds less than what
	 * is requested by the Host such that we can return the IO before
	 * aborts are called.
	 */
	if ((CMD_TIMEOUT(cmd)/HZ) > QLA_CMD_TIMER_DELTA)
		qla2x00_add_timer_to_cmd(sp,
				(CMD_TIMEOUT(cmd)/HZ) - QLA_CMD_TIMER_DELTA);
	else
		qla2x00_add_timer_to_cmd(sp, (CMD_TIMEOUT(cmd)/HZ));

	if (l >= ha->max_luns) {
		CMD_RESULT(cmd) = DID_NO_CONNECT << 16;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif
		__sp_put(ha, sp);
		LEAVE(__func__);
		return (0);
	}

	if ((tq = (os_tgt_t *) TGT_Q(ha, t)) != NULL &&
		(lq = (os_lun_t *) LUN_Q(ha, t, l)) != NULL ) {

		fcport = lq->fclun->fcport;
		ha2 = fcport->ha;
	} else {
		lq = NULL;
		fcport = NULL;
		ha2 = ha;
	}

	/* Set an invalid handle until we issue the command to ISP */
	/* then we will set the real handle value.                 */
	handle = INVALID_HANDLE;
	CMD_HANDLE(cmd) = (unsigned char *)handle;

	DEBUG4(printk("scsi(%ld:%2d:%2d): (queuecmd) queue sp = %p, "
			"flags=0x%x fo retry=%d, pid=%ld, cmd flags= 0x%x\n",
			ha->host_no,t,l,sp,sp->flags,sp->fo_retry_cnt,
			cmd->serial_number,cmd->flags);)

	/* Bookkeeping information */
	sp->r_start = jiffies;       /* time the request was recieved */
	sp->u_start = 0;

	/* Setup device queue pointers. */
	sp->tgt_queue = tq;
	sp->lun_queue = lq;

	/*
	 * NOTE : q is NULL
	 *
	 * 1. When device is added from persistent binding but has not been
	 *    discovered yet.The state of loopid == PORT_AVAIL.
	 * 2. When device is never found on the bus.(loopid == UNUSED)
	 *
	 * IF Device Queue is not created, or device is not in a valid state
	 * and link down error reporting is enabled, reject IO.
	 */
	if (fcport == NULL) {
		DEBUG3(printk("scsi(%ld:%2d:%2d): port unavailable\n",
				ha->host_no,t,l);)

		CMD_RESULT(cmd) = DID_NO_CONNECT << 16;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif
		__sp_put(ha, sp);
		return (0);
	}

	DEBUG5(printk("%s(): pid=%ld, opcode=%d, timeout= %d\n",
			__func__,
			cmd->serial_number,
			cmd->cmnd[0],
			CMD_TIMEOUT(cmd));)
	DEBUG5(qla2x00_print_scsi_cmd(cmd);)

	sp->flags &= ~SRB_ISP_COMPLETED;

	sp->fclun = lq->fclun;
	sp->ha = ha2;

	sp->cmd_length = CMD_CDBLEN(cmd);

	if (cmd->sc_data_direction == SCSI_DATA_UNKNOWN &&
		cmd->request_bufflen != 0) {

		DEBUG2(printk(KERN_WARNING
				"%s(): Incorrect data direction - transfer "
				"length=%d, direction=%d, pid=%ld, opcode=%x\n",
				__func__,
				cmd->request_bufflen,
				cmd->sc_data_direction,
				cmd->serial_number,
				cmd->cmnd[0]);)
	}

	/* Final pre-check */
	if (atomic_read(&fcport->state) == FC_DEVICE_DEAD) {
		/*
		 * Add the command to the done-queue for later failover
		 * processing
		 */
		CMD_RESULT(cmd) = DID_NO_CONNECT << 16;
		add_to_done_queue(ha, sp);
#if QLA2X_PERFORMANCE
		tasklet_schedule(&ha->run_qla_task);
#else
		qla2x00_done(ha);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif
		return (0);
	}

	add_to_pending_queue(ha, sp);

	/* First start cmds for this lun if possible */
	qla2x00_next(ha);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	spin_lock_irq(&io_request_lock);
#else
	spin_lock_irq(ha->host->host_lock);
#endif

	LEAVE(__func__);
	return (0);
}

/*
 * qla2x00_eh_wait_on_command
 *    Waits for the command to be returned by the Firmware for some
 *    max time.
 *
 * Input:
 *    ha = actual ha whose done queue will contain the command
 *	      returned by firmware.
 *    cmd = Scsi Command to wait on.
 *    flag = Abort/Reset(Bus or Device Reset)
 *
 * Return:
 *    Not Found : 0
 *    Found : 1
 */
STATIC int
qla2x00_eh_wait_on_command(scsi_qla_host_t *ha, Scsi_Cmnd *cmd)
{
#define ABORT_WAIT_TIME	10 /* seconds */
#define EH_ACTIVE       1  /* Error Handler Active */	

	int		found = 0;
	int		done = 0;
	srb_t		*rp;
	struct list_head *list, *temp;
	u_long		cpu_flags = 0;
	u_long		max_wait_time = ABORT_WAIT_TIME;

	ENTER(__func__);

	do {
		/* Check on done queue */
		if (!found) {
			spin_lock_irqsave(&ha->list_lock, cpu_flags);
			list_for_each_safe(list, temp, &ha->done_queue) {
				rp = list_entry(list, srb_t, list);

				/*
				* Found command.  Just exit and wait for the
				* cmd sent to OS.
			 	*/
				if (cmd == rp->cmd) {
					found++;
					DEBUG3(printk("%s: found in done "
							"queue.\n", __func__);)
					break;
				}
			}
			spin_unlock_irqrestore(&ha->list_lock, cpu_flags);
		}

		/* Checking to see if its returned to OS */
		rp = (srb_t *) CMD_SP(cmd);
		if (rp == NULL ) {
			done++;
			break;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_unlock_irq(&io_request_lock);
#else
		spin_unlock_irq(ha->host->host_lock);
#endif

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(2*HZ);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif

	} while ((max_wait_time--));

	DEBUG2(if (done))
	DEBUG2(printk("%s: found cmd=%p.\n", __func__, cmd);)

	LEAVE(__func__);

	return(done);
}

/**************************************************************************
* qla2xxx_eh_abort
*
* Description:
*    The abort function will abort the specified command.
*
* Input:
*    cmd = Linux SCSI command packet to be aborted.
*
* Returns:
*    Either SUCCESS or FAILED.
*
* Note:
**************************************************************************/
int
qla2xxx_eh_abort(Scsi_Cmnd *cmd)
{
	int		i;
	int		return_status = FAILED;
	os_lun_t	*q;
	scsi_qla_host_t *ha;
	scsi_qla_host_t *vis_ha;
	srb_t		*sp;
	srb_t		*rp;
	struct list_head *list, *temp;
	struct Scsi_Host *host;
	uint8_t		found = 0;
	uint32_t	b, t, l;
	unsigned long	flags;


	ENTER("qla2xxx_eh_abort");

	/* Get the SCSI request ptr */
	sp = (srb_t *) CMD_SP(cmd);

	/*
	 * If sp is NULL, command is already returned.
	 * sp is NULLed just before we call back scsi_done
	 *
	 */
	if ((sp == NULL)) {
		/* no action - we don't have command */
		DEBUG(printk("qla2xxx_eh_abort: cmd already done sp=%p\n",sp);)
		return(SUCCESS);
	}
	if (sp) {
		DEBUG(printk("qla2xxx_eh_abort: refcount %i \n",
		    atomic_read(&sp->ref_count));)
	}

	vis_ha = (scsi_qla_host_t *) cmd->host->hostdata;
	vis_ha->eh_start=0;
	if (vis_ha->flags.failover_enabled)
		/* Get Actual HA pointer */
		ha = (scsi_qla_host_t *)sp->ha;
	else
		ha = (scsi_qla_host_t *)cmd->host->hostdata;

	host = ha->host;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,7)
	/* Check for possible pending interrupts. */
	qla2x00_process_risc_intrs(ha);
#endif

	/* Generate LU queue on bus, target, LUN */
	b = SCSI_BUS_32(cmd);
	t = SCSI_TCN_32(cmd);
	l = SCSI_LUN_32(cmd);
	q = GET_LU_Q(vis_ha, t, l);

	/*
	 * if no LUN queue then something is very wrong!!!
	 */
	if (q == NULL) {
		printk(KERN_WARNING
			"qla2x00: (%x:%x:%x) No LUN queue.\n", b, t, l);

		/* no action - we don't have command */
		return(FAILED);
	}

	DEBUG2(printk("scsi(%ld): ABORTing cmd=%p sp=%p jiffies = 0x%lx, "
	    "timeout=%x, dpc_flags=%lx, vis_ha->dpc_flags=%lx\n",
	    ha->host_no,
	    cmd,
	    sp,
	    jiffies,
	    CMD_TIMEOUT(cmd)/HZ,
	    ha->dpc_flags,
	    vis_ha->dpc_flags);)
	DEBUG2(qla2x00_print_scsi_cmd(cmd));
	DEBUG2(qla2x00_print_q_info(q);)

	/* Search done queue */
	spin_lock_irqsave(&ha->list_lock,flags);
	list_for_each_safe(list, temp, &ha->done_queue) {
		rp = list_entry(list, srb_t, list);

		if (cmd != rp->cmd)
			continue;

		/*
		 * Found command. No need to remove command from done list.
		 * Just proceed to call done.
		 */
		return_status = SUCCESS;
		found++;
		qla2x00_delete_from_done_queue(ha, sp);

		break;
	} /* list_for_each_safe() */
	spin_unlock_irqrestore(&ha->list_lock, flags);

	/*
	 * Return immediately if the aborted command was already in the done
	 * queue
	 */
	if (found) {
		printk(KERN_INFO "qla2xxx_eh_abort: Returning completed "
			"command=%p sp=%p\n", cmd, sp);
		__sp_put(ha, sp);
		return (return_status);
	}
	

	/*
	 * See if this command is in the retry queue
	 */
	if (!found) {
		DEBUG3(printk("qla2xxx_eh_abort: searching sp %p "
		    "in retry queue.\n", sp);)

		spin_lock_irqsave(&ha->list_lock, flags);
		list_for_each_safe(list, temp, &ha->retry_queue) {
			rp = list_entry(list, srb_t, list);

			if (cmd != rp->cmd)
				continue;


			DEBUG2(printk("qla2xxx_eh_abort: found "
			    "in retry queue. SP=%p\n", sp);)

			__del_from_retry_queue(ha, rp);
			CMD_RESULT(rp->cmd) = DID_ABORT << 16;
			__add_to_done_queue(ha, rp);

			return_status = SUCCESS;
			found++;

			break;

		} /* list_for_each_safe() */
		spin_unlock_irqrestore(&ha->list_lock, flags);
	}

	/*
	 * Search failover queue
	 */
	if (ha->flags.failover_enabled) {
		if (!found) {
			DEBUG3(printk("qla2xxx_eh_abort: searching sp %p "
					"in failover queue.\n", sp);)

			spin_lock_irqsave(&ha->list_lock, flags);
			list_for_each_safe(list, temp, &ha->failover_queue) {
				rp = list_entry(list, srb_t, list);

				if (cmd != rp->cmd)
					continue;

				DEBUG2(printk(KERN_WARNING
						"qla2xxx_eh_abort: found "
						"in failover queue. SP=%p\n",
						sp);)

				/* Remove srb from failover queue. */
				__del_from_failover_queue(ha, rp);
				CMD_RESULT(rp->cmd) = DID_ABORT << 16;
				__add_to_done_queue(ha, rp);

				return_status = SUCCESS;
				found++;

				break;

			} /* list_for_each_safe() */
			spin_unlock_irqrestore(&ha->list_lock, flags);
		} /*End of if !found */
	}

	/*
	 * Our SP pointer points at the command we want to remove from the
	 * pending queue providing we haven't already sent it to the adapter.
	 */
	if (!found) {
		DEBUG3(printk("qla2xxx_eh_abort: searching sp %p "
		    "in pending queue.\n", sp);)

		spin_lock_irqsave(&vis_ha->list_lock, flags);
		list_for_each_safe(list, temp, &vis_ha->pending_queue) {
			rp = list_entry(list, srb_t, list);
			if (rp->cmd != cmd)
				continue;

			/* Remove srb from LUN queue. */
			rp->flags |=  SRB_ABORTED;

			DEBUG2(printk("qla2xxx_eh_abort: Cmd in pending queue."
			    " serial_number %ld.\n",
			    sp->cmd->serial_number);)

			__del_from_pending_queue(vis_ha, rp);
			CMD_RESULT(cmd) = DID_ABORT << 16;

			__add_to_done_queue(vis_ha, rp);

			return_status = SUCCESS;

			found++;
			break;
		} /* list_for_each_safe() */
		spin_unlock_irqrestore(&vis_ha->list_lock, flags);
	} /*End of if !found */

	if (!found) {  /* find the command in our active list */
		DEBUG3(printk("qla2xxx_eh_abort: searching sp %p "
		    "in outstanding queue.\n", sp);)

		spin_lock_irqsave(&ha->hardware_lock, flags);
		for (i = 1; i < MAX_OUTSTANDING_COMMANDS; i++) {
			sp = ha->outstanding_cmds[i];

			if (sp == NULL)
				continue;

			if (sp->cmd != cmd)
				continue;


			DEBUG2(printk("qla2xxx_eh_abort(%ld): aborting sp %p "
			    "from RISC. pid=%d sp->state=%x\n",
			    ha->host_no, 
			    sp, 
			    (int)sp->cmd->serial_number,
			    sp->state);)
			DEBUG(qla2x00_print_scsi_cmd(cmd);)
			DEBUG(qla2x00_print_q_info(q);)

			/* Get a reference to the sp and drop the lock.*/
			sp_get(ha,sp);

			spin_unlock_irqrestore(&ha->hardware_lock, flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
			spin_unlock(&io_request_lock);
#else
			spin_unlock(ha->host->host_lock);
#endif

			if (qla2x00_abort_command(ha, sp)) {
				DEBUG2(printk("qla2xxx_eh_abort: abort_command "
				    "mbx failed.\n");)
				return_status = FAILED;
			} else {
				DEBUG3(printk("qla2xxx_eh_abort: abort_command "
				    " mbx success.\n");)
				return_status = SUCCESS;
			}

			sp_put(ha,sp);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
			spin_lock_irq(&io_request_lock);
#else
			spin_lock_irq(ha->host->host_lock);
#endif
			spin_lock_irqsave(&ha->hardware_lock, flags);

			/*
			 * Regardless of mailbox command status, go check on
			 * done queue just in case the sp is already done.
			 */
			break;

		}/*End of for loop */
		spin_unlock_irqrestore(&ha->hardware_lock, flags);

	} /*End of if !found */

	  /*Waiting for our command in done_queue to be returned to OS.*/
	if (qla2x00_eh_wait_on_command(ha, cmd) != 0) {
		DEBUG2(printk("qla2xxx_eh_abort: cmd returned back to OS.\n");)
		return_status = SUCCESS;
	}

	if (return_status == FAILED) {
		printk(KERN_INFO "qla2xxx_eh_abort Exiting: status=Failed\n");
		return FAILED;
	}

	DEBUG(printk("qla2xxx_eh_abort: Exiting. return_status=0x%x.\n",
	    return_status));

	LEAVE("qla2xxx_eh_abort");

	return(return_status);
}

/**************************************************************************
* qla2x00_eh_wait_for_pending_target_commands
*
* Description:
*    Waits for all the commands to come back from the specified target.
*
* Input:
*    ha - pointer to scsi_qla_host structure.
*    t  - target 	
* Returns:
*    Either SUCCESS or FAILED.
*
* Note:
**************************************************************************/
int
qla2x00_eh_wait_for_pending_target_commands(scsi_qla_host_t *ha, int t)
{
	int	cnt;
	int	status;
	unsigned long	flags;
	srb_t		*sp;
	Scsi_Cmnd	*cmd;

	status = 0;

	/*
	 * Waiting for all commands for the designated target in the active
	 * array
	 */
	for (cnt = 1; cnt < MAX_OUTSTANDING_COMMANDS; cnt++) {
		spin_lock_irqsave(&ha->hardware_lock, flags);
		sp = ha->outstanding_cmds[cnt];
		if (sp) {
			cmd = sp->cmd;
			spin_unlock_irqrestore(&ha->hardware_lock, flags);
			if (SCSI_TCN_32(cmd) == t) {
				qla2x00_eh_wait_on_command(ha, cmd);
			}
		}
		else {
			spin_unlock_irqrestore(&ha->hardware_lock, flags);
		}
	}
	return (status);
}


/**************************************************************************
* qla2xxx_eh_device_reset
*
* Description:
*    The device reset function will reset the target and abort any
*    executing commands.
*
*    NOTE: The use of SP is undefined within this context.  Do *NOT*
*          attempt to use this value, even if you determine it is 
*          non-null.
*
* Input:
*    cmd = Linux SCSI command packet of the command that cause the
*          bus device reset.
*
* Returns:
*    SUCCESS/FAILURE (defined as macro in scsi.h).
*
**************************************************************************/
int
qla2xxx_eh_device_reset(Scsi_Cmnd *cmd)
{
	int		return_status = SUCCESS;
	uint32_t	b, t, l;
	scsi_qla_host_t	*ha;

#if defined(LOGOUT_AFTER_DEVICE_RESET)
	os_lun_t	*lq;
	fc_port_t	*fcport;
#endif

	ENTER(__func__);

	if (cmd == NULL) {
		printk(KERN_INFO
			"%s(): **** SCSI mid-layer passing in NULL cmd\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

	/* Verify the device exists. */
	ha = (scsi_qla_host_t *)cmd->host->hostdata;
	ha->eh_start = 0;
	b = SCSI_BUS_32(cmd);
	t = SCSI_TCN_32(cmd);
	l = SCSI_LUN_32(cmd);
	if (TGT_Q(ha, t) == NULL) {
		printk(KERN_INFO
			"%s(): **** CMD derives a NULL TGT_Q\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

	ha = (scsi_qla_host_t *)cmd->host->hostdata;

#if STOP_ON_RESET
	printk(debug_buff,"Resetting Device= 0x%x\n", (int)cmd);
/* WE SHOULD NOT call this function, since it dereferences SP */
 	//qla2x00_print_scsi_cmd(cmd);
	qla2x00_panic(__func__, ha->host);
#endif

	if (qla2x00_verbose)
		printk(KERN_INFO
			"scsi(%ld:%d:%d:%d): DEVICE RESET ISSUED.\n",
			ha->host_no, (int)b, (int)t, (int)l);

	DEBUG2(printk("scsi(%ld): DEVICE_RESET cmd=%p jiffies = 0x%lx, "
		"timeout=%x, dpc_flags=%lx, status=%x allowed=%d "
		"cmd.state=%x\n",
		ha->host_no,
		cmd,
		jiffies,
		CMD_TIMEOUT(cmd)/HZ,
		ha->dpc_flags,
		cmd->result,
		cmd->allowed,
		cmd->state);)
/* WE SHOULD NOT call this function, since it dereferences SP */
	//qla2x00_print_scsi_cmd(cmd);

	if (!((test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
		(test_bit(LOOP_RESET_NEEDED, &ha->dpc_flags)) ||
		(test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags)) ||
		(!atomic_read(&ha->loop_down_timer) &&
		 ha->loop_state == LOOP_DOWN)||
		test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
		ha->loop_state != LOOP_READY)) {

		clear_bit(DEVICE_RESET_NEEDED, &ha->dpc_flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_unlock_irq(&io_request_lock);
#else
		spin_unlock_irq(ha->host->host_lock);
#endif

		if (qla2x00_device_reset(ha, t, l) != 0) {
			return_status = FAILED;
		}

#if defined(LOGOUT_AFTER_DEVICE_RESET)
		if (return_status == SUCCESS) {
			lq = (os_lun_t *)LUN_Q(ha, t, l);
			fcport = lq->fclun->fcport;

			if (fcport->flags & FC_FABRIC_DEVICE) {
				qla2x00_fabric_logout(ha,
						ha->fc_db[t].loop_id & 0xff);
				ha->fc_db[t].flag |= DEV_RELOGIN;
				qla2x00_mark_device_lost(ha, fcport);
			}
		}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif

	} else {
		/*
		 * Wait a while for the loop to come back. Return SUCCESS
		 * for the kernel to try again.
		 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_unlock_irq(&io_request_lock);
#else
		spin_unlock_irq(ha->host->host_lock);
#endif

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(5 * HZ);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif

		return_status = SUCCESS;
	}

	if (return_status == FAILED) {
		DEBUG2(printk("%s() Exiting: Reset Failed\n", __func__);)
		return (FAILED);
	}

	/* Waiting for all commands to complete for the device */
	if (qla2x00_eh_wait_for_pending_target_commands(ha, t))
		return_status = FAILED;

	if (return_status == FAILED) {
		printk(KERN_INFO "%s() Exiting: status = Failed\n", __func__);
		return (FAILED);
	}

	LEAVE(__func__);

	return (return_status);
}

/**************************************************************************
* qla2x00_eh_wait_for_pending_commands
*
* Description:
*    Waits for all the commands to come back from the specified host.
*
* Input:
*    ha - pointer to scsi_qla_host structure.
*
* Returns:
*    1 : SUCCESS
*    0 : FAILED
*
* Note:
**************************************************************************/
int
qla2x00_eh_wait_for_pending_commands(scsi_qla_host_t *ha)
{
	int	cnt;
	int	status;
	unsigned long	flags;
	srb_t		*sp;
	Scsi_Cmnd	*cmd;

	status = 1;

	/*
	 * Waiting for all commands for the designated target in the active
	 * array
	 */
	for (cnt = 1; cnt < MAX_OUTSTANDING_COMMANDS; cnt++) {
		spin_lock_irqsave(&ha->hardware_lock, flags);
		sp = ha->outstanding_cmds[cnt];
		if (sp) {
			cmd = sp->cmd;
			spin_unlock_irqrestore(&ha->hardware_lock, flags);
				status = qla2x00_eh_wait_on_command(ha, cmd);
		}
		else {
			spin_unlock_irqrestore(&ha->hardware_lock, flags);
		}
	}
	return (status);
}


/**************************************************************************
* qla2xxx_eh_bus_reset
*
* Description:
*    The bus reset function will reset the bus and abort any executing
*    commands.
*
* Input:
*    cmd = Linux SCSI command packet of the command that cause the
*          bus reset.
*
* Returns:
*    SUCCESS/FAILURE (defined as macro in scsi.h).
*
**************************************************************************/
int
qla2xxx_eh_bus_reset(Scsi_Cmnd *cmd)
{
	int        return_status = SUCCESS;
	uint32_t   b, t, l;
	srb_t      *sp;
	scsi_qla_host_t *ha, *search_ha = NULL;

	ENTER("qla2xxx_eh_bus_reset");

	if (cmd == NULL) {
		printk(KERN_INFO
			"%s(): **** SCSI mid-layer passing in NULL cmd\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

	b = SCSI_BUS_32(cmd);
	t = SCSI_TCN_32(cmd);
	l = SCSI_LUN_32(cmd);

	ha = (scsi_qla_host_t *) cmd->host->hostdata;
	ha->eh_start=0;
	sp = (srb_t *) CMD_SP(cmd);

	if (ha == NULL) {
		printk(KERN_INFO
			"%s(): **** CMD derives a NULL HA\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

	for (search_ha = qla2x00_hostlist;
		(search_ha != NULL) && search_ha != ha;
		search_ha = search_ha->next)
		continue;

	if (search_ha == NULL) {
		printk(KERN_INFO
			"%s(): **** CMD derives a NULL search HA\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

#if  STOP_ON_RESET
	printk("Resetting the Bus= 0x%x\n", (int)cmd);
	qla2x00_print_scsi_cmd(cmd);
	qla2x00_panic("qla2100_reset", ha->host);
#endif

	if (qla2x00_verbose)
		printk(KERN_INFO
			"scsi(%ld:%d:%d:%d): LOOP RESET ISSUED.\n",
			ha->host_no, (int)b, (int)t, (int)l);

	if (!((test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
		(test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags)) ||
		(!atomic_read(&ha->loop_down_timer) &&
		 ha->loop_state == LOOP_DOWN)||
		test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
		ha->loop_state != LOOP_READY)) {

		clear_bit(LOOP_RESET_NEEDED, &ha->dpc_flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_unlock_irq(&io_request_lock);
#else
		spin_unlock_irq(ha->host->host_lock);
#endif

		if (qla2x00_loop_reset(ha) != 0) {
			return_status = FAILED;
		}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif
	} else {
		/*
		 * Wait a while for the loop to come back. Return SUCCESS
		 * for the kernel to try again.
		 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_unlock_irq(&io_request_lock);
#else
		spin_unlock_irq(ha->host->host_lock);
#endif

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(5 * HZ);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif

		return_status = SUCCESS;
	}

	if (return_status == FAILED) {
		DEBUG2(printk("qla2xxx_eh_bus_reset Exiting: Reset Failed\n");)
		printk("qla2xxx_eh_bus_reset Exiting: Reset Failed\n");
		return FAILED;
	}

	/* Blocking Call. It goes to sleep waiting for cmd to get to done q */
	 /* Waiting for our command in done_queue to be returned to OS.*/

	if ( qla2x00_eh_wait_for_pending_commands(ha) == 0) {
		return_status = FAILED;
	}

	if(return_status == FAILED) {
		printk(KERN_INFO "qla2xxx_eh_bus_reset Exiting: status=Failed\n");
		return FAILED;
	} else
		printk(KERN_INFO "qla2xxx_eh_bus_reset Exiting: status=SUCCESS\n");

	LEAVE("qla2xxx_eh_bus_reset");

	return (return_status);
}

/**************************************************************************
* qla2xxx_eh_host_reset
*
* Description:
*    The reset function will reset the Adapter.
*
* Input:
*      cmd = Linux SCSI command packet of the command that cause the
*            adapter reset.
*
* Returns:
*      Either SUCCESS or FAILED.
*
* Note:
**************************************************************************/
int
qla2xxx_eh_host_reset(Scsi_Cmnd *cmd)
{
	int		return_status = SUCCESS;
	scsi_qla_host_t	*ha; /* actual ha to reset. */
	scsi_qla_host_t	*search_ha;
	srb_t		*sp;
	uint32_t        b, t, l;

	ENTER("qla2xxx_eh_host_reset");

	if (cmd == NULL) {
		printk(KERN_INFO
			"%s(): **** SCSI mid-layer passing in NULL cmd\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

	ha = (scsi_qla_host_t *)cmd->host->hostdata;
	ha->eh_start= 0;
	/* Find actual ha */
	sp = (srb_t *)CMD_SP(cmd);
	if (ha->flags.failover_enabled && sp != NULL)
		ha = sp->ha;
	else
		ha = (scsi_qla_host_t *)cmd->host->hostdata;

	if (ha == NULL) {
		printk(KERN_INFO
			"%s(): **** CMD derives a NULL HA\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

	for (search_ha = qla2x00_hostlist;
		(search_ha != NULL) && search_ha != ha;
		search_ha = search_ha->next)
		continue;

	if (search_ha == NULL) {
		printk(KERN_INFO
			"%s(): **** CMD derives a NULL search HA\n",
			__func__);
                DEBUG2(BUG();)

		return (FAILED);
	}

	/* Display which one we're actually resetting for debug. */
	DEBUG(printk("qla2xxx_eh_host_reset: entered for scsi%ld. "
			"Resetting host_no %ld.\n", 
			((scsi_qla_host_t *)cmd->host->hostdata)->host_no,
			ha->host_no);)

#if  STOP_ON_RESET
	printk("Host Reset...  Command=\n");
	qla2x00_print_scsi_cmd(cmd);
	qla2x00_panic("qla2xxx_eh_host_reset", ha->host);
#endif

	/*
	 *  Now issue reset.
	 */
	b = SCSI_BUS_32(cmd);
	t = SCSI_TCN_32(cmd);
	l = SCSI_LUN_32(cmd);

	if (qla2x00_verbose) {
		printk(KERN_INFO
			"scsi(%ld:%d:%d:%d): now issue ADAPTER RESET.\n",
			((scsi_qla_host_t *)cmd->host->hostdata)->host_no,
			(int)b, 
			(int)t, 
			(int)l);
	}

	DEBUG2(printk(KERN_INFO
			"scsi(%ld:%d:%d:%d): now issue ADAPTER RESET "
			"to ha %ld.\n",
			((scsi_qla_host_t *)cmd->host->hostdata)->host_no,
			(int)b, (int)t, (int)l, ha->host_no);)

	if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags))) {
		set_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_unlock_irq(&io_request_lock);
#else
		spin_unlock_irq(ha->host->host_lock);
#endif

		if (qla2x00_abort_isp(ha, 1)) {
			/* failed. try later */
			set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
			return_status = FAILED;

			DEBUG2(printk(KERN_WARNING
					"scsi(%ld:%d:%d:%d): ha %ld "
					"ADAPTER RESET failed. Scheduled "
					"retry later.\n",
					((scsi_qla_host_t *)
						 cmd->host->hostdata)->host_no,
					(int)b, 
					(int)t, 
					(int)l,
					ha->host_no);)
		} else {
			return_status = SUCCESS;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif
		clear_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags);
	} else {
		/*
		 * Already active. Sleep a while then return SUCCESS for kernel
		 * to retry the IO.
		 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_unlock_irq(&io_request_lock);
#else
		spin_unlock_irq(ha->host->host_lock);
#endif

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(5 * HZ);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		spin_lock_irq(&io_request_lock);
#else
		spin_lock_irq(ha->host->host_lock);
#endif

		return_status = SUCCESS;
	}
	if ( return_status == FAILED) {
		DEBUG2(printk("qla2xxx_eh_host_reset Exiting: Reset Failed\n");)
		return FAILED;
	}

    /* Waiting for our command in done_queue to be returned to OS.*/
	if ( qla2x00_eh_wait_for_pending_commands(ha) == 0) {
		return_status = FAILED;
	}

	if(return_status == FAILED) {
		printk(KERN_INFO "qla2xxx_eh_host_reset Exiting: status=Failed\n");
		return FAILED;
	} else
		printk(KERN_INFO "qla2xxx_eh_host_reset Exiting: status=SUCCESS\n");

	LEAVE("qla2xxx_eh_host_reset");

#if EH_DEBUG
	my_reset_success = 1;
#endif

	return(return_status);
}

/**************************************************************************
* qla1200_biosparam
*
* Description:
*   Return the disk geometry for the given SCSI device.
**************************************************************************/
int
qla2x00_biosparam(Disk *disk, kdev_t dev, int geom[])
{
	int heads, sectors, cylinders;

	heads = 64;
	sectors = 32;
	cylinders = disk->capacity / (heads * sectors);
	if (cylinders > 1024) {
		heads = 255;
		sectors = 63;
		cylinders = disk->capacity / (heads * sectors);
	}

	geom[0] = heads;
	geom[1] = sectors;
	geom[2] = cylinders;

	return (0);
}

/**************************************************************************
* qla2x00_intr_handler
*
* Description:
*   Handles the actual interrupt from the adapter.
*
* Context: Interrupt
**************************************************************************/
void
qla2x00_intr_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned long flags = 0;
	unsigned long mbx_flags = 0;
	scsi_qla_host_t *ha;
	uint16_t    data;
	uint8_t     got_mbx = 0;
	device_reg_t *reg;
	unsigned long		intr_loop = 50; /* don't loop forever, interrupt are OFF */

	ENTER_INTR("qla2x00_intr_handler");

	ha = (scsi_qla_host_t *) dev_id;
	if (!ha) {
		printk(KERN_INFO
			"qla2x00_intr_handler: NULL host ptr\n");

		return;
	}
	qla2x00_stats.irqhba = ha;

	reg = ha->iobase;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	/* Check for pending interrupts. */
#if defined(ISP2100) || defined(ISP2200)
	while (((data = RD_REG_WORD(&reg->istatus)) & RISC_INT)
			&& intr_loop-- )
#else
	while (((data = RD_REG_WORD(&reg->host_status_lo)) & HOST_STATUS_INT)
			&& intr_loop-- )
#endif
	{
		ha->total_isr_cnt++;
		qla2x00_isr(ha, data, &got_mbx);
	}
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	if (test_bit(MBX_INTR_WAIT, &ha->mbx_cmd_flags) &&
		got_mbx && ha->flags.mbox_int) {
		/* There was a mailbox completion */
		DEBUG3(printk("qla2x00_intr_handler: going to "
				"get mbx reg lock.\n");)

		QLA_MBX_REG_LOCK(ha);
		MBOX_TRACE(ha,BIT_5);
		got_mbx = 0;

		if (ha->mcp == NULL) {
			DEBUG3(printk("qla2x00_intr_handler: error mbx "
					"pointer.\n");)
		} else {
			DEBUG3(printk("qla2x00_intr_handler: going to set mbx "
					"intr flags. cmd=%x.\n",
					ha->mcp->mb[0]);)
		}
		set_bit(MBX_INTERRUPT, &ha->mbx_cmd_flags);

		DEBUG3(printk("qla2x00_intr_handler(%ld): going to wake up "
				"mbx function for completion.\n",
				ha->host_no);)
		MBOX_TRACE(ha,BIT_6);
		up(&ha->mbx_intr_sem);

		DEBUG3(printk("qla2x00_intr_handler: going to unlock mbx "
				"reg.\n");)
		QLA_MBX_REG_UNLOCK(ha);
	}

	if (!list_empty(&ha->done_queue))
#if QLA2X_PERFORMANCE
		tasklet_schedule(&ha->run_qla_task);
#else
		qla2x00_done(ha);
#endif

	/* Wakeup the DPC routine */
	if ((!ha->flags.mbox_busy &&
		(test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags) ||
		 test_bit(RESET_MARKER_NEEDED, &ha->dpc_flags) ||
		 test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags) ) ) && 
		ha->dpc_wait && !ha->dpc_active) {  /* v2.19.4 */

		up(ha->dpc_wait);
	}

	LEAVE_INTR("qla2x00_intr_handler");
}


#if QLA2X_PERFORMANCE
/*
 * qla2x00_done_tasklet
 *
 * This is a task to process completion only similar to a
 * bottom half handler.
 *
 *      Input:
 *      p -- pointer to hba struct
 *
 */
void
qla2x00_done_tasklet(long p)
{
	scsi_qla_host_t *ha = (scsi_qla_host_t *) p;

	ENTER(__func__);

	set_bit(TASKLET_SCHED, &ha->dpc_flags);

	if (!list_empty(&ha->done_queue))
		qla2x00_done(ha);
	
	clear_bit(TASKLET_SCHED, &ha->dpc_flags);

	LEAVE(__func__);
}
#endif


/**************************************************************************
* qla2x00_do_dpc
*   This kernel thread is a task that is schedule by the interrupt handler
*   to perform the background processing for interrupts.
*
* Notes:
* This task always run in the context of a kernel thread.  It
* is kick-off by the driver's detect code and starts up
* up one per adapter. It immediately goes to sleep and waits for
* some fibre event.  When either the interrupt handler or
* the timer routine detects a event it will one of the task
* bits then wake us up.
**************************************************************************/
void
qla2x00_do_dpc(void *p)
{
#ifndef __VMWARE__
	DECLARE_MUTEX_LOCKED(sem);
#endif
	fcdev_t         dev;
	fc_port_t	*fcport;
	os_lun_t        *q;
	scsi_qla_host_t *ha = (scsi_qla_host_t *) p;
	srb_t           *sp;
	uint8_t		status;
	uint32_t        t;
	unsigned long	flags = 0;
	struct list_head *list, *templist;
	int	dead_cnt, online_cnt;

	ENTER(__func__);

#ifdef __VMWARE__
	/*
	 * We are not a real Linux thread so no need to handle all the
	 * task setup.
	 */
	printk("qla: DPC init\n");
	ha->wait_sema = (struct semaphore)__SEMAPHORE_INITIALIZER(ha->wait_sema, 0);
	ha->dpc_wait = &ha->wait_sema;
	ha->dpc_handler = (struct task_struct *)1;
#else
#if defined(MODULE)
	siginitsetinv(&current->blocked, SHUTDOWN_SIGS);
#else
	siginitsetinv(&current->blocked, 0);
#endif

	lock_kernel();

	/* Flush resources */
	daemonize();

	/*
	 * FIXME(dg) this is still a child process of the one that did
	 * the insmod.  This needs to be attached to task[0] instead.
	 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,9)
	/* As mentioned in kernel/sched.c(RA).....
	 * Reparent the calling kernel thread to the init task.
	 * 
	 * If a kernel thread is launched as a result of a system call,
	 * or if it ever exists,it should generally reparent itself to init
	 * so that it is correctly cleaned up on exit.
	 *
	 * The various task state such as scheduling policy and priority
	 * may have been inherited from a user process, so we reset them
	 * to sane values here.
	 *
	 * NOTE that reparent_to_init() gives the caller full capabilities.
	 *
	 */
	reparent_to_init();
#endif
#endif

	/*
	 * Set the name of this process.
	 */
	sprintf(current->comm, "%s_dpc%ld", DRIVER_NAME, ha->host_no);
	ha->dpc_wait = &sem;

	ha->dpc_handler = current;

	unlock_kernel();
#endif //__VMWARE__

	/*
	 * Wake up the thread that created us.
	 */
	DEBUG(printk("%s(): Wake up parent %d\n",
			__func__,
			ha->dpc_notify->count.counter);)

	up(ha->dpc_notify);

	while (1) {
		/*
		 * If we get a signal, it means we are supposed to go
		 * away and die.  This typically happens if the user is
		 * trying to unload a module.
		 */
		DEBUG3(printk("qla2x00: DPC handler sleeping\n");)

#ifdef __VMWARE__
		down_interruptible(ha->dpc_wait);

		if (ha->should_die)
			break;	/* get out */
#else
		down_interruptible(&sem);

		if (signal_pending(current))
			break;   /* get out */
#endif //__VMWARE__

		if (!list_empty(&ha->done_queue))
#if QLA2X_PERFORMANCE
			tasklet_schedule(&ha->run_qla_task);
#else
			qla2x00_done(ha);
#endif

		DEBUG3(printk("qla2x00: DPC handler waking up\n");)

		/* Initialization not yet finished. Don't do anything yet. */
		if (!ha->init_done || ha->dpc_active)
			continue;

		DEBUG3(printk("scsi(%ld): DPC handler\n", ha->host_no);)

		/* spin_lock_irqsave(&io_request_lock, ha->cpu_flags);*/
		ha->dpc_active = 1;

		/* Determine what action is necessary */

		/* Process commands in retry queue */
		if (test_and_clear_bit(PORT_RESTART_NEEDED, &ha->dpc_flags)) {
			DEBUG(printk("%s(%ld): DPC checking retry_q. "
					"total=%d\n",
					__func__,
					ha->host_no,
					ha->retry_q_cnt);)

			spin_lock_irqsave(&ha->list_lock, flags);
			dead_cnt = online_cnt = 0;
			list_for_each_safe(list, templist, &ha->retry_queue) {
				sp = list_entry(list, srb_t, list);
				q = sp->lun_queue;
				DEBUG3(printk("qla2x00_retry_q: pid=%ld "
						"sp=%p, spflags=0x%x, "
						"q_flag= 0x%lx\n",
						sp->cmd->serial_number,
						sp,
						sp->flags,
						q->q_flag);)

				if (q == NULL)
					continue;
				fcport = q->fclun->fcport;

				if (atomic_read(&fcport->state) == 
					FC_DEVICE_DEAD) {

					__del_from_retry_queue(ha, sp);
					CMD_RESULT(sp->cmd) = 
						DID_NO_CONNECT << 16;
					CMD_HANDLE(sp->cmd) = 
						(unsigned char *) NULL;
					__add_to_done_queue(ha, sp);
					dead_cnt++;
				} else if (atomic_read(&fcport->state) != 
						FC_DEVICE_LOST) {

					__del_from_retry_queue(ha, sp);
					CMD_RESULT(sp->cmd) = 
						DID_BUS_BUSY << 16;
					CMD_HANDLE(sp->cmd) = 
						(unsigned char *) NULL;
					__add_to_done_queue(ha, sp);
					online_cnt++;
				}
			} /* list_for_each_safe() */
			spin_unlock_irqrestore(&ha->list_lock, flags);

			DEBUG(printk("%s(%ld): done processing retry queue - "
					"dead=%d, online=%d\n ",
					__func__,
					ha->host_no,
					dead_cnt,
					online_cnt);)
		}
		/* Process commands in scsi retry queue */
		if (test_and_clear_bit(SCSI_RESTART_NEEDED, &ha->dpc_flags)) {
			/*
			 * Any requests we want to delay for some period is put
			 * in the scsi retry queue with a delay added. The
			 * timer will schedule a "scsi_restart_needed" every 
			 * second as long as there are requests in the scsi
			 * queue. 
			 */
			DEBUG(printk("%s(%ld): DPC checking scsi "
					"retry_q.total=%d\n",
					__func__,
					ha->host_no,
					ha->scsi_retry_q_cnt);)

			online_cnt = 0;
			spin_lock_irqsave(&ha->list_lock, flags);
			list_for_each_safe(list,
						templist,
						&ha->scsi_retry_queue) {

				sp = list_entry(list, srb_t, list);
				q = sp->lun_queue;

				DEBUG3(printk("qla2x00_scsi_retry_q: pid=%ld "
						"sp=%p, spflags=0x%x, "
						"q_flag= 0x%lx,q_state=%d\n",
						sp->cmd->serial_number,
						sp,
						sp->flags,
						q->q_flag,
						q->q_state);)

				/* Was this lun suspended */
				if (q->q_state != LUN_STATE_WAIT) {
					online_cnt++;
					__del_from_scsi_retry_queue(ha, sp);
					__add_to_retry_queue(ha,sp);
				}

				/* Was this command suspended for N secs */
				if (sp->delay != 0) {
					sp->delay--;
					if (sp->delay == 0) {
						online_cnt++;
						__del_from_scsi_retry_queue(
								ha, sp);
						__add_to_retry_queue(ha,sp);
					}
				}
			}
			spin_unlock_irqrestore(&ha->list_lock, flags);

			DEBUG(if (online_cnt > 0))
			DEBUG(printk("scsi%ld: dpc() found scsi reqs "
					"to restart= %d\n",
					ha->host_no, online_cnt););
		}

		/* Process any pending mailbox commands */
		if (!ha->flags.mbox_busy) {
			if (test_and_clear_bit(ISP_ABORT_NEEDED,
						&ha->dpc_flags)) {

				DEBUG(printk("scsi%ld: dpc: sched "
						"qla2x00_abort_isp ha = %p\n",
						ha->host_no, ha);)
				if (!(test_and_set_bit(ABORT_ISP_ACTIVE,
							&ha->dpc_flags))) {

					if (qla2x00_abort_isp(ha, 0)) {
						/* failed. retry later */
						set_bit(ISP_ABORT_NEEDED,
								&ha->dpc_flags);
					}
					clear_bit(ABORT_ISP_ACTIVE,
							&ha->dpc_flags);
				}
				DEBUG(printk("scsi%ld: dpc: qla2x00_abort_isp "
						"end\n",
						ha->host_no);)
			}

			if (test_and_clear_bit(LOOP_RESET_NEEDED,
						&ha->dpc_flags)) {

				DEBUG(printk("dpc: loop_reset_needed(%ld) "
						"calling loop_reset.\n",
						ha->host_no);)

				qla2x00_loop_reset(ha);
			}
			if (test_and_clear_bit(DEVICE_ABORT_NEEDED,
						&ha->dpc_flags)) {

				DEBUG(printk("dpc: device_abort_needed(%ld) "
						"calling device_abort.\n",
						ha->host_no);)

				t = ha->reset_tgt_id;
				if (ha->otgt[t] && ha->otgt[t]->vis_port)
					qla2x00_abort_device(ha,
						ha->otgt[t]->vis_port->loop_id,
						ha->reset_lun);
			}

			if (test_and_clear_bit(RESET_MARKER_NEEDED,
						&ha->dpc_flags)) {

				if (!(test_and_set_bit(RESET_ACTIVE,
							&ha->dpc_flags))) {

					DEBUG(printk("dpc(%ld): "
						"qla2x00_reset_marker \n",
						ha->host_no);)

					qla2x00_rst_aen(ha);
					clear_bit(RESET_ACTIVE, &ha->dpc_flags);
				}
			}

			/* v2.19.8 Retry each device up to login retry count */
			if ((test_and_clear_bit(RELOGIN_NEEDED,
							&ha->dpc_flags)) &&
				!test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags) &&
				ha->loop_state != LOOP_DOWN) { /* v2.19.5 */

				DEBUG(printk("dpc%ld: qla2x00_port_login\n",
						ha->host_no);)

				for (fcport = ha->fcport;
					fcport != NULL;
					fcport = fcport->next) {
					
					/*
					 * If the port is not ONLINE then try
					 * to login to it if we haven't run
					 * out of retries.
					 */
					if (atomic_read(&fcport->state) != FC_ONLINE &&
						fcport->login_retry) {

						fcport->login_retry--;
						memset(&dev, 0, sizeof(fcdev_t));
						dev.loop_id = fcport->old_loop_id;
						dev.d_id.b24 = fcport->d_id.b24;
						if(ha->fc_db[fcport->dev_id].flag & DEV_PUBLIC)	
							status = qla2x00_fabric_login(ha, &dev);
						else 	
							status = qla2x00_local_device_login(ha, (dev.loop_id & 0xff));

						if (status == QL_STATUS_SUCCESS) {
							ha->fc_db[fcport->dev_id].loop_id = dev.loop_id;
							fcport->loop_id = dev.loop_id;
							fcport->old_loop_id = dev.loop_id;

							DEBUG(printk("dpc%ld port login OK: logged in ID 0x%x\n",
									ha->host_no, fcport->loop_id);)
							
							fcport->port_login_retry_count = ha->port_down_retry_count *
												PORT_RETRY_TIME;
							atomic_set(&fcport->state, FC_ONLINE);
							atomic_set(&fcport->port_down_timer,
									ha->port_down_retry_count * PORT_RETRY_TIME);

							fcport->login_retry = 0;
						} else if (status == 1) {
							if (dev.loop_id != fcport->old_loop_id) {
								fcport->old_loop_id = dev.loop_id;
								ha->fc_db[fcport->dev_id].loop_id = dev.loop_id;
							}

							set_bit(RELOGIN_NEEDED, &ha->dpc_flags);
							/* retry the login again */
							DEBUG(printk("dpc: Retrying %d login again loop_id 0x%x\n",
									fcport->login_retry, fcport->loop_id);)
						} else {
							fcport->login_retry = 0;
						}
					}
					if (test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags))
						break;
				}
				DEBUG(printk("dpc%ld: qla2x00_port_login - end\n",
						ha->host_no);)
			}

			/* v2.19.5 */
			if ((test_bit(LOGIN_RETRY_NEEDED, &ha->dpc_flags)) &&
				ha->loop_state != LOOP_DOWN ) { /* v2.19.5 */

				clear_bit(LOGIN_RETRY_NEEDED, &ha->dpc_flags);
				DEBUG(printk("dpc(%ld): qla2x00_login_retry\n",
						ha->host_no);)
					
				set_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags);

				DEBUG(printk("dpc: qla2x00_login_retry end.\n");)
			}

			/* v2.19.5b5 */
			if (test_and_clear_bit(LOOP_RESYNC_NEEDED,
						&ha->dpc_flags)) {

				DEBUG(printk("dpc(%ld): qla2x00_LOOP_RESYNC\n",
						ha->host_no);)

				if (!(test_and_set_bit(LOOP_RESYNC_ACTIVE,
							&ha->dpc_flags))) {

					qla2x00_loop_resync(ha);

					clear_bit(LOOP_RESYNC_ACTIVE,
							&ha->dpc_flags);

				}
				DEBUG(printk("dpc(%ld): qla2x00_LOOP_RESYNC "
						"done\n",
						ha->host_no);)
			}

			if (ha->flags.failover_enabled) {
				/*
				 * If we are not processing a ioctl or one of
				 * the ports are still MISSING or need a resync
				 * then process the failover event.
				*/  
				if (!test_bit(CFG_ACTIVE, &ha->cfg_flags)) {

					if (qla2x00_check_for_devices_online(ha)) {
						if (test_and_clear_bit(FAILOVER_EVENT,
								&ha->dpc_flags)) {

							DEBUG(printk("dpc(%ld): "
								"qla2x00_cfg_event_notify\n",
								ha->host_no);)

							if (ha->flags.online) {
								qla2x00_cfg_event_notify(ha, ha->failover_type);
							}

							DEBUG(printk("dpc(%ld): "
								"qla2x00_cfg_event_notify - done\n",
								ha->host_no);)
						}
					}

					if (test_and_clear_bit(FAILOVER_NEEDED,
								&ha->dpc_flags)) {

						/*
						 * Get any requests from failover queue
						 */
						DEBUG(printk("dpc: qla2x00_process "
								"failover\n");)

						qla2x00_process_failover(ha);

						DEBUG(printk("dpc: qla2x00_process "
								"failover - done\n");)
					}
				}
			}

			if (test_bit(RESTART_QUEUES_NEEDED, &ha->dpc_flags)) {
				DEBUG(printk("dpc: qla2x00_restart_queues\n");)

				qla2x00_restart_queues(ha,FALSE);

				DEBUG(printk("dpc: qla2x00_restart_queues "
						"- done\n");)
			}

			if (test_bit(ABORT_QUEUES_NEEDED, &ha->dpc_flags)) {
				DEBUG(printk("dpc:(%ld) "
					"qla2x00_abort_queues\n", ha->host_no);)
					
				qla2x00_abort_queues(ha, FALSE);
			}
			if (!ha->interrupts_on)
				qla2x00_enable_intrs(ha);
		}

		if (!list_empty(&ha->done_queue))
#if QLA2X_PERFORMANCE
			tasklet_schedule(&ha->run_qla_task);
#else
			qla2x00_done(ha);
#endif

		/* spin_unlock_irqrestore(&io_request_lock, ha->cpu_flags);*/

		ha->dpc_active = 0;

		/* The spinlock is really needed up to this point. (DB) */
	} /* End of while(1) */

	DEBUG(printk("dpc: DPC handler exiting\n");)

	/*
	 * Make sure that nobody tries to wake us up again.
	 */
	ha->dpc_wait = NULL;
	ha->dpc_handler = NULL;
	ha->dpc_active = 0;

	/*
	 * If anyone is waiting for us to exit (i.e. someone trying to unload a
	 * driver), then wake up that process to let them know we are on the
	 * way out the door.  This may be overkill - I *think* that we could
	 * probably just unload the driver and send the signal, and when the
	 * error handling thread wakes up that it would just exit without
	 * needing to touch any memory associated with the driver itself.
	 */
	if (ha->dpc_notify != NULL)
		up(ha->dpc_notify);

	LEAVE(__func__);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
/**************************************************************************
* qla2x00_device_queue_depth
*   Determines the queue depth for a given device.  There are two ways
*   a queue depth can be obtained for a tagged queueing device.  One
*   way is the default queue depth which is determined by whether
*   If it is defined, then it is used
*   as the default queue depth.  Otherwise, we use either 4 or 8 as the
*   default queue depth (dependent on the number of hardware SCBs).
**************************************************************************/
void
qla2x00_device_queue_depth(scsi_qla_host_t *p, Scsi_Device *device)
{
	int default_depth = 32;

	device->queue_depth = default_depth;
	if (device->tagged_supported) {
		device->tagged_queue = 1;
		device->current_tag = 0;
#if defined(MODULE)
		if (!(ql2xmaxqdepth == 0 || ql2xmaxqdepth > 256))
			device->queue_depth = ql2xmaxqdepth;
#endif

		printk(KERN_INFO
			"scsi(%ld:%d:%d:%d): Enabled tagged queuing, "
			"queue depth %d.\n",
			p->host_no,
			device->channel,
			device->id,
			device->lun, 
			device->queue_depth);
	}

}

/**************************************************************************
*   qla2x00_select_queue_depth
*
* Description:
*   Sets the queue depth for each SCSI device hanging off the input
*   host adapter.  We use a queue depth of 2 for devices that do not
*   support tagged queueing.
**************************************************************************/
STATIC void
qla2x00_select_queue_depth(struct Scsi_Host *host, Scsi_Device *scsi_devs)
{
	Scsi_Device *device;
	scsi_qla_host_t  *p = (scsi_qla_host_t *) host->hostdata;

	ENTER(__func__);

	for (device = scsi_devs; device != NULL; device = device->next) {
		if (device->host == host)
			qla2x00_device_queue_depth(p, device);
	}

	LEAVE(__func__);
}
#endif

#if defined (CONFIG_SCSIFCHOTSWAP) || defined(CONFIG_GAMAP)
union wwnmap {
	unsigned long long wwn;
	unsigned char wwn_u8[8];
};

int qla2x00_get_scsi_info_from_wwn (int mode,
	unsigned long long wwn,
	int *host,
	int *channel,
	int *lun,
	int *id) {

scsi_qla_host_t *list;
Scsi_Device *scsi_device;
union wwnmap wwncompare;
union wwnmap wwncompare2;
int i, j, k;

	/*
	 * Retrieve big endian version of world wide name
	 */
	wwncompare2.wwn = wwn;
	for (j = 0, k=7; j < 8; j++, k--) {
		wwncompare.wwn_u8[j] = wwncompare2.wwn_u8[k];
	}

	/*
	 * query all hosts searching for WWN
	 */
	for (list = qla2x00_hostlist; list; list = list->next) {
		for (i = 0; i < MAX_FIBRE_DEVICES; i++) {
			/*
			 * Scan all devices in FibreChannel database
			 * if WWN match found, return SCSI device information
			 */
			if (memcmp (wwncompare.wwn_u8, list->fc_db[i].name, 8) == 0) {
				/*
				 * If inserting, avoid scan for channel and lun information
				 */
				if (mode == 0) {
					*channel = 0;
					*lun = 0;
					*host = list->host->host_no;
					*id = i;
					return (0);
				}
			

				/*
				 * WWN matches, find channel and lun information from scsi
				 * device
				 */
				for (scsi_device = list->host->host_queue; scsi_device; scsi_device = scsi_device->next) {
					if (scsi_device->id == i) {
						*channel = scsi_device->channel;
						*lun = scsi_device->lun;
						break;
					}
				}
				if (scsi_device == 0) {
					return (-ENOENT);
				}
				/*
				 * Device found, return all data
				 */
				*host = list->host->host_no;
				*id = i;
				return (0);
			} /* memcmp */
		} /* i < MAXFIBREDEVICES */
	}
	return (-ENOENT);
}

int qla2x00_get_wwn_from_scsi_info (int host, int id, unsigned long long *wwn) {
scsi_qla_host_t *list;
union wwnmap wwnendian;
union wwnmap wwnendian2;
int j, k;

	/*
	 * Examine all QLogic hosts
	 */
	for (list = qla2x00_hostlist; list; list = list->next) {
		if (host == list->host->host_no) {
			/*
			 * Get endian corrected 64 bit WWN
			 */

			memcpy (&wwnendian2.wwn, list->fc_db[id].name, 8);
			for (j = 0, k=7; j < 8; j++, k--) {
				wwnendian.wwn_u8[j] = wwnendian2.wwn_u8[k];
			}
			*wwn = wwnendian.wwn;
			return (0);
		}
	}
	return (-ENOENT);
}
#endif /* CONFIG_SCSIFCHOTSWAP || CONFIG_GAMAP */

/**************************************************************************
* ** Driver Support Routines **
*
* qla2x00_enable_intrs
* qla2x00_disable_intrs
**************************************************************************/
static inline void 
qla2x00_enable_intrs(scsi_qla_host_t *ha)
{
	unsigned long flags = 0;
	device_reg_t *reg;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	reg = ha->iobase;
	ha->interrupts_on = 1;
	/* enable risc and host interrupts */
	WRT_REG_WORD(&reg->ictrl, (ISP_EN_INT+ ISP_EN_RISC));
	CACHE_FLUSH(&reg->ictrl);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);
}

static inline void 
qla2x00_disable_intrs(scsi_qla_host_t *ha)
{
	unsigned long flags = 0;
	device_reg_t *reg;

	spin_lock_irqsave(&ha->hardware_lock, flags);
	reg = ha->iobase;
	ha->interrupts_on = 0;
	/* disable risc and host interrupts */
	WRT_REG_WORD(&reg->ictrl, 0);
	CACHE_FLUSH(&reg->ictrl);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);
}


STATIC inline void 
qla2x00_delete_from_done_queue(scsi_qla_host_t *dest_ha, srb_t *sp) 
{
	/* remove command from done list */
	list_del_init(&sp->list);
	dest_ha->done_q_cnt--;
	sp->state = SRB_NO_QUEUE_STATE;

	if (sp->flags & SRB_DMA_VALID) {
		sp->flags &= ~SRB_DMA_VALID;

#ifndef __VMWARE__
		/* Release memory used for this I/O */
		if (sp->cmd->use_sg) {
			pci_unmap_sg(dest_ha->pdev,
					sp->cmd->request_buffer,
					sp->cmd->use_sg,
					scsi_to_pci_dma_dir(
						sp->cmd->sc_data_direction));
		} else if (sp->cmd->request_bufflen) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,13)
			pci_unmap_page(dest_ha->pdev,
					sp->saved_dma_handle,
					sp->cmd->request_bufflen,
					scsi_to_pci_dma_dir(
						sp->cmd->sc_data_direction));
#else
			pci_unmap_single(dest_ha->pdev,
					sp->saved_dma_handle,
					sp->cmd->request_bufflen,
					scsi_to_pci_dma_dir(
						sp->cmd->sc_data_direction));
#endif
		}
#endif
	}
}

/**************************************************************************
* qla2x00_done
*      Process completed commands.
*
* Input:
*      old_ha           = adapter block pointer.
*
* Returns:
* int     
**************************************************************************/
STATIC int
qla2x00_done(scsi_qla_host_t *old_ha)
{
	srb_t           *sp;
	os_lun_t	*lq;
	Scsi_Cmnd	*cmd;
	unsigned long	flags = 0;
	scsi_qla_host_t	*ha;
	scsi_qla_host_t	*vis_ha;
	int	cnt;
	int	send_marker_once = 0;
	srb_t *done_queue_first = NULL;
	srb_t *done_queue_last = NULL;

	ENTER(__func__);

	if (test_bit(DONE_RUNNING, &old_ha->dpc_flags))
		return (0);

	set_bit(DONE_RUNNING, &old_ha->dpc_flags);
	cnt = 0;

	/*
	 * Get into local queue such that we do not wind up calling done queue
	 * takslet for the same IOs from DPC or any other place.
	 */
	spin_lock_irqsave(&old_ha->list_lock,flags);
	while (!list_empty(&old_ha->done_queue)) {
		sp = list_entry(old_ha->done_queue.next, srb_t, list);
		/* remove command from done list */
		list_del_init(&sp->list);

		old_ha->done_q_cnt--;
		sp->s_next = NULL;
        	sp->state = SRB_NO_QUEUE_STATE;
		/* insert in local queue */
		if (done_queue_first == NULL) {
			done_queue_first = sp;
			done_queue_last = sp;
		} else {
			done_queue_last->s_next = sp;
			done_queue_last = sp;
		}
	} /* end of while list_empty(&ha->done_queue) */
	spin_unlock_irqrestore(&old_ha->list_lock, flags);

	/*
	 * All done commands are in local queue. Now do the call back
	 */
	while ((sp = done_queue_first) != NULL) {
		done_queue_first = sp->s_next;
		if (sp->s_next == NULL)
			done_queue_last = NULL;
		sp->s_next = NULL;

		cnt++;

		cmd = sp->cmd;
		if (cmd == NULL) {
#if  0
			panic("qla2x00_done: SP %p already freed - %s %d.\n",
			    sp, __FILE__,__LINE__);
#else
		 	continue;
#endif
		}

		vis_ha = (scsi_qla_host_t *)cmd->host->hostdata;
		lq = sp->lun_queue;
		ha = lq->fclun->fcport->ha;

		if (sp->flags & SRB_DMA_VALID) {
			sp->flags &= ~SRB_DMA_VALID;

			/* 4.10   64 and 32 bit */
			/* Release memory used for this I/O */
#ifndef __VMWARE__
			if (cmd->use_sg) {
				pci_unmap_sg(ha->pdev,
				    cmd->request_buffer,
				    cmd->use_sg,
				    scsi_to_pci_dma_dir(
					    cmd->sc_data_direction));
			} else if (cmd->request_bufflen) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,13)
				pci_unmap_page(ha->pdev,
					sp->saved_dma_handle,
					cmd->request_bufflen,
					scsi_to_pci_dma_dir(
						cmd->sc_data_direction));
#else
				pci_unmap_single(ha->pdev,
				    sp->saved_dma_handle,
				    cmd->request_bufflen,
				    scsi_to_pci_dma_dir(
					    cmd->sc_data_direction));
#endif
			}
#endif
		}

		if (!(sp->flags & SRB_IOCTL) &&
			ha->flags.failover_enabled) {
			/*
			 * This routine checks for DID_NO_CONNECT to decide
			 * whether to failover to another path or not. We only
			 * failover on that status.
			 */
			if (qla2x00_fo_check(ha,sp)) {
				if ((sp->state != SRB_FAILOVER_STATE)) {
					/*
					 * Retry the command on this path
					 * several times before selecting a new
					 * path.
					 */
					add_to_pending_queue_head(vis_ha, sp);
					qla2x00_next(vis_ha);
				}
				else {
					/* we failover this path */
					qla2x00_extend_timeout(sp->cmd,
							EXTEND_CMD_TIMEOUT);
				}
				continue;
			}
			
		}

		switch ((CMD_RESULT(cmd)>>16)) {

			case DID_OK:
			case DID_ERROR:
				break;

			case DID_RESET:
				/*
				 * set marker needed, so we don't have to
				 * send multiple markers
				 */

				/* ra 01/10/02 */
				if (!send_marker_once) {
					ha->marker_needed = 1;
					send_marker_once++;
				}

				/*
				 * WORKAROUND
				 *
				 * A backdoor device-reset requires different
				 * error handling.  This code differentiates
				 * between normal error handling and the
				 * backdoor method.
				 *
				 */
				if (ha->host->eh_active != EH_ACTIVE)
					CMD_RESULT(sp->cmd) =
						DID_BUS_BUSY << 16;
				break;


			case DID_ABORT:
				sp->flags &= ~SRB_ABORT_PENDING;
				sp->flags |= SRB_ABORTED;

				if (sp->flags & SRB_TIMEOUT)
					CMD_RESULT(cmd)= DID_TIME_OUT << 16;

				break;

			default:
				DEBUG2(printk("scsi(%ld:%d:%d) %s: did_error "
						"= %d, comp-scsi= 0x%x-0x%x.\n",
				vis_ha->host_no,
				SCSI_TCN_32(cmd),
				SCSI_LUN_32(cmd),
				__func__,
				(CMD_RESULT(cmd)>>16),
				CMD_COMPL_STATUS(cmd),
				CMD_SCSI_STATUS(cmd));)
				break;
		}

		/*
		 * Call the mid-level driver interrupt handler -- via sp_put()
		 */
		sp_put(ha, sp);

		qla2x00_next(vis_ha);

	} /* end of while */
	clear_bit(DONE_RUNNING, &old_ha->dpc_flags);

	LEAVE(__func__);

	return (cnt);
}

STATIC uint8_t
qla2x00_suspend_lun(scsi_qla_host_t *ha, os_lun_t *lq, int time, int count)
{
	return (__qla2x00_suspend_lun(ha, lq, time, count, 0));
}

STATIC uint8_t
qla2x00_delay_lun(scsi_qla_host_t *ha, os_lun_t *lq, int time)
{
	return (__qla2x00_suspend_lun(ha, lq, time, 1, 1));
}

/*
 *  qla2x00_suspend_lun
 *	Suspend lun and start port down timer
 *
 * Input:
 *	ha = visable adapter block pointer.
 *  lq = lun queue
 *  cp = Scsi command pointer 
 *  time = time in seconds
 *  count = number of times to let time expire
 *  delay_lun = non-zero, if lun should be delayed rather than suspended
 *
 * Return:
 *     QL_STATUS_SUCCESS  -- suspended lun 
 *     QL_STATUS_ERROR  -- Didn't suspend lun
 *
 * Context:
 *	Interrupt context.
 */
STATIC uint8_t
__qla2x00_suspend_lun(scsi_qla_host_t *ha,
		os_lun_t *lq, int time, int count, int delay_lun)
{
	srb_t *sp;
	struct list_head *list, *temp;
	unsigned long flags;
	uint8_t	status;

	/* if the lun_q is already suspended then don't do it again */
	if (lq->q_state == LUN_STATE_READY ||
		lq->q_state == LUN_STATE_RUN) {

		spin_lock_irqsave(&lq->q_lock, flags);
		if (lq->q_state == LUN_STATE_READY) {
			lq->q_max = count;
			lq->q_count = 0;
		}
		/* Set the suspend time usually 6 secs */
		atomic_set(&lq->q_timer, time);

		/* now suspend the lun */
		lq->q_state = LUN_STATE_WAIT;

		if (delay_lun) {
			set_bit(LUN_EXEC_DELAYED, &lq->q_flag);
			DEBUG(printk(KERN_INFO 
					"scsi%ld: Delay lun execution for %d "
					"secs, count=%d, max count=%d, "
					"state=%d\n",
					ha->host_no,
					time,
					lq->q_count,
					lq->q_max,
					lq->q_state);)
		} else {
			DEBUG(printk(KERN_INFO 
					"scsi%ld: Suspend lun for %d secs, "
					"count=%d, max count=%d, state=%d\n",
					ha->host_no,
					time,
					lq->q_count,
					lq->q_max,
					lq->q_state);)
		}
		spin_unlock_irqrestore(&lq->q_lock, flags);

		/*
		 * Remove all pending commands from request queue and  put them
		 * in the scsi_retry queue.
		 */
		spin_lock_irqsave(&ha->list_lock, flags);
		list_for_each_safe(list, temp, &ha->pending_queue) {
			sp = list_entry(list, srb_t, list);
			if (sp->lun_queue != lq)
				continue;

			__del_from_pending_queue(ha, sp);

			if( sp->cmd->allowed < count)
				sp->cmd->allowed = count;
			__add_to_scsi_retry_queue(ha,sp);

		} /* list_for_each_safe */
		spin_unlock_irqrestore(&ha->list_lock, flags);
		status = QL_STATUS_SUCCESS;
	} else
		status = QL_STATUS_ERROR;
	return( status );

}

/*
 *  qla2x00_flush_failover_queue
 *	Return cmds of a "specific" LUN from the failover queue with
 *      DID_BUS_BUSY status.
 *
 * Input:
 *	ha = adapter block pointer.
 *      q  = lun queue.
 *
 * Context:
 *	Interrupt context.
 */
void
qla2x00_flush_failover_q(scsi_qla_host_t *ha, os_lun_t *q)
{
	srb_t  *sp;
	struct list_head *list, *temp;
	unsigned long flags;

	spin_lock_irqsave(&ha->list_lock, flags);
	list_for_each_safe(list, temp, &ha->failover_queue) {
		sp = list_entry(list, srb_t, list);
		/*
		 * If request originated from the same lun_q then delete it
		 * from the failover queue 
		 */
		if (q == sp->lun_queue) {
			/* Remove srb from failover queue. */
			__del_from_failover_queue(ha,sp);
			CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;
			CMD_HANDLE(sp->cmd) = (unsigned char *) NULL;
			__add_to_done_queue(ha, sp);
		}
	} /* list_for_each_safe() */
	spin_unlock_irqrestore(&ha->list_lock, flags);
}

/*
 *  qla2x00_check_sense
 *
 * Input:
 * cp = SCSI command structure
 * lq = lun queue
 *
 * Return:
 *     QL_STATUS_SUCCESS  -- Lun suspended 
 *     QL_STATUS_ERROR  -- Lun not suspended
 *
 * Context:
 *	Interrupt context.
 */
STATIC uint8_t 
qla2x00_check_sense(Scsi_Cmnd *cp, os_lun_t *lq)
{
	scsi_qla_host_t *ha = (scsi_qla_host_t *) cp->host->hostdata;
	srb_t		*sp;
	fc_port_t	*fcport;

	ha = ha;
	if (((cp->sense_buffer[0] & 0x70) >> 4) != 7) {
		return QL_STATUS_ERROR;
	}

	sp = (srb_t * )CMD_SP(cp);
	sp->flags |= SRB_GOT_SENSE;

	switch (cp->sense_buffer[2] & 0xf) {
		case RECOVERED_ERROR:
			CMD_RESULT(cp)  = DID_OK << 16;
			cp->sense_buffer[0] = 0;
			break;

		case NOT_READY:
			/*
			 * if current suspend count is greater than max suspend
			 * count then no more suspends. 
			 */
			fcport = lq->fclun->fcport;
			/*
			 * Suspend the lun only for hard disk device type.
			 */
			if (!(fcport->flags & FC_TAPE_DEVICE) &&
				lq->q_state != LUN_STATE_TIMEOUT) {

#if defined(COMPAQ)
				/* COMPAQ*/
				if ((lq->q_flag & LUN_SCSI_SCAN_DONE)) {
					DEBUG(printk(
						"scsi%ld: check_sense: "
						"lun%d, suspend count="
						"%d, max count=%d\n",
						ha->host_no,
						(int)SCSI_LUN_32(cp),
						lq->q_count,
						lq->q_max);)

					/*
					 * HSG80 can take awhile to
					 * become ready.
					 */
					if (cp->allowed != HSG80_SUSPEND_COUNT)
						cp->allowed =
							HSG80_SUSPEND_COUNT;
					qla2x00_suspend_lun(ha, lq, 6,
							HSG80_SUSPEND_COUNT);

					return (QL_STATUS_SUCCESS);
				}
#else
				/* non-COMPAQ*/
				/*
				 * if target is "in process of being 
				 * ready then suspend lun for 6 secs and
				 * retry all the commands.
				 */
				if ((cp->sense_buffer[12] == 0x4 &&
					cp->sense_buffer[13] == 0x1)) {

					/* Suspend the lun for 6 secs */
					qla2x00_suspend_lun(ha, lq, 6,
					    ql2xsuspendcount);

					return (QL_STATUS_SUCCESS);
				}
#endif /* COMPAQ */

			} /* EO if (lq->q_state != LUN_STATE_TIMEOUT )*/

			break;
	} /* end of switch */

	return (QL_STATUS_ERROR);
}

/**************************************************************************
*   qla2x00_timer
*
* Description:
*   One second timer
*
* Context: Interrupt
***************************************************************************/
STATIC void
qla2x00_timer(scsi_qla_host_t *ha)
{
	int		t,l;
	unsigned long	cpu_flags = 0;
	fc_port_t	*fcport;
	os_lun_t *lq;
	os_tgt_t *tq;
	int		start_dpc = 0;

	/*
	 * We try and restart any request in the retry queue every second.
	 */
	if (!list_empty(&ha->retry_queue)) {
		set_bit(PORT_RESTART_NEEDED, &ha->dpc_flags);
		start_dpc++;
	}

	/*
	 * We try and restart any request in the scsi_retry queue every second.
	 */
	if (!list_empty(&ha->scsi_retry_queue)) {
		set_bit(SCSI_RESTART_NEEDED, &ha->dpc_flags);
		start_dpc++;
	}

	/*
	 * We try and failover any request in the failover queue every second.
	 */
	if (!list_empty(&ha->failover_queue)) {
		set_bit(FAILOVER_NEEDED, &ha->dpc_flags);
		start_dpc++;
	}

	/*
	 * Ports - Port down timer.
	 *
	 * Whenever, a port is in the LOST state we start decrementing its port
	 * down timer every second until it reaches zero. Once  it reaches zero
	 * the port it marked DEAD. 
	 */
	for (t=0, fcport = ha->fcport; 
		fcport != NULL;
		fcport = fcport->next, t++) {

		if (atomic_read(&fcport->state) == FC_DEVICE_LOST) {

			if (atomic_read(&fcport->port_down_timer) == 0)
				continue;

			if (atomic_dec_and_test(&fcport->port_down_timer) != 0) 
				atomic_set(&fcport->state, FC_DEVICE_DEAD);
			
			DEBUG(printk("scsi%ld: fcport-%d - port retry count "
					":%d remainning\n",
					ha->host_no, 
					t,
					atomic_read(&fcport->port_down_timer));)
		}
	} /* End of for fcport  */

	/*
	 * LUNS - lun suspend timer.
	 *
	 * Whenever, a lun is suspended the timer starts decrementing its
	 * suspend timer every second until it reaches zero. Once  it reaches
	 * zero the lun retry count is decremented. 
	 */

	/*
	 * FIXME(dg) - Need to convert this linear search of luns into a search
	 * of a list of suspended luns.
	 */
	for (t = 0; t < ha->max_targets; t++) {
		if ((tq = ha->otgt[t]) == NULL)
			continue;

		for (l = 0; l < ha->max_luns; l++) {
			if ((lq = (os_lun_t *) tq->olun[l]) == NULL)
				continue;

			spin_lock_irqsave(&lq->q_lock, cpu_flags);
			if (lq->q_state == LUN_STATE_WAIT &&
				atomic_read(&lq->q_timer) != 0) {

				if (atomic_dec_and_test(&lq->q_timer) != 0) {
					/*
					 * A delay should immediately
					 * transition to a READY state
					 */
					if (test_and_clear_bit(LUN_EXEC_DELAYED,
								&lq->q_flag)) {
						lq->q_state = LUN_STATE_READY;
					}
					else {
						lq->q_count++;
						if (lq->q_count == lq->q_max)
							lq->q_state =
							      LUN_STATE_TIMEOUT;
						else
							lq->q_state =
								LUN_STATE_RUN;
					}
				}
				DEBUG3(printk("scsi%ld: lun%d - timer %d, "
						"count=%d, max=%d, state=%d\n",
						ha->host_no, 
						l, 
						atomic_read(&lq->q_timer),
						lq->q_count,
						lq->q_max,
						lq->q_state);)
			}
			spin_unlock_irqrestore(&lq->q_lock, cpu_flags);
		} /* End of for luns  */
	} /* End of for targets  */

	/* Loop down handler. */
	if (atomic_read(&ha->loop_down_timer) > 0 && 
		!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) &&
		ha->flags.online) {

		/* dg 10/30 if (atomic_read(&ha->loop_down_timer) == LOOP_DOWN_TIME) { */
		if (atomic_read(&ha->loop_down_timer) == 
			ha->loop_down_abort_time ) {
			DEBUG(printk("qla%ld: Loop Down - aborting the queues "
					"before time expire\n",
					ha->instance);)

			set_bit(ABORT_QUEUES_NEEDED, &ha->dpc_flags);
			start_dpc++;
		}

		/* if the loop has been down for 4 minutes, reinit adapter */
		if (atomic_dec_and_test(&ha->loop_down_timer) != 0) {
			DEBUG(printk("qla%ld: Loop down exceed 4 mins - "
					"restarting queues.\n",
					ha->instance);)

			set_bit(RESTART_QUEUES_NEEDED, &ha->dpc_flags);
			start_dpc++;
			if (!(ha->device_flags & DFLG_NO_CABLE) &&
			     qla2x00_reinit && !ha->flags.failover_enabled) {
				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
			DEBUG(printk("qla%ld: Loop down - aborting ISP.\n",
					ha->instance);)
			}
		}
		DEBUG3(printk("qla%ld: Loop Down - seconds remainning %d\n",
				ha->instance, 
				atomic_read(&ha->loop_down_timer));)
	}

	/*
	 * Done Q Handler -- dgFIXME This handler will kick off doneq if we
	 * haven't process it in 2 seconds.
	 */
	if (!list_empty(&ha->done_queue)) {
#if QLA2X_PERFORMANCE
		tasklet_schedule(&ha->run_qla_task);
#else
		start_dpc++;
		/* qla2x00_done(ha); */
#endif
	}

#if QLA2100_LIPTEST
	/*
	 * This block is used to periodically schedule isp abort after
	 * qla2x00_lip flag is set. 
	 */

	/*
	   if (qla2x00_lip && (ha->forceLip++) == (60*2)) {
	   printk("timer: schedule isp abort.\n");
	   set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
	   ha->forceLip = 0;
	   }
	 */

	/*
	 * This block is used to periodically schedule mailbox cmd timeout
	 * simulation
	 */
	if (qla2x00_lip && (ha->forceLip++) == (60*6)) {
		printk("qla2x00_timer: Going to force mbx timeout\n");

		ha->forceLip = 0;
		mbxtimeout = 1;
	}
#endif

#if defined(EH_WAKEUP_WORKAROUND)
	if (ha->host->in_recovery &&
#if defined(EH_WAKEUP_WORKAROUND_REDHAT)
		(atomic_read(&(ha->host->host_busy)) ==
		      ha->host->host_failed) &&
#else
		(ha->host->host_busy == ha->host->host_failed) &&
#endif
		!ha->host->eh_active) {	

		if ((ha->eh_start++) == 60) {
			if (ha->host->eh_wait)
				up(ha->host->eh_wait);
			ha->eh_start=0;
			printk("qla%ld: !!! Waking up error handler "
				"for scsi layer\n",
				ha->host_no);
		}
	}
#endif /* EH_WAKEUP_WORKAROUND */

	if (test_bit(FAILOVER_EVENT_NEEDED, &ha->dpc_flags)) {
		if (ha->failback_delay)  {
			ha->failback_delay--;
			if (ha->failback_delay == 0)  {
				set_bit(FAILOVER_EVENT, &ha->dpc_flags);
				clear_bit(FAILOVER_EVENT_NEEDED,
						&ha->dpc_flags);
			}
		} else {
			set_bit(FAILOVER_EVENT, &ha->dpc_flags);
			clear_bit(FAILOVER_EVENT_NEEDED, &ha->dpc_flags);
		}
	}

	/* Schedule the DPC routine if needed */
	if ((test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags) ||
		test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags) ||
		start_dpc ||
		test_bit(LOGIN_RETRY_NEEDED, &ha->dpc_flags) ||
		test_bit(FAILOVER_EVENT, &ha->dpc_flags) ||
		test_bit(FAILOVER_NEEDED, &ha->dpc_flags) ||
		test_bit(MAILBOX_CMD_NEEDED, &ha->dpc_flags)) &&
		ha->dpc_wait && !ha->dpc_active ) {  /* v2.19.4 */

		up(ha->dpc_wait);
	}

	RESTART_TIMER(qla2x00_timer,ha,WATCH_INTERVAL);
}


#if  NO_LONG_DELAYS
/*
 * This would normally need to get the IO request lock, but as it doesn't
 * actually touch anything that needs to be locked we can avoid the lock here..
 */
STATIC void 
qla2x00_sleep_done(struct semaphore * sem)
{
	if (sem != NULL)
	{
		up(sem);
	}
}
#endif

/*
* qla2x00_callback
*      Returns the completed SCSI command to LINUX.
*
* Input:
*	ha -- Host adapter structure
*	cmd -- SCSI mid-level command structure.
* Returns:
*      None
* Note:From failover point of view we always get the sp
*      from vis_ha pool in queuecommand.So when we put it 
*      back to the pool it has to be the vis_ha.	 
*      So rely on Scsi_Cmnd to get the vis_ha and not on sp. 		 	
*/
static inline void
qla2x00_callback(scsi_qla_host_t *ha, Scsi_Cmnd *cmd)
{
	srb_t *sp = (srb_t *) CMD_SP(cmd);
	scsi_qla_host_t *vis_ha;
	os_lun_t *lq;
	int got_sense;
	unsigned long	cpu_flags = 0;

	ENTER(__func__);

	CMD_HANDLE(cmd) = (unsigned char *) NULL;
	vis_ha = (scsi_qla_host_t *) cmd->host->hostdata;

	if (sp == NULL) {
		printk(KERN_INFO
			"%s(): **** CMD derives a NULL SP\n",
			__func__);
                DEBUG2(BUG();)
		return;
	}

	/*
	 * If command status is not DID_BUS_BUSY then go ahead and freed sp.
	 */
	/*
	 * Cancel command timeout
	 */
	qla2x00_delete_timer_from_cmd(sp);

	/*
	 * Put SP back in the free queue
	 */
	sp->cmd   = NULL;
	CMD_SP(cmd) = NULL;
	lq = sp->lun_queue;
	got_sense = (sp->flags & SRB_GOT_SENSE)? 1: 0;
	add_to_free_queue(vis_ha, sp);

	if ((CMD_RESULT(cmd)>>16) == DID_OK) {
		/* device ok */
		ha->total_bytes += cmd->bufflen;
		if (!got_sense) {
			/* COMPAQ*/
#if defined(COMPAQ)
			/*
			 * When we detect the first good Read capability scsi
			 * command we assume the SCSI layer finish the scan.
			 */
			if (cmd->cmnd[0] == 0x25 &&
				!(lq->q_flag & LUN_SCSI_SCAN_DONE)) {
				/* mark lun with finish scan */
				lq->q_flag |= LUN_SCSI_SCAN_DONE;
			}
#endif /* COMPAQ */
			/*
			 * If lun was suspended then clear retry count.
			 */
			spin_lock_irqsave(&lq->q_lock, cpu_flags);
			if (!test_bit(LUN_EXEC_DELAYED, &lq->q_flag))
				lq->q_state = LUN_STATE_READY;
			spin_unlock_irqrestore(&lq->q_lock, cpu_flags);
		}
	} else if ((CMD_RESULT(cmd)>>16) == DID_ERROR) {
		/* device error */
		ha->total_dev_errs++;
	}

	if (cmd->flags & IS_RESETTING) {
		CMD_RESULT(cmd) = (int)DID_RESET << 16;
	}

	/* Call the mid-level driver interrupt handler */
	(*(cmd)->scsi_done)(cmd);

	LEAVE(__func__);
}

/*
* qla2x00_mem_alloc
*      Allocates adapter memory.
*
* Returns:
*      0  = success.
*      1  = failure.
*/
static uint8_t
qla2x00_mem_alloc(scsi_qla_host_t *ha)
{
	uint8_t   status = 1;
	uint8_t   i;
	int	retry= 10;
	mbx_cmdq_t	*ptmp;
	mbx_cmdq_t	*tmp_q_head;
	mbx_cmdq_t	*tmp_q_tail;

	ENTER(__func__);

	do {
		/*
		 * This will loop only once if everything goes well, else some
		 * number of retries will be performed to get around a kernel
		 * bug where available mem is not allocated until after a
		 * little delay and a retry.
		 */

#if defined(FC_IP_SUPPORT)
		ha->risc_rec_q = pci_alloc_consistent(ha->pdev,
					((IP_BUFFER_QUEUE_DEPTH) * 
					 (sizeof(struct risc_rec_entry))),
					&ha->risc_rec_q_dma);
		if (ha->risc_rec_q == NULL) {
			/* error */
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - "
				"risc_rec_q\n",
				ha->host_no);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
			continue;
		}
#endif	/* #if defined(FC_IP_SUPPORT) */

		ha->request_ring = pci_alloc_consistent(ha->pdev,
					((REQUEST_ENTRY_CNT + 1) * 
					 (sizeof(request_t))),
					&ha->request_dma);
		if (ha->request_ring == NULL) {
			/* error */
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - "
				"request_ring\n",
				ha->host_no);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
			continue;
		}

		ha->response_ring = pci_alloc_consistent(ha->pdev,
					((RESPONSE_ENTRY_CNT + 1) * 
					 (sizeof(response_t))),
					&ha->response_dma);
		if (ha->response_ring == NULL) {
			/* error */
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - "
				"response_ring\n",
				ha->host_no);
			qla2x00_mem_free(ha);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
			continue;
		}

		/* get consistent memory allocated for init control block */
		ha->init_cb = pci_alloc_consistent(ha->pdev,
				sizeof(init_cb_t),
				&ha->init_cb_dma);
		if (ha->init_cb == NULL) {
			/* error */
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - "
				"init_cb\n",
				ha->host_no);
			qla2x00_mem_free(ha);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
			continue;
		}
		memset(ha->init_cb, 0, sizeof(init_cb_t));

		/* Allocate ioctl related memory. */
		if (qla2x00_alloc_ioctl_mem(ha)) {
			/* error */
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - "
				"ioctl_mem\n",
				ha->host_no);
			qla2x00_mem_free(ha);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
			continue;
		}

		if (qla2x00_allocate_sp_pool(ha)) {
			/* error */
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - "
				"qla2x00_allocate_sp_pool\n",
				ha->host_no);
			qla2x00_mem_free(ha);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
			continue;
		}

		/*
		 * Allocate an initial list of mailbox semaphore queue to be
		 * used for serialization of the mailbox commands.
		 */
		tmp_q_head = (void *)KMEM_ZALLOC(sizeof(mbx_cmdq_t), 20);
		if (tmp_q_head == NULL) {
			/* error */
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - "
				"mbx_cmd_q",
				ha->host_no);
			qla2x00_mem_free(ha);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
			continue;
		}
		ha->mbx_sem_pool_head = tmp_q_head;
		tmp_q_tail = tmp_q_head;
		/* Now try to allocate more */
		for (i = 1; i < MBQ_INIT_LEN; i++) {
			ptmp = (void *)KMEM_ZALLOC(sizeof(mbx_cmdq_t), 20 + i);
			if (ptmp == NULL) {
				/*
				 * Error. Just exit. If more is needed later
				 * they will be allocated at that time.
				 */
				break;
			}
			tmp_q_tail->pnext = ptmp;
			tmp_q_tail = ptmp;
		}
		ha->mbx_sem_pool_tail = tmp_q_tail;

		/* Done all allocations without any error. */
		status = 0;

	} while (retry-- && status != 0);

	if (status) {
		printk(KERN_WARNING
			"%s(): **** FAILED ****\n", __func__);
	}

	LEAVE(__func__);

	return(status);
}

/*
* qla2x00_mem_free
*      Frees all adapter allocated memory.
*
* Input:
*      ha = adapter block pointer.
*/
STATIC void
qla2x00_mem_free(scsi_qla_host_t *ha)
{
	uint32_t	t;
	fc_lun_t	*fclun, *fclun_next;
	fc_port_t	*fcport, *fcport_next;
	mbx_cmdq_t	*ptmp;
	mbx_cmdq_t	*tmp_q_head;
	unsigned long	wtime;/* max wait time if mbx cmd is busy. */
	struct list_head *fcil, *fcitemp;
	fc_initiator_t	*fcinitiator;

	ENTER(__func__);

	if (ha == NULL) {
		/* error */
		DEBUG2(printk("%s(): ERROR invalid ha pointer.\n", __func__);)
		return;
	}

#if defined(QLA2XXX_CONFIG_BOOTIMG)
	/* Detach the bootimg notifier. */
	notifier_chain_unregister(&bootimg_notifiers, &(ha->bootimg_notifier));
#endif

	/* Free the target queues */
	for (t = 0; t < MAX_TARGETS; t++) {
		qla2x00_tgt_free(ha, t);
	}

	/* Make sure all other threads are stopped. */
	wtime = 60 * HZ;
	while ((ha->dpc_wait != NULL || 
		ha->mbx_q_head != NULL) && 
		wtime) {

		set_current_state(TASK_INTERRUPTIBLE);
		wtime = schedule_timeout(wtime);
	}

	/* Now free the mbx sem pool */
	tmp_q_head = ha->mbx_sem_pool_head;
	while (tmp_q_head != NULL) {
		ptmp = tmp_q_head->pnext;
		KMEM_FREE(tmp_q_head, sizeof(mbx_cmdq_t));
		tmp_q_head = ptmp;
	}
	ha->mbx_sem_pool_head = NULL;

	/* free ioctl memory */
	qla2x00_free_ioctl_mem(ha);

	/* Free host database. */
	list_for_each_safe(fcil, fcitemp, &ha->fcinitiators) {
		fcinitiator = list_entry(fcil, fc_initiator_t, list);

		list_del(&fcinitiator->list);
		kfree(fcinitiator);
	}
	INIT_LIST_HEAD(&ha->fcinitiators);

	/* free sp pool */
	qla2x00_free_sp_pool(ha);

	/* 4.10 */
	/* free memory allocated for init_cb */
	if (ha->init_cb) {
		pci_free_consistent(ha->pdev, 
				sizeof(init_cb_t),
				ha->init_cb, 
				ha->init_cb_dma);
	}

	if (ha->request_ring) {
		pci_free_consistent(ha->pdev,
				((REQUEST_ENTRY_CNT + 1) * 
				 (sizeof(request_t))),
				ha->request_ring, 
				ha->request_dma);
	}

	if (ha->response_ring) {
		pci_free_consistent(ha->pdev,
				((RESPONSE_ENTRY_CNT + 1) * 
				 (sizeof(response_t))),
				ha->response_ring, 
				ha->response_dma);
	}

#if defined(FC_IP_SUPPORT)
	if (ha->risc_rec_q) {
		pci_free_consistent(ha->pdev,
				((IP_BUFFER_QUEUE_DEPTH) * 
				 (sizeof(struct risc_rec_entry))),
				ha->risc_rec_q, 
				ha->risc_rec_q_dma);
	}
	ha->risc_rec_q = NULL;
	ha->risc_rec_q_dma = 0;
#endif

	ha->init_cb = NULL;
	ha->request_ring = NULL;
	ha->request_dma = 0;
	ha->response_ring = NULL;
	ha->response_dma = 0;

	/* fc ports */
	for (fcport = ha->fcport; 
		fcport != NULL;
		fcport = fcport_next) {

		fcport_next = fcport->next;

		/* fc luns */
		for (fclun = fcport->fclun; 
			fclun != NULL;
			fclun = fclun_next) {

			fclun_next = fclun->next;
			kfree(fclun);
		}
		kfree(fcport);
	}

	LEAVE(__func__);
}

#if 0
/*
*  qla2x00_abort_pending_queue
*      Abort all commands on the pending queue.
*
* Input:
*      ha = adapter block pointer.
*/
STATIC void
qla2x00_abort_pending_queue(scsi_qla_host_t *ha, uint32_t stat)
{
	unsigned long		flags;
	struct list_head	*list, *temp;

	ENTER("qla2x00_abort_pending_queue");

	DEBUG5(printk("Abort pending queue ha(%d)\n", ha->host_no);)

	/* abort all commands on LUN queue. */
	spin_lock_irqsave(&ha->list_lock, flags);
	list_for_each_safe(list, temp, &ha->pending_queue) {
		srb_t *sp;

		sp = list_entry(list, srb_t, list);
		__del_from_pending_queue(ha, sp);
		CMD_RESULT(sp->cmd) = stat << 16;
		__add_to_done_queue(ha, sp);
	} /* list_for_each_safe */
	spin_unlock_irqrestore(&ha->list_lock, flags);

	LEAVE("qla2x00_abort_pending_queue");
}
#endif


/****************************************************************************/
/*                QLogic ISP2x00 Hardware Support Functions.                */
/****************************************************************************/

/*
* qla2x00_initialize_adapter
*      Initialize board.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success
*/
uint8_t
qla2x00_initialize_adapter(scsi_qla_host_t *ha)
{
	device_reg_t *reg;
	uint8_t      status;
	uint8_t      isp_init = 0;
	uint8_t      restart_risc = 0;
	uint8_t      retry;

	ENTER(__func__);

	/* Clear adapter flags. */
	ha->forceLip = 0;
	ha->flags.online = FALSE;
	ha->flags.disable_host_adapter = FALSE;
	ha->flags.reset_active = FALSE;
	ha->flags.watchdog_enabled = FALSE;
	atomic_set(&ha->loop_down_timer, LOOP_DOWN_TIME);
	ha->loop_state = LOOP_DOWN;
	ha->device_flags = 0;
	ha->sns_retry_cnt = 0;
	ha->device_flags = 0;
	ha->dpc_flags = 0;
	ha->sns_retry_cnt = 0;
	ha->failback_delay = 0;
	ha->iocb_cnt = 0;
	ha->iocb_overflow_cnt = 0;
	/* 4.11 */
	ha->flags.management_server_logged_in = 0;
	/* ra 11/27/01 */
	ha->marker_needed = 0;
	ha->mbx_flags = 0;
	ha->isp_abort_cnt = 0;

	DEBUG(printk("Configure PCI space for adapter...\n"));

	if (!(status = qla2x00_pci_config(ha))) {
		reg = ha->iobase;

		qla2x00_reset_chip(ha);

		/* Initialize Fibre Channel database. */
		qla2x00_init_fc_db(ha);

		/* Initialize target map database. */
		qla2x00_init_tgt_map(ha);

		/* Get Flash Version */
		qla2x00_get_flash_version(ha);

		if (qla2x00_verbose)
			printk("scsi(%ld): Configure NVRAM parameters...\n",
				ha->host_no);

#if defined(ISP2100)
		qla2100_nvram_config(ha);
#else
		qla2x00_nvram_config(ha);
#endif

		ha->retry_count = ql2xretrycount;
#if USE_PORTNAME
		ha->flags.port_name_used =1;
#else
		ha->flags.port_name_used =0;
#endif

		if (qla2x00_verbose)
			printk("scsi(%ld): Verifying loaded RISC code...\n",
				ha->host_no);

		qla2x00_set_cache_line(ha);

		/*
		 * If the user specified a device configuration on the command
		 * line then use it as the configuration.  Otherwise, we scan
		 * for all devices.
		 */
		if (ql2xdevconf) {
			ha->cmdline = ql2xdevconf;
			if (!ha->flags.failover_enabled)
				qla2x00_get_properties(ha, ql2xdevconf);
		}

		retry = QLA2XXX_LOOP_RETRY_COUNT;
		/*
		 * Try an configure the loop.
		 */
		do {
			restart_risc = 0;
			isp_init = 0;
			DEBUG(printk("%s(): check if firmware needs to be "
					"loaded\n",
					__func__);)

			/* If firmware needs to be loaded */
			if (qla2x00_isp_firmware(ha)) {
				if (qla2x00_verbose)
					printk("scsi(%ld): Verifying chip...\n",
						ha->host_no);

				if (!(status = qla2x00_chip_diag(ha)))
					status = qla2x00_setup_chip(ha);

				if (!status) {
					DEBUG(printk("scsi(%ld): Chip verified "
							"and RISC loaded...\n",
							ha->host_no));
				}
			}
			if (!status && !(status = qla2x00_init_rings(ha))) {

				/* dg - 7/3/1999
				 *
				 * Wait for a successful LIP up to a maximum 
				 * of (in seconds): RISC login timeout value,
				 * RISC retry count value, and port down retry
				 * value OR a minimum of 4 seconds OR If no 
				 * cable, only 5 seconds.
				 */
				DEBUG(printk("qla2x00_init_rings OK, call "
						"qla2x00_fw_ready...\n");)

				if (!qla2x00_fw_ready(ha)) {
					clear_bit(RESET_MARKER_NEEDED,
							&ha->dpc_flags);
					clear_bit(COMMAND_WAIT_NEEDED,
							&ha->dpc_flags);

					/*
					 * Go setup flash database devices 
					 * with proper Loop ID's.
					 */
					do {
						clear_bit(LOOP_RESYNC_NEEDED,
								&ha->dpc_flags);
						status = qla2x00_configure_loop(ha);

						if ( test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags) ){
							restart_risc = 1;
							break;
						}

						/*
						 *	If loop state change while we were discovering devices
						 *	then wait for LIP to complete
						 */

						if ( ha->loop_state == LOOP_DOWN ) {
							while( atomic_read(&ha->loop_down_timer) &&
								ha->loop_state != LOOP_UPDATE ) {
								qla2x00_check_fabric_devices(ha);
								set_current_state(TASK_INTERRUPTIBLE);
								schedule_timeout(5);
							}
						}
					} while (!atomic_read(&ha->loop_down_timer) &&
						(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)) );
				}

				if (ha->flags.update_config_needed) {
					struct qla2x00_additional_firmware_options additional_firmware_options;

					*((uint16_t *) &additional_firmware_options) =
					    le16_to_cpu(*((uint16_t *) &ha->init_cb->additional_firmware_options));

					additional_firmware_options.connection_options = ha->operating_mode;

					*((uint16_t *) &ha->init_cb->additional_firmware_options) =
					    cpu_to_le16( *((uint16_t *) &additional_firmware_options));

					restart_risc = 1;
				}

				if (ha->mem_err) {
					restart_risc = 1;
				}
				isp_init = 1;

			}
		} while (restart_risc && retry--);

		if (isp_init) {
			clear_bit(RESET_MARKER_NEEDED, &ha->dpc_flags);
			ha->marker_needed = 1;
			qla2x00_marker(ha, 0, 0, MK_SYNC_ALL);
			ha->marker_needed = 0;

			ha->flags.online = TRUE;

			/* Enable target response to SCSI bus. */
			if (ha->flags.enable_target_mode)
				qla2x00_enable_lun(ha);
		}

	}

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status)
		printk("%s(): **** FAILED ****\n", __func__);
#endif

	LEAVE(__func__);

	return (status);
}

/*
* ISP Firmware Test
*      Checks if present version of RISC firmware is older than
*      driver firmware.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = firmware does not need to be loaded.
*/
STATIC uint8_t
qla2x00_isp_firmware(scsi_qla_host_t *ha)
{
	uint8_t  status = 1; /* assume loading risc code */

	ENTER(__func__);

	if (ha->flags.disable_risc_code_load) {
		/* Verify checksum of loaded RISC code. */
		status = qla2x00_verify_checksum(ha);
		printk(KERN_INFO "%s RISC CODE NOT loaded\n",__func__);
		DEBUG2(printk("%s RISC CODE NOT loaded\n",__func__);)

	}

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status)
		printk("%s: **** Load RISC code ****\n", __func__);
#endif

	LEAVE(__func__);

	return (status);
}

/*
* (08/05/99)
*
* PCI configuration
*      Setup device PCI configuration registers.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_pci_config(scsi_qla_host_t *ha)
{
	uint8_t		status = 1;
#if MEMORY_MAPPED_IO
	uint32_t	page_offset, base;
	uint32_t	mmapbase;
#endif
	int		pci_ret;
	uint16_t	buf_wd;

	ENTER(__func__);

	/* 
	 * Turn on PCI master; for system BIOSes that don't turn it on by
	 * default.
	 */
	pci_set_master(ha->pdev);
	pci_read_config_word(ha->pdev, PCI_REVISION_ID, &buf_wd);
	ha->revision = buf_wd;

	if (ha->iobase)
		return 0;

	do { /* Quick exit */
		/* Get command register. */
		pci_ret = pci_read_config_word(ha->pdev, PCI_COMMAND, &buf_wd);
		if (pci_ret != PCIBIOS_SUCCESSFUL)
			break;

		/*
		 * Set Bus Master Enable (bit-2), Memory Address Space Enable
		 * and reset any error bits.
		 */
		buf_wd &= ~0x7;

#if MEMORY_MAPPED_IO
		DEBUG(printk("%s(): I/O SPACE and MEMORY MAPPED I/O is "
				"enabled.\n",
				__func__));
		buf_wd |= (PCI_COMMAND_MASTER |
				PCI_COMMAND_MEMORY |
				PCI_COMMAND_IO);
#else
		DEBUG(printk("%s(): I/O SPACE Enabled and MEMORY MAPPED "
				"I/O is disabled.\n",
				__func__));
		buf_wd |= (PCI_COMMAND_MASTER | PCI_COMMAND_IO);
#endif

		pci_ret = pci_write_config_word(ha->pdev, PCI_COMMAND, buf_wd);
		if (pci_ret != PCIBIOS_SUCCESSFUL)
			printk(KERN_WARNING
				"%s(): Could not write config word.\n",
				__func__);

		/* Get expansion ROM address. */
		pci_ret = pci_read_config_word(ha->pdev,
				PCI_ROM_ADDRESS, &buf_wd);
		if (pci_ret != PCIBIOS_SUCCESSFUL)
			break;

		/* Reset expansion ROM address decode enable */
		buf_wd &= ~PCI_ROM_ADDRESS_ENABLE;

		pci_ret = pci_write_config_word(ha->pdev, 
					PCI_ROM_ADDRESS, buf_wd);
		if (pci_ret != PCIBIOS_SUCCESSFUL)
			break;

#if MEMORY_MAPPED_IO
		/* Get memory mapped I/O address */
		pci_read_config_dword(ha->pdev, PCI_BASE_ADDRESS_1, &mmapbase);
		mmapbase &= PCI_BASE_ADDRESS_MEM_MASK;

		/* Find proper memory chunk for memory map I/O reg */
		base = mmapbase & PAGE_MASK;
		page_offset = mmapbase - base;

		/* Get virtual address for I/O registers  */
		ha->mmpbase = ioremap(base, page_offset + 256);
		if (ha->mmpbase) {
			ha->mmpbase += page_offset;
			ha->iobase = ha->mmpbase;
			status = 0;
		}
#else /* MEMORY_MAPPED_IO */
		status = 0;
#endif /* MEMORY_MAPPED_IO */
	} while (0);

	LEAVE(__func__);

	return (status);
}

/*
* qla2x00_set_cache_line
*      Sets PCI cache line parameter.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success.
*/
static uint8_t
qla2x00_set_cache_line(struct scsi_qla_host * ha)
{
	unsigned char cache_size;

	ENTER(__func__);

	/* Set the cache line. */
	if (!ha->flags.set_cache_line_size_1) {
		LEAVE(__func__);
		return 0;
	}

	/* taken from drivers/net/acenic.c */
	pci_read_config_byte(ha->pdev, PCI_CACHE_LINE_SIZE, &cache_size);
	cache_size <<= 2;
	if (cache_size != SMP_CACHE_BYTES) {
		printk(KERN_INFO
			"  PCI cache line size set incorrectly (%d bytes) by "
			"BIOS/FW, ",
			cache_size);

		if (cache_size > SMP_CACHE_BYTES) {
			printk("expecting %d.\n", SMP_CACHE_BYTES);
		} else {
			printk("correcting to %d.\n", SMP_CACHE_BYTES);
			pci_write_config_byte(ha->pdev,
						PCI_CACHE_LINE_SIZE,
						SMP_CACHE_BYTES >> 2);
		}
	}

	LEAVE(__func__);

	return 0;
}


/*
* Chip diagnostics
*      Test chip for proper operation.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_chip_diag(scsi_qla_host_t *ha)
{
	uint8_t		status = 0;
	device_reg_t	*reg = ha->iobase;
	unsigned long	flags = 0;
#if defined(ISP2300)
	uint16_t	buf_wd;
#endif
	uint16_t	data;
	uint32_t	cnt;

	ENTER(__func__);

	DEBUG3(printk("%s(): testing device at %lx.\n",
			__func__,
			(u_long)&reg->flash_address);)

	spin_lock_irqsave(&ha->hardware_lock, flags);

	/* Reset ISP chip. */
	WRT_REG_WORD(&reg->ctrl_status, CSR_ISP_SOFT_RESET);
	data = qla2x00_debounce_register(&reg->ctrl_status);
	for (cnt = 6000000 ; cnt && (data & CSR_ISP_SOFT_RESET); cnt--) {
		udelay(5);
		data = RD_REG_WORD(&reg->ctrl_status);
		barrier();
	}

	if (cnt) {
		DEBUG3(printk("%s(): reset register cleared by chip reset\n",
				__func__);)

#if defined(ISP2300)
		pci_read_config_word(ha->pdev, PCI_COMMAND, &buf_wd);
		buf_wd |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
		data = RD_REG_WORD(&reg->mailbox6);

		if ((ha->device_id == QLA2312_DEVICE_ID) ||
			((data & 0xff) == FPM_2310))
			/* Enable Memory Write and Invalidate. */
			buf_wd |= PCI_COMMAND_INVALIDATE;
		else
			buf_wd &= ~PCI_COMMAND_INVALIDATE;
		pci_write_config_word(ha->pdev, PCI_COMMAND, buf_wd);
#endif
		/* Reset RISC processor. */
		WRT_REG_WORD(&reg->host_cmd, HC_RESET_RISC);
		WRT_REG_WORD(&reg->host_cmd, HC_RELEASE_RISC);

#if defined(ISP2300)
		/* Workaround for QLA2312 PCI parity error */
		if (ha->device_id == QLA2312_DEVICE_ID)
			udelay(10);
		else {
			data = qla2x00_debounce_register(&reg->mailbox0);

			for (cnt = 6000000; cnt && (data == MBS_BUSY); cnt--) {
				udelay(5);
				data = RD_REG_WORD(&reg->mailbox0);
				barrier(); 
			}
		}
#else
		data = qla2x00_debounce_register(&reg->mailbox0);

		for (cnt = 6000000; cnt && (data == MBS_BUSY); cnt--) {
			udelay(5);
			data = RD_REG_WORD(&reg->mailbox0);
			barrier(); 
		}
#endif

		if (cnt) {
			/* Check product ID of chip */
			DEBUG3(printk("%s(): Checking product ID of chip\n",
					__func__);)

			if (RD_REG_WORD(&reg->mailbox1) != PROD_ID_1 ||
				(RD_REG_WORD(&reg->mailbox2) != PROD_ID_2 &&
				 RD_REG_WORD(&reg->mailbox2) != PROD_ID_2a) ||
				RD_REG_WORD(&reg->mailbox3) != PROD_ID_3 ||
				(qla2x00_debounce_register(&reg->mailbox4) !=
							 PROD_ID_4 &&
				 qla2x00_debounce_register(&reg->mailbox4) !=
							 PROD_ID_4a) ) {
				printk(KERN_WARNING
					"qla2x00: Wrong product ID = "
					"0x%x,0x%x,0x%x,0x%x\n",
					RD_REG_WORD(&reg->mailbox1),
					RD_REG_WORD(&reg->mailbox2),
					RD_REG_WORD(&reg->mailbox3),
					RD_REG_WORD(&reg->mailbox4));
				status = 1;
			} else {
#if defined(ISP2200)
				/* Now determine if we have a 2200A board */
				if ((ha->device_id == QLA2200_DEVICE_ID ||
					ha->device_id == QLA2200A_DEVICE_ID) &&
					RD_REG_WORD(&reg->mailbox7) ==
						QLA2200A_RISC_ROM_VER) {
					ha->device_id = QLA2200A_DEVICE_ID;

					DEBUG3(printk("%s(): Found QLA2200A "
							"chip.\n",
							__func__);)
				}
#endif
				spin_unlock_irqrestore(&ha->hardware_lock,
						flags);

				DEBUG3(printk("%s(): Checking mailboxes.\n",
						__func__);)

				/* Wrap Incoming Mailboxes Test. */
				status = qla2x00_mbx_reg_test(ha);
				if (status) {
					printk(KERN_WARNING
						"%s(): failed mailbox send "
						"register test\n",
						__func__);
					DEBUG(printk("%s(): Failed mailbox "
							"send register test\n",
							__func__);)
				}
				spin_lock_irqsave(&ha->hardware_lock, flags);
			}
		} else
			status = 1;
	} else
		status = 1;

	if (status)
		DEBUG2_3(printk("%s(): **** FAILED ****\n", __func__);)

	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	LEAVE(__func__);

	return(status);
}

/*
* Setup chip
*      Load and start RISC firmware.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_setup_chip(scsi_qla_host_t *ha)
{
	uint8_t		status = 0;
	uint16_t	cnt;
	uint16_t	risc_address;
	uint16_t	*risc_code_address;
	unsigned long	risc_code_size;
	int		num;
#if defined(WORD_FW_LOAD)
	uint16_t	data;
	uint16_t	*ql21_risc_code_addr01;
	uint16_t	ql21_risc_code_length01;
	uint8_t		dump_status;
#endif

	ENTER(__func__);

	/* Load RISC code. */
	risc_address = *QLBoardTbl_fc[ha->devnum].fwstart;
	risc_code_address = QLBoardTbl_fc[ha->devnum].fwcode;
	risc_code_size = *QLBoardTbl_fc[ha->devnum].fwlen;

	DEBUG(printk("%s(): Loading RISC code size =(0x%lx) req virt=%p "
			"phys=%llx\n",
			__func__,
			risc_code_size,
			ha->request_ring,
			(u64)ha->request_dma);)

	/*
	 * Save active FC4 type depending on firmware support. This info is
	 * needed by ioctl cmd.
	 */
	ha->active_fc4_types = EXT_DEF_FC4_TYPE_SCSI;
#if defined(FC_IP_SUPPORT)
	ha->active_fc4_types |= EXT_DEF_FC4_TYPE_IP;
#endif
#if defined(FC_SCTP_SUPPORT)
	if (risc_address == fw2300sctp_code01)
		ha->active_fc4_types |= EXT_DEF_FC4_TYPE_SCTP;
#endif

	num = 0;
	while (risc_code_size > 0 && !status) {
		cnt = REQUEST_ENTRY_SIZE * REQUEST_ENTRY_CNT >> 1;
#if defined(ISP2200)
		/* for 2200A set transfer size to 128 bytes */
		if (ha->device_id == QLA2200A_DEVICE_ID)
			cnt = 128 >> 1;
#endif

		if (cnt > risc_code_size)
			cnt = risc_code_size;

		DEBUG7(printk("%s(): loading risc segment@ addr %p, number of "
				"bytes 0x%x, offset 0x%x.\n",
				__func__,
				risc_code_address,
				cnt,
				risc_address);)

#if defined(__LITTLE_ENDIAN)
		memcpy(ha->request_ring, risc_code_address, (cnt << 1));
#else
	{
		int i;
		uint16_t *req_ring;

		req_ring = (uint16_t *)ha->request_ring;
		for (i = 0; i < cnt; i++)
			req_ring[i] = cpu_to_le16(risc_code_address[i]);
	};
#endif

		/*
		 * Flush written firmware to the ha->request_ring buffer before
		 * DMA
		 */
		flush_cache_all();

		status = qla2x00_load_ram(ha,
				ha->request_dma, risc_address, cnt);

		if (status) {
			qla2x00_dump_regs(ha->host);
			printk(KERN_WARNING
				"qla2x00: [ERROR] Failed to load segment "
				"%d of FW\n",
				num);
			DEBUG(printk("%s(): Failed to load segment %d of FW\n",
					__func__,
					num);)
			break;
		}

		risc_address += cnt;
		risc_code_size -= cnt;
		risc_code_address += cnt;
		num++;
	}

#if defined(WORD_FW_LOAD)
	{
		int i;

		risc_address = *QLBoardTbl_fc[ha->devnum].fwstart;
		ql21_risc_code_addr01  = QLBoardTbl_fc[ha->devnum].fwcode;
		ql21_risc_code_length01 = *QLBoardTbl_fc[ha->devnum].fwlen;

		for (i = 0; i < ql21_risc_code_length01 ; i++) {
			dump_status = qla2x00_write_ram_word(ha,
					risc_address + i, 
					*(ql21_risc_code_addr01 + i));

			if (dump_status) {
				printk(KERN_WARNING
					"qla2x00: [ERROR] firmware load "
					"failure\n");
				break;
			}

			dump_status = qla2x00_read_ram_word(ha,
					risc_address + i, &data);

			if (dump_status) {
				printk(KERN_WARNING
					"qla2x00: [ERROR] RISC FW Read "
					"Failure\n");
				break;
			}

			if (data != *(ql21_risc_code_addr01 + i)) {
				printk(KERN_WARNING
					"qla2x00: [ERROR] RISC FW Compare "
					"ERROR @ (0x%p)\n",
					(void *)(ql21_risc_code_addr01+i));
			}
		}
		printk(KERN_INFO
			"qla2x00: RISC FW download confirmed... \n");
	}
#endif /* WORD_FW_LOAD */

	/* Verify checksum of loaded RISC code. */
	if (!status) {
		DEBUG(printk("%s(): Verifying Check Sum of loaded RISC code.\n",
				__func__);)

		status = (uint8_t)qla2x00_verify_checksum(ha);

		if (status == QL_STATUS_SUCCESS) {
			/* Start firmware execution. */
			DEBUG(printk("%s(): CS Ok, Start firmware running\n",
					__func__);)
			status = qla2x00_execute_fw(ha);
		}
#if defined(QL_DEBUG_LEVEL_2)
		else {
			printk(KERN_INFO
				"%s(): ISP FW Failed Check Sum\n", __func__);
		}
#endif
	}

	if (status) {
		DEBUG2_3(printk("%s(): **** FAILED ****\n", __func__);)
	} else {
		DEBUG3(printk("%s(): Returning Good Status\n", __func__);)
	}

	return (status);
}

/*
* qla2x00_init_rings
*      Initializes firmware.
*
*      Beginning of request ring has initialization control block
*      already built by nvram config routine.
*
* Input:
*      ha                = adapter block pointer.
*      ha->request_ring  = request ring virtual address
*      ha->response_ring = response ring virtual address
*      ha->request_dma   = request ring physical address
*      ha->response_dma  = response ring physical address
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_init_rings(scsi_qla_host_t *ha)
{
	unsigned long flags = 0;
	uint8_t  status;
	int cnt;
	device_reg_t *reg = ha->iobase;

	ENTER(__func__);

	spin_lock_irqsave(&ha->hardware_lock, flags);

	/* Clear outstanding commands array. */
	for (cnt = 0; cnt < MAX_OUTSTANDING_COMMANDS; cnt++)
		ha->outstanding_cmds[cnt] = 0;

	ha->current_outstanding_cmd = 0;

	/* Clear RSCN queue. */
	ha->rscn_in_ptr = 0;
	ha->rscn_out_ptr = 0;

	/* Initialize firmware. */
	ha->request_ring_ptr  = ha->request_ring;
	ha->req_ring_index    = 0;
	ha->req_q_cnt         = REQUEST_ENTRY_CNT;
	ha->response_ring_ptr = ha->response_ring;
	ha->rsp_ring_index    = 0;

#if defined(ISP2300)
	WRT_REG_WORD(&reg->req_q_in, 0);
	WRT_REG_WORD(&reg->req_q_out, 0);
	WRT_REG_WORD(&reg->rsp_q_in, 0);
	WRT_REG_WORD(&reg->rsp_q_out, 0);
#else
	WRT_REG_WORD(&reg->mailbox4, 0);
	WRT_REG_WORD(&reg->mailbox4, 0);
	WRT_REG_WORD(&reg->mailbox5, 0);
	WRT_REG_WORD(&reg->mailbox5, 0);
#endif

	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	DEBUG(printk("%s(%ld): issue init firmware.\n",
			__func__,
			ha->host_no);)
	status = qla2x00_init_firmware(ha, sizeof(init_cb_t));
	if (status) {
		DEBUG2_3(printk("%s(%ld): **** FAILED ****.\n",
				__func__,
				ha->host_no);)
	} else {
		/* Setup seriallink options */
		uint16_t	opt10, opt11;

		DEBUG2(printk("%s(%ld): Serial link options:\n",
		    __func__, ha->host_no);)
		DEBUG2(qla2x00_dump_buffer(
		    (uint8_t *)&ha->fw_seriallink_options,
		    sizeof(ha->fw_seriallink_options));)

		qla2x00_get_firmware_options(ha,
		    &ha->fw_options1, &ha->fw_options2, &ha->fw_options3);

		ha->fw_options1 &= ~BIT_8;
		if (ha->fw_seriallink_options.output_enable)
			ha->fw_options1 |= BIT_8;

		opt10 = (ha->fw_seriallink_options.output_emphasis_1g << 14) |
		    (ha->fw_seriallink_options.output_swing_1g << 8) | 0x3;
		opt11 = (ha->fw_seriallink_options.output_emphasis_2g << 14) |
		    (ha->fw_seriallink_options.output_swing_2g << 8) | 0x3;

		qla2x00_set_firmware_options(ha, ha->fw_options1,
		    ha->fw_options2, ha->fw_options3, opt10, opt11);

		DEBUG3(printk("%s(%ld): exiting normally.\n",
				__func__,
				ha->host_no);)
	}

	return (status);
}

/*
* qla2x00_fw_ready
*      Waits for firmware ready.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_fw_ready(scsi_qla_host_t *ha)
{
	uint8_t  status = 0;
	uint8_t  loop_forever = 1;
	unsigned long wtime, mtime;
	uint16_t min_wait; /* minimum wait time if loop is down */
	uint16_t wait_time;/* wait time if loop is becoming ready */
	uint16_t pause_time;
	uint16_t fw_state;

	ENTER(__func__);

	min_wait = QLA2XXX_LOOP_DOWN_TIMEOUT;
	ha->device_flags &= ~DFLG_NO_CABLE;

	/*
	 * Firmware should take at most one RATOV to login, plus 5 seconds for
	 * our own processing.
	 */
	if ((wait_time = (ha->retry_count*ha->login_timeout) + 5) < min_wait) {
		wait_time = min_wait;
	}

	pause_time = 1000;	/* 1000 usec */

	/* min wait time if loop down */
	mtime = jiffies + (min_wait * HZ);

	/* wait time before firmware ready */
	wtime = jiffies + (wait_time * HZ);

	/* Wait for ISP to finish LIP */
	if (!qla2x00_quiet)
		printk(KERN_INFO
			"scsi(%ld): Waiting for LIP to complete...\n",
			ha->host_no);

	DEBUG3(printk("scsi(%ld): Waiting for LIP to complete...\n",
			ha->host_no);)

	do {
		status = qla2x00_get_firmware_state(ha, &fw_state);

		if (status == QL_STATUS_SUCCESS) {
			if (fw_state == FSTATE_READY) {
				qla2x00_get_retry_cnt(ha, 
						&ha->retry_count,
						&ha->login_timeout);
				status = QL_STATUS_SUCCESS;

				DEBUG(printk("%s(%ld): F/W Ready - OK \n",
						__func__,
						ha->host_no);)

				break;
			}

			status = QL_STATUS_ERROR;

			if (atomic_read(&ha->loop_down_timer) ||
				fw_state == FSTATE_LOSS_OF_SYNC) {
				/* Loop down. Timeout on min_wait */
				if (time_after_eq(jiffies, mtime)) {
					printk(KERN_INFO
						"scsi(%ld): Cable is "
						"unplugged...\n",
						ha->host_no);
					ha->device_flags |= DFLG_NO_CABLE;
					break;
				}
			}
		} else {
			/* Mailbox cmd failed. Timeout on min_wait. */
			if (time_after_eq(jiffies, mtime))
				break;
		}

		if (time_after_eq(jiffies, wtime))
			break;

		/* Delay for a while */
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 2);

		DEBUG3(printk("%s(): fw_state=%x curr time=%lx.\n",
				__func__,
				fw_state,
				jiffies);)
	} while (loop_forever);

	DEBUG(printk("%s(%ld): fw_state=%x curr time=%lx.\n",
			__func__,
			ha->host_no,
			fw_state,
			jiffies);)

	if (status) {
		DEBUG2_3(printk("%s(%ld): **** FAILED ****.\n",
					__func__,
					ha->host_no);)
	} else {
		DEBUG3(printk("%s(%ld): exiting normally.\n",
					__func__,
					ha->host_no);)
	}

	return (status);
}

/*
*  qla2x00_configure_hba
*      Setup adapter context.
*
* Input:
*      ha = adapter state pointer.
*
* Returns:
*      0 = success
*
* Context:
*      Kernel context.
*/
STATIC uint8_t
qla2x00_configure_hba(scsi_qla_host_t *ha)
{
	uint8_t       rval;
	uint16_t      loop_id;
	uint16_t      topo;
	uint8_t       al_pa;
	uint8_t       area;
	uint8_t       domain;
	char		connect_type[22];

	ENTER(__func__);

	/* Get host addresses. */
	rval = qla2x00_get_adapter_id(ha,
			&loop_id, &al_pa, &area, &domain, &topo);
	if (rval != QL_STATUS_SUCCESS) {
		printk(KERN_WARNING
			"%s(%ld): ERROR Get host loop ID.\n",
			__func__,
			ha->host_no);
		return (rval);
	}

	if (topo == 4) {
		printk(KERN_INFO
			"scsi(%ld): Cannot get topology - retrying.\n",
			ha->host_no);
		return (QL_STATUS_ERROR);
	}

	ha->loop_id = loop_id;

#if defined(ISP2100)
	/* Make sure 2100 only has loop, in case of any firmware bug. */
	topo = 0;
#endif

	/* initialize */
	ha->min_external_loopid = SNS_FIRST_LOOP_ID;
	ha->operating_mode = LOOP;

	switch (topo) {
		case 0:
			DEBUG3(printk("qla2x00(%ld): HBA in NL topology.\n",
					ha->host_no);)
			ha->current_topology = ISP_CFG_NL;
			strcpy(connect_type, "(Loop)");
			break;

		case 1:
			DEBUG3(printk("qla2x00(%ld): HBA in FL topology.\n",
					ha->host_no);)
			ha->current_topology = ISP_CFG_FL;
			strcpy(connect_type, "(FL_Port)");
			break;

		case 2:
			DEBUG3(printk("qla2x00(%ld): HBA in N P2P topology.\n",
					ha->host_no);)
			ha->operating_mode = P2P;
			ha->current_topology = ISP_CFG_N;
			strcpy(connect_type, "(N_Port-to-N_Port)");
			break;

		case 3:
			DEBUG3(printk("qla2x00(%ld): HBA in F P2P topology.\n",
					ha->host_no);)
			ha->operating_mode = P2P;
			ha->current_topology = ISP_CFG_F;
			strcpy(connect_type, "(F_Port)");
			break;

		default:
			DEBUG3(printk("qla2x00(%ld): HBA in unknown "
					"topology %x. Using NL.\n", 
					ha->host_no, topo);)
			ha->current_topology = ISP_CFG_NL;
			strcpy(connect_type, "(Loop)");
			break;
	}

	/* Save Host port and loop ID. */
	/* byte order - Big Endian */
	ha->d_id.b.domain = domain;
	ha->d_id.b.area = area;
	ha->d_id.b.al_pa = al_pa;

	if (!qla2x00_quiet)
		printk(KERN_INFO
			"scsi(%ld): Topology - %s, Host Loop address 0x%x\n",
			ha->host_no, connect_type, ha->loop_id);

	if (rval != 0) {
		/* Empty */
		DEBUG2_3(printk("%s(%ld): FAILED.\n", __func__, ha->host_no);)
	} else {
		/* Empty */
		DEBUG3(printk("%s(%ld): exiting normally.\n",
				__func__,
				ha->host_no);)
	}

	return(rval);
}

/**
 * qla2x00_config_dma_addressing() - Configure OS DMA addressing method.
 * @ha: HA context
 *
 * At exit, the @ha's flags.enable_64bit_addressing set to indicated
 * supported addressing method.
 */
static inline void qla2x00_config_dma_addressing(scsi_qla_host_t *ha);
static inline void
qla2x00_config_dma_addressing(scsi_qla_host_t *ha)
{
	/*
	 * Given the two variants pci_set_dma_mask(), allow the compiler to
	 * assist in setting the proper dma mask.
	 */
	if (sizeof(dma_addr_t) > 4) {
		ha->flags.enable_64bit_addressing = 1;
		/* Update our PCI device dma_mask for full 64 bit mask */
		if (pci_set_dma_mask(ha->pdev, 0xffffffffffffffffULL)) {
			printk("qla2x00: failed to set 64 bit PCI DMA mask, "
				"using 32 bits\n");
			ha->flags.enable_64bit_addressing = 0;
			pci_set_dma_mask(ha->pdev, 0xffffffff);
		}
	}
	else {
		ha->flags.enable_64bit_addressing = 0;
		pci_set_dma_mask(ha->pdev, 0xffffffff);
	}
	printk(KERN_INFO
		"scsi(%ld): %d Bit PCI Addressing Enabled.\n",
		ha->host_no,
		(ha->flags.enable_64bit_addressing ? 64 : 32));
}

#if defined(ISP2100)
/*
* NVRAM configuration for 2100.
*
* Input:
*      ha                = adapter block pointer.
*      ha->request_ring  = request ring virtual address
*      ha->response_ring = response ring virtual address
*      ha->request_dma   = request ring physical address
*      ha->response_dma  = response ring physical address
*
* Output:
*      initialization control block in response_ring
*      host adapters parameters in host adapter block
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2100_nvram_config(scsi_qla_host_t *ha)
{
	uint8_t   status = 0;
	uint16_t  cnt;
	init_cb_t *icb   = ha->init_cb;
	nvram21_t *nv    = (nvram21_t *)ha->request_ring;
	uint16_t  *wptr  = (uint16_t *)ha->request_ring;
	uint8_t   chksum = 0;

	ENTER(__func__);

	/* Only complete configuration once */
	if (ha->flags.nvram_config_done) {
		LEAVE(__func__);

		return (status);
	}

	/* Verify valid NVRAM checksum. */
	for (cnt = 0; cnt < sizeof(nvram21_t)/2; cnt++) {
		*wptr = qla2x00_get_nvram_word(ha, cnt);
		chksum += (uint8_t)*wptr;
		chksum += (uint8_t)(*wptr >> 8);
		wptr++;
	}

#if  DEBUG_PRINT_NVRAM
	printk("%s(): Contents of NVRAM\n", __func__);
	qla2x00_dump_buffer((uint8_t *)ha->request_ring, sizeof(nvram21_t));
#endif

	/* Bad NVRAM data, set defaults parameters. */
	if (chksum ||
		nv->id[0] != 'I' ||
		nv->id[1] != 'S' ||
		nv->id[2] != 'P' ||
		nv->id[3] != ' ' ||
		nv->nvram_version < 1) {

		/* Reset NVRAM data. */
		DEBUG(printk("Using defaults for NVRAM: \n"));
		DEBUG(printk("checksum=0x%x, Id=%c, version=0x%x\n",
				chksum,
				nv->id[0],
				nv->nvram_version));

		memset(nv, 0, sizeof(nvram21_t));

		/*
		 * Set default initialization control block.
		 */
		nv->parameter_block_version = ICB_VERSION;
		nv->firmware_options.enable_fairness = 1;
		nv->firmware_options.enable_fast_posting = 1;
		nv->firmware_options.enable_full_login_on_lip = 1;

		nv->frame_payload_size  = 1024;
		nv->max_iocb_allocation = 256;
		nv->execution_throttle  = 16;
		nv->retry_count         = 8;
		nv->retry_delay         = 1;
		nv->node_name[0]        = 32;
		nv->node_name[3]        = 224;
		nv->node_name[4]        = 139;
		nv->login_timeout       = 4;

		/*
		 * Set default host adapter parameters
		 */
		nv->host_p.enable_lip_full_login = 1;
		nv->reset_delay = 5;
		nv->port_down_retry_count = 8;
		nv->maximum_luns_per_target = 8;
		status = 1;
	}

	/*
	 * Copy over NVRAM RISC parameter block to initialization control
	 * block.
	 */
	cnt = (uint8_t *)&nv->host_p - (uint8_t *)&nv->parameter_block_version;
	memcpy((uint8_t *)icb,
			(uint8_t *)&nv->parameter_block_version, cnt);

	/* HBA node name 0 correction */
	for (cnt=0 ; cnt < 8 ; cnt++) {
		if (icb->node_name[cnt] != 0)
			break;
	}
	if (cnt == 8) {
		for (cnt= 0 ; cnt < 8 ; cnt++)
			icb->node_name[cnt] = icb->port_name[cnt];
		icb->node_name[0] = icb->node_name[0] & ~BIT_0;
		icb->port_name[0] = icb->port_name[0] |  BIT_0;
	}

	/*
	 * Setup driver firmware options.
	 */
	icb->firmware_options.enable_target_mode       = 0;
	icb->firmware_options.disable_initiator_mode   = 0;
	icb->firmware_options.enable_port_update_event = 1;
	icb->firmware_options.enable_full_login_on_lip = 1;

	/*
	 * Set host adapter parameters
	 */
	ha->flags.enable_target_mode = icb->firmware_options.enable_target_mode;
	ha->flags.disable_luns            = nv->host_p.disable_luns;
	ha->flags.disable_risc_code_load  = nv->host_p.disable_risc_code_load;
	ha->flags.set_cache_line_size_1   = nv->host_p.set_cache_line_size_1;
	ha->flags.enable_64bit_addressing = nv->host_p.enable_64bit_addressing;

	qla2x00_config_dma_addressing(ha);

	ha->flags.link_down_error_enable  = 1;

	ha->flags.enable_lip_reset        = nv->host_p.enable_lip_reset;
	ha->flags.enable_lip_full_login   = nv->host_p.enable_lip_full_login;
	ha->flags.enable_target_reset     = nv->host_p.enable_target_reset;
	ha->flags.enable_flash_db_update  = nv->host_p.enable_database_storage;

	/* new for IOCTL support of APIs */
	ha->node_name[0] = icb->node_name[0];
	ha->node_name[1] = icb->node_name[1];
	ha->node_name[2] = icb->node_name[2];
	ha->node_name[3] = icb->node_name[3];
	ha->node_name[4] = icb->node_name[4];
	ha->node_name[5] = icb->node_name[5];
	ha->node_name[6] = icb->node_name[6];
	ha->node_name[7] = icb->node_name[7];
	ha->nvram_version = nv->nvram_version;
	/* empty data for QLA2100s OEM stuff */
	ha->oem_id     = 0;
	ha->oem_spare0 = 0;
	for (cnt= 0 ; cnt < 8 ; cnt++) {
		ha->oem_string[cnt] = 0; 
		ha->oem_part[cnt]   = 0; 
		ha->oem_fru[cnt]    = 0; 
		ha->oem_ec[cnt]     = 0; 
	}

	ha->hiwat               = icb->iocb_allocation;
	ha->execution_throttle  = nv->execution_throttle;

	ha->retry_count         = nv->retry_count;
	ha->login_timeout       = nv->login_timeout;
	/* Set minimum login_timeout to 4 seconds. */
	if (ha->login_timeout < 4)
		ha->login_timeout = 4;
	ha->port_down_retry_count = nv->port_down_retry_count;
	ha->minimum_timeout = (ha->login_timeout * ha->retry_count)
				+ ha->port_down_retry_count;
	ha->loop_reset_delay = nv->reset_delay;

	/* Will get the value from nvram. */
	ha->loop_down_timeout     = LOOP_DOWN_TIMEOUT;
	ha->loop_down_abort_time  = LOOP_DOWN_TIME - ha->loop_down_timeout;

	/* save HBA serial number */
	ha->serial0 = nv->node_name[5];
	ha->serial1 = nv->node_name[6];
	ha->serial2 = nv->node_name[7];

	ha->max_probe_luns = le16_to_cpu(nv->maximum_luns_per_target);
	if (ha->max_probe_luns == 0)
		ha->max_probe_luns = MIN_LUNS;

	/* High-water mark of IOCBs */
	ha->iocb_hiwat = MAX_IOCBS_AVAILBALE;

#if  USE_BIOS_MAX_LUNS
	if (!nv->maximum_luns_per_target)
		ha->max_luns = MAX_LUNS-1;
	else
		ha->max_luns = nv->maximum_luns_per_target;
#else
	ha->max_luns = MAX_LUNS-1;
#endif

	ha->binding_type = Bind;
	if ((ha->binding_type != BIND_BY_PORT_NAME) &&
		(ha->binding_type != BIND_BY_PORT_ID) &&
		(ha->binding_type != BIND_BY_NODE_NAME)) {

		printk(KERN_WARNING
			"scsi(%ld): Invalid binding type specified "
			"(%d), defaulting to BIND_BY_PORT_NAME!!!\n",
			ha->host_no,
			ha->binding_type);
		ha->binding_type = BIND_BY_PORT_NAME;
	}

	/*
	 * Setup ring parameters in initialization control block
	 */
	icb->request_q_outpointer  = 0;
	icb->response_q_inpointer  = 0;
	icb->request_q_length      = REQUEST_ENTRY_CNT;
	icb->response_q_length     = RESPONSE_ENTRY_CNT;
	icb->request_q_address[0]  = LS_64BITS(ha->request_dma);
	icb->request_q_address[1]  = MS_64BITS(ha->request_dma);
	icb->response_q_address[0] = LS_64BITS(ha->response_dma);
	icb->response_q_address[1] = MS_64BITS(ha->response_dma);

	ha->flags.nvram_config_done = 1;

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status)
		printk(KERN_WARNING
			"%s(): **** FAILED ****\n", __func__);
#endif

	LEAVE(__func__);

	return(status);
}
#else
/*
* NVRAM configuration for the 2200/2300/2312
*
* Input:
*      ha                = adapter block pointer.
*      ha->request_ring  = request ring virtual address
*      ha->response_ring = response ring virtual address
*      ha->request_dma   = request ring physical address
*      ha->response_dma  = response ring physical address
*
* Output:
*      initialization control block in response_ring
*      host adapters parameters in host adapter block
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_nvram_config(scsi_qla_host_t *ha)
{
#if defined(ISP2300)
	device_reg_t *reg = ha->iobase;
	uint16_t  data;
#endif
	struct qla2xxx_host_p host_p;
	struct qla2x00_firmware_options firmware_options;
	struct qla2x00_additional_firmware_options additional_firmware_options;
	struct qla2x00_seriallink_firmware_options serial_options; 

	uint8_t   status = 0;
	uint8_t   chksum = 0;
	uint16_t  cnt, base;
	uint8_t   *dptr1, *dptr2;
	init_cb_t *icb   = ha->init_cb;
	nvram22_t *nv    = (nvram22_t *)ha->request_ring;
	uint16_t  *wptr  = (uint16_t *)ha->request_ring;

	ENTER(__func__);

	if (!ha->flags.nvram_config_done) {
#if defined(ISP2300)
		if (ha->device_id == QLA2312_DEVICE_ID) {
			data = RD_REG_WORD(&reg->ctrl_status);
			if ((data >> 14) == 1)
				base = 0x80;
			else
				base = 0;
			data = RD_REG_WORD(&reg->nvram);
			while (data & NV_BUSY) {
				UDELAY(100);
				data = RD_REG_WORD(&reg->nvram);
			}

			/* Lock resource */
			WRT_REG_WORD(&reg->host_semaphore, 0x1);
			UDELAY(5);
			data = RD_REG_WORD(&reg->host_semaphore);
			while ((data & BIT_0) == 0) {
				/* Lock failed */
				UDELAY(100);
				WRT_REG_WORD(&reg->host_semaphore, 0x1);
				UDELAY(5);
				data = RD_REG_WORD(&reg->host_semaphore);
			}
		} else
			base = 0;
#else
		base = 0;
#endif
		/* Verify valid NVRAM checksum. */
		for (cnt = 0; cnt < sizeof(nvram22_t)/2; cnt++) {
	 	 	*wptr = cpu_to_le16(
			    qla2x00_get_nvram_word(ha, (cnt+base)));
			chksum += (uint8_t)*wptr;
			chksum += (uint8_t)(*wptr >> 8);
			wptr++;
		}
#if defined(ISP2300)
		if (ha->device_id == QLA2312_DEVICE_ID) {
			/* Unlock resource */
			WRT_REG_WORD(&reg->host_semaphore, 0);
		}
#endif

#if  DEBUG_PRINT_NVRAM
		printk("%s(): Contents of NVRAM\n", __func__);
		qla2x00_dump_buffer((uint8_t *)ha->request_ring,
					sizeof(nvram22_t));
#endif
		/* Bad NVRAM data, set defaults parameters. */
		if (chksum ||
			nv->id[0] != 'I' || 
			nv->id[1] != 'S' || 
			nv->id[2] != 'P' ||
			nv->id[3] != ' ' || 
			nv->nvram_version < 1) {

			/* Reset NVRAM data. */
			DEBUG(printk("Using defaults for NVRAM: \n"));
			DEBUG(printk("checksum=0x%x, Id=%c, version=0x%x\n",
					chksum,
					nv->id[0],
					nv->nvram_version));

			memset(nv, 0, sizeof(nvram22_t));

			/*
			 * Set default initialization control block.
			 */
			nv->parameter_block_version = ICB_VERSION;

			*((uint16_t *) &firmware_options) =
			    le16_to_cpu(*((uint16_t *) &nv->firmware_options));

			firmware_options.enable_fairness = 1;
			firmware_options.enable_fast_posting = 1;
			firmware_options.enable_full_login_on_lip = 1;
			firmware_options.expanded_ifwcb = 1;

			*((uint16_t *) &nv->firmware_options) =
			    cpu_to_le16(*((uint16_t *) &firmware_options));

			nv->frame_payload_size  = __constant_cpu_to_le16(1024);
			nv->max_iocb_allocation = __constant_cpu_to_le16(256);
			nv->execution_throttle  = __constant_cpu_to_le16(16);

			nv->retry_count         = 8;
			nv->retry_delay         = 1;
			nv->port_name[0]        = 32;
			nv->port_name[3]        = 224;
			nv->port_name[4]        = 139;
			nv->login_timeout       = 4;

			*((uint16_t *) &additional_firmware_options) =
			    le16_to_cpu(*((uint16_t *)
				&nv->additional_firmware_options));

			additional_firmware_options.connection_options =
#if defined(ISP2200)
					P2P_LOOP;
#else
					LOOP_P2P;
#endif

			*((uint16_t *) &nv->additional_firmware_options) =
			    cpu_to_le16(*((uint16_t *)
				&additional_firmware_options));

			/*
			 * Set default host adapter parameters
			 */

			*((uint16_t *) &host_p) =
			    le16_to_cpu(*((uint16_t *) &nv->host_p));

			host_p.enable_lip_full_login = 1;

			*((uint16_t *) &nv->host_p) =
			    cpu_to_le16(*((uint16_t *) &host_p));

			nv->reset_delay = 5;
			nv->port_down_retry_count = 30;
			nv->maximum_luns_per_target = __constant_cpu_to_le16(8);
			status = 1;
		}

		/* Reset NVRAM data. */
		memset(icb, 0, sizeof(init_cb_t));

		/*
		 * Copy over NVRAM RISC parameter block to initialization
		 * control block.
		 */
		dptr1 = (uint8_t *)icb;
		dptr2 = (uint8_t *)&nv->parameter_block_version;
		cnt = (uint8_t *)&nv->additional_firmware_options - 
			(uint8_t *)&nv->parameter_block_version;
		while (cnt--)
			*dptr1++ = *dptr2++;

		dptr1 += (uint8_t *)&icb->additional_firmware_options - 
				(uint8_t *)&icb->request_q_outpointer;
		cnt = (uint8_t *)&nv->serial_options - 
			(uint8_t *)&nv->additional_firmware_options;
		while (cnt--)
			*dptr1++ = *dptr2++;

		/*
		 * Get the three bit fields.
		 */
		*((uint16_t *) &firmware_options) =
		    le16_to_cpu(*((uint16_t *) &icb->firmware_options));

		*((uint16_t *) &additional_firmware_options) =
		    le16_to_cpu(*((uint16_t *)
			&icb->additional_firmware_options));

		*((uint16_t *) &host_p) =
		    le16_to_cpu(*((uint16_t *) &nv->host_p));
        
		if (!firmware_options.node_name_option) {
			/*
			 * Firmware will apply the following mask if the
			 * nodename was not provided.
			 */
			memcpy(icb->node_name, icb->port_name, WWN_SIZE);
			icb->node_name[0] &= 0xF0;
		}

		/*
		 * Setup driver firmware options.
		 */

		firmware_options.enable_full_duplex       = 0;
		firmware_options.enable_target_mode       = 0;
		firmware_options.disable_initiator_mode   = 0;
		firmware_options.enable_port_update_event = 1;
		firmware_options.enable_full_login_on_lip = 1;
#if defined(ISP2300)
		firmware_options.enable_fast_posting = 0;
#endif
#if !defined(FC_IP_SUPPORT)
		/* Enable FC-Tape support */
		firmware_options.expanded_ifwcb = 1;
		additional_firmware_options.enable_fc_tape = 1;
		additional_firmware_options.enable_fc_confirm = 1;
#endif
		/*
		 * Set host adapter parameters
		 */
		ha->flags.enable_target_mode = firmware_options.enable_target_mode;
		ha->flags.disable_luns = host_p.disable_luns;
		ha->flags.disable_risc_code_load = host_p.disable_risc_code_load;
		ha->flags.set_cache_line_size_1 = host_p.set_cache_line_size_1;
		ha->flags.enable_64bit_addressing = host_p.enable_64bit_addressing;

		qla2x00_config_dma_addressing(ha);

		ha->flags.enable_lip_reset = host_p.enable_lip_reset;
		ha->flags.enable_lip_full_login = host_p.enable_lip_full_login;
		ha->flags.enable_target_reset = host_p.enable_target_reset;
		ha->flags.enable_flash_db_update = host_p.enable_database_storage;
		ha->operating_mode = additional_firmware_options.connection_options;

		/*
		 * Set serial firmware options
		 */
		*((uint16_t *) &serial_options) = 
			le16_to_cpu(*((uint16_t *) &nv->serial_options));
		ha->fw_seriallink_options = serial_options;

		/*
		 * Put back any changes made to the bit fields.
		 */
		*((uint16_t *) &icb->firmware_options) =
		    cpu_to_le16(*((uint16_t *) &firmware_options));

		*((uint16_t *) &icb->additional_firmware_options) =
		    cpu_to_le16(*((uint16_t *) &additional_firmware_options));

		/* new for IOCTL support of APIs */
		ha->node_name[0] = icb->node_name[0];
		ha->node_name[1] = icb->node_name[1];
		ha->node_name[2] = icb->node_name[2];
		ha->node_name[3] = icb->node_name[3];
		ha->node_name[4] = icb->node_name[4];
		ha->node_name[5] = icb->node_name[5];
		ha->node_name[6] = icb->node_name[6];
		ha->node_name[7] = icb->node_name[7];
		ha->nvram_version = nv->nvram_version;

		ha->hiwat = le16_to_cpu(icb->iocb_allocation);
		ha->execution_throttle = le16_to_cpu(nv->execution_throttle);
		if (nv->login_timeout < ql2xlogintimeout)
			nv->login_timeout = ql2xlogintimeout;

		icb->execution_throttle = __constant_cpu_to_le16(0xFFFF);
		ha->retry_count = nv->retry_count;
		/* Set minimum login_timeout to 4 seconds. */
		if (nv->login_timeout < 4)
			nv->login_timeout = 4;
		ha->login_timeout = nv->login_timeout;
		icb->login_timeout = nv->login_timeout;
		ha->port_down_retry_count = nv->port_down_retry_count;
		ha->minimum_timeout = (ha->login_timeout * ha->retry_count) +
					ha->port_down_retry_count;
		ha->loop_reset_delay = nv->reset_delay;
		/* Will get the value from nvram. */
		ha->loop_down_timeout = LOOP_DOWN_TIMEOUT;
		ha->loop_down_abort_time = LOOP_DOWN_TIME - 
						ha->loop_down_timeout;

		/* save HBA serial number */
		ha->serial0 = nv->port_name[5];
		ha->serial1 = nv->port_name[6];
		ha->serial2 = nv->port_name[7];
		ha->flags.link_down_error_enable  = 1;
		/* save OEM related items for QLA2200s and QLA2300s */
		ha->oem_id = nv->oem_id;
		ha->oem_spare0 = nv->oem_spare0;
		for (cnt = 2; cnt < 8; cnt++)
			ha->oem_string[cnt] = nv->oem_string[cnt];

		for (cnt = 0; cnt < 8; cnt++) {
			ha->oem_part[cnt] = nv->oem_part[cnt];
			ha->oem_fru[cnt] = nv->oem_fru[cnt];
			ha->oem_ec[cnt] = nv->oem_ec[cnt];
		}

#if defined(FC_IP_SUPPORT)
		memcpy(ha->ip_port_name, nv->port_name, WWN_SIZE);
#endif

		ha->max_probe_luns = le16_to_cpu(nv->maximum_luns_per_target);
		if (ha->max_probe_luns == 0)
			ha->max_probe_luns = MIN_LUNS;

		/* High-water mark of IOCBs */
		ha->iocb_hiwat = MAX_IOCBS_AVAILBALE;

#if USE_BIOS_MAX_LUNS
		if (!nv->maximum_luns_per_target)
			ha->max_luns = MAX_LUNS;
		else if (nv->maximum_luns_per_target < MAX_LUNS)
			ha->max_luns = le16_to_cpu(nv->maximum_luns_per_target);
		else
			ha->max_luns = MAX_LUNS;
#else
		ha->max_luns = MAX_LUNS;
#endif

		ha->binding_type = Bind;
		if ((ha->binding_type != BIND_BY_PORT_NAME) &&
			(ha->binding_type != BIND_BY_PORT_ID) &&
			(ha->binding_type != BIND_BY_NODE_NAME)) {

			printk(KERN_WARNING
				"scsi(%ld): Invalid binding type specified "
				"(%d), defaulting to BIND_BY_PORT_NAME!!!\n",
				ha->host_no,
				ha->binding_type);
			ha->binding_type = BIND_BY_PORT_NAME;
		}

		/*
		 * Need enough time to try and get the port back.
		 */
		if (qlport_down_retry)
			ha->port_down_retry_count = qlport_down_retry;
#if defined(COMPAQ)
		else if (ha->port_down_retry_count < HSG80_PORT_RETRY_COUNT)
			ha->port_down_retry_count = HSG80_PORT_RETRY_COUNT;
#endif
		/* Set login_retry_count */
		ha->login_retry_count  = nv->retry_count;
		if (ha->port_down_retry_count == nv->port_down_retry_count &&
			ha->port_down_retry_count > 3)
			ha->login_retry_count = ha->port_down_retry_count;
		else if ( ha->port_down_retry_count > ha->login_retry_count )
			ha->login_retry_count = ha->port_down_retry_count;

		/*
		 * Setup ring parameters in initialization control block
		 */
		icb->request_q_outpointer  = __constant_cpu_to_le16(0);
		icb->response_q_inpointer  = __constant_cpu_to_le16(0);
		icb->request_q_length      =
			__constant_cpu_to_le16(REQUEST_ENTRY_CNT);
		icb->response_q_length     =
			__constant_cpu_to_le16(RESPONSE_ENTRY_CNT);
		icb->request_q_address[0]  =
			cpu_to_le32(LS_64BITS(ha->request_dma));
		icb->request_q_address[1]  =
			cpu_to_le32(MS_64BITS(ha->request_dma));
		icb->response_q_address[0] =
			cpu_to_le32(LS_64BITS(ha->response_dma));
		icb->response_q_address[1] =
			cpu_to_le32(MS_64BITS(ha->response_dma));

		icb->lun_enables = __constant_cpu_to_le16(0);
		icb->command_resource_count = 0;
		icb->immediate_notify_resource_count = 0;
		icb->timeout = __constant_cpu_to_le16(0);
		icb->reserved_3 = __constant_cpu_to_le16(0);

		ha->flags.nvram_config_done = 1;
	}

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status)
		printk(KERN_WARNING
			"%s(): **** FAILED ****\n", __func__);
#endif

	LEAVE(__func__);

	return (status);
}
#endif	/* #if defined(ISP2100) */

/*
* Get NVRAM data word
*      Calculates word position in NVRAM and calls request routine to
*      get the word from NVRAM.
*
* Input:
*      ha      = adapter block pointer.
*      address = NVRAM word address.
*
* Returns:
*      data word.
*/
STATIC uint16_t
qla2x00_get_nvram_word(scsi_qla_host_t *ha, uint32_t address)
{
	uint32_t nv_cmd;
	uint16_t data;

#if defined(QL_DEBUG_ROUTINES)
	uint8_t  saved_print_status = ql2x_debug_print;
#endif

	DEBUG4(printk("qla2100_get_nvram_word: entered\n");)

	nv_cmd = address << 16;
	nv_cmd |= NV_READ_OP;

#if defined(QL_DEBUG_ROUTINES)
	ql2x_debug_print = FALSE;
#endif

	data = qla2x00_nvram_request(ha, nv_cmd);
#if defined(QL_DEBUG_ROUTINES)
	ql2x_debug_print = saved_print_status;
#endif

	DEBUG4(printk("qla2100_get_nvram_word: exiting normally "
			"NVRAM data=%lx.\n",
			(u_long)data);)

	return(data);
}

/*
* NVRAM request
*      Sends read command to NVRAM and gets data from NVRAM.
*
* Input:
*      ha     = adapter block pointer.
*      nv_cmd = Bit 26     = start bit
*               Bit 25, 24 = opcode
*               Bit 23-16  = address
*               Bit 15-0   = write data
*
* Returns:
*      data word.
*/
STATIC uint16_t
qla2x00_nvram_request(scsi_qla_host_t *ha, uint32_t nv_cmd)
{
	uint8_t      cnt;
	device_reg_t *reg = ha->iobase;
	uint16_t     data = 0;
	uint16_t     reg_data;

	/* Send command to NVRAM. */
	nv_cmd <<= 5;
	for (cnt = 0; cnt < 11; cnt++) {
		if (nv_cmd & BIT_31)
			qla2x00_nv_write(ha, NV_DATA_OUT);
		else
			qla2x00_nv_write(ha, 0);
		nv_cmd <<= 1;
	}

	/* Read data from NVRAM. */
	for (cnt = 0; cnt < 16; cnt++) {
		WRT_REG_WORD(&reg->nvram, NV_SELECT+NV_CLOCK);
		/* qla2x00_nv_delay(ha); */
		NVRAM_DELAY();
		data <<= 1;
		reg_data = RD_REG_WORD(&reg->nvram);
		if (reg_data & NV_DATA_IN)
			data |= BIT_0;
		WRT_REG_WORD(&reg->nvram, NV_SELECT);
		/* qla2x00_nv_delay(ha); */
		NVRAM_DELAY();
	}

	/* Deselect chip. */
	WRT_REG_WORD(&reg->nvram, NV_DESELECT);
	/* qla2x00_nv_delay(ha); */
	NVRAM_DELAY();

	return(data);
}

STATIC void
qla2x00_nv_write(scsi_qla_host_t *ha, uint16_t data)
{
	device_reg_t *reg = ha->iobase;

	WRT_REG_WORD(&reg->nvram, data | NV_SELECT);
	NVRAM_DELAY();
	/* qla2x00_nv_delay(ha); */
	WRT_REG_WORD(&reg->nvram, data | NV_SELECT | NV_CLOCK);
	/* qla2x00_nv_delay(ha); */
	NVRAM_DELAY();
	WRT_REG_WORD(&reg->nvram, data | NV_SELECT);
	/* qla2x00_nv_delay(ha); */
	NVRAM_DELAY();
}

STATIC void
qla2x00_nv_deselect(scsi_qla_host_t *ha)
{
	device_reg_t *reg = ha->iobase;

	WRT_REG_WORD(&reg->nvram, NV_DESELECT);
	NVRAM_DELAY();
}

/*
* qla2x00_poll
*      Polls ISP for interrupts.
*
* Input:
*      ha = adapter block pointer.
*/
STATIC void
qla2x00_poll(scsi_qla_host_t *ha)
{
	unsigned long flags = 0;
	device_reg_t *reg   = ha->iobase;
	uint8_t     discard;
	uint16_t     data;

	ENTER(__func__);

#ifdef __VMWARE__
   /* This function is only called from qla2x00_ms_req_pkt and
    * qla2x00_req_pkt. Since these functions drop the hardware
    * lock and we immediately regrab it here, we drop releasing
    * it there and drop grabbing it here. -- Thor
    */
#else
	/* Acquire interrupt specific lock */
	spin_lock_irqsave(&ha->hardware_lock, flags);
#endif

	/* Check for pending interrupts. */
#if defined(ISP2100) || defined(ISP2200)
	data = RD_REG_WORD(&reg->istatus);
	if (data & RISC_INT)
		qla2x00_isr(ha, data, &discard);
#else
	if (ha->device_id == QLA2312_DEVICE_ID) {
		data = RD_REG_WORD(&reg->istatus);
		if (data & RISC_INT) {
			data = RD_REG_WORD(&reg->host_status_lo);
			qla2x00_isr(ha, data, &discard);
		}

	} else {
		data = RD_REG_WORD(&reg->host_status_lo);
		if (data & HOST_STATUS_INT)
			qla2x00_isr(ha, data, &discard);
	}
#endif
#ifdef __VMWARE__
   /* Again, we do not grab and release this lock since the caller
    * already has this lock. The effect is that tasklet_schedule below
    * is called with interrupts disabled, which is fine. -- Thor
    */
#else
	/* Release interrupt specific lock */
	spin_unlock_irqrestore(&ha->hardware_lock, flags);
#endif

	if (!list_empty(&ha->done_queue))
#if QLA2X_PERFORMANCE
		tasklet_schedule(&ha->run_qla_task);
#else
		qla2x00_done(ha);
#endif

	LEAVE(__func__);
}

/*
*  qla2x00_restart_isp
*      restarts the ISP after a reset
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success
*/
int
qla2x00_restart_isp(scsi_qla_host_t *ha)
{
	uint8_t		status = 0;
#if defined(ISP2300) && defined(CONFIG_SCSI_QLOGIC_23XX_PARITY)
	device_reg_t	*reg;
	unsigned long	flags = 0;
#endif

	/* If firmware needs to be loaded */
	if (qla2x00_isp_firmware(ha)) {
		ha->flags.online = FALSE;
		if (!(status = qla2x00_chip_diag(ha))) {
#if defined(ISP2300) && defined(CONFIG_SCSI_QLOGIC_23XX_PARITY)
			reg = ha->iobase;
			spin_lock_irqsave(&ha->hardware_lock, flags);
			/* Disable SRAM, Instruction RAM and GP RAM parity. */
			WRT_REG_WORD(&reg->host_cmd, (HC_ENABLE_PARITY + 0x0));
			spin_unlock_irqrestore(&ha->hardware_lock, flags);
#endif

			status = qla2x00_setup_chip(ha);

#if defined(ISP2300) && defined(CONFIG_SCSI_QLOGIC_23XX_PARITY)
			spin_lock_irqsave(&ha->hardware_lock, flags);

			/* Enable proper parity */
			if (ha->device_id == QLA2312_DEVICE_ID)
				/* SRAM, Instruction RAM and GP RAM parity */
				WRT_REG_WORD(&reg->host_cmd,
				    (HC_ENABLE_PARITY + 0x7));
			else
				/* SRAM parity */
				WRT_REG_WORD(&reg->host_cmd,
				    (HC_ENABLE_PARITY + 0x1));

			spin_unlock_irqrestore(&ha->hardware_lock, flags);
#endif
		}
	}
	if (!status && !(status = qla2x00_init_rings(ha))) {
		clear_bit(RESET_MARKER_NEEDED, &ha->dpc_flags);
		clear_bit(COMMAND_WAIT_NEEDED, &ha->dpc_flags);
		if (!(status = qla2x00_fw_ready(ha))) {
			DEBUG(printk("%s(): Start configure loop, "
					"status = %d\n",
					__func__,
					status);)
			ha->flags.online = TRUE;
			do {
				clear_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags);
				qla2x00_configure_loop(ha);
			} while (!atomic_read(&ha->loop_down_timer) &&
				!(test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags)) &&
				(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)));
		}

		/* if no cable then assume it's good */
		if ((ha->device_flags & DFLG_NO_CABLE)) 
			status = 0;

		DEBUG(printk("%s(): Configure loop done, status = 0x%x\n",
				__func__,
				status);)
	}
	return (status);
}

/*
*  qla2x00_abort_isp
*      Resets ISP and aborts all outstanding commands.
*
* Input:
*      ha           = adapter block pointer.
*
* Returns:
*      0 = success
*/
STATIC uint8_t
qla2x00_abort_isp(scsi_qla_host_t *ha, uint8_t flag)
{
	unsigned long flags = 0;
	uint16_t       cnt;
	srb_t          *sp;
	uint8_t        status = 0;
#ifdef PERF_MONITORING
	os_lun_t	*lq;
#endif

	ENTER("qla2x00_abort_isp");

	if (ha->flags.online) {
		ha->flags.online = FALSE;
		clear_bit(COMMAND_WAIT_NEEDED, &ha->dpc_flags);
		clear_bit(COMMAND_WAIT_ACTIVE, &ha->dpc_flags);
		clear_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		qla2x00_stats.ispAbort++;
		ha->total_isp_aborts++;  /* used by ioctl */
		ha->sns_retry_cnt = 0;

		printk(KERN_INFO
			"qla2x00(%ld): Performing ISP error recovery - ha= %p.\n", 
			ha->host_no,ha);
		qla2x00_reset_chip(ha);

		if (ha->loop_state != LOOP_DOWN) {
			ha->loop_state = LOOP_DOWN;
			atomic_set(&ha->loop_down_timer, LOOP_DOWN_TIME);
			qla2x00_mark_all_devices_lost(ha);
		}

#if defined(FC_IP_SUPPORT)
		/* Return all IP send packets */
		for (cnt = 0; cnt < MAX_SEND_PACKETS; cnt++) {
			if (ha->active_scb_q[cnt] != NULL) {
				/* Via IP callback */
				(*ha->send_completion_routine)
					(ha->active_scb_q[cnt]);

				ha->active_scb_q[cnt] = NULL;
			}
		}
#endif

		spin_lock_irqsave(&ha->hardware_lock, flags);
		/* Requeue all commands in outstanding command list. */
		for (cnt = 1; cnt < MAX_OUTSTANDING_COMMANDS; cnt++) {
			sp = ha->outstanding_cmds[cnt];
			if (sp) {
				ha->outstanding_cmds[cnt] = 0;
				if( ha->actthreads )
					ha->actthreads--;
				sp->lun_queue->out_cnt--;
				ha->iocb_cnt -= sp->iocb_cnt;
#ifdef PERF_MONITORING
				/* update stats */
				lq = sp->lun_queue;
				lq->resp_time += jiffies - sp->u_start; 
				lq->act_time += jiffies - sp->r_start;  
#endif
				
				sp->flags = 0;

				/* 
				 * We need to send the command back to OS now 
				 * if returning RESET status for kernel's 
				 * error handling.
				 */
				if (flag == 0) {
					CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;
				} else {
					CMD_RESULT(sp->cmd) = DID_RESET << 16;
				}
				CMD_HANDLE(sp->cmd) = (unsigned char *) NULL;
				add_to_done_queue(ha, sp);
			}
		}

		spin_unlock_irqrestore(&ha->hardware_lock, flags);

#if defined(ISP2100)
		qla2100_nvram_config(ha);
#else
		qla2x00_nvram_config(ha);
#endif

		if (!qla2x00_restart_isp(ha)) {
			clear_bit(RESET_MARKER_NEEDED, &ha->dpc_flags);

			if (!atomic_read(&ha->loop_down_timer)) {
				/*
				 * Issue marker command only when we are going
				 * to start the I/O .
				 */
				ha->marker_needed = 1;
			}

			ha->flags.online = TRUE;

			/* Enable target response to SCSI bus. */
			if (ha->flags.enable_target_mode)
				qla2x00_enable_lun(ha);

#if defined(FC_IP_SUPPORT)
			/* Reenable IP support */
			if (ha->flags.enable_ip) {
				set_bit(REGISTER_FC4_NEEDED, &ha->dpc_flags);
				qla2x00_ip_initialize(ha);
			}
#endif
			/* Enable ISP interrupts. */
			qla2x00_enable_intrs(ha);

			/* v2.19.5b6 Return all commands */
			qla2x00_abort_queues(ha, TRUE);

			/* Restart queues that may have been stopped. */
			qla2x00_restart_queues(ha,TRUE);
			ha->isp_abort_cnt = 0; 
			clear_bit(ISP_ABORT_RETRY, &ha->dpc_flags);
		} else {	/* failed the ISP abort */
			ha->flags.online = TRUE;
			if( test_bit(ISP_ABORT_RETRY, &ha->dpc_flags) ){
				if( ha->isp_abort_cnt == 0 ){
					printk(KERN_WARNING
					"qla2x00(%ld): ISP error recovery failed - "
					"board disabled\n",ha->host_no);
					/* 
					 * The next call disables the board
					 * completely.
					 */
					qla2x00_reset_adapter(ha);
					qla2x00_abort_queues(ha, FALSE);
					ha->flags.online = TRUE;
					clear_bit(ISP_ABORT_RETRY, &ha->dpc_flags);
					status = 0;
				} else { /* schedule another ISP abort */
					ha->isp_abort_cnt--;
					DEBUG(printk("qla%ld: ISP abort - retry remainning %d\n",
					ha->host_no, 
					ha->isp_abort_cnt);)
					status = 1;
				}
			} else {
				ha->isp_abort_cnt = MAX_RETRIES_OF_ISP_ABORT;
				DEBUG(printk( "qla2x00(%ld): ISP error recovery - "
				"retrying (%d) more times\n",ha->host_no,
				ha->isp_abort_cnt);)
				set_bit(ISP_ABORT_RETRY, &ha->dpc_flags);
				status = 1;
			}
		}
		       
	}

	if (status) {
		printk(KERN_INFO
			"qla2x00_abort_isp(%ld): **** FAILED ****\n",
			ha->host_no);
	} else {
		DEBUG(printk(KERN_INFO
				"qla2x00_abort_isp(%ld): exiting.\n",
				ha->host_no);)
	}

	return(status);
}

/*
* qla2x00_init_fc_db
*      Initializes Fibre Channel Device Database.
*
* Input:
*      ha = adapter block pointer.
*
* Output:
*      ha->fc_db = initialized
*/
STATIC void
qla2x00_init_fc_db(scsi_qla_host_t *ha)
{
	uint16_t cnt;

	ENTER(__func__);

	/* Initialize fc database if it is not initialized. */
	if (!ha->fc_db[0].loop_id && !ha->fc_db[1].loop_id) {
		ha->flags.updated_fc_db = FALSE;

		/* Initialize target database. */
		for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
			ha->fc_db[cnt].name[0] = 0L;
			ha->fc_db[cnt].name[1] = 0L;
			ha->fc_db[cnt].loop_id = PORT_UNUSED;
			ha->fc_db[cnt].port_login_retry_count =
				ha->port_down_retry_count * PORT_RETRY_TIME;
			ha->fc_db[cnt].flag = 0;   /* v2.19.5b3 */
		}

#if USE_FLASH_DATABASE
		/* Move flash database to driver database. */
		qla2x00_get_database(ha);
#endif
	}

	LEAVE(__func__);
}


/*
* qla2x00_init_tgt_map
*      Initializes target map.
*
* Input:
*      ha = adapter block pointer.
*
* Output:
*      TGT_Q initialized
*/
STATIC void
qla2x00_init_tgt_map(scsi_qla_host_t *ha)
{
	uint32_t t;

	ENTER(__func__);

	for (t = 0; t < MAX_TARGETS; t++)
		TGT_Q(ha, t) = (os_tgt_t *) NULL;

	LEAVE(__func__);
}


/*
* qla2x00_reset_adapter
*      Reset adapter.
*
* Input:
*      ha = adapter block pointer.
*/
STATIC void
qla2x00_reset_adapter(scsi_qla_host_t *ha)
{
	unsigned long flags = 0;
	device_reg_t *reg = ha->iobase;

	ENTER(__func__);

	ha->flags.online = FALSE;
	qla2x00_disable_intrs(ha);
	/* WRT_REG_WORD(&reg->ictrl, 0); */
	/* Reset RISC processor. */
	spin_lock_irqsave(&ha->hardware_lock, flags);
	WRT_REG_WORD(&reg->host_cmd, HC_RESET_RISC);
	WRT_REG_WORD(&reg->host_cmd, HC_RELEASE_RISC);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	LEAVE(__func__);
}

/*
* qla2x00_loop_reset
*      Issue loop reset.
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success
*/
STATIC uint8_t
qla2x00_loop_reset(scsi_qla_host_t *ha)
{
	uint8_t  status = QL_STATUS_SUCCESS;
	uint16_t t;
	os_tgt_t        *tq;

	ENTER(__func__);

	if (ha->flags.enable_lip_reset) {
		status = qla2x00_lip_reset(ha);
	}

	if (status == QL_STATUS_SUCCESS && ha->flags.enable_target_reset) {
		for (t = 0; t < MAX_FIBRE_DEVICES; t++) {
			if ((tq = TGT_Q(ha, t)) == NULL)
				continue;

			if (tq->vis_port == NULL)
				continue;

			status = qla2x00_target_reset(ha, 0, t);
#ifndef __VMWARE__NO_BUG_FIX
			/* Ignore error from qla2x00_target_reset(),
			 * because it is always returning an error in the
			 * multipath driver. */
#else
			if (status != QL_STATUS_SUCCESS) {
				break;
			}
#endif
		}
	}

	if (

#ifdef __VMWARE__NO_BUG_FIX
		status == QL_STATUS_SUCCESS &&
#else
   /* Do not look at status, since it may very well be the result of
    * the last target reset and will not tell whether the lip_reset
    * was completed successfully!!!
    */
#endif
		((!ha->flags.enable_target_reset && 
		  !ha->flags.enable_lip_reset) ||
		ha->flags.enable_lip_full_login)) {

#ifdef __VMWARE__
        printk("Doing full login LIP\n");
#endif
		status = qla2x00_full_login_lip(ha);
	}

	/* Issue marker command only when we are going to start the I/O */
	ha->marker_needed = 1;

	if (status) {
		/* Empty */
		DEBUG2_3(printk("%s(%ld): **** FAILED ****\n",
				__func__,
				ha->host_no);)
	} else {
		/* Empty */
		DEBUG3(printk("%s(%ld): exiting normally.\n",
				__func__,
				ha->host_no);)
	}

	LEAVE(__func__);

	return(status);
}

/*
 * qla2x00_device_reset
 *	Issue bus device reset message to the target.
 *
 * Input:
 *	ha = adapter block pointer.
 *	t = SCSI ID.
 *	TARGET_QUEUE_LOCK must be released.
 *	ADAPTER_STATE_LOCK must be released.
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_device_reset(scsi_qla_host_t *vis_ha, uint16_t tgt, uint16_t lun)
{
#if !USE_ABORT_TGT
	uint16_t	l;
	fc_port_t	*fcport;
#endif
	os_lun_t	*lq;
	uint8_t		status = 0;

	ENTER(__func__);

#if USE_ABORT_TGT
	/* Abort Target command will clear Reservation */
	lq = GET_LU_Q(vis_ha, tgt, lun);
	if (lq && lq->fclun)
		 status = qla2x00_abort_target(lq->fclun->fcport);
#else
	/* Abort Device command will not clear Reservation */
	for (l = 0; l < MAX_LUNS; l++) {
		lq = GET_LU_Q(vis_ha, tgt, l);
		if (lq == NULL)
			continue;

		fcport = lq->fclun->fcport;
		if (LOOP_RDY(fcport->ha)) {
			qla2x00_abort_device(fcport->ha, 
					fcport->loop_id,
					lq->fclun->lun);
		}
	}
#endif

	LEAVE(__func__);

	return( status );
}

/*
 *  Issue marker command.
 *	Function issues marker IOCB.
 *
 * Input:
 *	ha = adapter block pointer.
 *	loop_id = loop ID
 *	lun = LUN
 *	type = marker modifier
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel/Interrupt context.
 */
STATIC int
__qla2x00_marker(scsi_qla_host_t *ha, uint16_t loop_id, 
		uint16_t lun, uint8_t type)
{
	mrk_entry_t	*pkt;

	ENTER(__func__);

	pkt = (mrk_entry_t *)qla2x00_req_pkt(ha);
	if (pkt == NULL) {
		DEBUG2_3(printk("%s(): **** FAILED ****\n", __func__);)

		return (QLA2X00_FUNCTION_FAILED);
	}

	pkt->entry_type = MARKER_TYPE;
	pkt->modifier = type;

	if (type != MK_SYNC_ALL) {
		pkt->lun = cpu_to_le16(lun);
		pkt->target = (uint8_t)loop_id;
	}

	/* Issue command to ISP */
	qla2x00_isp_cmd(ha);

	LEAVE(__func__);

	return (QLA2X00_SUCCESS);
}


/**
 * qla2x00_check_request_ring() - Checks request ring for additional IOCB space.
 * @ha: HA context
 * @tot_iocbs: Number of IOCBs required
 * @req_ring_index: Current index to request ring
 * @req_q_cnt: Number of free request entries
 *
 * Returns non-zero if no additional room available on request ring, else zero.
 */
static inline uint16_t
qla2x00_check_request_ring(
		scsi_qla_host_t *ha, uint16_t tot_iocbs,
		uint16_t req_ring_index, uint16_t *req_q_cnt)
{
	uint16_t	status;
	uint16_t	cnt;
	device_reg_t	*reg;

	reg = ha->iobase;

	/*
	 * If room for request in request ring for at least N IOCB
	 */
	status = 0;
	if ((tot_iocbs + 2) >= *req_q_cnt) {
		/*
		 * Calculate number of free request entries.
		 */
#if defined(ISP2100) || defined(ISP2200)
		cnt = RD_REG_WORD(&reg->mailbox4);
#else
		cnt = RD_REG_WORD(&reg->req_q_out);
#endif
		if (req_ring_index < cnt)
			*req_q_cnt = cnt - req_ring_index;
		else
			*req_q_cnt = REQUEST_ENTRY_CNT - (req_ring_index - cnt);
	}
	if ((tot_iocbs + 2) >= *req_q_cnt) {
		DEBUG5(printk("%s(): in-ptr=%x req_q_cnt=%x tot_iocbs=%x.\n",
				__func__,
				req_ring_index,
				*req_q_cnt,
				tot_iocbs);)

		status = 1;
	}
	if ((ha->iocb_cnt + tot_iocbs) >= ha->iocb_hiwat) {
		DEBUG5(printk("%s(): Not Enough IOCBS for request. "
				"iocb_cnt=%x, tot_iocbs=%x, hiwat=%x.\n",
				__func__,
				ha->iocb_cnt,
				tot_iocbs,
				ha->iocb_hiwat);)
#if defined(IOCB_HIT_RATE)
		ha->iocb_overflow_cnt++;
#endif
		status = 1;
	}
	return (status);
}

/**
 * qla2x00_prep_cont_packet() - Initialize a continuation packet.
 * @ha: HA context
 * @req_ring_index: Current index to request ring
 * @req_ring_ptr: Current pointer to request ring
 *
 * Returns a pointer to the continuation packet.
 */
static inline cont_entry_t *
qla2x00_prep_cont_packet(
		scsi_qla_host_t *ha,
		uint16_t *req_ring_index, request_t **request_ring_ptr)
{
	cont_entry_t *cont_pkt;

	/* Adjust ring index. */
	*req_ring_index += 1;
	if (*req_ring_index == REQUEST_ENTRY_CNT) {
		*req_ring_index = 0;
		*request_ring_ptr = ha->request_ring;
	} else
		*request_ring_ptr += 1;

	cont_pkt = (cont_entry_t *)(*request_ring_ptr);

	/* Load packet defaults. */
	*((uint32_t *)(&cont_pkt->entry_type)) =
		__constant_cpu_to_le32(CONTINUE_TYPE);
	//cont_pkt->entry_type = CONTINUE_TYPE;
	//cont_pkt->entry_count = 0;
	//cont_pkt->sys_define = (uint8_t)req_ring_index;

	return (cont_pkt);
}

/**
 * qla2x00_prep_a64_cont_packet() - Initialize an A64 continuation packet.
 * @ha: HA context
 * @req_ring_index: Current index to request ring
 * @req_ring_ptr: Current pointer to request ring
 *
 * Returns a pointer to the continuation packet.
 */
static inline cont_a64_entry_t *
qla2x00_prep_a64_cont_packet(
		scsi_qla_host_t *ha,
		uint16_t *req_ring_index, request_t **request_ring_ptr)
{
	cont_a64_entry_t *cont_pkt;

	/* Adjust ring index. */
	*req_ring_index += 1;
	if (*req_ring_index == REQUEST_ENTRY_CNT) {
		*req_ring_index = 0;
		*request_ring_ptr = ha->request_ring;
	} else
		*request_ring_ptr += 1;

	cont_pkt = (cont_a64_entry_t *)(*request_ring_ptr);

	/* Load packet defaults. */
	*((uint32_t *)(&cont_pkt->entry_type)) =
		__constant_cpu_to_le32(CONTINUE_A64_TYPE);
	//cont_pkt->entry_type = CONTINUE_A64_TYPE;
	//cont_pkt->entry_count = 0;
	//cont_pkt->sys_define = (uint8_t)req_ring_index;

	return (cont_pkt);
}

/**
 * qla2x00_64bit_start_scsi() - Send a SCSI command to the ISP
 * @sp: command to send to the ISP
 *
 * Returns non-zero if a failure occured, else zero.
 */
STATIC uint8_t
qla2x00_64bit_start_scsi(srb_t *sp)
{
	unsigned long   flags;
	uint16_t        failed;
	scsi_qla_host_t	*ha;
	fc_lun_t	*fclun;
	Scsi_Cmnd	*cmd;
	uint16_t	req_q_cnt;
	uint16_t	req_ring_index;
	request_t	*request_ring_ptr;
	uint32_t	*clr_ptr;
	uint32_t	found;
	uint32_t        index;
	uint32_t	handle;
	uint16_t	tot_iocbs;
	uint16_t	tot_dsds;
	uint16_t	avail_dsds;
	uint32_t	*cur_dsd;
	uint16_t        cdb_len;
	uint8_t		*cdb;
	cmd_a64_entry_t		*cmd_pkt;
	cont_a64_entry_t	*cont_pkt;
	uint32_t        timeout;

	device_reg_t	*reg;
	uint16_t        reg_flushed;

	ENTER(__func__);

	/* Setup device pointers. */
	fclun = sp->lun_queue->fclun;
	ha = fclun->fcport->ha;

	cmd = sp->cmd;
	reg = ha->iobase;

	DEBUG3(printk("64bit_start: cmd=%p sp=%p CDB=%x\n",
			cmd,
			sp,
			cmd->cmnd[0]);)

	/* Send marker if required */
	if (ha->marker_needed != 0) {
		if(qla2x00_marker(ha, 0, 0, MK_SYNC_ALL) != QLA2X00_SUCCESS) {
			return (1);
		}
		ha->marker_needed = 0;
	}

	/* Acquire ring specific lock */
	spin_lock_irqsave(&ha->hardware_lock, flags);

	/* Save ha fields for post-update */
	req_ring_index = ha->req_ring_index;
	request_ring_ptr = ha->request_ring_ptr;
	req_q_cnt = ha->req_q_cnt;

	tot_dsds = 0;
	tot_iocbs = 1;

	/* Allocate space for an additional IOCB */
	failed = qla2x00_check_request_ring(ha,
			tot_iocbs, req_ring_index, &req_q_cnt);
	if (failed)
		goto queuing_error_64;

	/* Check for room in outstanding command list. */
	found = 0;
	handle = ha->current_outstanding_cmd;
	for (index = 1; index < MAX_OUTSTANDING_COMMANDS; index++) {
		handle++;
		if (handle == MAX_OUTSTANDING_COMMANDS)
			handle = 1;
		if (ha->outstanding_cmds[handle] == 0) {
			found = 1;
			ha->current_outstanding_cmd = handle;
			break;
		}
	}
	if (!found) {
		DEBUG5(printk("%s(): NO ROOM IN OUTSTANDING ARRAY. "
				"req_q_cnt=%lx.\n",
				__func__,
				(u_long)ha->req_q_cnt);)
		goto queuing_error_64;
	}

	/*
	 * Build command packet.
	 */
	cmd_pkt = request_ring_ptr;

	*((uint32_t *)(&cmd_pkt->entry_type)) = 
			 __constant_cpu_to_le32(COMMAND_A64_TYPE);
	//cmd_pkt->entry_type = COMMAND_A64_TYPE;
	//cmd_pkt->entry_count = (uint8_t)tot_iocbs;
	//cmd_pkt->sys_define = (uint8_t)ha->req_ring_index;
	//cmd_pkt->entry_status = 0;

	cmd_pkt->handle = handle;

	/* Zero out remaining portion of packet. */
	clr_ptr = (uint32_t *)cmd_pkt + 2;
	for (index = 2; index < REQUEST_ENTRY_SIZE / 4; index++)
		*clr_ptr++ = 0;

	/* Two DSDs are available in the command IOCB */
	avail_dsds = 2;
	cur_dsd = (uint32_t *)&cmd_pkt->dseg_0_address;

	/* Set target ID */
	cmd_pkt->target = (uint8_t)fclun->fcport->loop_id;

	/* Set LUN number*/
#if VSA
	if ((cmd->data_cmnd[0] == 0x26) ||
		(cmd->data_cmnd[0] == 0xA0) ||
		(cmd->data_cmnd[0] == 0xCB) ) {

		cmd_pkt->lun = cpu_to_le16(fclun->lun);
	} else if ((fclun->fcport->flags & FC_VSA))
		cmd_pkt->lun = cpu_to_le16(fclun->lun | 0x4000);
	else
		cmd_pkt->lun = cpu_to_le16(fclun->lun);
#else
	cmd_pkt->lun = cpu_to_le16(fclun->lun);
#endif

	/* Update tagged queuing modifier */
	cmd_pkt->control_flags = __constant_cpu_to_le16(CF_SIMPLE_TAG);
	if (cmd->device->tagged_queue) {
		switch (cmd->tag) {
			case HEAD_OF_QUEUE_TAG:
				cmd_pkt->control_flags =
					__constant_cpu_to_le16(CF_HEAD_TAG);
				break;
			case ORDERED_QUEUE_TAG:
				cmd_pkt->control_flags =
					__constant_cpu_to_le16(CF_ORDERED_TAG);
				break;
		}
	}

	/*
	 * Allocate at least 5 (+ QLA_CMD_TIMER_DELTA) seconds for RISC timeout.
	 */
	timeout = (uint32_t) CMD_TIMEOUT(cmd)/HZ;
	if (timeout > 65535)
		cmd_pkt->timeout = __constant_cpu_to_le16(0);
	if (timeout > 25)
		cmd_pkt->timeout = cpu_to_le16((uint16_t)timeout -
				(5 + QLA_CMD_TIMER_DELTA));
	else
		cmd_pkt->timeout = cpu_to_le16((uint16_t)timeout);

	/* Load SCSI command packet. */
	cdb_len = (uint16_t)CMD_CDBLEN(cmd);
	if (cdb_len > MAX_COMMAND_SIZE)
		cdb_len = MAX_COMMAND_SIZE;
	cdb = (uint8_t *) &(CMD_CDBP(cmd));
	memcpy(cmd_pkt->scsi_cdb, cdb, cdb_len);
	if (sp->cmd_length > MAX_COMMAND_SIZE) {
		for (index = MAX_COMMAND_SIZE; index < MAX_CMDSZ; index++) {
			cmd_pkt->scsi_cdb[index] =
				sp->more_cdb[index - MAX_COMMAND_SIZE];
		}
	}

	cmd_pkt->byte_count = cpu_to_le32((uint32_t)cmd->request_bufflen);

	if (cmd->request_bufflen == 0 || 
		cmd->sc_data_direction == SCSI_DATA_NONE) {
		/* No data transfer */
		cmd_pkt->byte_count = __constant_cpu_to_le32(0);
		DEBUG5(printk("%s(): No data, command packet data - "
				"b%dt%dd%d\n",
				__func__,
				(uint32_t)SCSI_BUS_32(cmd),
				(uint32_t)SCSI_TCN_32(cmd),
				(uint32_t)SCSI_LUN_32(cmd));)
		DEBUG5(qla2x00_dump_buffer((uint8_t *)cmd_pkt,
						REQUEST_ENTRY_SIZE);)
	}
	else {
#if defined(SANE_USAGE_OF_CMD_DIRECTION)
		/* Set transfer direction */
#ifndef __VMWARE__
		if (cmd->sc_data_direction == SCSI_DATA_WRITE) {
			cmd_pkt->control_flags |=
			    __constant_cpu_to_le16(CF_WRITE);
		} else if (cmd->sc_data_direction == SCSI_DATA_READ) {
			cmd_pkt->control_flags |=
			    __constant_cpu_to_le16(CF_READ);
		} else
#else
		/* Always set the data direction here, since the vmkernel
		 * does not do it for us (otherwise it will hold a default
		 * value of zero, which means SCSI_DATA_WRITE)
		 */
		if (1)
#endif //__VMWARE__
		{
			switch (cmd->data_cmnd[0]) {
				case FORMAT_UNIT:
				case WRITE_6:
				case MODE_SELECT:
				case SEND_DIAGNOSTIC:
				case WRITE_10:
				case WRITE_BUFFER:
				case WRITE_LONG:
				case WRITE_SAME:
				case MODE_SELECT_10:
				case WRITE_12:
				case WRITE_VERIFY:
				case WRITE_VERIFY_12:
				case SEND_VOLUME_TAG:
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_WRITE);
					break;
				default:
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_READ);
					break;
			}
		}
#else
		switch (cmd->data_cmnd[0]) {
			case FORMAT_UNIT:
			case WRITE_6:
			case MODE_SELECT:
			case SEND_DIAGNOSTIC:
			case WRITE_10:
			case WRITE_BUFFER:
			case WRITE_LONG:
			case WRITE_SAME:
			case MODE_SELECT_10:
			case WRITE_12:
			case WRITE_VERIFY:
			case WRITE_VERIFY_12:
			case SEND_VOLUME_TAG:
				cmd_pkt->control_flags |=
					__constant_cpu_to_le16(CF_WRITE);
				break;
			default:
#ifdef __VMWARE__
				cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_READ);
#else
				if (cmd->sc_data_direction == SCSI_DATA_WRITE)
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_WRITE);
				else
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_READ);
#endif //__VMWARE__
				break;
		}
#endif
		sp->dir = cmd_pkt->control_flags &
				  __constant_cpu_to_le16(CF_READ | CF_WRITE);

		/* Load data segments */
		if (cmd->use_sg != 0) {
			struct	scatterlist *cur_seg;
			struct	scatterlist *end_seg;
			int	nseg;

			cur_seg = (struct scatterlist *)cmd->request_buffer;
#ifdef __VMWARE__
			/*
			* The dma addresses in sg have already been set up.
			*/
			nseg = cmd->use_sg;
#else
			nseg = pci_map_sg(ha->pdev, cur_seg, cmd->use_sg,
				scsi_to_pci_dma_dir(cmd->sc_data_direction));
#endif
			end_seg = cur_seg + nseg;

			while (cur_seg < end_seg) {
				dma_addr_t	sle_dma;
				uint32_t	sle_len;
				dma_addr_t	nml_dma;
				uint32_t	nml_len;
				uint32_t	normalized;

				/* Allocate additional continuation packets? */
				if (avail_dsds == 0) {
					tot_iocbs++;
					failed = qla2x00_check_request_ring(ha,
							tot_iocbs,
							req_ring_index,
							&req_q_cnt);
					if (failed) {
						goto mapped_queuing_error_64;
					}

					cont_pkt = qla2x00_prep_a64_cont_packet(
							ha,
							&req_ring_index,
							&request_ring_ptr);

					cur_dsd = (uint32_t *)
						&cont_pkt->dseg_0_address;
					avail_dsds = 5;
				}

				sle_dma = sg_dma_address(cur_seg);
				sle_len = sg_dma_len(cur_seg);

				normalized = qla2x00_normalize_dma_addr(
						&sle_dma, &sle_len,
						&nml_dma, &nml_len);

				/* One entry always consumed */
				*cur_dsd++ = cpu_to_le32(
					pci_dma_lo32(sle_dma));
				*cur_dsd++ = cpu_to_le32(
					pci_dma_hi32(sle_dma));
#ifdef PERF_MONITORING
	 			if ( pci_dma_hi32(sle_dma) != 0L) {
	    				qla2x00_stats.highmem_io++;
				}
#endif
				*cur_dsd++ = cpu_to_le32(sle_len);
				tot_dsds++;
				avail_dsds--;

				if (normalized) {
					/*
					 * Allocate additional continuation
					 * packets?
					 */
					if (avail_dsds == 0) {
						tot_iocbs++;
						failed =
						  qla2x00_check_request_ring(ha,
								tot_iocbs,
								req_ring_index,
								&req_q_cnt);
						if (failed)
							goto
							   mapped_queuing_error_64;

						cont_pkt =
						  qla2x00_prep_a64_cont_packet(
							ha,
							&req_ring_index,
							&request_ring_ptr);

						cur_dsd = (uint32_t *)
						  &cont_pkt->dseg_0_address;
						avail_dsds = 5;
					}

					*cur_dsd++ = cpu_to_le32(
							pci_dma_lo32(nml_dma));
					*cur_dsd++ = cpu_to_le32(
							pci_dma_hi32(nml_dma));
					*cur_dsd++ = cpu_to_le32(nml_len);
					tot_dsds++;
					avail_dsds--;
				}
				cur_seg++;
			}
		}
		else {
			/*
			 * No more than 1 (one) IOCB is needed for this type
			 * of request, even if the DMA address spans the 4GB
			 * page boundary.
			 *
			 * @tot_dsds == 1 if non-spanning, else 2
			 */
			dma_addr_t	req_dma;
			uint32_t	req_len;
			dma_addr_t	nml_dma;
			uint32_t	nml_len;
			uint32_t	normalized;

#ifdef __VMWARE__
			/*
			* We already have the machine address.
			*/
			req_dma = (unsigned long)cmd->request_buffer;
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,13)
			struct page *page = virt_to_page(cmd->request_buffer);
			unsigned long offset = ((unsigned long)
						 cmd->request_buffer 
						 & ~PAGE_MASK);

			req_dma = pci_map_page(ha->pdev,
					page,
					offset,
					cmd->request_bufflen,
					scsi_to_pci_dma_dir(
					cmd->sc_data_direction));
#else
			req_dma = pci_map_single(ha->pdev,
					cmd->request_buffer,
					cmd->request_bufflen,
					scsi_to_pci_dma_dir(
						cmd->sc_data_direction));
#endif
#endif
			req_len = cmd->request_bufflen;

			sp->saved_dma_handle = req_dma;

			normalized = qla2x00_normalize_dma_addr(
					&req_dma, &req_len,
					&nml_dma, &nml_len);

			/* One entry always consumed */
			*cur_dsd++ = cpu_to_le32(
				pci_dma_lo32(req_dma));
			*cur_dsd++ = cpu_to_le32(
				pci_dma_hi32(req_dma));
			*cur_dsd++ = cpu_to_le32(req_len);
			tot_dsds++;

#ifdef PERF_MONITORING
	 		if ( pci_dma_hi32(req_dma) != 0L) {
	    			qla2x00_stats.highmem_io++;
			}
#endif
			if (normalized) {
				*cur_dsd++ = cpu_to_le32(
						pci_dma_lo32(nml_dma));
				*cur_dsd++ = cpu_to_le32(
						pci_dma_hi32(nml_dma));
				*cur_dsd++ = cpu_to_le32(nml_len);
				tot_dsds++;
			}

		}
	}

	/* Set total data segment count. */
	cmd_pkt->dseg_count = cpu_to_le16(tot_dsds);
	cmd_pkt->entry_count = (uint8_t)tot_iocbs;

	/* Update ha fields */
	ha->req_ring_index = req_ring_index;
	ha->request_ring_ptr = request_ring_ptr;
	ha->req_q_cnt = req_q_cnt;
	ha->req_q_cnt -= tot_iocbs;
	ha->iocb_cnt += tot_iocbs;

	sp->iocb_cnt = tot_iocbs;

	/* Add command to the active array */
	ha->outstanding_cmds[handle] = sp;
	CMD_HANDLE(sp->cmd) = (unsigned char *)(u_long)handle;

	/* Adjust ring index. */
	ha->req_ring_index++;
	if (ha->req_ring_index == REQUEST_ENTRY_CNT) {
		ha->req_ring_index = 0;
		ha->request_ring_ptr = ha->request_ring;
	} else
		ha->request_ring_ptr++;

	ha->actthreads++;
	ha->total_ios++;
	sp->ha = ha;
	sp->lun_queue->out_cnt++;
	sp->flags |= SRB_DMA_VALID;
	sp->state = SRB_ACTIVE_STATE;
	sp->u_start = jiffies;

	/* Set chip new ring index. */
#if WATCH_THREADS_SIZE
	DEBUG3(printk("%s(): actthreads=%ld.\n", 
			__func__,
			ha->actthreads);)
#endif

#if defined(ISP2100) || defined(ISP2200)
	reg_flushed = CACHE_FLUSH(&reg->mailbox4);
	WRT_REG_WORD(&reg->mailbox4, ha->req_ring_index);
#else
	reg_flushed = CACHE_FLUSH(&reg->req_q_in);
	WRT_REG_WORD(&reg->req_q_in, ha->req_ring_index);
#endif

	spin_unlock_irqrestore(&ha->hardware_lock, flags);
	return (0);

mapped_queuing_error_64:
#ifndef __VMWARE__
	pci_unmap_sg(ha->pdev, (struct scatterlist *)cmd->request_buffer,
		cmd->use_sg, scsi_to_pci_dma_dir(cmd->sc_data_direction));
#endif

queuing_error_64:
	spin_unlock_irqrestore(&ha->hardware_lock, flags);
	return (1);
}

/*
* qla2x00_32bit_start_scsi
*      The start SCSI is responsible for building request packets on
*      request ring and modifying ISP input pointer.
*
*      The Qlogic firmware interface allows every queue slot to have a SCSI
*      command and up to 4 scatter/gather (SG) entries.  If we need more
*      than 4 SG entries, then continuation entries are used that can
*      hold another 7 entries each.  The start routine determines if there
*      is eought empty slots then build the combination of requests to
*      fulfill the OS request.
*
* Input:
*      ha = adapter block pointer.
*      sp = SCSI Request Block structure pointer.
*
* Returns:
*      0 = success, was able to issue command.
*/
STATIC uint8_t
qla2x00_32bit_start_scsi(srb_t *sp)
{
	unsigned long   flags;
	uint16_t        failed;
	scsi_qla_host_t	*ha;
	fc_lun_t	*fclun;
	Scsi_Cmnd	*cmd;
	uint16_t	req_q_cnt;
	uint16_t	req_ring_index;
	request_t	*request_ring_ptr;
	uint32_t	*clr_ptr;
	uint32_t	found;
	uint32_t        index;
	uint32_t	handle;
	uint16_t	tot_iocbs;
	uint16_t	tot_dsds;
	uint16_t	avail_dsds;
	uint32_t	*cur_dsd;
	uint16_t        cdb_len;
	uint8_t		*cdb;
	cmd_entry_t	*cmd_pkt;
	cont_entry_t	*cont_pkt;
	uint32_t        timeout;

	device_reg_t	*reg;
	uint16_t        reg_flushed;

	ENTER(__func__);

	/* Setup device pointers. */
	fclun = sp->lun_queue->fclun;
	ha = fclun->fcport->ha;

	cmd = sp->cmd;
	reg = ha->iobase;

	DEBUG3(printk("32bit_start: cmd=%p sp=%p CDB=%x\n",
			cmd,
			sp,
			cmd->cmnd[0]);)

	/* Send marker if required */
	if (ha->marker_needed != 0) {
		if(qla2x00_marker(ha, 0, 0, MK_SYNC_ALL) != QLA2X00_SUCCESS) {
			return (1);
		}
		ha->marker_needed = 0;
	}

	/* Acquire ring specific lock */
	spin_lock_irqsave(&ha->hardware_lock, flags);

	/* Save ha fields for post-update */
	req_ring_index = ha->req_ring_index;
	request_ring_ptr = ha->request_ring_ptr;
	req_q_cnt = ha->req_q_cnt;

	tot_dsds = 0;
	tot_iocbs = 1;

	/* Allocate space for an additional IOCB */
	failed = qla2x00_check_request_ring(ha,
			tot_iocbs, req_ring_index, &req_q_cnt);
	if (failed)
		goto queuing_error_32;

	/* Check for room in outstanding command list. */
	found = 0;
	handle = ha->current_outstanding_cmd;
	for (index = 1; index < MAX_OUTSTANDING_COMMANDS; index++) {
		handle++;
		if (handle == MAX_OUTSTANDING_COMMANDS)
			handle = 1;
		if (ha->outstanding_cmds[handle] == 0) {
			found = 1;
			ha->current_outstanding_cmd = handle;
			break;
		}
	}
	if (!found) {
		DEBUG5(printk("%s(): NO ROOM IN OUTSTANDING ARRAY. "
				"req_q_cnt=%lx.\n",
				__func__,
				(u_long)ha->req_q_cnt);)
		goto queuing_error_32;
	}

	/*
	 * Build command packet.
	 */
	cmd_pkt = (cmd_entry_t *)request_ring_ptr;

	*((uint32_t *)(&cmd_pkt->entry_type)) = 
			 __constant_cpu_to_le32(COMMAND_TYPE);
	//cmd_pkt->entry_type = COMMAND_TYPE;
	//cmd_pkt->entry_count = (uint8_t)tot_iocbs;
	//cmd_pkt->sys_define = (uint8_t)ha->req_ring_index;
	//cmd_pkt->entry_status = 0;

	cmd_pkt->handle = handle;

	/* Zero out remaining portion of packet. */
	clr_ptr = (uint32_t *)cmd_pkt + 2;
	for (index = 2; index < REQUEST_ENTRY_SIZE / 4; index++)
		*clr_ptr++ = 0;

	/* Three DSDs are available in the command IOCB */
	avail_dsds = 3;
	cur_dsd = (uint32_t *)&cmd_pkt->dseg_0_address;

	/* Set target ID */
	cmd_pkt->target = (uint8_t)fclun->fcport->loop_id;

	/* Set LUN number*/
#if VSA
	if ((cmd->data_cmnd[0] == 0x26) ||
		(cmd->data_cmnd[0] == 0xA0) ||
		(cmd->data_cmnd[0] == 0xCB) ) {

		cmd_pkt->lun = cpu_to_le16(fclun->lun);
	} else if ((fclun->fcport->flags & FC_VSA))
		cmd_pkt->lun = cpu_to_le16(fclun->lun | 0x4000);
	else
		cmd_pkt->lun = cpu_to_le16(fclun->lun);
#else
	cmd_pkt->lun = cpu_to_le16(fclun->lun);
#endif

	/* Update tagged queuing modifier */
	cmd_pkt->control_flags = __constant_cpu_to_le16(CF_SIMPLE_TAG);
	if (cmd->device->tagged_queue) {
		switch (cmd->tag) {
			case HEAD_OF_QUEUE_TAG:
				cmd_pkt->control_flags =
					__constant_cpu_to_le16(CF_HEAD_TAG);
				break;
			case ORDERED_QUEUE_TAG:
				cmd_pkt->control_flags =
					__constant_cpu_to_le16(CF_ORDERED_TAG);
				break;
		}
	}

	/*
	 * Allocate at least 5 (+ QLA_CMD_TIMER_DELTA) seconds for RISC timeout.
	 */
	timeout = (uint32_t) CMD_TIMEOUT(cmd)/HZ;
	if (timeout > 65535)
		cmd_pkt->timeout = __constant_cpu_to_le16(0);
	if (timeout > 25)
		cmd_pkt->timeout = cpu_to_le16((uint16_t)timeout -
				(5 + QLA_CMD_TIMER_DELTA));
	else
		cmd_pkt->timeout = cpu_to_le16((uint16_t)timeout);

	/* Load SCSI command packet. */
	cdb_len = (uint16_t)CMD_CDBLEN(cmd);
	if (cdb_len > MAX_COMMAND_SIZE)
		cdb_len = MAX_COMMAND_SIZE;
	cdb = (uint8_t *) &(CMD_CDBP(cmd));
	memcpy(cmd_pkt->scsi_cdb, cdb, cdb_len);
	if (sp->cmd_length > MAX_COMMAND_SIZE) {
		for (index = MAX_COMMAND_SIZE; index < MAX_CMDSZ; index++) {
			cmd_pkt->scsi_cdb[index] =
				sp->more_cdb[index - MAX_COMMAND_SIZE];
		}
	}

	cmd_pkt->byte_count = cpu_to_le32((uint32_t)cmd->request_bufflen);

	if (cmd->request_bufflen == 0 ||
		cmd->sc_data_direction == SCSI_DATA_NONE) {
		/* No data transfer */
		cmd_pkt->byte_count = __constant_cpu_to_le32(0);
		DEBUG5(printk("%s(): No data, command packet data - "
				"b%dt%dd%d\n",
				__func__,
				(uint32_t)SCSI_BUS_32(cmd),
				(uint32_t)SCSI_TCN_32(cmd),
				(uint32_t)SCSI_LUN_32(cmd));)
		DEBUG5(qla2x00_dump_buffer((uint8_t *)cmd_pkt,
						REQUEST_ENTRY_SIZE);)
	}
	else {
#if defined(SANE_USAGE_OF_CMD_DIRECTION)
#ifndef __VMWARE__
		/* Set transfer direction */
		if (cmd->sc_data_direction == SCSI_DATA_WRITE) {
			cmd_pkt->control_flags |=
				__constant_cpu_to_le16(CF_WRITE);
		} else if (cmd->sc_data_direction == SCSI_DATA_READ) {
			cmd_pkt->control_flags |=
				__constant_cpu_to_le16(CF_READ);
		} else
#else
		/* Always set the data direction here, since the vmkernel
		 * does not do it for us (otherwise it will hold a default
		 * value of zero, which means SCSI_DATA_WRITE)
		 */
		if (1)
#endif //__VMWARE__
		{
			switch (cmd->data_cmnd[0]) {
				case FORMAT_UNIT:
				case WRITE_6:
				case MODE_SELECT:
				case SEND_DIAGNOSTIC:
				case WRITE_10:
				case WRITE_BUFFER:
				case WRITE_LONG:
				case WRITE_SAME:
				case MODE_SELECT_10:
				case WRITE_12:
				case WRITE_VERIFY:
				case WRITE_VERIFY_12:
				case SEND_VOLUME_TAG:
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_WRITE);
					break;
				default:
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_READ);
					break;
			}
		}
#else
		switch (cmd->data_cmnd[0]) {
			case FORMAT_UNIT:
			case WRITE_6:
			case MODE_SELECT:
			case SEND_DIAGNOSTIC:
			case WRITE_10:
			case WRITE_BUFFER:
			case WRITE_LONG:
			case WRITE_SAME:
			case MODE_SELECT_10:
			case WRITE_12:
			case WRITE_VERIFY:
			case WRITE_VERIFY_12:
			case SEND_VOLUME_TAG:
				cmd_pkt->control_flags |=
					__constant_cpu_to_le16(CF_WRITE);
				break;
			default:
#ifdef __VMWARE__
				cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_READ);
#else
				if (cmd->sc_data_direction == SCSI_DATA_WRITE)
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_WRITE);
				else
					cmd_pkt->control_flags |=
					   __constant_cpu_to_le16(CF_READ);
#endif //__VMWARE__
				break;
		}
#endif
		sp->dir = cmd_pkt->control_flags &
				  __constant_cpu_to_le16(CF_READ | CF_WRITE);

		/* Load data segments */
		if (cmd->use_sg != 0) {
			struct	scatterlist *cur_seg;
			struct	scatterlist *end_seg;
			int	nseg;

			cur_seg = (struct scatterlist *)cmd->request_buffer;
#ifdef __VMWARE__
			/*
			 * The dma addresses in sg have already been set up.
			 */
			nseg = cmd->use_sg;
#else
			nseg = pci_map_sg(ha->pdev, cur_seg, cmd->use_sg,
				scsi_to_pci_dma_dir(cmd->sc_data_direction));
#endif
			end_seg = cur_seg + nseg;

			while (cur_seg < end_seg) {
				dma_addr_t	sle_dma;
				uint32_t	sle_len;

				/* Allocate additional continuation packets? */
				if (avail_dsds == 0) {
					tot_iocbs++;
					failed = qla2x00_check_request_ring(ha,
							tot_iocbs,
							req_ring_index,
							&req_q_cnt);
					if (failed) {
						goto mapped_queuing_error_32;
					}

					cont_pkt = qla2x00_prep_cont_packet(
							ha,
							&req_ring_index,
							&request_ring_ptr);

					cur_dsd = (uint32_t *)
						&cont_pkt->dseg_0_address;
					avail_dsds = 7;
				}

				sle_dma = sg_dma_address(cur_seg);
				sle_len = sg_dma_len(cur_seg);

				/* One entry always consumed */
				*cur_dsd++ = cpu_to_le32(
					pci_dma_lo32(sle_dma));
				*cur_dsd++ = cpu_to_le32(sle_len);
				tot_dsds++;
				avail_dsds--;

				cur_seg++;
			}
		}
		else {
			/*
			 * No more than 1 (one) IOCB is needed for this type
			 * of request.
			 */
			dma_addr_t	req_dma;
			uint32_t	req_len;

#ifdef __VMWARE__
			/*
			 * We already have the machine address.
			 */
			req_dma = (unsigned long)cmd->request_buffer;
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,13)
			struct page *page = virt_to_page(cmd->request_buffer);
			unsigned long offset = ((unsigned long)
						 cmd->request_buffer 
						 & ~PAGE_MASK);

			req_dma = pci_map_page(ha->pdev,
					page,
					offset,
					cmd->request_bufflen,
					scsi_to_pci_dma_dir(
					cmd->sc_data_direction));
#else
			req_dma = pci_map_single(ha->pdev,
					cmd->request_buffer,
					cmd->request_bufflen,
					scsi_to_pci_dma_dir(
						cmd->sc_data_direction));
#endif
#endif
			req_len = cmd->request_bufflen;

			sp->saved_dma_handle = req_dma;

			/* One entry always consumed */
			*cur_dsd++ = cpu_to_le32(
				pci_dma_lo32(req_dma));
			*cur_dsd++ = cpu_to_le32(req_len);
			tot_dsds++;
		}
	}

	/* Set total data segment count. */
	cmd_pkt->dseg_count = cpu_to_le16(tot_dsds);
	cmd_pkt->entry_count = (uint8_t)tot_iocbs;

	/* Update ha fields */
	ha->req_ring_index = req_ring_index;
	ha->request_ring_ptr = request_ring_ptr;
	ha->req_q_cnt = req_q_cnt;
	ha->req_q_cnt -= tot_iocbs;
	ha->iocb_cnt += tot_iocbs;

	sp->iocb_cnt = tot_iocbs;

	/* Add command to the active array */
	ha->outstanding_cmds[handle] = sp;
	CMD_HANDLE(sp->cmd) = (unsigned char *)(u_long)handle;

	/* Adjust ring index. */
	ha->req_ring_index++;
	if (ha->req_ring_index == REQUEST_ENTRY_CNT) {
		ha->req_ring_index = 0;
		ha->request_ring_ptr = ha->request_ring;
	} else
		ha->request_ring_ptr++;

	ha->actthreads++;
	ha->total_ios++;
	sp->ha = ha;
	sp->lun_queue->out_cnt++;
	sp->flags |= SRB_DMA_VALID;
	sp->state = SRB_ACTIVE_STATE;
	sp->u_start = jiffies;

	/* Set chip new ring index. */
#if WATCH_THREADS_SIZE
	DEBUG3(printk("%s(): actthreads=%ld.\n",
			__func__,
			ha->actthreads);)
#endif

#if defined(ISP2100) || defined(ISP2200)
	reg_flushed = CACHE_FLUSH(&reg->mailbox4);
	WRT_REG_WORD(&reg->mailbox4, ha->req_ring_index);
#else
	reg_flushed = CACHE_FLUSH(&reg->req_q_in);
	WRT_REG_WORD(&reg->req_q_in, ha->req_ring_index);
#endif

	spin_unlock_irqrestore(&ha->hardware_lock, flags);
	return (0);

mapped_queuing_error_32:
#ifndef __VMWARE__
	pci_unmap_sg(ha->pdev, (struct scatterlist *)cmd->request_buffer,
		cmd->use_sg, scsi_to_pci_dma_dir(cmd->sc_data_direction));
#endif

queuing_error_32:
	spin_unlock_irqrestore(&ha->hardware_lock, flags);
	return (1);
}

/*
* qla2x00_ms_req_pkt
*      Function is responsible for locking ring and
*      getting a zeroed out Managment Server request packet.
*
* Input:
*      ha  = adapter block pointer.
*      sp  = srb_t pointer to handle post function call
* Returns:
*      0 = failed to get slot.
*
* Note: Need to hold the hardware lock before calling this routine.
*/
STATIC request_t *
qla2x00_ms_req_pkt(scsi_qla_host_t *ha, srb_t  *sp)
{
	device_reg_t *reg = ha->iobase;
	request_t    *pkt = 0;
	uint16_t     cnt, i, index;
	uint32_t     *dword_ptr;
	uint32_t     timer;
	uint8_t      found = 0;
	uint16_t     req_cnt = 1;

	ENTER(__func__);

	/* Wait 1 second for slot. */
	for (timer = HZ; timer; timer--) {
		/* Acquire ring specific lock */

		if ((uint16_t)(req_cnt + 2) >= ha->req_q_cnt) {
			/* Calculate number of free request entries. */
#if defined(ISP2100) || defined(ISP2200)
			cnt = qla2x00_debounce_register(&reg->mailbox4);
#else
			cnt = qla2x00_debounce_register(&reg->req_q_out);
#endif

			if (ha->req_ring_index < cnt) {
				ha->req_q_cnt = cnt - ha->req_ring_index;
			} else {
				ha->req_q_cnt = REQUEST_ENTRY_CNT -
					(ha->req_ring_index - cnt);
			}
		}

		/* Check for room in outstanding command list. */
		cnt = ha->current_outstanding_cmd;
		for (index = 1; index < MAX_OUTSTANDING_COMMANDS; index++) {
			cnt++;
			if (cnt == MAX_OUTSTANDING_COMMANDS)
				cnt = 1;

			if (ha->outstanding_cmds[cnt] == 0) {
				found = 1;
				ha->current_outstanding_cmd = cnt;
				break;
			}
		}

		/* If room for request in request ring. */
		if (found && (uint16_t)(req_cnt + 2) < ha->req_q_cnt) {

			pkt = ha->request_ring_ptr;

			/* Zero out packet. */
			dword_ptr = (uint32_t *)pkt;
			for( i = 0; i < REQUEST_ENTRY_SIZE/4; i++ )
				*dword_ptr++ = 0;

			DEBUG5(printk("%s(): putting sp=%p in "
					"outstanding_cmds[%x]\n",
					__func__,
					sp,cnt);)

			ha->outstanding_cmds[cnt] = sp;

			/* save the handle */
			CMD_HANDLE(sp->cmd) = (unsigned char *) (u_long) cnt;
			CMD_SP(sp->cmd) = (void *)sp;

			ha->req_q_cnt--;
			pkt->handle = (uint32_t)cnt;

			/* Set system defined field. */
			pkt->sys_define = (uint8_t)ha->req_ring_index;
			pkt->entry_status = 0;

			break;
		}

#ifdef __VMWARE__
		/* The qla2x00_poll function is only called from here and
		 * from qla2x00_req_pkt (in a similar way). Since
		 * qla2x00_poll will immediately regrab the hardware lock,
		 * we drop releasing it here and drop grabbing it in the
		 * poll function. -- Thor
		 */
		udelay(20);
		qla2x00_poll(ha);
#else
		/* Release ring specific lock */
		spin_unlock(&ha->hardware_lock);
		udelay(20);

		/* Check for pending interrupts. */
		qla2x00_poll(ha);
		spin_lock_irq(&ha->hardware_lock);
#endif
	}

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (!pkt)
		printk("%s(): **** FAILED ****\n", __func__);
#endif

	LEAVE(__func__);

	return (pkt);
}

/*
* qla2x00_req_pkt
*      Function is responsible for locking ring and
*      getting a zeroed out request packet.
*
* Input:
*      ha  = adapter block pointer.
*
* Returns:
*      0 = failed to get slot.
*/
STATIC request_t *
qla2x00_req_pkt(scsi_qla_host_t *ha)
{
	device_reg_t *reg = ha->iobase;
	request_t    *pkt = 0;
	uint16_t     cnt;
	uint32_t     *dword_ptr;
	uint32_t     timer;
	uint16_t     req_cnt = 1;

	ENTER(__func__);

	/* Wait 1 second for slot. */
	for (timer = HZ; timer; timer--) {
		/* Acquire ring specific lock */

		if ((uint16_t)(req_cnt + 2) >= ha->req_q_cnt) {
			/* Calculate number of free request entries. */
#if defined(ISP2100) || defined(ISP2200)
			cnt = qla2x00_debounce_register(&reg->mailbox4);
#else
			cnt = qla2x00_debounce_register(&reg->req_q_out);
#endif
			if  (ha->req_ring_index < cnt)
				ha->req_q_cnt = cnt - ha->req_ring_index;
			else
				ha->req_q_cnt = REQUEST_ENTRY_CNT - 
					(ha->req_ring_index - cnt);
		}
		/* If room for request in request ring. */
		if ((uint16_t)(req_cnt + 2) < ha->req_q_cnt) {
			ha->req_q_cnt--;
			pkt = ha->request_ring_ptr;

			/* Zero out packet. */
			dword_ptr = (uint32_t *)pkt;
			for (cnt = 0; cnt < REQUEST_ENTRY_SIZE/4; cnt++)
				*dword_ptr++ = 0;

			/* Set system defined field. */
			pkt->sys_define = (uint8_t)ha->req_ring_index;

			/* Set entry count. */
			pkt->entry_count = 1;

			break;
		}

#ifdef __VMWARE__
		/* The qla2x00_poll function is only called from here and
		 * from qla2x00_ms_req_pkt (in a similar way). Since
		 * qla2x00_poll will immediately regrab the hardware lock,
		 * we drop releasing it here and drop grabbing it in the
		 * poll function. -- Thor
		 */
		udelay(2);
		if (!ha->marker_needed)
			qla2x00_poll(ha);
#else
		/* Release ring specific lock */
		spin_unlock(&ha->hardware_lock);

		udelay(2);   /* 2 us */

		/* Check for pending interrupts. */
		/* During init we issue marker directly */
		if (!ha->marker_needed)
			qla2x00_poll(ha);

		spin_lock_irq(&ha->hardware_lock);
#endif
	}

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (!pkt)
		printk("%s(): **** FAILED ****\n", __func__);
#endif

	LEAVE(__func__);

	return(pkt);
}

/*
* qla2x00_isp_cmd
*      Function is responsible for modifying ISP input pointer.
*      Releases ring lock.
*
* Input:
*      ha  = adapter block pointer.
*/
STATIC void
qla2x00_isp_cmd(scsi_qla_host_t *ha)
{
	device_reg_t *reg = ha->iobase;

	ENTER(__func__);

	DEBUG5(printk("%s(): IOCB data:\n", __func__);)
	DEBUG5(qla2x00_dump_buffer((uint8_t *)ha->request_ring_ptr,
				REQUEST_ENTRY_SIZE);)

	/* Adjust ring index. */
	ha->req_ring_index++;
	if (ha->req_ring_index == REQUEST_ENTRY_CNT) {
		ha->req_ring_index = 0;
		ha->request_ring_ptr = ha->request_ring;
	} else
		ha->request_ring_ptr++;

	/* Set chip new ring index. */
#if defined(ISP2100) || defined(ISP2200)
	WRT_REG_WORD(&reg->mailbox4, ha->req_ring_index);
#else
	WRT_REG_WORD(&reg->req_q_in, ha->req_ring_index);
#endif

	LEAVE(__func__);
}

/*
* qla2x00_enable_lun
*      Issue enable LUN entry IOCB.
*
* Input:
*      ha = adapter block pointer.
*/
STATIC void
qla2x00_enable_lun(scsi_qla_host_t *ha)
{
	unsigned long flags = 0;
	elun_entry_t *pkt;

	ENTER("qla2x00_enable_lun");

	spin_lock_irqsave(&ha->hardware_lock, flags);
	/* Get request packet. */
	if ((pkt = (elun_entry_t *)qla2x00_req_pkt(ha)) != NULL) {
		pkt->entry_type = ENABLE_LUN_TYPE;
		pkt->command_count = 32;
		pkt->immed_notify_count = 1;
		pkt->timeout = __constant_cpu_to_le16(0xFFFF);

		/* Issue command to ISP */
		qla2x00_isp_cmd(ha);
	}
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (!pkt)
		printk("qla2100_enable_lun: **** FAILED ****\n");
#endif

	LEAVE("qla2x00_enable_lun");
}


/*
 * qla2x00_process_good_request
 * Mark request denoted by "index" in the outstanding commands array
 * as complete and handle the stuff needed for that.
 *
 * Input:
 *      ha   = adapter block pointer.
 *      index = srb handle.
 *      async_event_status_code 
 *
 * Note: To be called from the ISR only.
 */
STATIC void
qla2x00_process_good_request(struct scsi_qla_host * ha, int index, 
					int async_event_status_code)
{
	srb_t *sp;
	struct scsi_qla_host *vis_ha;
#ifdef PERF_MONITORING
	os_lun_t	*lq;
#endif

	ENTER(__func__);

	/* Validate handle. */
	if (index < MAX_OUTSTANDING_COMMANDS) {
		sp = ha->outstanding_cmds[index];
	} else {
		DEBUG2(printk("%s(%ld): invalid scsi completion handle %d.\n",
				__func__,
				ha->host_no, 
				index);)
		sp = NULL;
	}

	if (sp) {
		/* Free outstanding command slot. */
		ha->outstanding_cmds[index] = 0;
		ha->iocb_cnt -= sp->iocb_cnt;
		vis_ha =(scsi_qla_host_t *)sp->cmd->host->hostdata;
		if( ha->actthreads )
			ha->actthreads--;
		sp->lun_queue->out_cnt--;
		sp->flags |= SRB_ISP_COMPLETED;
		CMD_COMPL_STATUS(sp->cmd) = 0L;
		CMD_SCSI_STATUS(sp->cmd) = 0L;

#ifdef PERF_MONITORING
		/* update stats */
		lq = sp->lun_queue;
		lq->resp_time += jiffies - sp->u_start; 
		lq->act_time += jiffies - sp->r_start;  
#endif

		/* Save ISP completion status */
		CMD_RESULT(sp->cmd) = DID_OK << 16;
		sp->fo_retry_cnt = 0;
		add_to_done_queue(ha,sp);
	} else {
		DEBUG2(printk("scsi(%ld): %s(): ISP invalid handle\n",
				ha->host_no,
				__func__);)
		printk(KERN_WARNING
			"%s(): ISP invalid handle", __func__);

		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
	}

	LEAVE(__func__);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,7)
/*
*  qla2x00_process_risc_intrs
*      Check and process multiple pending interrupts.
*
* Input:
*      ha           = adapter block pointer.
*      io_request_lock must be already obtained.
*      
*/
STATIC void
qla2x00_process_risc_intrs(scsi_qla_host_t *ha)
{
	unsigned long mbx_flags = 0 , flags = 0;
	uint16_t    data;
	uint8_t     got_mbx = 0;
	device_reg_t *reg;

	reg = ha->iobase;

	DEBUG(printk("%s(): check and process pending intrs.\n", __func__);)

	spin_lock_irqsave(&ha->hardware_lock, flags);
	/* Check and process pending interrupts. */
#if defined(ISP2100) || defined(ISP2200)
	while (!(ha->flags.in_isr) &&
		((data = RD_REG_WORD(&reg->istatus)) & RISC_INT))
#else
	while (!(ha->flags.in_isr) &&
		((data = RD_REG_WORD(&reg->host_status_lo)) & HOST_STATUS_INT))
#endif
	{
		ha->total_isr_cnt++;
		qla2x00_isr(ha, data, &got_mbx);
	}

	spin_unlock_irqrestore(&ha->hardware_lock, flags);
	if (test_bit(MBX_INTR_WAIT, &ha->mbx_cmd_flags) &&
		 got_mbx && ha->flags.mbox_int) {
		/* There was a mailbox completion */
		DEBUG3(printk("%s(): going to get mbx reg lock.\n", __func__);)

		QLA_MBX_REG_LOCK(ha);
		MBOX_TRACE(ha,BIT_5);
		got_mbx = 0;

		if (ha->mcp == NULL) {
			DEBUG3(printk("%s(): error mbx pointer.\n", __func__);)
		} else {
			DEBUG3(printk("%s(): going to set mbx intr flags. "
					"cmd=%x.\n",
					__func__,
					ha->mcp->mb[0]);)
		}
		set_bit(MBX_INTERRUPT, &ha->mbx_cmd_flags);

		DEBUG3(printk("%s(%ld): going to wake up mbx function for "
				"completion.\n",
				__func__,
				ha->host_no);)
		MBOX_TRACE(ha,BIT_6);
		up(&ha->mbx_intr_sem);

		DEBUG3(printk("%s: going to unlock mbx reg.\n", __func__);)
		QLA_MBX_REG_UNLOCK(ha);
	}

	LEAVE(__func__);
}
#endif

/****************************************************************************/
/*                        Interrupt Service Routine.                        */
/****************************************************************************/

/*
*  qla2x00_isr
*      Calls I/O done on command completion.
*
* Input:
*      ha           = adapter block pointer.
*      INTR_LOCK must be already obtained.
*/
STATIC void
qla2x00_isr(scsi_qla_host_t *ha, uint16_t data, uint8_t *got_mbx)
{
	device_reg_t *reg = ha->iobase;
	uint32_t     index;
	uint16_t     *iptr, *mptr;
	uint16_t     mailbox[MAILBOX_REGISTER_COUNT];
	uint16_t     cnt, temp1;
	uint16_t     response_index = RESPONSE_ENTRY_CNT;
#if defined(ISP2300)
	uint16_t     temp2;
	uint8_t      mailbox_int;
	uint16_t     hccr;
#endif
	uint8_t      rscn_queue_index;

	ENTER(__func__);

#if defined(ISP2300)
	/*
	 * Check for a paused RISC -- schedule an isp abort 
	 */
	if (data & BIT_8) {
		hccr = RD_REG_WORD(&reg->host_cmd);
		printk(KERN_INFO
		    "%s(%ld): RISC paused, dumping HCCR (%x) and schedule "
		    "an ISP abort (big-hammer)\n",
		    __func__,
		    ha->host_no,
		    hccr);
		printk("%s(%ld): RISC paused, dumping HCCR (%x) and schedule "
		    "an ISP abort (big-hammer)\n",
		    __func__,
		    ha->host_no,
		    hccr);

		/* Issuing a "HARD" reset in order for the RISC interrupt
		 * bit to be cleared and scheduling a big hammmer to
		 * get out of the RISC PAUSED state.
		 */
		WRT_REG_WORD(&reg->host_cmd, HC_RESET_RISC);
		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
	}
#endif

	/* Check for mailbox interrupt. */
	MBOX_TRACE(ha,BIT_2);
#if defined(ISP2100) || defined(ISP2200)
	response_index = qla2x00_debounce_register(&reg->mailbox5);
	temp1 = RD_REG_WORD(&reg->semaphore);
	if (temp1 & BIT_0) {
		temp1 = RD_REG_WORD(&reg->mailbox0);
#else
	temp2 = RD_REG_WORD(&reg->host_status_hi);
	mailbox_int = 0;
	switch (data & 0xFF) {
		case ROM_MB_CMD_COMP:
		case ROM_MB_CMD_ERROR:
		case MB_CMD_COMP:
		case MB_CMD_ERROR:
		case ASYNC_EVENT:
			mailbox_int = 1;
			temp1 = temp2;
			break;
		case FAST_SCSI_COMP:
			mailbox_int = 1;
			temp1 = MBA_SCSI_COMPLETION;
			break;
		case RESPONSE_QUEUE_INT:
			response_index = temp2;
			goto response_queue_int;
			break;

#if defined(FC_IP_SUPPORT)
		case RHS_IP_SEND_COMPLETE:
			/* Clear RISC interrupt and do IP send completion */
			WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
			qla2x00_ip_send_complete(ha, temp2, CS_COMPLETE);
			return;

		case RHS_IP_RECV_COMPLETE:
			/* Handle IP receive */
			/*
			 * Note: qla2x00_ip_receive_fastpost will clear RISC
			 * interrupt
			 */
			qla2x00_ip_receive_fastpost(ha,
					MBA_IP_RECEIVE_COMPLETE);
			return;

		case RHS_IP_RECV_DA_COMPLETE:
			/* Handle IP receive with data alignment */
			/*
			 * Note: qla2x00_ip_receive_fastpost will clear RISC
			 * interrupt
			 */
			qla2x00_ip_receive_fastpost(ha,
					MBA_IP_RECEIVE_COMPLETE_SPLIT);
			return;
#endif /* FC_IP_SUPPORT */

		default:
			WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
			goto isr_end;
			break;
	}

	if (mailbox_int) {
		MBOX_TRACE(ha,BIT_3);
#endif

#if defined(FC_IP_SUPPORT)
		if (temp1 == MBA_IP_TRANSMIT_COMPLETE) {
			uint16_t handle = RD_REG_WORD(&reg->mailbox1);

			/* Clear interrupt and do IP send completion */
			WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
#if defined(ISP2100) || defined(ISP2200)
			WRT_REG_WORD(&reg->semaphore, 0);
#endif
			qla2x00_ip_send_complete(ha, handle, CS_COMPLETE);
			return;
		}

		if (temp1 == MBA_IP_RECEIVE_COMPLETE ||
			temp1 == MBA_IP_RECEIVE_COMPLETE_SPLIT) {
			/* Handle IP receive */
			/*
			 * Note: qla2x00_ip_receive_fastpost will clear RISC
			 * interrupt
			 */
			qla2x00_ip_receive_fastpost(ha, temp1);
			return;
		}
#endif /* FC_IP_SUPPORT */

		/*
		   if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
		   QLA_MBX_REG_LOCK(ha);
		 */
		if (temp1 == MBA_SCSI_COMPLETION) {
#if defined(ISP2100) || defined(ISP2200)
			mailbox[1] = RD_REG_WORD(&reg->mailbox1);
#else
			mailbox[1] = temp2;
#endif

			mailbox[2] = RD_REG_WORD(&reg->mailbox2);
		} else {
			MBOX_TRACE(ha,BIT_4);
			mailbox[0] = temp1;
			DEBUG3(printk("%s(): Saving return mbx data\n",
					__func__);)

			/* Get mailbox data. */
			mptr = &mailbox[1];
			iptr = (uint16_t *)&reg->mailbox1;
			for (cnt = 1; cnt < MAILBOX_REGISTER_COUNT; cnt++) {
#if defined(ISP2200)
				if (cnt == 8)
					iptr = (uint16_t *)&reg->mailbox8;
#endif
				if (cnt == 4 || cnt == 5)
					*mptr = qla2x00_debounce_register(iptr);
				else
					*mptr = RD_REG_WORD(iptr);
				mptr++;
				iptr++;
			}
		}

		/*
		   if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
		   QLA_MBX_REG_UNLOCK(ha);
		 */
		/* Release mailbox registers. */
		WRT_REG_WORD(&reg->semaphore, 0);
		WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);

		DEBUG5(printk("%s(): mailbox interrupt mailbox[0] = %x.\n",
				__func__,
				temp1);)

		/* Handle asynchronous event */
		switch (temp1) {

			case MBA_SCSI_COMPLETION:	/* Completion */
				
				DEBUG5(printk("%s(): mailbox response "
						"completion.\n",
						__func__);)

				if (!ha->flags.online)
					break;

				/* Get outstanding command index  */
				index = (uint32_t)
						(mailbox[2] << 16 | mailbox[1]);
				qla2x00_process_good_request(ha,
						index, MBA_SCSI_COMPLETION);
				break;

			case MBA_RESET:			/* Reset */

				DEBUG2(printk("scsi(%ld): %s: asynchronous "
						"RESET.\n",
						ha->host_no,
						__func__);)

				set_bit(RESET_MARKER_NEEDED, &ha->dpc_flags);
				break;

			case MBA_SYSTEM_ERR:		/* System Error */

				printk(KERN_INFO
					"qla2x00: ISP System Error - mbx1=%xh, "
					"mbx2=%xh, mbx3=%xh.",
					mailbox[1],
					mailbox[2],
					mailbox[3]);

				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
				break;

			case MBA_REQ_TRANSFER_ERR:  /* Request Transfer Error */

				printk(KERN_WARNING
					"qla2x00: ISP Request Transfer "
					"Error.\n");

				DEBUG2(printk("%s(): ISP Request Transfer "
						"Error.\n",
						__func__);)

				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
				break;


			case MBA_RSP_TRANSFER_ERR: /* Response Transfer Error */

				printk(KERN_WARNING
					"qla2100: ISP Response Transfer "
					"Error.\n");

				DEBUG2(printk("%s(): ISP Response Transfer "
						"Error.\n",
						__func__);)

				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
				break;

			case MBA_WAKEUP_THRES:	/* Request Queue Wake-up */

				DEBUG2(printk("%s(): asynchronous "
						"WAKEUP_THRES.\n",
						__func__);)
				break;

			case MBA_LIP_OCCURRED:	/* Loop Initialization	*/
						/*  Procedure		*/

				if (!qla2x00_quiet)
					printk(KERN_INFO
						"scsi(%ld): LIP occurred.\n",
						    ha->host_no);

				DEBUG2(printk(
					"%s(): asynchronous "
					"MBA_LIP_OCCURRED.\n",
					__func__);)

				/* Save LIP sequence. */
				ha->lip_seq = mailbox[1];
				if (ha->loop_state != LOOP_DOWN) {
					ha->loop_state = LOOP_DOWN;
					atomic_set(&ha->loop_down_timer,
							LOOP_DOWN_TIME);
					qla2x00_mark_all_devices_lost(ha);
				}
				set_bit(COMMAND_WAIT_NEEDED, &ha->dpc_flags);
#if REG_FC4_ENABLED
				set_bit(REGISTER_FC4_NEEDED, &ha->dpc_flags);
#endif

				ha->flags.management_server_logged_in = 0;

				if (ha->ioctl->flags &
						IOCTL_AEN_TRACKING_ENABLE) {
					/* Update AEN queue. */
					qla2x00_enqueue_aen(ha,
							MBA_LIP_OCCURRED, NULL);
				}

				ha->total_lip_cnt++;

				break;

			case MBA_LOOP_UP:

				printk(KERN_INFO
					"scsi(%ld): LOOP UP detected.\n",
					ha->host_no);

				DEBUG2(printk("%s(): asynchronous "
						"MBA_LOOP_UP.\n",
						__func__);)

				ha->flags.management_server_logged_in = 0;
				if (ha->ioctl->flags &
						IOCTL_AEN_TRACKING_ENABLE) {
					/* Update AEN queue. */
					qla2x00_enqueue_aen(ha,
							MBA_LOOP_UP, NULL);
				}

				/*
				 * Save the current speed for use by ioctl and
				 * IP driver.
				 */
				ha->current_speed = EXT_DEF_PORTSPEED_1GBIT;
#if defined(ISP2300)
				if (mailbox[1] == 1)
					ha->current_speed =
						EXT_DEF_PORTSPEED_2GBIT;
#endif
				break;

			case MBA_LOOP_DOWN:

				printk(KERN_INFO
					"scsi(%ld): LOOP DOWN detected.\n",
					ha->host_no);

				DEBUG2(printk("scsi(%ld) %s: asynchronous "
						"MBA_LOOP_DOWN.\n",
						ha->host_no, __func__);)

				if (ha->loop_state != LOOP_DOWN) {
					ha->loop_state = LOOP_DOWN;
					atomic_set(&ha->loop_down_timer,
							LOOP_DOWN_TIME);
					qla2x00_mark_all_devices_lost(ha);
				}

				ha->flags.management_server_logged_in = 0;
				ha->current_speed = 0; /* reset value */

				/* no wait 10/19/2000 */
				if (ha->ioctl->flags &
						IOCTL_AEN_TRACKING_ENABLE) {
					/* Update AEN queue. */
					qla2x00_enqueue_aen(ha,
							MBA_LOOP_DOWN, NULL);
				}
				break;

			case MBA_LIP_RESET:	/* LIP reset occurred */

				printk(KERN_INFO
					"scsi(%ld): LIP reset occurred.\n",
					ha->host_no);

				DEBUG2(printk("scsi(%ld) %s: "
					"asynchronous MBA_LIP_RESET.\n",
					ha->host_no, __func__);)

				set_bit(COMMAND_WAIT_NEEDED, &ha->dpc_flags);
				set_bit(RESET_MARKER_NEEDED, &ha->dpc_flags);

				if( ha->loop_state != LOOP_DOWN ) {
					atomic_set(&ha->loop_down_timer, 
							LOOP_DOWN_TIME);
					ha->loop_state = LOOP_DOWN;
					qla2x00_mark_all_devices_lost(ha);
				}
				ha->operating_mode = LOOP;
				ha->flags.management_server_logged_in = 0;

				if (ha->ioctl->flags &
						IOCTL_AEN_TRACKING_ENABLE) {
					/* Update AEN queue. */
					qla2x00_enqueue_aen(ha,
							MBA_LIP_RESET, NULL);
				}

				ha->total_lip_cnt++;
				break;

#if !defined(ISP2100)
			case MBA_LINK_MODE_UP:	/* Link mode up. */

				DEBUG(printk("scsi(%ld): Link node is up.\n",
						ha->host_no);)

				DEBUG2(printk("%s(%ld): asynchronous "
						"MBA_LINK_MODE_UP.\n",
						__func__,
						ha->host_no);)

				/*
				 * Until there's a transition from loop down to
				 * loop up, treat this as loop down only.
				 */
				if (!(test_bit(ABORT_ISP_ACTIVE,
							&ha->dpc_flags))) {
					set_bit(COMMAND_WAIT_NEEDED,
							&ha->dpc_flags);
					set_bit(RESET_MARKER_NEEDED,
							&ha->dpc_flags);
				}
#if REG_FC4_ENABLED
				set_bit(REGISTER_FC4_NEEDED, &ha->dpc_flags);
#endif

				if (ha->loop_state != LOOP_DOWN) {
					if (!atomic_read(&ha->loop_down_timer))
						atomic_set(&ha->loop_down_timer,
								LOOP_DOWN_TIME);

					ha->loop_state = LOOP_DOWN;
					qla2x00_mark_all_devices_lost(ha);
				}
				break;

			case MBA_UPDATE_CONFIG:      /* Update Configuration. */

				printk(KERN_INFO
					"scsi(%ld): Configuration change "
					"detected: value %d.\n",
					ha->host_no,
					mailbox[1]);

				DEBUG2(printk("scsi(%ld) %s: asynchronous "
						"MBA_UPDATE_CONFIG.\n",
						ha->host_no, __func__);)

				if (ha->loop_state != LOOP_DOWN) {
					/* dg - 03/30 */
					ha->loop_state = LOOP_DOWN;  
					if (!atomic_read(&ha->loop_down_timer))
						atomic_set(&ha->loop_down_timer,
								LOOP_DOWN_TIME);
					qla2x00_mark_all_devices_lost(ha);
				}
				set_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags);
				set_bit(LOCAL_LOOP_UPDATE, &ha->dpc_flags);
				break;

#endif	/* #if !defined(ISP2100) */

			case MBA_PORT_UPDATE:	/* Port database update */

			     /* If PORT UPDATE is global(recieved 
			      * LIP_OCCURED/LIP_RESET event etc earlier 
			      * indicating loop is down) then process
			      * it.Otherwise ignore it and Wait for RSCN
			      * to come in.
			      */
				
			     if (ha->loop_state == LOOP_DOWN) {
				DEBUG(printk("scsi(%ld): Port database "
						"changed.\n",
						ha->host_no);)

				DEBUG2(printk("scsi%ld %s: asynchronous "
						"MBA_PORT_UPDATE.\n",
						ha->host_no, __func__);)

				/* dg - 06/19/01
				 *
				 * Mark all devices as missing so we will
				 * login again.
				 */
				ha->flags.rscn_queue_overflow = 1;

				atomic_set(&ha->loop_down_timer, 0);
				ha->loop_state = LOOP_UP;
				qla2x00_mark_all_devices_lost(ha);
				set_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags);
				set_bit(LOCAL_LOOP_UPDATE, &ha->dpc_flags);

				/* 9/23
				 *
				 * ha->flags.loop_resync_needed = TRUE;
				 */
				ha->loop_state = LOOP_UPDATE;
				if (ha->ioctl->flags &
						IOCTL_AEN_TRACKING_ENABLE) {
					/* Update AEN queue. */
					qla2x00_enqueue_aen(ha,
							MBA_PORT_UPDATE, NULL);
				}

			     }else{
				printk(KERN_INFO "scsi(%ld) %s MBA_PORT_UPDATE"
					         " ignored\n",
						 ha->host_no, __func__);
				DEBUG2(printk("scsi(%ld) %s: asynchronous "
						"MBA_PORT_UPDATE ignored.\n",
						ha->host_no, __func__);)
			     }
				break;

			case MBA_SCR_UPDATE:	/* State Change Registration */

				DEBUG(printk("scsi(%ld): RSCN database changed "
						"-0x%x,0x%x.\n",
						ha->host_no,
						mailbox[1],
						mailbox[2]);)

				DEBUG2(printk("scsi%ld %s: asynchronous "
						"MBA_RSCR_UPDATE.\n",
						ha->host_no, __func__);)

				rscn_queue_index = ha->rscn_in_ptr + 1;
				if (rscn_queue_index == MAX_RSCN_COUNT)
					rscn_queue_index = 0;
				if (rscn_queue_index != ha->rscn_out_ptr) {
					ha->rscn_queue[ha->rscn_in_ptr].
						format =
						   (uint8_t)(mailbox[1] >> 8);
					ha->rscn_queue[ha->rscn_in_ptr].
						d_id.b.domain =
						   (uint8_t)mailbox[1];
					ha->rscn_queue[ha->rscn_in_ptr].
						d_id.b.area =
						   (uint8_t)(mailbox[2] >> 8);
					ha->rscn_queue[ha->rscn_in_ptr].
						d_id.b.al_pa =
						   (uint8_t)mailbox[2];
					ha->rscn_in_ptr =
						(uint8_t)rscn_queue_index;
				} else {
					ha->flags.rscn_queue_overflow = 1;
				}

				set_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags);
				set_bit(RSCN_UPDATE, &ha->dpc_flags);
				atomic_set(&ha->loop_down_timer, 0);
				ha->flags.management_server_logged_in = 0;

				ha->loop_state = LOOP_UPDATE;
				if (ha->ioctl->flags &
						IOCTL_AEN_TRACKING_ENABLE) {
					/* Update AEN queue. */
					qla2x00_enqueue_aen(ha,
							MBA_RSCN_UPDATE,
							&mailbox[0]);
				}
				break;

			case MBA_CTIO_COMPLETION:

				DEBUG2(printk("%s(): asynchronous "
						"MBA_CTIO_COMPLETION.\n",
						__func__);)

				break;

			default:

				if (temp1 >= MBA_ASYNC_EVENT)
					break;

				/* mailbox completion */
				*got_mbx = TRUE;
				memcpy((void *)ha->mailbox_out,
					mailbox,
					sizeof(ha->mailbox_out));
				ha->flags.mbox_int = TRUE;
				if (ha->mcp) {
					DEBUG3(printk("%s(): got mailbox "
							"completion. cmd=%x.\n",
							__func__,
							ha->mcp->mb[0]);)
				} else {
					DEBUG2_3(printk("%s(): mbx pointer "
							"ERROR.\n",
							__func__);)
				}
				DEBUG5(printk("%s(): Returning mailbox data\n",
						__func__);)
				break;
		}
	} else
#if defined(ISP2300)
response_queue_int:
#endif
	{
		WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);

		/* Process response ring */
		if (ha->flags.online) {
			if (response_index < RESPONSE_ENTRY_CNT) {
				qla2x00_response_pkt(ha, response_index);
			} else {
				/* Invalid response pointer value. */
				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
				DEBUG(printk("%s(): Response Pointer Error. "
						"mb5=%x.\n",
						__func__,
						response_index);)
			}
		}
	}

#if defined(ISP2300)
isr_end:
#endif

	LEAVE(__func__);
}

/*
*  qla2x00_rst_aen
*      Processes asynchronous reset.
*
* Input:
*      ha  = adapter block pointer.
*/
STATIC void
qla2x00_rst_aen(scsi_qla_host_t *ha) 
{
	ENTER(__func__);

	if (ha->flags.online && !ha->flags.reset_active &&
		!atomic_read(&ha->loop_down_timer) && 
		!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ) {
		/* 10/15 ha->flags.reset_active = TRUE; */
		do {
			clear_bit(RESET_MARKER_NEEDED, &ha->dpc_flags);

			/*
			 * Issue marker command only when we are going to start
			 * the I/O .
			 */
			ha->marker_needed = 1;
		} while (!atomic_read(&ha->loop_down_timer) &&
			(test_bit(RESET_MARKER_NEEDED, &ha->dpc_flags)) );
		/* 10/15 ha->flags.reset_active = FALSE; */
	}

	LEAVE(__func__);
}


STATIC void
qla2x00_response_pkt(scsi_qla_host_t *ha, uint16_t index) 
{
	device_reg_t	*reg = ha->iobase;
	response_t	*pkt;

	ENTER(__func__);

	while (ha->rsp_ring_index != index) {
		pkt = ha->response_ring_ptr;

		DEBUG5(printk("%s(): ha->rsp_ring_index=%ld index=%ld.\n",
				__func__,
				(u_long)ha->rsp_ring_index, 
				(u_long)index);)
		DEBUG5(printk("%s(): response packet data:", __func__);)
		DEBUG5(qla2x00_dump_buffer((uint8_t *)pkt,
				RESPONSE_ENTRY_SIZE);)

		ha->rsp_ring_index++;
		if (ha->rsp_ring_index == RESPONSE_ENTRY_CNT) {
			ha->rsp_ring_index = 0;
			ha->response_ring_ptr = ha->response_ring;
		} else {
			ha->response_ring_ptr++;
		}

#if defined(FC_IP_SUPPORT)
		/*
		 * This code is temporary until FW is fixed.  FW is mistakenly
		 * setting bit 6 on Mailbox IOCB response
		 */
		pkt->entry_status &= 0x3f;
#endif

		if (pkt->entry_status != 0) {
			DEBUG3(printk(KERN_INFO
					"%s(): process error entry.\n",
					__func__);)
			qla2x00_error_entry(ha, pkt);
			continue;
		}

		DEBUG3(printk(KERN_INFO
				"%s(): process response entry.\n",
				__func__);)

		switch (pkt->entry_type) {
			case STATUS_TYPE:
				qla2x00_status_entry(ha, (sts_entry_t *)pkt);
				break;

			case STATUS_CONT_TYPE:
				qla2x00_status_cont_entry(ha,
						(sts_cont_entry_t *)pkt);
				break;

			case MS_IOCB_TYPE:
				qla2x00_ms_entry(ha, (ms_iocb_entry_t *)pkt);
				break;

#if defined(FC_IP_SUPPORT)
			case ET_IP_COMMAND_64:
				/* Handle IP send completion */
				qla2x00_ip_send_complete(ha,
						pkt->handle,
						le16_to_cpu(pkt->comp_status));
				break;

			case ET_IP_RECEIVE:
				/* Handle IP receive packet */
				qla2x00_ip_receive(ha, pkt);
				break;

			case ET_MAILBOX_COMMAND:
				if (pkt->sys_define == SOURCE_IP) {
					qla2x00_ip_mailbox_iocb_done(ha,
						(struct mbx_entry *)pkt);
					break;
				}       
#endif  /* FC_IP_SUPPORT */

			default:
				/* Type Not Supported. */
				DEBUG4(printk(KERN_WARNING
						"%s(): received unknown "
						"response pkt type %x "
						"entry status=%x.\n",
						__func__,
						pkt->entry_type, 
						pkt->entry_status);)
				break;
		}
	} /* while (ha->rsp_ring_index != index) */

	/* Adjust ring index -- once, instead of for all entries. */
#if defined(ISP2100) || defined(ISP2200)
	WRT_REG_WORD(&reg->mailbox5, ha->rsp_ring_index);
#else
	WRT_REG_WORD(&reg->rsp_q_out, ha->rsp_ring_index);
#endif

	LEAVE(__func__);
}

static inline void qla2x00_filter_command(scsi_qla_host_t *ha, srb_t *sp);
static inline void
qla2x00_filter_command(scsi_qla_host_t *ha, srb_t *sp)
{
	Scsi_Cmnd	*cp = sp->cmd;
	uint8_t		*strp;

	/*
	 * Special case considertaion on an Inquiry command (0x12) for Lun 0,
	 * device responds with no devices (0x7F), then Linux will not scan
	 * further Luns. While reporting that some device exists on Lun 0 Linux
	 * will scan all devices on this target.
	 */
	if (qla2xenbinq && (cp->cmnd[0] == INQUIRY) && (cp->lun == 0)) {
		strp = (uint8_t *)cp->request_buffer;
		if (*strp == 0x7f) {
			/* Make lun unassigned and processor type */
			*strp = 0x23;
		}
	}
}

/*
 *  qla2x00_status_entry
 *      Processes received ISP status entry.
 *
 * Input:
 *      ha           = adapter block pointer.
 *      pkt          = entry pointer.
 *      done_q_first = done queue first pointer.
 *      done_q_last  = done queue last pointer.
 */
STATIC void
qla2x00_status_entry(scsi_qla_host_t *ha, sts_entry_t *pkt ) 
{
#if defined(QL_DEBUG_LEVEL_2)
	uint32_t	b, l;
#endif
	uint32_t	t; /*target*/
	uint8_t		sense_sz = 0;
	srb_t		*sp;
	os_lun_t	*lq;
	os_tgt_t	*tq;
	uint32_t	resid;
	Scsi_Cmnd	*cp;
	uint16_t	comp_status;
	uint16_t	scsi_status;
	uint8_t		lscsi_status;
	fc_port_t	*fcport;
	scsi_qla_host_t	*vis_ha;


	ENTER(__func__);

	/* Validate handle. */
	if (pkt->handle < MAX_OUTSTANDING_COMMANDS) {
		sp = ha->outstanding_cmds[pkt->handle];
		/* Free outstanding command slot. */
		ha->outstanding_cmds[pkt->handle] = 0;
	} else
		sp = NULL;

	if (sp == NULL) {
		printk(KERN_WARNING
			"qla2x00: Status Entry invalid handle.\n");

		DEBUG2(printk("qla2x00: Status Entry invalid handle.\n");)
		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		if (ha->dpc_wait && !ha->dpc_active) 
			up(ha->dpc_wait);
		return;
	}

	cp = sp->cmd;
	if (cp == NULL) {
		printk(KERN_WARNING 
			"%s(): cmd is NULL: already returned to OS (sp=%p)\n",
			__func__,
			sp);
		DEBUG2(printk("%s(): cmd already returned back to OS "
				"pkt->handle:%d sp=%p sp->state:%d\n",
				__func__,
				pkt->handle,
				sp,
				sp->state);)
		return;
	}

	/*
	 * Set the visible adapter for lun Q access.
	 */
	vis_ha = (scsi_qla_host_t *)cp->host->hostdata;
	if (ha->actthreads)
		ha->actthreads--;

	if (sp->lun_queue == NULL) {
		printk(KERN_WARNING
			"qla2x00: Status Entry invalid lun pointer.\n");
		DEBUG2(printk("qla2x00: Status Entry invalid lun pointer.\n");)
		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		if (ha->dpc_wait && !ha->dpc_active) 
			up(ha->dpc_wait);
		return;
	}

	sp->lun_queue->out_cnt--;
	ha->iocb_cnt -= sp->iocb_cnt;

	comp_status = le16_to_cpu(pkt->comp_status);
	/* Mask of reserved bits 12-15.  Before we examine the scsi status */
	scsi_status = le16_to_cpu(pkt->scsi_status) & SS_MASK;
	lscsi_status = scsi_status & STATUS_MASK;

	CMD_ENTRY_STATUS(cp) = pkt->entry_status;
	CMD_COMPL_STATUS(cp) = comp_status;
	CMD_SCSI_STATUS(cp) = scsi_status;

	/* dg 10/11 */
	sp->flags |= SRB_ISP_COMPLETED;

#if defined(QL_DEBUG_LEVEL_2)
	/* Generate LU queue on cntrl, target, LUN */
	b = SCSI_BUS_32(cp);
	t = SCSI_TCN_32(cp);
	l = SCSI_LUN_32(cp);
#endif
	tq = sp->tgt_queue;
	lq = sp->lun_queue;

#ifdef PERF_MONITORING
	/* update stats */
	lq->resp_time += jiffies - sp->u_start; 
	lq->act_time += jiffies - sp->r_start;  
#endif

	/*
	 * If loop is in transient state Report DID_BUS_BUSY
	 */
	if (!(sp->flags & SRB_IOCTL) &&
		(atomic_read(&ha->loop_down_timer) ||
		 ha->loop_state != LOOP_READY) &&
		(comp_status != CS_COMPLETE ||
		 scsi_status != 0)) {

		DEBUG2(printk("scsi(%ld:%d:%d:%d): Loop Not Ready - pid=%lx.\n",
				ha->host_no, 
				b, t, l, 
				sp->cmd->serial_number);)
#if DG
		CMD_RESULT(cp) = DID_BUS_BUSY << 16;
		add_to_done_queue(ha, sp);
#else
		qla2x00_extend_timeout(sp->cmd, EXTEND_CMD_TIMEOUT);
		add_to_retry_queue(ha, sp);
#endif
		return;
	}

	/*
	 * Based on Host and scsi status generate status code for Linux
	 */
	switch (comp_status) {
		case CS_COMPLETE:
			/*
			 * Host complted command OK.  Check SCSI Status to
			 * determine the correct Host status.
			 */
			if (scsi_status == 0) {
				CMD_RESULT(cp) = DID_OK << 16;

#ifndef __VMWARE__
				/*
				 * Special case consideration On an Inquiry
				 * command (0x12) for Lun 0, device responds
				 * with no devices (0x7F), then Linux will not
				 * scan further Luns. While reporting that some
				 * device exists on Lun 0 Linux will scan all
				 * devices on this target.
				 */
				/* Perform any post command processing */
				qla2x00_filter_command(ha, sp);
#endif
			} else {   /* Check for non zero scsi status */
				if (lscsi_status == SS_BUSY_CONDITION) {
					CMD_RESULT(cp) = DID_BUS_BUSY << 16 |
							 lscsi_status;
				} else {
					CMD_RESULT(cp) = DID_OK << 16 |
							 lscsi_status;

					if (lscsi_status != SS_CHECK_CONDITION)
						break;

					/*
					 * Copy Sense Data into sense buffer
					 */
					memset(cp->sense_buffer, 0, 
						sizeof(cp->sense_buffer));

					if (!(scsi_status & SS_SENSE_LEN_VALID))
						break;

					if (le16_to_cpu(pkt->req_sense_length) <
							CMD_SNSLEN(cp))
						sense_sz = le16_to_cpu(
							pkt->req_sense_length);
					else
						sense_sz = CMD_SNSLEN(cp) - 1;

					CMD_ACTUAL_SNSLEN(cp) = sense_sz;
					sp->request_sense_length = sense_sz;
				       	sp->request_sense_ptr =
					       	(void *)cp->sense_buffer;

				       	if (sp->request_sense_length > 32) 
						sense_sz = 32;

					memcpy(cp->sense_buffer,
							pkt->req_sense_data,
							sense_sz);

					sp->request_sense_ptr += sense_sz;
					sp->request_sense_length -= sense_sz;
					if (sp->request_sense_length != 0)
						ha->status_srb = sp;

					if (!(sp->flags & SRB_IOCTL) &&
						qla2x00_check_sense(cp, lq) ==
							QL_STATUS_SUCCESS) {
						/*
						 * Throw away status_cont
						 * if any
						 */
					       	ha->status_srb = NULL;
						add_to_scsi_retry_queue(ha, sp);
						return;
					}
#if defined(QL_DEBUG_LEVEL_5)
					printk("%s(): Check condition Sense "
						"data, scsi(%ld:%d:%d:%d) "
						"cmd=%p pid=%ld\n",
						__func__,
						ha->host_no, 
						b, t, l,
						cp, cp->serial_number);
					if (sense_sz)
						qla2x00_dump_buffer(
							cp->sense_buffer,
							CMD_ACTUAL_SNSLEN(cp));
#endif
				}
			}
			break;

		case CS_DATA_UNDERRUN:
			DEBUG2(printk(KERN_INFO
					"qla%ld:%d:%d UNDERRUN status detected "
					"0x%x-0x%x.\n",
					ha->host_no, 
					t,l,
					comp_status, 
					scsi_status);)
			resid = le32_to_cpu(pkt->residual_length);
			CMD_RESID_LEN(cp) = resid;

			/*
			 * Check to see if SCSI Status is non zero.  If so
			 * report SCSI Status
			 */
			if (lscsi_status != 0) {
				if (lscsi_status == SS_BUSY_CONDITION) {
					CMD_RESULT(cp) = DID_BUS_BUSY << 16 |
						 lscsi_status;
				} else {
					CMD_RESULT(cp) = DID_OK << 16 |
						 lscsi_status;

					if (lscsi_status != SS_CHECK_CONDITION)
						break;

					/*
					 * Copy Sense Data into sense buffer
					 */
					memset(cp->sense_buffer, 0, 
						sizeof(cp->sense_buffer));

					if (!(scsi_status & SS_SENSE_LEN_VALID))
						break;

					if (le16_to_cpu(pkt->req_sense_length) <
							CMD_SNSLEN(cp))
						sense_sz = le16_to_cpu(
							pkt->req_sense_length);
					else
						sense_sz = CMD_SNSLEN(cp) - 1;

					CMD_ACTUAL_SNSLEN(cp) = sense_sz;
					sp->request_sense_length = sense_sz;
				       	sp->request_sense_ptr =
					       	(void *)cp->sense_buffer;

				       	if (sp->request_sense_length > 32) 
						sense_sz = 32;

					memcpy(cp->sense_buffer,
							pkt->req_sense_data,
							sense_sz);

					sp->request_sense_ptr += sense_sz;
					sp->request_sense_length -= sense_sz;
					if (sp->request_sense_length != 0)
						ha->status_srb = sp;

					if (!(sp->flags & SRB_IOCTL) && 
						(qla2x00_check_sense(cp, lq) ==
							QL_STATUS_SUCCESS)) {
						ha->status_srb = NULL;
						add_to_scsi_retry_queue(ha,sp);
						return;
					}
#if defined(QL_DEBUG_LEVEL_5)
					printk("scsi: Check condition Sense "
						"data, scsi(%ld:%d:%d:%d)\n",
						ha->host_no, b, t, l);
					if (sense_sz)
						qla2x00_dump_buffer(
							cp->sense_buffer,
							CMD_ACTUAL_SNSLEN(cp));
#endif
				}
			} else {
				/*
				 * If RISC reports underrun and target does not
				 * report it then we must have a lost frame, so
				 * tell upper layer to retry it by reporting a
				 * bus busy.
				 */
				if (!(scsi_status & SS_RESIDUAL_UNDER)) {
					ha->dropped_frame_error_cnt++;
					CMD_RESULT(cp) = DID_BUS_BUSY << 16;
					DEBUG2(printk("scsi(%ld): Dropped "
						"frame(s) detected (%x of %x "
						"bytes)...retrying command.\n",
						ha->host_no,
						resid,
						CMD_XFRLEN(cp));)
					break;
				}

				/*
				 * Handle mid-layer underflow???
				 *
				 * For kernels less than 2.4, the driver must
				 * return an error if an underflow is detected.
				 * For kernels equal-to and above 2.4, the
				 * mid-layer will appearantly handle the
				 * underflow by detecting the residual count --
				 * unfortunately, we do not see where this is
				 * actually being done.  In the interim, we
				 * will return DID_ERROR.
				 */
				cp->resid = resid;
				if ((unsigned)(CMD_XFRLEN(cp) - resid) <
							cp->underflow) {
					CMD_RESULT(cp) = DID_ERROR << 16;
					printk(KERN_INFO 
						"scsi(%ld): Mid-layer "
						"underflow detected "
						"(%x of %x bytes) wanted "
						"%x bytes...returning "
						"DID_ERROR status!\n",
						ha->host_no,
						resid,
						CMD_XFRLEN(cp),
						cp->underflow);
					break;
				}

				/* Everybody online, looking good... */
				CMD_RESULT(cp) = DID_OK << 16;

#ifndef __VMWARE__
				/*
				 * Special case consideration On an Inquiry
				 * command (0x12) for Lun 0, device responds
				 * with no devices (0x7F), then Linux will not
				 * scan further Luns. While reporting that some
				 * device exists on Lun 0 Linux will scan all
				 * devices on this target.
				 */
				/* Perform any post command processing */
				qla2x00_filter_command(ha, sp);
#endif
			}
			break;

		case CS_PORT_LOGGED_OUT:
		case CS_PORT_CONFIG_CHG:
		case CS_PORT_BUSY:
		case CS_INCOMPLETE:
		case CS_PORT_UNAVAILABLE:
			/*
			 * If the port is in Target Down state, return all IOs
			 * for this Target with DID_NO_CONNECT ELSE Queue the
			 * IOs in the retry_queue
			 */
			fcport = lq->fclun->fcport;
			DEBUG2(printk("scsi(%ld:%2d:%2d): status_entry: "
					"Port Down pid=%ld, compl "
					"status=0x%x, port state=0x%x\n",
					ha->host_no,
					t, l,
					sp->cmd->serial_number,
					comp_status,
					atomic_read(&fcport->state));)
			if ((sp->flags & SRB_IOCTL) ||
			    (atomic_read(&fcport->state) == FC_DEVICE_DEAD)) {
				CMD_RESULT(cp) = DID_NO_CONNECT << 16;
				add_to_done_queue(ha, sp);
			} else {
				qla2x00_extend_timeout(cp,
						EXTEND_CMD_TIMEOUT);
				add_to_retry_queue(ha, sp);
			}

			if (atomic_read(&fcport->state) == FC_ONLINE) {
				qla2x00_mark_device_lost(ha, fcport);
			}

			return;
			break;

		case CS_RESET:
			DEBUG2(printk(KERN_INFO 
					"scsi(%ld): RESET status detected "
					"0x%x-0x%x.\n",
					ha->host_no, 
					comp_status, 
					scsi_status);)

			if (sp->flags & SRB_IOCTL) {
				CMD_RESULT(cp) = DID_RESET << 16;
			}
			else {
				qla2x00_extend_timeout(cp,
						EXTEND_CMD_TIMEOUT);
				add_to_retry_queue(ha, sp);
				return;
			}
			break;

		case CS_ABORTED:
			/* 
			 * hv2.19.12 - DID_ABORT does not retry the request if
			 * we aborted this request then abort otherwise it must
			 * be a reset 
			 */
			DEBUG2(printk(KERN_INFO 
					"scsi(%ld): ABORT status detected "
					"0x%x-0x%x.\n",
					ha->host_no, 
					comp_status, 
					scsi_status);)
			CMD_RESULT(cp) = DID_RESET << 16;
			break;

		case CS_TIMEOUT:
			DEBUG2(printk(KERN_INFO
					"qla%ld TIMEOUT status detected "
					"0x%x-0x%x.\n",
					ha->host_no, 
					comp_status, 
					scsi_status);)

			fcport = lq->fclun->fcport;
			CMD_RESULT(cp) = DID_BUS_BUSY << 16;

			/* 
			 * v2.19.8 if timeout then check to see if logout
			 * occurred
			 */
			t = SCSI_TCN_32(cp);
			if ((le16_to_cpu(pkt->status_flags) &
						IOCBSTAT_SF_LOGO)) {

				DEBUG2(printk("scsi: Timeout occurred with "
						"Logo, status flag (%x) with "
						"public device loop id (%x), "
						"attempt new recovery\n",
						le16_to_cpu(pkt->status_flags), 
						ha->fc_db[t].loop_id);)
				ha->fc_db[t].flag |= DEV_RELOGIN;
				fcport->login_retry = ha->login_retry_count;
				set_bit(RELOGIN_NEEDED, &ha->dpc_flags);
			}
			break;

		case CS_QUEUE_FULL:
			DEBUG2(printk(KERN_INFO
				       "scsi(%ld): QUEUE FULL status detected "
				       "0x%x-0x%x.\n",
					ha->host_no, 
					comp_status, 
					scsi_status);)
			/*
			 * SCSI Mid-Layer handles device queue full
			 */				 
			CMD_RESULT(cp) = DID_OK << 16 | lscsi_status;
			break;

		default:
			printk(KERN_INFO
				"scsi(%ld): Unknown status detected "
				"0x%x-0x%x.\n",
				ha->host_no, 
				comp_status, 
				scsi_status);
			DEBUG3(printk("scsi: Error detected 0x%x-0x%x.\n",
					comp_status, 
					scsi_status);)

			CMD_RESULT(cp) = DID_ERROR << 16;

			break;
	} /* end of switch comp_status */

	/* Place command on done queue. */
	if (ha->status_srb == NULL)
		add_to_done_queue(ha, sp);

	LEAVE(__func__);
}

/*
 *  qla2x00_status_cont_entry
 *      Processes status continuation entry.
 *
 * Input:
 *      ha           = adapter block pointer.
 *      pkt          = entry pointer.
 *
 * Context:
 *      Interrupt context.
 */
STATIC void
qla2x00_status_cont_entry(scsi_qla_host_t *ha, sts_cont_entry_t *pkt )
{
	uint8_t    sense_sz = 0;
	srb_t      *sp = ha->status_srb;
	Scsi_Cmnd      *cp;

	ENTER(__func__);

	if (sp != NULL && sp->request_sense_length != 0) {
		cp = sp->cmd;
		if (cp == NULL) {
			printk(KERN_INFO
				"%s(): cmd is NULL: already returned to OS "
				"(sp=%p)\n",
				__func__,
				sp); 
			DEBUG2(printk("%s(): cmd already returned back to OS "
					"sp=%p sp->state:%d\n",
					__func__,
					sp,
					sp->state);)
			ha->status_srb = NULL;
			return;
		}

		if (sp->request_sense_length > sizeof (pkt->req_sense_data)) {
			sense_sz = sizeof (pkt->req_sense_data);
		} else {
			sense_sz = sp->request_sense_length;
		}

		/* Move sense data. */
		memcpy(sp->request_sense_ptr, pkt->req_sense_data, sense_sz);
		DEBUG5(qla2x00_dump_buffer(sp->request_sense_ptr, sense_sz);)

		sp->request_sense_ptr += sense_sz;
		sp->request_sense_length -= sense_sz;

		/* Place command on done queue. */
		if (sp->request_sense_length == 0) {
			add_to_done_queue(ha, sp);
			ha->status_srb = NULL;
		}
	}

	LEAVE(__func__);
}


/*
*  qla2x00_error_entry
*      Processes error entry.
*
* Input:
*      ha           = adapter block pointer.
*      pkt          = entry pointer.
*/
STATIC void
qla2x00_error_entry(scsi_qla_host_t *ha, response_t *pkt) 
{
	srb_t *sp;
#ifdef PERF_MONITORING
	os_lun_t	*lq;
#endif

	ENTER(__func__);

#if defined(QL_DEBUG_LEVEL_2)
	if (pkt->entry_status & RF_INV_E_ORDER)
		printk("%s: Invalid Entry Order\n", __func__);
	else if (pkt->entry_status & RF_INV_E_COUNT)
		printk("%s: Invalid Entry Count\n", __func__);
	else if (pkt->entry_status & RF_INV_E_PARAM)
		printk("%s: Invalid Entry Parameter\n", __func__);
	else if (pkt->entry_status & RF_INV_E_TYPE)
		printk("%s: Invalid Entry Type\n", __func__);
	else if (pkt->entry_status & RF_BUSY)
		printk("%s: Busy\n", __func__);
	else
		printk("%s: UNKNOWN flag error\n", __func__);
#endif

	/* Validate handle. */
	if (pkt->handle < MAX_OUTSTANDING_COMMANDS)
		sp = ha->outstanding_cmds[pkt->handle];
	else
		sp = NULL;

	if (sp) {
		/* Free outstanding command slot. */
		ha->outstanding_cmds[pkt->handle] = 0;
		if (ha->actthreads)
			ha->actthreads--;
		sp->lun_queue->out_cnt--;
		ha->iocb_cnt -= sp->iocb_cnt;
#ifdef PERF_MONITORING
		/* update stats */
		lq = sp->lun_queue;
		lq->resp_time += jiffies - sp->u_start; 
		lq->act_time += jiffies - sp->r_start;  
#endif

		sp->flags |= SRB_ISP_COMPLETED;

		/* Bad payload or header */
		if (pkt->entry_status &
			(RF_INV_E_ORDER | RF_INV_E_COUNT |
			 RF_INV_E_PARAM | RF_INV_E_TYPE)) {
			CMD_RESULT(sp->cmd) = DID_ERROR << 16;
		} else if (pkt->entry_status & RF_BUSY) {
			CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;
		} else {
			CMD_RESULT(sp->cmd) = DID_ERROR << 16;
		}
		/* Place command on done queue. */
		add_to_done_queue(ha, sp);

	} else if (pkt->entry_type == COMMAND_A64_TYPE ||
			pkt->entry_type == COMMAND_TYPE) {

		DEBUG2(printk("%s(): ISP Invalid handle\n", __func__);)
		printk(KERN_WARNING
			"qla2x00: Error Entry invalid handle");
		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		if (ha->dpc_wait && !ha->dpc_active) 
			up(ha->dpc_wait);
	}

	LEAVE(__func__);
}

STATIC void
qla2x00_ms_entry(scsi_qla_host_t *ha, ms_iocb_entry_t *pkt) 
{
	srb_t          *sp;

	ENTER(__func__);

	DEBUG3(printk("%s(): pkt=%p pkthandle=%d.\n",
	    __func__, pkt, pkt->handle1);)
	
	/* Validate handle. */
	if (pkt->handle1 < MAX_OUTSTANDING_COMMANDS)
		sp = ha->outstanding_cmds[pkt->handle1];
	else
		sp = NULL;

	if (sp == NULL) {
		printk(KERN_WARNING
			"qla2x00: MS Entry invalid handle.\n");

		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		return;
	}

	CMD_COMPL_STATUS(sp->cmd) = le16_to_cpu(pkt->status);
	CMD_ENTRY_STATUS(sp->cmd) = pkt->entry_status;

	/* Free outstanding command slot. */
	ha->outstanding_cmds[pkt->handle1] = 0;
	sp->flags |= SRB_ISP_COMPLETED;

	add_to_done_queue(ha, sp);

	LEAVE(__func__);
}

/*
 *  qla2x00_restart_queues
 *	Restart device queues.
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Context:
 *	Kernel/Interrupt context.
 */
void
qla2x00_restart_queues(scsi_qla_host_t *ha, uint8_t flush) 
{
	srb_t  		*sp;
	int		retry_q_cnt = 0;
	int		pending_q_cnt = 0;
	struct list_head *list, *temp;
	unsigned long flags = 0;
	scsi_qla_host_t *vis_ha;

	ENTER(__func__);

	clear_bit(RESTART_QUEUES_NEEDED, &ha->dpc_flags);

	/*
	 * start pending queue
	 */
	pending_q_cnt = ha->qthreads;
	if (flush) {
		spin_lock_irqsave(&ha->list_lock,flags);
		list_for_each_safe(list, temp, &ha->pending_queue) {
			sp = list_entry(list, srb_t, list);
			/* 
			 * When time expire return request back to OS as BUSY 
			 */
			__del_from_pending_queue(ha, sp);
			CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;
			CMD_HANDLE(sp->cmd) = (unsigned char *)NULL;
			__add_to_done_queue(ha, sp);
		}
		spin_unlock_irqrestore(&ha->list_lock, flags);
	} else {
		if (!list_empty(&ha->pending_queue))
			qla2x00_next(ha);
	}

	/*
	 * Clear out our retry queue
	 */
	if (flush) {
		spin_lock_irqsave(&ha->list_lock, flags);
		retry_q_cnt = ha->retry_q_cnt;
		list_for_each_safe(list, temp, &ha->retry_queue) {
			sp = list_entry(list, srb_t, list);
			/* when time expire return request back to OS as BUSY */
			__del_from_retry_queue(ha, sp);
			CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;
			CMD_HANDLE(sp->cmd) = (unsigned char *) NULL;
			__add_to_done_queue(ha, sp);
		}
		spin_unlock_irqrestore(&ha->list_lock, flags);

		DEBUG2(printk("%s(%ld): callback %d commands.\n",
				__func__,
				ha->host_no,
				retry_q_cnt);)
	}

	DEBUG2(printk("%s(%ld): active=%ld, retry=%d, pending=%d, "
			"done=%ld, failover=%d, scsi retry=%d commands.\n",
			__func__,
			ha->host_no,
			ha->actthreads,
			ha->retry_q_cnt,
			pending_q_cnt,
			ha->done_q_cnt,
			ha->failover_cnt,
			ha->scsi_retry_q_cnt);)

	if (ha->flags.failover_enabled) {
		/* Try and start all visible adapters */
		for (vis_ha=qla2x00_hostlist;
				(vis_ha != NULL); vis_ha=vis_ha->next) {

			if (!list_empty(&vis_ha->pending_queue))
				qla2x00_next(vis_ha);

			DEBUG2(printk("host(%ld):Commands active=%d busy=%d "
					"failed=%d\nin_recovery=%d "
					"eh_active=%d\n ",
					vis_ha->host_no,
					atomic_read(&vis_ha->host->host_active),
					vis_ha->host->host_busy,
					vis_ha->host->host_failed,
					vis_ha->host->in_recovery,
					vis_ha->host->eh_active);)	
		}
	}

	if (!list_empty(&ha->done_queue))
#if QLA2X_PERFORMANCE
		tasklet_schedule(&ha->run_qla_task);
#else
		qla2x00_done(ha);
#endif

	LEAVE(__func__);
}

/*
 *  qla2x00_abort_queues
 *	Abort all commands on queues on device
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Context:
 *	Interrupt context.
 */
STATIC void
qla2x00_abort_queues(scsi_qla_host_t *ha, uint8_t doneqflg) 
{

	srb_t       *sp;
	struct list_head *list, *temp;
	unsigned long flags;

	ENTER(__func__);

	clear_bit(ABORT_QUEUES_NEEDED, &ha->dpc_flags);

	/* Return all commands device queues. */
	spin_lock_irqsave(&ha->list_lock,flags);
	list_for_each_safe(list, temp, &ha->pending_queue) {
		sp = list_entry(list, srb_t, list);

		if (sp->flags & SRB_ABORTED)
			continue;

		/* Remove srb from LUN queue. */
		__del_from_pending_queue(ha, sp);

		/* Set ending status. */
		CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;

		__add_to_done_queue(ha, sp);
	}
	spin_unlock_irqrestore(&ha->list_lock, flags);

	LEAVE(__func__);
}


/*
 * qla2x00_reset_lun_fo_counts
 *	Reset failover retry counts
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Context:
 *	Interrupt context.
 */
void 
qla2x00_reset_lun_fo_counts(scsi_qla_host_t *ha, os_lun_t *lq) 
{
	srb_t		*tsp;
	os_lun_t	*orig_lq;
	struct list_head *list;
	unsigned long	flags ;

	spin_lock_irqsave(&ha->list_lock, flags);
	/*
	 * the pending queue.
	 */
	list_for_each(list,&ha->pending_queue) {
		tsp = list_entry(list, srb_t, list);
		orig_lq = tsp->lun_queue;
		if (orig_lq == lq)
			tsp->fo_retry_cnt = 0;
	}
	/*
	 * the retry queue.
	 */
	list_for_each(list,&ha->retry_queue) {
		tsp = list_entry(list, srb_t, list);
		orig_lq = tsp->lun_queue;
		if (orig_lq == lq)
			tsp->fo_retry_cnt = 0;
	}

	/*
	 * the done queue.
	 */
	list_for_each(list, &ha->done_queue) {
		tsp = list_entry(list, srb_t, list);
		orig_lq = tsp->lun_queue;
		if (orig_lq == lq)
			tsp->fo_retry_cnt = 0;
	}
	spin_unlock_irqrestore(&ha->list_lock, flags);
}

/*
 *  qla2x00_failover_cleanup
 *	Cleanup queues after a failover.
 *
 * Input:
 *	sp = command pointer
 *
 * Context:
 *	Interrupt context.
 */
STATIC void
qla2x00_failover_cleanup(srb_t *sp) 
{

	CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;
	CMD_HANDLE(sp->cmd) = (unsigned char *) NULL;

	/* turn-off all failover flags */
	sp->flags = sp->flags & ~(SRB_RETRY|SRB_FAILOVER|SRB_FO_CANCEL);
}


/*
 *  qla2x00_process_failover
 *	Process any command on the failover queue.
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Context:
 *	Interrupt context.
 */
STATIC void
qla2x00_process_failover(scsi_qla_host_t *ha) 
{

	os_tgt_t	*tq;
	os_lun_t	*lq;
	srb_t       *sp;
	fc_port_t *fcport;
	struct list_head *list, *temp;
	unsigned long flags;
	uint32_t    t, l;
	scsi_qla_host_t *vis_ha = NULL;

	DEBUG(printk("%s(): Processing failover for hba %ld\n",
			__func__,
			ha->host_no);)

	/*
	 * Process all the commands in the failover queue. Attempt to failover
	 * then either complete the command as is or requeue for retry.
	 */

	/* Prevent or allow acceptance of new I/O requests. */
	spin_lock_irqsave(&ha->list_lock, flags);

	/*
	 * Get first entry to find our visible adapter.  We could never get
	 * here if the list is empty
	 */
	list = ha->failover_queue.next;
	sp = list_entry(list, srb_t, list);
	vis_ha = (scsi_qla_host_t *) sp->cmd->host->hostdata;
	list_for_each_safe(list, temp, &ha->failover_queue) {
		sp = list_entry(list, srb_t, list);

		tq = sp->tgt_queue;
		lq = sp->lun_queue;
		fcport = lq->fclun->fcport;

		/* Remove srb from failover queue. */
		__del_from_failover_queue(ha, sp);

		DEBUG2(printk("%s(): pid %ld retrycnt=%d\n",
				__func__,
				sp->cmd->serial_number,
				sp->cmd->retries);)

		/*** Select an alternate path ***/
		/* 
		 * If the path has already been change by a previous request
		 * sp->fclun != lq->fclun
		 */
		if (sp->fclun != lq->fclun || 
		  	atomic_read(&fcport->state) != FC_DEVICE_DEAD) {

			qla2x00_failover_cleanup(sp);
		} else if (qla2x00_cfg_failover(ha, lq->fclun,
						tq, sp) == NULL) {
			/*
			 * We ran out of paths, so just post the status which
			 * is already set in the cmd.
			 */
			printk(KERN_INFO
				"%s(): Ran out of paths - pid %ld\n",
				__func__,
				sp->cmd->serial_number);
		} else {
			qla2x00_failover_cleanup(sp);

		}
		__add_to_done_queue(ha, sp);
	} /* list_for_each_safe */
	spin_unlock_irqrestore(&ha->list_lock,flags);

	for (t = 0; t < vis_ha->max_targets; t++) {
		if ((tq = vis_ha->otgt[t]) == NULL)
			continue;
		for (l = 0; l < vis_ha->max_luns; l++) {
			if ((lq = (os_lun_t *) tq->olun[l]) == NULL)
				continue;

			if( test_and_clear_bit(LUN_MPIO_BUSY, &lq->q_flag) ) {
				/* EMPTY */
				DEBUG(printk("%s(): remove suspend for "
						"lun %d\n",
						__func__,
						lq->fclun->lun);)
			}
		}
	}

	//qla2x00_restart_queues(ha,TRUE);
	qla2x00_restart_queues(ha, FALSE);

	DEBUG(printk("%s() - done", __func__);)
}

/*
 *  qla2x00_loop_resync
 *      Resync with fibre channel devices.
 *
 * Input:
 *      ha = adapter block pointer.
 *
 * Returns:
 *      0 = success
 */
STATIC uint8_t
qla2x00_loop_resync(scsi_qla_host_t *ha) 
{
	uint8_t   status;

	ENTER(__func__);

	DEBUG(printk("%s(): entered\n", __func__);)

	ha->loop_state = LOOP_UPDATE;
	qla2x00_stats.loop_resync++;
	clear_bit(ISP_ABORT_RETRY, &ha->dpc_flags);
	if (ha->flags.online) {
		if (!(status = qla2x00_fw_ready(ha))) {
			do {
				/* v2.19.05b6 */
				ha->loop_state = LOOP_UPDATE;

				/*
				 * Issue marker command only when we are going
				 * to start the I/O .
				 */
				ha->marker_needed = 1;

				/* Remap devices on Loop. */
				clear_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags);

				qla2x00_configure_loop(ha);

			} while (!atomic_read(&ha->loop_down_timer) &&
				!(test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags)) &&
				(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)));
		}
		qla2x00_restart_queues(ha,TRUE);
	} else
		status = 0;

	if (test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags)) {
		return (1);
	}

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status)
		printk("%s(): **** FAILED ****\n", __func__);
#endif

	LEAVE(__func__);

	return(status);
}

/*
 * qla2x00_debounce_register
 *      Debounce register.
 *
 * Input:
 *      port = register address.
 *
 * Returns:
 *      register value.
 */
STATIC uint16_t
qla2x00_debounce_register(volatile uint16_t *addr) 
{
	volatile uint16_t ret;
	volatile uint16_t ret2;

	do {
		ret = RD_REG_WORD(addr);
		barrier();
		ret2 = RD_REG_WORD(addr);
	} while (ret != ret2);

	return(ret);
}


/*
 * __qla2x00_reset_chip
 *      Reset ISP chip.
 *
 * Input:
 *      ha = adapter block pointer.
 */
STATIC void
__qla2x00_reset_chip(scsi_qla_host_t *ha) 
{
	device_reg_t	*reg = ha->iobase;
	uint32_t	cnt;
	unsigned long	mbx_flags = 0;

	ENTER(__func__);

/* ??? -- Safely remove??? */
#if 1
	/* Pause RISC. */
	WRT_REG_WORD(&reg->host_cmd, HC_PAUSE_RISC);
#if defined(ISP2300)
	if (ha->device_id == QLA2312_DEVICE_ID) {
		UDELAY(10);
	} else {
		for (cnt = 0; cnt < 30000; cnt++) {
			if ((RD_REG_WORD(&reg->host_cmd) & HC_RISC_PAUSE) != 0)
				break;
			else
				UDELAY(100);
		}
	}
#else
	for (cnt = 0; cnt < 30000; cnt++) {
		if ((RD_REG_WORD(&reg->host_cmd) & HC_RISC_PAUSE) != 0)
			break;
		else
			UDELAY(100);
	}
#endif

	/* Select FPM registers. */
	WRT_REG_WORD(&reg->ctrl_status, 0x20);

	/* FPM Soft Reset. */
	WRT_REG_WORD(&reg->fpm_diag_config, 0x100);
#if defined(ISP2300)
	WRT_REG_WORD(&reg->fpm_diag_config, 0x0); /* Toggle Fpm Reset */
#endif
	/* Select frame buffer registers. */
	WRT_REG_WORD(&reg->ctrl_status, 0x10);

	/* Reset frame buffer FIFOs. */
	WRT_REG_WORD(&reg->fb_cmd, 0xa000);

	/* Select RISC module registers. */
	WRT_REG_WORD(&reg->ctrl_status, 0);

	WRT_REG_WORD(&reg->semaphore, 0);

	WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
	WRT_REG_WORD(&reg->host_cmd, HC_CLR_HOST_INT);

	/* Reset ISP chip. */
	WRT_REG_WORD(&reg->ctrl_status, CSR_ISP_SOFT_RESET);

#if defined(ISP2300)
	if (ha->device_id == QLA2312_DEVICE_ID) {
		UDELAY(10);
	} else {
		/* Wait for RISC to recover from reset. */
		for (cnt = 30000; cnt; cnt--) {
			if (!(RD_REG_WORD(&reg->ctrl_status) &
						CSR_ISP_SOFT_RESET))
				break;
			UDELAY(100);
		}
	}
#else
	/* Wait for RISC to recover from reset. */
	for (cnt = 30000; cnt; cnt--) {
		if (!(RD_REG_WORD(&reg->ctrl_status) & CSR_ISP_SOFT_RESET))
			break;
		UDELAY(100);
	}
#endif

	/* Reset RISC processor. */
	WRT_REG_WORD(&reg->host_cmd, HC_RESET_RISC);
	WRT_REG_WORD(&reg->host_cmd, HC_RELEASE_RISC);

#if defined(ISP2300)
	if (ha->device_id == QLA2312_DEVICE_ID) {
		UDELAY(10);
	} else {
		for (cnt = 0; cnt < 30000; cnt++) {
			/* ra 12/30/01 */
			if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
				QLA_MBX_REG_LOCK(ha);

			if (RD_REG_WORD(&reg->mailbox0) != MBS_BUSY) {
				if (!(test_bit(ABORT_ISP_ACTIVE,
							&ha->dpc_flags)))
					QLA_MBX_REG_UNLOCK(ha);
				break;
			}

			if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
				QLA_MBX_REG_UNLOCK(ha);

			UDELAY(100);
		}
	}
#else
	for (cnt = 0; cnt < 30000; cnt++) {
		/* ra 12/30/01 */
		if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
			QLA_MBX_REG_LOCK(ha);

		if (RD_REG_WORD(&reg->mailbox0) != MBS_BUSY) {
			if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
				QLA_MBX_REG_UNLOCK(ha);
			break;
		}

		if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
			QLA_MBX_REG_UNLOCK(ha);

		UDELAY(100);
	}
#endif

#if defined(ISP2200) || defined(ISP2300)
	/* Disable RISC pause on FPM parity error. */
	WRT_REG_WORD(&reg->host_cmd, HC_DISABLE_PARITY_PAUSE);
#endif

#else
	/* Insure mailbox registers are free. */
	WRT_REG_WORD(&reg->semaphore, 0);
	WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
	WRT_REG_WORD(&reg->host_cmd, HC_CLR_HOST_INT);

	/* clear mailbox busy */
	ha->flags.mbox_busy = FALSE;

	/* Reset ISP chip. */
	WRT_REG_WORD(&reg->ctrl_status, CSR_ISP_SOFT_RESET);

	/*
	 * Delay after reset, for chip to recover.  Otherwise causes system
	 * PANIC
	 */
	mdelay(2);

	for (cnt = 30000; cnt; cnt--) {
		if (!(RD_REG_WORD(&reg->ctrl_status) & CSR_ISP_SOFT_RESET))
			break;
		UDELAY(100);
	}

	/* Reset RISC processor. */
	WRT_REG_WORD(&reg->host_cmd, HC_RESET_RISC);
	WRT_REG_WORD(&reg->host_cmd, HC_RELEASE_RISC);
	for (cnt = 30000; cnt; cnt--) {
		if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
			QLA_MBX_REG_LOCK(ha);
		if (RD_REG_WORD(&reg->mailbox0) != MBS_BUSY ) {
			if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
				QLA_MBX_REG_UNLOCK(ha);
			break;
		}
		if (!(test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)))
			QLA_MBX_REG_UNLOCK(ha);
		UDELAY(100);
	}
#endif

	LEAVE(__func__);
}

/*
 * qla2x00_reset_chip
 *      Reset ISP chip.
 *
 * Input:
 *      ha = adapter block pointer.
 */
STATIC void
qla2x00_reset_chip(scsi_qla_host_t *ha) 
{
	unsigned long   flags = 0;

	ENTER(__func__);

	/* Disable ISP interrupts. */
	qla2x00_disable_intrs(ha);
	/* WRT_REG_WORD(&reg->ictrl, 0); */

	spin_lock_irqsave(&ha->hardware_lock, flags);
	__qla2x00_reset_chip(ha);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	LEAVE(__func__);
}

/*
 * This routine will wait for fabric devices for
 * the reset delay.
 */
void qla2x00_check_fabric_devices(scsi_qla_host_t *ha) 
{
	uint16_t	fw_state;

	qla2x00_get_firmware_state(ha, &fw_state);
}

/*
 * qla2x00_extend_timeout
 *      This routine will extend the timeout to the specified value.
 *
 * Input:
 *      cmd = SCSI command structure
 *
 * Returns:
 *      None.
 */
static void 
qla2x00_extend_timeout(Scsi_Cmnd *cmd, int timeout) 
{
	srb_t *sp = (srb_t *) CMD_SP(cmd);
	u_long our_jiffies = (timeout * HZ) + jiffies;

    	sp->ext_history= 0; 
	sp->e_start = jiffies;
	if (cmd->eh_timeout.function) {
		mod_timer(&cmd->eh_timeout,our_jiffies);
    	 	 sp->ext_history |= 1;
	}
	if (sp->timer.function != NULL) {
		/* 
		 * Our internal timer should timeout before the midlayer has a
		 * chance begin the abort process
		 */
		mod_timer(&sp->timer,our_jiffies - (QLA_CMD_TIMER_DELTA * HZ));

    	 	sp->ext_history |= 2;
	}
}

/*
* qla2x00_display_fc_names
*      This routine will the node names of the different devices found
*      after port inquiry.
*
* Input:
*      cmd = SCSI command structure
*
* Returns:
*      None.
*/
STATIC void
qla2x00_display_fc_names(scsi_qla_host_t *ha) 
{
	uint16_t	tgt;
	os_tgt_t	*tq;

	/* Display the node name for adapter */
	printk(KERN_INFO
		"scsi-qla%d-adapter-node=%02x%02x%02x%02x%02x%02x%02x%02x\\;\n",
		(int)ha->instance,
		ha->init_cb->node_name[0],
		ha->init_cb->node_name[1],
		ha->init_cb->node_name[2],
		ha->init_cb->node_name[3],
		ha->init_cb->node_name[4],
		ha->init_cb->node_name[5],
		ha->init_cb->node_name[6],
		ha->init_cb->node_name[7]);

	/* display the port name for adapter */
	printk(KERN_INFO
		"scsi-qla%d-adapter-port=%02x%02x%02x%02x%02x%02x%02x%02x\\;\n",
		(int)ha->instance,
		ha->init_cb->port_name[0],
		ha->init_cb->port_name[1],
		ha->init_cb->port_name[2],
		ha->init_cb->port_name[3],
		ha->init_cb->port_name[4],
		ha->init_cb->port_name[5],
		ha->init_cb->port_name[6],
		ha->init_cb->port_name[7]);

	/* Print out device port names */
	for (tgt = 0; tgt < MAX_TARGETS; tgt++) {
		if ((tq = ha->otgt[tgt]) == NULL)
			continue;

		if (tq->vis_port == NULL)
			continue;

		switch (ha->binding_type) {
			case BIND_BY_PORT_NAME:
				printk(KERN_INFO
					"scsi-qla%d-tgt-%d-di-0-port="
					"%02x%02x%02x%02x%02x%02x%02x%02x\\;\n",
					(int)ha->instance, 
					tgt,
					tq->port_name[0], 
					tq->port_name[1],
					tq->port_name[2], 
					tq->port_name[3],
					tq->port_name[4], 
					tq->port_name[5],
					tq->port_name[6], 
					tq->port_name[7]);

				break;

			case BIND_BY_PORT_ID:
				printk(KERN_INFO
					"scsi-qla%d-tgt-%d-di-0-pid=%06x\\;\n",
					(int)ha->instance, 
					tgt,
					tq->d_id.b24);
				break;

			case BIND_BY_NODE_NAME:
				printk(KERN_INFO
					"scsi-qla%d-tgt-%d-di-0-node="
					"%02x%02x%02x%02x%02x%02x%02x%02x\\;\n",
					(int)ha->instance, 
					tgt,
					tq->node_name[0], 
					tq->node_name[1],
					tq->node_name[2], 
					tq->node_name[3],
					tq->node_name[4], 
					tq->node_name[5],
					tq->node_name[6], 
					tq->node_name[7]);
				break;
		}

#if VSA
		printk(KERN_INFO
			"scsi-qla%d-target-%d-vsa=01;\n",
			(int)ha->instance, tgt);
#endif
	}
}

/*
 * qla2x00_find_propname
 *	Get property in database.
 *
 * Input:
 *	ha = adapter structure pointer.
 *      db = pointer to database
 *      propstr = pointer to dest array for string
 *	propname = name of property to search for.
 *	siz = size of property
 *
 * Returns:
 *	0 = no property
 *      size = index of property
 *
 * Context:
 *	Kernel context.
 */
STATIC uint8_t
qla2x00_find_propname(scsi_qla_host_t *ha, 
			char *propname, char *propstr, 
			char *db, int siz) 
{
	char	*cp;

	/* find the specified string */
	if (db) {
		/* find the property name */
		if ((cp = strstr(db,propname)) != NULL) {
			while ((*cp)  && *cp != '=')
				cp++;
			if (*cp) {
				strncpy(propstr, cp, siz+1);
				propstr[siz+1] = '\0';
				DEBUG(printk("qla2x00_find_propname: found "
						"property = {%s}\n",
						propstr);)
				return (siz);   /* match */
			}
		}
	}

	return (0);
}


/*
 * qla2x00_get_prop_16chars
 *	Get an 8-byte property value for the specified property name by
 *      converting from the property string found in the configuration file.
 *      The resulting converted value is in big endian format (MSB at byte0).
 *
 * Input:
 *	ha = adapter state pointer.
 *	propname = property name pointer.
 *	propval  = pointer to location for the converted property val.
 *      db = pointer to database
 *
 * Returns:
 *	0 = value returned successfully.
 *
 * Context:
 *	Kernel context.
 */
static int
qla2x00_get_prop_16chars(scsi_qla_host_t *ha,
				char *propname, char *propval, char *db) 
{
	char		*propstr;
	int		i, k;
	int		rval;
	uint8_t		nval;
	uint8_t		*pchar;
	uint8_t		*ret_byte;
	uint8_t		*tmp_byte;
	uint8_t		*retval = (uint8_t*)propval;
	uint8_t		tmpval[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t	max_byte_cnt = 8; /* 16 chars = 8 bytes */
	uint16_t	max_strlen = 16;
	static char	buf[LINESIZE];

	rval = qla2x00_find_propname(ha, propname, buf, db, max_strlen);

	propstr = &buf[0];
	if (*propstr == '=')
		propstr++;   /* ignore equal sign */

	if (rval == 0) {
		return (1);
	}

	/* Convert string to numbers. */
	pchar = (uint8_t *)propstr;
	tmp_byte = (uint8_t *)tmpval;

	rval = 0;
	for (i = 0; i < max_strlen; i++) {
		/*
		 * Check for invalid character, two at a time,
		 * then convert them starting with first byte.
		 */

		if ((pchar[i] >= '0') && (pchar[i] <= '9')) {
			nval = pchar[i] - '0';
		} else if ((pchar[i] >= 'A') && (pchar[i] <= 'F')) {
			nval = pchar[i] - 'A' + 10;
		} else if ((pchar[i] >= 'a') && (pchar[i] <= 'f')) {
			nval = pchar[i] - 'a' + 10;
		} else {
			/* invalid character */
			rval = 1;
			break;
		}

		if (i & BIT_0) {
			*tmp_byte = *tmp_byte | nval;
			tmp_byte++;
		} else {
			*tmp_byte = *tmp_byte | nval << 4;
		}
	}

	if (rval != 0) {
		/* Encountered invalid character. */
		return (rval);
	}

	/* Copy over the converted value. */
	ret_byte = retval;
	tmp_byte = tmpval;

	i = max_byte_cnt;
	k = 0;
	while (i--) {
		*ret_byte++ = *tmp_byte++;
	}

	/* big endian retval[0]; */
	return (0);
}

/*
* qla2x00_get_properties
*	Find all properties for the specified adapeter in
*      command line.
*
* Input:
*	ha = adapter block pointer.
*	cmdline = pointer to command line string
*
* Context:
*	Kernel context.
*/
static void
qla2x00_get_properties(scsi_qla_host_t *ha, char *cmdline) 
{
	static char	propbuf[LINESIZE];
	uint8_t		tmp_name[8];

	/* Adapter FC node names. */
	sprintf(propbuf, "scsi-qla%d-adapter-node", (int) ha->instance);
	qla2x00_get_prop_16chars (ha, 
			propbuf,
			(uint8_t *)(&ha->init_cb->node_name), 
			cmdline);

	sprintf(propbuf, "scsi-qla%d-adapter-port", (int) ha->instance);

	/* DG 04/07 check portname of adapter */
	qla2x00_get_prop_16chars (ha, 
			propbuf,
			tmp_name, 
			cmdline);
	if (memcmp(ha->init_cb->port_name, tmp_name, 8) != 0) {
		/*
		 * Adapter port name is WWN, and cannot be changed.
		 * Inform users of the mismatch, then just continue driver
		 * loading using the original adapter port name in NVRAM.
		 */
		printk(KERN_WARNING
			"qla2x00: qla%ld found mismatch in "
			"adapter port names.\n",
			ha->instance);
		printk(KERN_INFO
			"       qla%ld port name found in NVRAM "
			"-> %02x%02x%02x%02x%02x%02x%02x%02x\n",
			ha->instance,
			ha->init_cb->port_name[0],
			ha->init_cb->port_name[1],
			ha->init_cb->port_name[2],
			ha->init_cb->port_name[3],
			ha->init_cb->port_name[4],
			ha->init_cb->port_name[5],
			ha->init_cb->port_name[6],
			ha->init_cb->port_name[7]);
		printk(KERN_INFO
			"      qla%ld port name found on command line "
			"-> %02x%02x%02x%02x%02x%02x%02x%02x\n",
			ha->instance,
			tmp_name[0],
			tmp_name[1],
			tmp_name[2],
			tmp_name[3],
			tmp_name[4],
			tmp_name[5],
			tmp_name[6],
			tmp_name[7]);
		printk(KERN_INFO
			"      Using port name from NVRAM.\n");
	}

	qla2x00_cfg_persistent_binding(ha);
}

/*
 * qla2x00_update_fc_database
 *      This routine updates the device data in the database.
 *
 * Input:
 *      ha = adapter block pointer.
 *      device = device data pointer.
 *
 * Returns:
 *      0 = success, if device found or added to database.
 *      BIT_0 = error
 *      BIT_1 = database was full and device was not configured.
 */
STATIC uint8_t
qla2x00_update_fc_database(scsi_qla_host_t *ha, 
				fcdev_t *device, uint8_t enable_slot_reuse) 
{
	int		rval;
	uint16_t	cnt, i;

	DEBUG(printk("qla2x00: Found device - "
	    "nodename=%02x%02x%02x%02x%02x%02x%02x%02x, "
	    "portname=%02x%02x%02x%02x%02x%02x%02x%02x, "
	    "port Id=%06x, loop id=%04x\n",
	    device->name[0], device->name[1],
	    device->name[2], device->name[3],
	    device->name[4], device->name[5],
	    device->name[6], device->name[7],
	    device->wwn[0], device->wwn[1],
	    device->wwn[2], device->wwn[3],
	    device->wwn[4], device->wwn[5],
	    device->wwn[6], device->wwn[7],
	    device->d_id.b24, device->loop_id);)

	/* Look for device in database. */
	for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
		if (ha->fc_db[cnt].loop_id == PORT_UNUSED)
			continue;

		rval = 1;
		switch (ha->binding_type) {
			case BIND_BY_PORT_NAME:
				rval = memcmp(device->wwn,
				    ha->fc_db[cnt].wwn, WWN_SIZE);
				break;

			case BIND_BY_PORT_ID:
				rval = (device->d_id.b24 !=
				    ha->fc_db[cnt].d_id.b24);
				break;

			case BIND_BY_NODE_NAME:
				rval = memcmp(device->name,
				    ha->fc_db[cnt].name, WWN_SIZE);
				break;
		}
		if (rval)
			continue;

		DEBUG(printk("qla2x00: Reusing slot %d "
		    "for device "
		    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		    cnt,
		    device->wwn[0],
		    device->wwn[1],
		    device->wwn[2],
		    device->wwn[3],
		    device->wwn[4],
		    device->wwn[5],
		    device->wwn[6],
		    device->wwn[7]);)

		if (device->flag & DEV_PUBLIC) {
			ha->fc_db[cnt].flag |= DEV_PUBLIC;
		} else {
			if (ha->fc_db[cnt].flag & DEV_PUBLIC) {
				ha->fc_db[cnt].flag &= ~DEV_PUBLIC;
				ha->fabricid[ha->fc_db[cnt].loop_id].in_use 
				    = FALSE;
			}
		}

		ha->fc_db[cnt].loop_id = device->loop_id;
		ha->fc_db[cnt].d_id.b24 = device->d_id.b24;

		/* Update volatile unbound fields for PortID binding only */
		if (ha->binding_type == BIND_BY_PORT_ID) {
			memcpy(ha->fc_db[cnt].name, device->name, WWN_SIZE);
			memcpy(ha->fc_db[cnt].wwn, device->wwn, WWN_SIZE);
		}

		return (0);
	}

	/* Find a empty slot and add device into database. */
	for (i = 0; i < MAX_FIBRE_DEVICES; i++) {

/* FlexServ Patch */
#if QLA2XXX_HOTSWAP_ENUMERATION
		/*
		 * Enumerate upon the actual ID so add-single-device works
		 */
		if (i != device->loop_id) {
			continue;
		}
#endif

		if ((ha->fc_db[i].loop_id == PORT_UNUSED) ||
			(ha->fc_db[i].loop_id == PORT_NEED_MAP)) {

			DEBUG(printk("qla2x00: New slot %d for device "
			    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
			    i,
			    device->wwn[0],
			    device->wwn[1],
			    device->wwn[2],
			    device->wwn[3],
			    device->wwn[4],
			    device->wwn[5],
			    device->wwn[6],
			    device->wwn[7]);)

			memcpy(ha->fc_db[i].name, device->name, WWN_SIZE);
 			memcpy(ha->fc_db[i].wwn, device->wwn, WWN_SIZE);
			ha->fc_db[i].loop_id = device->loop_id;
			ha->fc_db[i].d_id.b24 = device->d_id.b24;

			if (device->flag & DEV_PUBLIC)
				ha->fc_db[i].flag |= DEV_PUBLIC;

			ha->flags.updated_fc_db = TRUE;

			return (0);
		}
	}

	if (enable_slot_reuse) {
		for (i = 0; i < MAX_FIBRE_DEVICES; i++) {
			if (ha->fc_db[i].loop_id == PORT_AVAILABLE) {
				DEBUG(printk("qla2x00: Assigned slot %d "
				    "reuse for device "
				    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
				    i, 
				    device->wwn[0],
				    device->wwn[1],
				    device->wwn[2],
				    device->wwn[3],
				    device->wwn[4],
				    device->wwn[5],
				    device->wwn[6],
				    device->wwn[7]);)

				memcpy(ha->fc_db[i].name,
				    device->name, WWN_SIZE);
				memcpy(ha->fc_db[i].wwn,
				    device->wwn, WWN_SIZE);
				ha->fc_db[i].loop_id = device->loop_id;
				ha->fc_db[i].d_id.b24 = device->d_id.b24;

				if (device->flag & DEV_PUBLIC)
					ha->fc_db[i].flag |= DEV_PUBLIC;

				ha->flags.updated_fc_db = TRUE;

				return (0);
			}
		}
	}

	return(BIT_1);
}


/*
 * qla2x00_device_resync
 *	Marks devices in the database that needs resynchronization.
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Context:
 *	Kernel context.
 */
static void
qla2x00_device_resync(scsi_qla_host_t *ha) 
{
	uint16_t index;
	uint32_t mask;
	rscn_t dev;
	struct list_head *fcil;
	fc_initiator_t	*fcinitiator;

	ENTER(__func__);

	while (ha->rscn_out_ptr != ha->rscn_in_ptr ||
			ha->flags.rscn_queue_overflow) {

		memcpy(&dev, &ha->rscn_queue[ha->rscn_out_ptr], sizeof(rscn_t));

		DEBUG(printk("qla%ld: device_resync: rscn_queue[%d], "
				"portID=%06x\n",
				ha->instance,
				ha->rscn_out_ptr,
				ha->rscn_queue[ha->rscn_out_ptr].d_id.b24);)

		ha->rscn_out_ptr++;
		if (ha->rscn_out_ptr == MAX_RSCN_COUNT)
			ha->rscn_out_ptr = 0;

		/* Queue overflow, set switch default case. */
		if (ha->flags.rscn_queue_overflow) {
			DEBUG(printk("device_resync: rscn overflow\n");)

			dev.format = 3;
			ha->flags.rscn_queue_overflow = 0;
		}

		switch (dev.format) {
			case 0:
				mask = 0xffffff;
				break;
			case 1:
				mask = 0xffff00;
				break;
			case 2:
				mask = 0xff0000;
				break;
			default:
				mask = 0x0;
				dev.d_id.b24 = 0;
				ha->rscn_out_ptr = ha->rscn_in_ptr;
				break;
		}

		/* Mark target devices indicated by RSCN for later processing */
		for (index = 0; index < MAX_FIBRE_DEVICES; index++) {
			if ((ha->fc_db[index].flag & DEV_PUBLIC) &&
				(ha->fc_db[index].d_id.b24 & mask) ==
				 dev.d_id.b24) {

				/* fabric device */
				if (ha->fc_db[index].loop_id != PORT_UNUSED) {
					ha->fc_db[index].loop_id |=
								PORT_LOST_ID;

					DEBUG(printk("qla%d: RSCN port @ "
							"slot %d "
							"port_id=%06x\n",
							(int)ha->instance,
							index,
							ha->fc_db[index].d_id.b24);)
				}
			}
		}

		if (dev.format == 3)
			continue;

		/*
		 * Invalidate initiator devices indicated by RSCN so we know
		 * they are no longer logged in.
		 */
		list_for_each(fcil, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if ((fcinitiator->d_id.b24 & mask) != dev.d_id.b24)
				continue;
			if (fcinitiator->loop_id & PORT_LOST_ID ||
				fcinitiator->loop_id & PORT_LOGIN_NEEDED)
				continue;

			fcinitiator->loop_id |= PORT_LOST_ID;
			fcinitiator->d_id.b24 = 0;
		}
	}

	LEAVE(__func__);
}

/*
 * qla2x00_configure_fabric
 *      Setup SNS devices with loop ID's.
 *
 * Input:
 *      ha = adapter block pointer.
 *
 * Returns:
 *      0 = success.
 *      BIT_0 = error
 *      BIT_1 = database was full and device was not configured.
 */
#define MAX_PUBLIC_LOOP_IDS LAST_SNS_LOOP_ID + 1

STATIC uint8_t
qla2x00_configure_fabric(scsi_qla_host_t *ha, uint8_t enable_slot_reuse) 
{
	uint8_t     rval = 0;
	uint8_t     rval1;
	uint8_t     local_flags = 0;
	sns_cmd_rsp_t  *sns;
	uint8_t     tmp_name[8];
	fcdev_t     dev;
	uint16_t    i, index, found_cnt;
	dma_addr_t  phys_address = 0;
	uint16_t    new_dev_cnt;
	uint16_t    tmp_loop_id;
	uint16_t    tmp_topo;
	static struct new_dev new_dev_list[MAX_FIBRE_DEVICES];
	struct list_head *fcil, *fcitemp;
	fc_initiator_t	*fcinitiator;

	ENTER(__func__);

	DEBUG2(printk("scsi%ld: Enter qla2x00_configure_fabric: hba=%p\n",
			ha->host_no, ha);)

	/* If FL port exists, then SNS is present */
	rval1 = qla2x00_get_port_name(ha, SNS_FL_PORT, tmp_name, 0);
	if (rval1 || qla2x00_is_wwn_zero(tmp_name)) {
		DEBUG2(printk("%s(): MBC_GET_PORT_NAME Failed, No FL Port\n",
				__func__);)

		ha->device_flags &= ~SWITCH_FOUND;
		return (0);
	}

	ha->device_flags |= SWITCH_FOUND;

	/* Get adapter port ID. */
	rval = qla2x00_get_adapter_id(ha, &tmp_loop_id, &ha->d_id.b.al_pa,
			&ha->d_id.b.area, &ha->d_id.b.domain, &tmp_topo);

	sns = pci_alloc_consistent(ha->pdev, 
			sizeof(sns_cmd_rsp_t), 
			&phys_address);
	if (sns == NULL) {
		printk(KERN_WARNING
			"qla(%ld): Memory Allocation failed - sns.\n",
			ha->host_no);
		ha->mem_err++;
		return BIT_0;
	}

	memset(sns, 0, sizeof(sns_cmd_rsp_t));

	/* Mark devices that need re-synchronization. */
	qla2x00_device_resync(ha);
	found_cnt = 0;
	do {
#if REG_FC4_ENABLED
		if (test_and_clear_bit(REGISTER_FC4_NEEDED, &ha->dpc_flags)) {
			if (qla2x00_register_fc4(ha, sns, phys_address)) {
				/* EMPTY */
				DEBUG2(printk("%s(%ld): register_fc4 failed.\n",
						__func__,
						ha->host_no);)
			}
			if (qla2x00_register_fc4_feature(ha, sns, phys_address)) {
				/* EMPTY */
				DEBUG2(printk("%s(%ld): register_fc4_feature failed.\n",
						__func__,
						ha->host_no);)
			}
		}
#endif
		rval = qla2x00_find_all_fabric_devs(ha, 
				sns, phys_address,
				new_dev_list, &new_dev_cnt, 
				&local_flags);
		if (rval != 0)
			break;

		/*
		 * Logout all previous fabric devices marked lost, except
		 * tape devices.
		 */
		for (index = 0; index < MAX_FIBRE_DEVICES &&
			!atomic_read(&ha->loop_down_timer) &&
			!(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)); 
			index++) {

			if (ha->fc_db[index].loop_id & PORT_LOST_ID &&
			    (ha->fc_db[index].flag & DEV_PUBLIC) &&
			    !(ha->fc_db[index].flag & DEV_TAPE_DEVICE)) {

				qla2x00_fabric_logout(ha, 
						ha->fc_db[index].loop_id & 
						 0xff);
				local_flags |= LOGOUT_PERFORMED;
			}
		}

		/* Logout and remove any lost initiator devices */
		list_for_each_safe(fcil, fcitemp, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if ((fcinitiator->loop_id & PORT_LOST_ID) == 0)
				continue;

			qla2x00_fabric_logout(ha, fcinitiator->loop_id & 0xff);
			ha->fabricid[fcinitiator->loop_id &0xFF].in_use = FALSE;

			list_del(&fcinitiator->list);
			kfree(fcinitiator);
		}

#if 0
		/*
		 * Wait for all remaining IO's to finish if there was logout.
		 */
		if (local_flags & LOGOUT_PERFORMED) {
			local_flags &= ~LOGOUT_PERFORMED;

			if (ha->init_done) {
				if (!(ha->dpc_flags & COMMAND_WAIT_ACTIVE)) {
					ha->dpc_flags |= COMMAND_WAIT_ACTIVE;

					qla2x00_cmd_wait(ha);

					ha->dpc_flags &= ~COMMAND_WAIT_ACTIVE;
				}
			}
		}
#endif

		/*
		 * Scan through our database and login entries already in our
		 * database.
		 */
		for (index = 0; index < MAX_FIBRE_DEVICES &&
			!atomic_read(&ha->loop_down_timer) &&
			!(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)); index++) {

			if (!(ha->fc_db[index].loop_id & PORT_LOGIN_NEEDED))
				continue;

			ha->fc_db[index].loop_id &= ~PORT_LOGIN_NEEDED;
			if (ha->fc_db[index].loop_id <= LAST_SNS_LOOP_ID) {

				/* loop_id reusable */
				dev.loop_id = ha->fc_db[index].loop_id & 0xff;
			} else {
				for (i = ha->min_external_loopid;
					i < MAX_PUBLIC_LOOP_IDS; 
					i++) {

					if (!ha->fabricid[i].in_use) {
						ha->fabricid[i].in_use = TRUE;
						dev.loop_id = i;
						break;
					}
				}

				if (i == MAX_PUBLIC_LOOP_IDS)
					break;
			}

			dev.d_id.b24 = ha->fc_db[index].d_id.b24;

			/* login and update database */
			if (qla2x00_fabric_login(ha, &dev) == 0) {
				ha->fc_db[index].loop_id = dev.loop_id;
		 	 	found_cnt++;
			}
		}

		/*
		 * Scan through new device list and login and add to our
		 * database.
		 */
		for (index = 0; index < new_dev_cnt &&
			!atomic_read(&ha->loop_down_timer) &&
			!(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)); 
			index++) {

			memcpy(&dev, &new_dev_list[index],
					sizeof(struct new_dev));

			dev.flag = DEV_PUBLIC;

			for (i = ha->min_external_loopid;
				i < MAX_PUBLIC_LOOP_IDS; 
				i++) {

				if (!ha->fabricid[i].in_use) {
					ha->fabricid[i].in_use = TRUE;
					dev.loop_id = i;
					break;
				}
			}

			if (i == MAX_PUBLIC_LOOP_IDS)
				break;

			DEBUG(printk("%s(): calling qla2100_fabric_login()\n",
					__func__);)

			if (qla2x00_fabric_login(ha, &dev) == 0) {
		 	 	found_cnt++;
				if ((rval = 
					qla2x00_update_fc_database(ha, 
							&dev,
							enable_slot_reuse)) ) {

					qla2x00_fabric_logout(ha, dev.loop_id);
					ha->fabricid[i].in_use = FALSE;
					break;
				}
			}
		}
	} while(0);

	pci_free_consistent(ha->pdev, sizeof(sns_cmd_rsp_t), sns, phys_address);

	if (rval) {
		DEBUG2(printk("%s(%ld): error exit: rval=%d\n",
				__func__,
				ha->host_no,
				rval);)
	} else {
		/* EMPTY */
		DEBUG2(if (found_cnt))
		DEBUG2(printk("scsi%ld Found (%d) ports\n",
				ha->host_no, found_cnt);) 
		DEBUG2(printk("scsi%ld: %s: exit\n", ha->host_no, __func__);)
	}

	LEAVE(__func__);

	return(rval);
}


/*
 * qla2x00_find_all_fabric_devs
 *	Go through GAN list to find all fabric devices.  Will perform
 *	necessary logout of previously existed devices that have changed
 *	and save new devices in a new device list.
 *
 * Input:
 *	ha = adapter block pointer.
 *	dev = database device entry pointer.
 *
 * Returns:
 *	0 = success.
 *	BIT_0 = error.
 *
 * Context:
 *	Kernel context.
 */
static uint8_t
qla2x00_find_all_fabric_devs(scsi_qla_host_t *ha, 
    sns_cmd_rsp_t *sns, dma_addr_t phys_addr, 
    struct new_dev *new_dev_list, uint16_t *new_dev_cnt, uint8_t *flags) 
{
	fcdev_t		first_dev, dev;
	uint8_t		rval = 0;
	uint16_t	i;
	uint16_t	index;
	uint16_t	new_cnt;
	uint16_t	public_count;
	uint16_t	initiator;
	struct list_head *fcil;
	fc_initiator_t	*fcinitiator;


	ENTER(__func__);

#if defined(ISP2100)
	ha->max_public_loop_ids = LAST_SNS_LOOP_ID - SNS_FIRST_LOOP_ID + 1;
#else
	ha->max_public_loop_ids = MAX_PUBLIC_LOOP_IDS;
#endif

	/*
	 * Loop getting devices from switch.  Issue GAN to find all devices out
	 * there.  Logout the devices that were in our database but changed
	 * port ID.
	 */
	/* Calculate the max number of public ports */
#if defined(ISP2100)
	public_count = ha->max_public_loop_ids;
#else
	public_count = ha->max_public_loop_ids - ha->min_external_loopid + 2;
#endif

	/* Set start port ID scan at adapter ID. */
	dev.d_id.b24 = 0;
	first_dev.d_id.b24 = 0;

	new_cnt = 0;	/* new device count */

	for (i = 0; 
		i < public_count && !atomic_read(&ha->loop_down_timer) &&
		!(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)); 
		i++) {

		/* Send GAN to the switch */
		rval = 0;
		if (qla2x00_gan(ha, sns, phys_addr, &dev)) {
			rval = rval | BIT_0;
			break;
		}

		/* If wrap on switch device list, exit. */
		if (dev.d_id.b24 == first_dev.d_id.b24)
			break;

		DEBUG(printk("scsi(%ld): gan found fabric(%d) - "
				"port Id=%06x\n", 
				ha->host_no, 
				i, 
				dev.d_id.b24);)

		if (first_dev.d_id.b24 == 0)
			first_dev.d_id.b24 = dev.d_id.b24;

		/* If port type not equal to N or NL port, skip it. */
		if (sns->p.gan_rsp[16] != 1 && sns->p.gan_rsp[16] != 2) {
			continue;	/* needed for McData switch */
		}

		/* Bypass if host adapter. */
		if (dev.d_id.b24 == ha->d_id.b24)
			continue;

		/* Bypass reserved domain fields. */
		if ((dev.d_id.b.domain & 0xf0) == 0xf0)
			continue;

		/* Bypass if same domain and area of adapter. */
		if ((dev.d_id.b24 & 0xffff00) == (ha->d_id.b24 & 0xffff00))
			continue;

#if defined(FC_IP_SUPPORT)
		/* Check for IP device */
		if (sns->p.gan_rsp[579] & 0x20) {
			/* Found IP device */
			DEBUG12(printk("qla%ld: IP fabric WWN: "
				"%02x%02x%02x%02x%02x%02x%02x%02x DID:%06x\n",
				ha->instance,
				dev.name[0], dev.name[1],
			       	dev.name[2], dev.name[3],
				dev.name[4], dev.name[5],
				dev.name[6], dev.name[7],
				dev.d_id.b24);)

			qla2x00_update_ip_device_data(ha, &dev);
			continue;
		}
#endif

		/* Bypass if initiator */
		initiator = FALSE;
		list_for_each(fcil, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if (memcmp(dev.wwn, fcinitiator->port_name, 8) != 0)
				continue;

			initiator = TRUE;
			DEBUG(printk("qla%ld: found host "
				"%02x%02x%02x%02x%02x%02x%02x%02x, "
				"port Id=%06x\n",
				ha->instance,
				dev.name[0], dev.name[1],
				dev.name[2], dev.name[3],
				dev.name[4], dev.name[5],
				dev.name[6], dev.name[7], 
				dev.d_id.b24);)

			/*
			 * If the initiator was marked as lost, perform the
			 * required logout and relogin the initiator by
			 * assuming a new device.
			 */
			if ((fcinitiator->loop_id & PORT_LOST_ID) == 0)
				break;

			initiator = FALSE;
			break;
		}

		/* Bypass if initiator */
		if (initiator)
			continue;

		/* Locate matching device in database. */
		for (index = 0; index < MAX_FIBRE_DEVICES; index++) {
			if (ha->fc_db[index].loop_id == PORT_UNUSED)
				continue;

			rval = 1;
			switch (ha->binding_type) {
				case BIND_BY_PORT_NAME:
					rval = memcmp(dev.wwn,
							ha->fc_db[index].wwn,
							WWN_SIZE);
					break;

				case BIND_BY_PORT_ID:
					rval = (dev.d_id.b24 !=
						 ha->fc_db[index].d_id.b24);
					break;

				case BIND_BY_NODE_NAME:
					rval = memcmp(dev.name,
							ha->fc_db[index].name,
							WWN_SIZE);
					break;
			}
			if (rval)
				continue;

			/*
			 * Update volatile unbound fields for PortID binding
			 * only
			 */
			if (ha->binding_type == BIND_BY_PORT_ID) {
				memcpy(ha->fc_db[index].name,
						dev.name, WWN_SIZE);
				memcpy(ha->fc_db[index].wwn,
						dev.wwn, WWN_SIZE);
			}

			/* Now we found a matching device name */
			DEBUG(printk("qla%ld: found fabric dev %d in tgt %d "
					"db, flags= 0x%x, loop_id="
					"0x%04x, port=%06x, name="
					"%02x%02x%02x%02x%02x%02x%02x%02x\n",
					ha->instance,
					i, index,
					ha->fc_db[index].flag,
					ha->fc_db[index].loop_id,
					ha->fc_db[index].d_id.b24,
					dev.wwn[0], dev.wwn[1],
					dev.wwn[2], dev.wwn[3],
					dev.wwn[4], dev.wwn[5],
					dev.wwn[6], dev.wwn[7]);)

			if (!(ha->fc_db[index].flag & DEV_PUBLIC)) {
				/*
				 * This was in our database as a local device.
				 * Here we assume this device either has
				 * changed location so configure_local_loop has
				 * already done necessary clean up, or it's
				 * saved here due to persistent name binding.
				 * We'll just add it in as a fabric device.
				 */
				/* Copy port id and name fields. */
				ha->fc_db[index].flag |= DEV_PUBLIC;
				ha->fc_db[index].d_id.b24 = dev.d_id.b24;
				ha->fc_db[index].loop_id |= PORT_LOGIN_NEEDED;

				break;
			}

			/* This was in our database as a fabric device. */
			if ((ha->fc_db[index].d_id.b24 == dev.d_id.b24) &&
				(ha->fc_db[index].loop_id <= LAST_SNS_LOOP_ID))
				/* Device didn't change */
				break;

			if (ha->fc_db[index].loop_id == PORT_AVAILABLE) {
				ha->fc_db[index].flag |= DEV_PUBLIC;
				ha->fc_db[index].d_id.b24 = dev.d_id.b24;
				ha->fc_db[index].loop_id |= PORT_LOGIN_NEEDED;
				break;
			}

			/*
			 * Port ID changed or device was marked to be updated;
			 * logout and mark it for relogin later.
			 */
			qla2x00_fabric_logout(ha,
					ha->fc_db[index].loop_id & 0xff);
			ha->fc_db[index].flag |= DEV_PUBLIC;
			ha->fc_db[index].d_id.b24 = dev.d_id.b24;

			ha->fc_db[index].loop_id |= PORT_LOGIN_NEEDED;
			ha->fc_db[index].loop_id &= ~PORT_LOST_ID;

			*flags |= LOGOUT_PERFORMED;

			break;
		}

		if (index == MAX_FIBRE_DEVICES) {
			/*
			 * Did not find a match in our database.  This is a new
			 * device.
			 */
			DEBUG3(printk("%s(): new device "
					"%02x%02x%02x%02x%02x%02x%02x%02x.\n",
					__func__,
					dev.wwn[0], dev.wwn[1], 
					dev.wwn[2], dev.wwn[3],
					dev.wwn[4], dev.wwn[5], 
					dev.wwn[6], dev.wwn[7]);)

			memcpy(&new_dev_list[new_cnt], &dev,
					sizeof(struct new_dev));
			new_cnt++;
		}
	}

	*new_dev_cnt = new_cnt;

	if (new_cnt >  0)
		ha->device_flags |= DFLG_FABRIC_DEVICES;

	DEBUG(printk("%s(): exit. rval=%d.\n", __func__, rval);)

	LEAVE(__func__);

	return (rval);
}

#if REG_FC4_ENABLED
/*
 * qla2x00_register_fc4
 *	Register adapter as FC4 device to the switch, so the switch won't
 *	need to login to us later which generates an RSCN event.
 *
 * Input:
 *	ha = adapter block pointer.
 *	sns = pointer to buffer for sns command.
 *	phys_addr = DMA buffer address.
 *
 * Context:
 *	Kernel context.
 */
static uint8_t
qla2x00_register_fc4(scsi_qla_host_t *ha, 
		sns_cmd_rsp_t *sns, dma_addr_t phys_addr) 
{
	uint8_t rval;
	uint16_t	wc;

	ENTER(__func__);
	
	/* Get port ID for device on SNS. */
	memset(sns, 0, sizeof(sns_cmd_rsp_t));
	wc = RFT_DATA_SIZE / 2;
	sns->p.cmd.buffer_length = cpu_to_le16(wc);
	sns->p.cmd.buffer_address[0] = cpu_to_le32(LS_64BITS(phys_addr));
	sns->p.cmd.buffer_address[1] = cpu_to_le32(MS_64BITS(phys_addr));
	sns->p.cmd.subcommand_length = __constant_cpu_to_le16(22);
	sns->p.cmd.subcommand = __constant_cpu_to_le16(0x217);
	wc = (RFT_DATA_SIZE - 16) / 4;
	sns->p.cmd.size = cpu_to_le16(wc);
	sns->p.cmd.param[0] = ha->d_id.b.al_pa;
	sns->p.cmd.param[1] = ha->d_id.b.area;
	sns->p.cmd.param[2] = ha->d_id.b.domain;

#if defined(FC_IP_SUPPORT)
	if (ha->flags.enable_ip)
		sns->p.cmd.param[4] = 0x20;	/* Set type 5 code for IP */
#endif
	sns->p.cmd.param[5] = 0x01;		/* SCSI - FCP */

	rval = BIT_0;
	if (!qla2x00_send_sns(ha, phys_addr, 30, sizeof(sns_cmd_rsp_t))) {
		if (sns->p.rft_rsp[8] == 0x80 && sns->p.rft_rsp[9] == 0x2) {
			DEBUG2(printk("%s(%ld): exiting normally.\n", 
					__func__,
					ha->host_no);)
			rval = 0;
		}
	}

	if (rval != 0) {
		/* EMPTY */
		DEBUG2_3(printk("%s(%ld): failed.\n",
				__func__,
				ha->host_no);)
	}

	LEAVE(__func__);

	return (rval);
}

/*
 * qla2x00_register_fc4_feature
 *	Register adapter as FC4 feature to the name server, so the name
 *	server won't need to login to us later which generates an RSCN 
 *	event.
 *
 * Input:
 *	ha = adapter block pointer.
 *	sns = pointer to buffer for sns command.
 *	phys_addr = DMA buffer address.
 *
 * Context:
 *	Kernel context.
 */
static uint8_t
qla2x00_register_fc4_feature(scsi_qla_host_t *ha, 
		sns_cmd_rsp_t *sns, dma_addr_t phys_addr) 
{
	uint8_t rval;
	uint16_t	wc;

	ENTER(__func__);

	/* Get port ID for device on SNS. */
	memset(sns, 0, sizeof(sns_cmd_rsp_t));
	wc = RFF_DATA_SIZE / 2;
	sns->p.cmd.buffer_length = cpu_to_le16(wc);
	sns->p.cmd.buffer_address[0] = cpu_to_le32(LS_64BITS(phys_addr));
	sns->p.cmd.buffer_address[1] = cpu_to_le32(MS_64BITS(phys_addr));
	sns->p.cmd.subcommand_length = __constant_cpu_to_le16(8);
	sns->p.cmd.subcommand = __constant_cpu_to_le16(0x21f);
	wc = (RFF_DATA_SIZE - 16) / 4;
	sns->p.cmd.size = cpu_to_le16(wc);
	sns->p.cmd.param[0] = ha->d_id.b.al_pa;
	sns->p.cmd.param[1] = ha->d_id.b.area;
	sns->p.cmd.param[2] = ha->d_id.b.domain;

	sns->p.cmd.param[6] = 0x08;		/* SCSI - FCP */
	if (!ha->flags.enable_target_mode)
		sns->p.cmd.param[7] = 0x02;	/* SCSI Initiator */

	rval = BIT_0;
	if (!qla2x00_send_sns(ha, phys_addr, 16, sizeof(sns_cmd_rsp_t))) {
		if (sns->p.rff_rsp[8] == 0x80 && sns->p.rff_rsp[9] == 0x2) {
			DEBUG2(printk("%s(%ld): exiting normally.\n", 
					__func__,
					ha->host_no);)
			rval = 0;
		}
	}

	if (rval != 0) {
		/* EMPTY */
		DEBUG2_3(printk("%s(%ld): failed.\n",
				__func__,
				ha->host_no);)
	}

	LEAVE(__func__);

	return (rval);
}

#endif

/*
 * qla2x00_gan
 *	Issue Get All Next (GAN) Simple Name Server (SNS) command.
 *
 * Input:
 *	ha = adapter block pointer.
 *	sns = pointer to buffer for sns command.
 *	dev = FC device type pointer.
 *
 * Returns:
 *	qla2100 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
static uint8_t
qla2x00_gan(scsi_qla_host_t *ha, 
		sns_cmd_rsp_t *sns, 
		dma_addr_t phys_addr, fcdev_t *dev) 
{
	uint8_t		rval;
	uint16_t	wc;

	ENTER(__func__);

	/* Get port ID for device on SNS. */
	memset(sns, 0, sizeof(sns_cmd_rsp_t));
	wc = GAN_DATA_SIZE / 2;
	sns->p.cmd.buffer_length = cpu_to_le16(wc);
	sns->p.cmd.buffer_address[0] = cpu_to_le32(LS_64BITS(phys_addr));
	sns->p.cmd.buffer_address[1] = cpu_to_le32(MS_64BITS(phys_addr));
	sns->p.cmd.subcommand_length = __constant_cpu_to_le16(6);
	sns->p.cmd.subcommand = __constant_cpu_to_le16(0x100);	/* GA_NXT */
	wc = (GAN_DATA_SIZE - 16) / 4;
	sns->p.cmd.size = cpu_to_le16(wc);
	sns->p.cmd.param[0] = dev->d_id.b.al_pa;
	sns->p.cmd.param[1] = dev->d_id.b.area;
	sns->p.cmd.param[2] = dev->d_id.b.domain;

	rval = BIT_0;
	if (!qla2x00_send_sns(ha, phys_addr, 14, sizeof(sns_cmd_rsp_t))) {
		if (sns->p.gan_rsp[8] == 0x80 && sns->p.gan_rsp[9] == 0x2) {
			dev->d_id.b.al_pa = sns->p.gan_rsp[19];
			dev->d_id.b.area = sns->p.gan_rsp[18];
			dev->d_id.b.domain = sns->p.gan_rsp[17];
			dev->flag = DEV_PUBLIC;

			/* Save FC name */
			memcpy(dev->name, &sns->p.gan_rsp[284], WWN_SIZE);

			/* Extract portname */
			memcpy(dev->wwn, &sns->p.gan_rsp[20], WWN_SIZE);

			DEBUG3(printk("qla2x00: gan entry - portname "
					"%02x%02x%02x%02x%02x%02x%02x%02x "
					"port Id=%06x\n",
					sns->p.gan_rsp[20], sns->p.gan_rsp[21],
					sns->p.gan_rsp[22], sns->p.gan_rsp[23],
					sns->p.gan_rsp[24], sns->p.gan_rsp[25],
					sns->p.gan_rsp[26], sns->p.gan_rsp[27], 
					dev->d_id.b24);)
			rval = 0;
		}
	}

#if defined(QL_DEBUG_LEVEL_2)
	if (rval != 0)
		printk("%s(): exit, rval = %d\n", __func__, rval);
#endif

	LEAVE(__func__);

	return (rval);
}

/*
 * qla2x00_fabric_login
 *	Issue fabric login command.
 *
 * Input:
 *	ha = adapter block pointer.
 *	device = pointer to FC device type structure.
 *
 * Returns:
 *      0 - Login successfully
 *      1 - Login failed
 *      2 - Initiator device
 *      3 - Fatal error
 */
static uint8_t
qla2x00_fabric_login(scsi_qla_host_t *ha, fcdev_t *device) 
{
	uint16_t	status[3];

	for (;;) {
		DEBUG(printk("scsi(%ld): Trying Fabric Login w/loop id 0x%04x "
		    "for port %06x\n",
		    ha->host_no, device->loop_id, device->d_id.b24);)

		/* Login device on switch. */
		qla2x00_login_fabric(ha,
		    device->loop_id, device->d_id.b.domain,
		    device->d_id.b.area, device->d_id.b.al_pa, 
		    &status[0], BIT_0);

		if (status[0] == 0x4007) {
			ha->fabricid[device->loop_id].in_use = FALSE;
			device->loop_id = status[1];

			DEBUG(printk("Fabric Login: port in use - next "
			    "loop id=0x%04x, port Id=%06x\n",
			    device->loop_id, device->d_id.b24);)

			if (device->loop_id <= LAST_SNS_LOOP_ID)
				ha->fabricid[device->loop_id].in_use = TRUE;
			else
				return 1;

		} else if (status[0] == 0x4000) {
			if (status[1] & 0x0001) {
				/* Initiator only device */
				qla2x00_add_initiator_device(ha, device);

				return 2;
			}

			/* This is target capable device */
			qla2x00_get_port_database(ha, device, 0);

			DEBUG(printk("scsi(%ld): Fabric Login OK. loop "
			    "id=0x%04x, port Id=%06x\n",
			    ha->host_no, device->loop_id, device->d_id.b24);)
			return 0;

		} else if (status[0] == 0x4008) {

			if (device->loop_id++ <= LAST_SNS_LOOP_ID)
				ha->fabricid[device->loop_id].in_use = TRUE;
			else
				return 1;

		} else if (status[0] == 0x4006) {
			/* No more retry needed. */
			return 3;
		} else {
			DEBUG2(printk("%s(%ld): failed=%x port_id=%06x "
			    "loop_id=%x jiffies=%lx.\n", 
			    __func__, ha->host_no, status[0], 
			    device->d_id.b24, device->loop_id, jiffies);)
			return 1;
		}
	}
}

/*
 * qla2x00_local_device_login
 *	Issue local device login command.
 *
 * Input:
 *	ha = adapter block pointer.
 *	loop_id = loop id of device to login to.
 *
 * Returns (Where's the #define!!!!):
 *      0 - Login successfully
 *      1 - Login failed
 *      3 - Fatal error
 */
static uint8_t
qla2x00_local_device_login(scsi_qla_host_t *ha, uint16_t loop_id)
{
	int		rval;
	uint16_t	mb[MAILBOX_REGISTER_COUNT];

	memset(mb, 0, sizeof(mb));
	rval = qla2x00_login_local_device(ha, loop_id, mb, BIT_0);
	if (rval == QL_STATUS_SUCCESS) {
		/* Interrogate mailbox registers for any errors */
		if (mb[0] == 0x4005)
			rval = 1;
		else if (mb[0] == 0x4006)
			/* device not in PCB table */
			rval = 3;
	}
	return rval;
}

/*
 * qla2x00_configure_loop
 *      Updates Fibre Channel Device Database with what is actually on loop.
 *
 * Input:
 *      ha                = adapter block pointer.
 *
 * Output:
 *      ha->fc_db = updated
 *
 * Returns:
 *      0 = success.
 *      1 = error.
 *      2 = database was full and device was not configured.
 */
STATIC uint8_t
qla2x00_configure_loop(scsi_qla_host_t *ha) 
{
	uint8_t  rval = 0;
	uint8_t  rval1 = 0;
	uint8_t  enable_slot_reuse = FALSE;
	uint16_t  cnt;
	static unsigned long  flags, save_flags;
#if defined(FC_IP_SUPPORT)
	struct ip_device	*ipdev;
#endif

	DEBUG3(printk("%s(%ld): entered\n", __func__, ha->host_no);)
	DEBUG(printk("scsi%ld: Enter %s():\n", ha->host_no, __func__);)

	/* Get Initiator ID */
	if (qla2x00_configure_hba(ha)) {
		DEBUG(printk("scsi%ld: qla2x00_configure_loop: "
				"configure hba failed.\n",
				ha->host_no);)
		return(1);
	}

#if defined(FC_IP_SUPPORT)
	/* Disable all IP devices in linked list */
	for (ipdev = ha->ipdev_db_top; ipdev; ipdev = ipdev->next)
		ipdev->flags &= ~IP_DEV_FLAG_PRESENT;
#endif /* FC_IP_SUPPORT */

	save_flags = flags = ha->dpc_flags;
	DEBUG(printk("%s(): dpc flags =0x%lx\n", __func__, flags);)

	/* dg 02/26/02 ha->dpc_flags &= ~(LOCAL_LOOP_UPDATE | RSCN_UPDATE); */

	/*
	 * If we have both an RSCN and PORT UPDATE pending then handle them
	 * both at the same time.
	 */
	clear_bit(LOCAL_LOOP_UPDATE, &ha->dpc_flags);
	clear_bit(RSCN_UPDATE, &ha->dpc_flags);
	ha->mem_err = 0 ;

	/* Determine what we need to do */
	if (ha->current_topology == ISP_CFG_FL &&
		(test_bit(LOCAL_LOOP_UPDATE, &flags))) {

		ha->flags.rscn_queue_overflow = TRUE;
		set_bit(RSCN_UPDATE, &flags);

	} else if (ha->current_topology == ISP_CFG_F &&
		(test_bit(LOCAL_LOOP_UPDATE, &flags))) {

		ha->flags.rscn_queue_overflow = TRUE;
		set_bit(RSCN_UPDATE, &flags);
		clear_bit(LOCAL_LOOP_UPDATE, &flags);

	} else if (!ha->flags.online ||
		(test_bit(ABORT_ISP_ACTIVE, &flags))) {

		ha->flags.rscn_queue_overflow = TRUE;
		set_bit(RSCN_UPDATE, &flags);
		set_bit(LOCAL_LOOP_UPDATE, &flags);
	}

	do {
		if (test_bit(LOCAL_LOOP_UPDATE, &flags)) {
			rval = rval | 
				qla2x00_configure_local_loop(ha,
					enable_slot_reuse);
		}

		if (test_bit(RSCN_UPDATE, &flags)) {
			rval1 = qla2x00_configure_fabric(ha, enable_slot_reuse);
			if ((rval1 & BIT_0) && ha->sns_retry_cnt < 8) {
				ha->sns_retry_cnt++;
				set_bit(LOGIN_RETRY_NEEDED, &ha->dpc_flags);
			}
		}

		/* If devices not configured first time try reusing slots.*/
		if (enable_slot_reuse == FALSE && (rval & BIT_1))
			enable_slot_reuse = TRUE;
		else
			enable_slot_reuse = FALSE;

		/* Isolate error status. */
		if (rval & BIT_0) {
			rval = 1;
		} else {
			rval = 0;
		}

	} while (enable_slot_reuse == TRUE && rval == 0);

	if (!atomic_read(&ha->loop_down_timer) && 
		!(test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags))) {

		/* Mark devices that are not present as DEV_ABSENCE */
		for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
			if (ha->fc_db[cnt].loop_id & PORT_LOST_ID) {
				ha->fc_db[cnt].flag |= DEV_ABSENCE;
			} else {
				/* device returned */
				if (ha->fc_db[cnt].loop_id <=
						LAST_SNS_LOOP_ID && 
					ha->fc_db[cnt].flag & DEV_ABSENCE) {

					ha->fc_db[cnt].flag &= ~DEV_ABSENCE;
					ha->fc_db[cnt].flag |= DEV_RETURN;
					ha->fc_db[cnt].port_login_retry_count =
						ha->port_down_retry_count *
						 PORT_RETRY_TIME;
				}
			}
		}

		rval1 = qla2x00_build_fcport_list(ha);
		if (((rval1 & BIT_0) || 
			ha->mem_err != 0) && 
			ha->sns_retry_cnt < 8 ) {

			ha->sns_retry_cnt++;
			set_bit(LOGIN_RETRY_NEEDED, &ha->dpc_flags);
		}

		if(!ha->flags.failover_enabled)
			qla2x00_config_os(ha);

		/* If we found all devices then go ready */
		if (!(test_bit(LOGIN_RETRY_NEEDED, &ha->dpc_flags))) {
			ha->loop_state = LOOP_READY;

			if (ha->flags.failover_enabled) {
				DEBUG(printk("%s(%ld): schedule "
						"FAILBACK EVENT\n", 
						__func__,
						ha->host_no);)
				if (!(test_and_set_bit(FAILOVER_EVENT_NEEDED,
							&ha->dpc_flags))) {
					ha->failback_delay = failbackTime;
				}
				set_bit(COMMAND_WAIT_NEEDED, &ha->dpc_flags);
				ha->failover_type = MP_NOTIFY_LOOP_UP;
			}

			DEBUG(printk("%s(%ld): LOOP READY\n", 
					__func__,
					ha->host_no);)
		} else {
			if (test_bit(LOCAL_LOOP_UPDATE, &save_flags))
				set_bit(LOCAL_LOOP_UPDATE, &ha->dpc_flags);
			if (test_bit(RSCN_UPDATE, &save_flags))
				set_bit(RSCN_UPDATE, &ha->dpc_flags);
		}
	} else {
		DEBUG(printk("%s(%ld): Loop down counter running= %d or "
				"Resync needed- dpc flags= %ld\n",
				__func__,
				ha->host_no,
				atomic_read(&ha->loop_down_timer), 
				ha->dpc_flags);)
			/* ???? dg 02/26/02  rval = 1; */
	}

	if (rval) {
		DEBUG2_3(printk("%s(%ld): *** FAILED ***\n",
				__func__,
				ha->host_no);)
	} else {
		DEBUG3(printk("%s: exiting normally\n", __func__);)
	}

	return(rval);
}


/*
 * qla2x00_config_os
 *	Setup OS target and LUN structures.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Context:
 *	Kernel context.
 */
static void
qla2x00_config_os(scsi_qla_host_t *ha) 
{
	fc_port_t	*fcport;
	fc_lun_t	*fclun;
	os_lun_t	*lq;
	uint16_t	t, l;


	DEBUG3(printk("%s(%ld): entered.\n", __func__, ha->host_no);)

	for (fcport = ha->fcport; fcport != NULL; fcport = fcport->next) {
		/* Allocate target */
#if 0
		if (fcport->loop_id == FC_NO_LOOP_ID)
			continue;
#endif

		/* Bind fcport to target number. */
		DEBUG5(printk("%s(%ld): fcport bind= %p\n",
				__func__,
				ha->host_no,fcport);)

		if ((t = qla2x00_fcport_bind(ha, fcport)) == MAX_TARGETS)
			continue;

#if VSA
		if( (ha->fc_db[t].flag & DEV_FLAG_VSA) )
			fcport->flags |= FC_VSA;
#endif
		DEBUG5(printk("%s(%ld): going to alloc lun for tgt %d. mask="
				"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
				"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
				"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
				".\n",
				__func__,
				ha->host_no, 
				t, 
				fcport->lun_mask.mask[0],
				fcport->lun_mask.mask[1],
				fcport->lun_mask.mask[2],
				fcport->lun_mask.mask[3],
				fcport->lun_mask.mask[4],
				fcport->lun_mask.mask[5],
				fcport->lun_mask.mask[6],
				fcport->lun_mask.mask[7],
				fcport->lun_mask.mask[8],
				fcport->lun_mask.mask[9],
				fcport->lun_mask.mask[10],
				fcport->lun_mask.mask[11],
				fcport->lun_mask.mask[12],
				fcport->lun_mask.mask[13],
				fcport->lun_mask.mask[14],
				fcport->lun_mask.mask[15],
				fcport->lun_mask.mask[16],
				fcport->lun_mask.mask[17],
				fcport->lun_mask.mask[18],
				fcport->lun_mask.mask[19],
				fcport->lun_mask.mask[20],
				fcport->lun_mask.mask[21],
				fcport->lun_mask.mask[22],
				fcport->lun_mask.mask[23],
				fcport->lun_mask.mask[24],
				fcport->lun_mask.mask[25],
				fcport->lun_mask.mask[26],
				fcport->lun_mask.mask[27],
				fcport->lun_mask.mask[28],
				fcport->lun_mask.mask[29],
				fcport->lun_mask.mask[30],
				fcport->lun_mask.mask[31]);)

		/* Allocate LUNs */
		for (fclun = fcport->fclun;
			fclun != NULL; fclun = fclun->next) {

			l = fclun->lun;		/* Must not exceed MAX_LUN */

			/*
			 * Always alloc LUN 0 so kernel will scan past LUN 0.
			 */
			if (l != 0 &&
				(EXT_IS_LUN_BIT_SET(&(fcport->lun_mask), l))) {

				/* mask this LUN */
				continue;
			}

			if ((lq = qla2x00_lun_alloc(ha, t, l)) == NULL)
				continue;

			lq->fclun = fclun;
		}
	}

	DEBUG3(printk("%s(%ld): exiting normally.\n", __func__, ha->host_no);)
}

/*
 * qla2x00_fcport_bind
 *	Locates a target number for FC port.
 *
 * Input:
 *	ha = adapter state pointer.
 *	fcport = FC port structure pointer.
 *
 * Returns:
 *	target number
 *
 * Context:
 *	Kernel context.
 */
static uint16_t
qla2x00_fcport_bind(scsi_qla_host_t *ha, fc_port_t *fcport) 
{
	int		rval;
	uint16_t	t;
	os_tgt_t	*tq;

	ENTER(__func__);

	/* Check for tgt already allocated for persistent binding. */
	for (t = 0; t < MAX_TARGETS; t++) {
		if ((tq = TGT_Q(ha, t)) == NULL)
			continue;

		rval = 0;
		switch (ha->binding_type) {
			case BIND_BY_PORT_NAME:
				rval = memcmp(fcport->port_name,
						tq->port_name, WWN_SIZE);
				break;

			case BIND_BY_PORT_ID:
				rval = (fcport->d_id.b24 != tq->d_id.b24);
				break;

			case BIND_BY_NODE_NAME:
				rval = memcmp(fcport->node_name,
						tq->node_name, WWN_SIZE);
				break;
		}
		/* Found a persistently bound match */
		if (rval == 0)
			break;
	}

	if (fcport->loop_id == FC_NO_LOOP_ID) {
		DEBUG(tq = TGT_Q(ha, t);)
		DEBUG(printk("scsi%ld: Missing target ID %02x @ %p to "
				"loop id: %04x, port state=0x%x, "
				"port down retry=%d\n",
				ha->host_no,
				t,
				tq,
				fcport->loop_id,
				atomic_read(&fcport->state),
				atomic_read(&fcport->port_down_timer));)
		return (MAX_TARGETS);
	}

	if (t != MAX_TARGETS) {
		tq = TGT_Q(ha, t);
		tq->vis_port = fcport;

		DEBUG(printk("scsi%ld: Assigning target ID %02x @ %p to "
				"loop id: %04x, port state=0x%x, "
				"port down retry=%d\n",
				ha->host_no,
				t,
				tq,
				fcport->loop_id,
				atomic_read(&fcport->state),
				atomic_read(&fcport->port_down_timer));)
		return (t);
	}

	/* Check for persistent binding not yet configured. */
	for (t = 0; t < MAX_TARGETS; t++) {
		rval = 0;
		switch (ha->binding_type) {
			case BIND_BY_PORT_NAME:
				rval = memcmp(fcport->port_name,
						ha->fc_db[t].wwn, WWN_SIZE);
				break;

			case BIND_BY_PORT_ID:
				rval = (fcport->d_id.b24 !=
						ha->fc_db[t].d_id.b24);
				break;

			case BIND_BY_NODE_NAME:
				rval = memcmp(fcport->node_name,
						ha->fc_db[t].name, WWN_SIZE);
				break;
		}
		/* Found not-yet-allocated target at t */
		if (rval == 0)
			break;
	}

	if (t == MAX_TARGETS) {
		/* Check if slot at loop ID is available. */
		t = fcport->loop_id;
		if (TGT_Q(ha, t) != NULL) {
			/* Locate first free target id in db for device. */
			for (t = 0; t < MAX_TARGETS; t++) {
				if (TGT_Q(ha, t) == NULL)
					break;
			}
		}
	}

	if (t != MAX_TARGETS) {
	       	tq = qla2x00_tgt_alloc(ha, t);
		if (tq != NULL) {
			memcpy(tq->port_name, fcport->port_name, WWN_SIZE);
			tq->d_id.b24 = fcport->d_id.b24;
			memcpy(tq->node_name, fcport->node_name, WWN_SIZE);
		}
		else
			t = MAX_TARGETS;
	}

	if (t == MAX_TARGETS) {
		DEBUG2(printk("%s(): **** FAILED ****", __func__);)
		printk(KERN_WARNING
			"%s(): **** FAILED ****", __func__);
	} else {
		if (!ha->flags.failover_enabled) {
			/* fcport IS the visible port in non-failover mode */
			tq = TGT_Q(ha, t);
			tq->vis_port = fcport;
		}

		DEBUG(tq = TGT_Q(ha, t);)
		DEBUG(printk("scsi%ld: Assigning target ID %02x @ %p to "
				"loop id: %04x, port state=0x%x, "
				"port down retry=%d\n",
				ha->host_no,
				t,
				tq,
				fcport->loop_id,
				atomic_read(&fcport->state),
				atomic_read(&fcport->port_down_timer));)
	}

	LEAVE(__func__);

	return (t);
}

/*
 * qla2x00_build_fcport_list
 *	Updates device on list.
 *
 * Input:
 *	ha = adapter block pointer.
 *	fcport = port structure pointer.
 *
 * Return:
 *	0  - Success
 *  BIT_0 - error
 *
 * Context:
 *	Kernel context.
 */
static int 
qla2x00_build_fcport_list(scsi_qla_host_t *ha) 
{
	int	rval;
	fcdev_t		*dev;
	int	found = 0;
	int cnt, i;
	fc_port_t	*fcport;
	fc_port_t	*prev_fcport;

	ENTER(__func__);

	for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
		dev = &ha->fc_db[cnt];

		/* Skip if zero port name */
		if (qla2x00_is_wwn_zero(dev->wwn)) {
			continue;
		}

		DEBUG3(printk("%s(%ld): found tgt %d in fc_db.\n",
				__func__, ha->host_no, cnt);)

		/* Check for matching device in port list. */
		found = 0;
		prev_fcport = NULL;
		for (i=0, fcport = ha->fcport; 
			fcport != NULL;
			fcport = fcport->next, i++) {

			rval = 1;
			switch (ha->binding_type) {
				case BIND_BY_PORT_NAME:
					rval = memcmp(dev->wwn,
							fcport->port_name,
							WWN_SIZE);
					break;

				case BIND_BY_PORT_ID:
					rval = (dev->d_id.b24 !=
						 fcport->d_id.b24);
					break;

				case BIND_BY_NODE_NAME:
					rval = memcmp(dev->name,
							fcport->node_name,
							WWN_SIZE);
					break;
			}
			if (rval) {
				prev_fcport = fcport;
				continue;
			}

			/*
			 * Update volatile unbound fields for PortID binding
			 * only
			 */
			if (ha->binding_type == BIND_BY_PORT_ID) {
				memcpy(fcport->node_name, dev->name, WWN_SIZE);
				memcpy(fcport->port_name, dev->wwn, WWN_SIZE);
			}

			DEBUG(printk("%s(): Found matching port %06x, "
					"device flags= 0x%x\n",
					__func__,
					dev->d_id.b24, 
					dev->flag);)

			/* if device found is missing then mark it */
			if (dev->flag & DEV_ABSENCE) {
				DEBUG(printk("%s(): Port missing ---  "
						"(port_name) -> "
						"%02x%02x%02x%02x%02x"
						"%02x%02x%02x, "
						"loop id = 0x%04x\n",
						__func__,
						fcport->port_name[0],
						fcport->port_name[1],
						fcport->port_name[2],
						fcport->port_name[3],
						fcport->port_name[4],
						fcport->port_name[5],
						fcport->port_name[6],
						fcport->port_name[7],
						fcport->loop_id);)

				fcport->loop_id = FC_NO_LOOP_ID;

				qla2x00_mark_device_lost(ha, fcport);

				found++;
				break;
			}

			/* if device was missing but returned */
			if (fcport->loop_id == FC_NO_LOOP_ID ||
				!(dev->flag & DEV_PUBLIC) ||
				atomic_read(&fcport->state) != FC_ONLINE) {

				DEBUG(printk("%s(): Port returned +++  "
						"(port_name) -> "
						"%02x%02x%02x%02x%02x"
						"%02x%02x%02x, "
						"loop id = 0x%04x\n",
						__func__,
						fcport->port_name[0],
						fcport->port_name[1],
						fcport->port_name[2],
						fcport->port_name[3],
						fcport->port_name[4],
						fcport->port_name[5],
						fcport->port_name[6],
						fcport->port_name[7],
						fcport->loop_id);)

				fcport->loop_id = dev->loop_id;
				fcport->old_loop_id = dev->loop_id;
				fcport->d_id.b24 = dev->d_id.b24;

				break;
			}

			DEBUG(printk("%s(): Match - fcport[%d] = fc_db[%d] "
					"(ignored) -> "
					"%02x%02x%02x%02x%02x%02x%02x%02x, "
					"loop id = 0x%04x\n",
					__func__,
					i, 
					cnt,
					fcport->port_name[0],
					fcport->port_name[1],
					fcport->port_name[2],
					fcport->port_name[3],
					fcport->port_name[4],
					fcport->port_name[5],
					fcport->port_name[6],
					fcport->port_name[7],
					fcport->loop_id);)
			found++;
			break;
		}
		if (found)
			continue;

		/* Add device to port list. */
		if (fcport == NULL) {
			DEBUG3(printk("%s(%ld): adding new device to list.\n",
					__func__,
					ha->host_no);)

			fcport = kmalloc(sizeof(fc_port_t), GFP_ATOMIC);
			if (fcport == NULL)
				break;

			memset(fcport, 0, sizeof(fc_port_t));

			/* copy fields into fcport */
			memcpy(fcport->port_name, dev->wwn, WWN_SIZE);
			memcpy(fcport->node_name, dev->name, WWN_SIZE);

			fcport->dev_id = cnt;

			if (dev->flag & DEV_ABSENCE) {
				DEBUG(printk("%s(): Port missing --- "
						"(port_name) -> "
						"%02x%02x%02x%02x"
						"%02x%02x%02x%02x, "
						"loop id = 0x%04x\n",
						__func__,
						fcport->port_name[0],
						fcport->port_name[1],
						fcport->port_name[2],
						fcport->port_name[3],
						fcport->port_name[4],
						fcport->port_name[5],
						fcport->port_name[6],
						fcport->port_name[7],
						fcport->loop_id);)

				fcport->loop_id = FC_NO_LOOP_ID;

				qla2x00_mark_device_lost(ha, fcport);
			} else {
				fcport->loop_id = dev->loop_id;
				fcport->old_loop_id = dev->loop_id;
			}

			fcport->d_id.b24 = dev->d_id.b24;

			DEBUG(printk("%s(): New Device +++ (port_name) -> "
					"%02x%02x%02x%02x%02x%02x%02x%02x, "
					"loop id = 0x%04x\n",
					__func__,
					fcport->port_name[0],
					fcport->port_name[1],
					fcport->port_name[2],
					fcport->port_name[3],
					fcport->port_name[4],
					fcport->port_name[5],
					fcport->port_name[6],
					fcport->port_name[7],
					fcport->loop_id);)

			/* flags */
			if (dev->flag & DEV_PUBLIC)
				fcport->flags |= FC_FABRIC_DEVICE;

			if (dev->flag & DEV_INITIATOR)
				fcport->flags |= FC_INITIATOR_DEVICE;

			/* Assume the device supports RLC */
			fcport->flags |= FC_SUPPORT_RPT_LUNS;

			if (!ha->flags.failover_enabled)
				qla2x00_get_lun_mask_from_config(ha, 
				    fcport, cnt, 0);

			if (prev_fcport == NULL) {
				/* nothing in fcport list yet */
				ha->fcport = fcport;
			} else {
				/*
				 * prev_fcport should be pointing to last
				 * port in list
				 */
				prev_fcport->next = fcport;
			}

		} else {
			DEBUG3(printk("%s(%ld): updating device to list.\n", 
			    __func__, ha->host_no);)
			fcport->loop_id = dev->loop_id;
			fcport->loop_id = dev->loop_id;
			fcport->old_loop_id = dev->loop_id;
		}

		if (atomic_read(&fcport->state) != FC_ONLINE) {
			if (qla2x00_update_fcport(ha, fcport, cnt)) {
				DEBUG2(printk("%s(%ld): update_fcport "
				    "failed.\n",
				    __func__, ha->host_no);)

				return BIT_0;
			}
		}

	}

	LEAVE(__func__);

	return (0);
}

/*
 * qla2x00_mark_device_lost
 *	Updates fcport state when device goes offline.
 *
 * Input:
 *	ha = adapter block pointer.
 *	fcport = port structure pointer.
 *
 * Return:
 *	None.
 *
 * Context:
 */
STATIC void
qla2x00_mark_device_lost( scsi_qla_host_t *ha, fc_port_t *fcport ) 
{
#if 0
	/*
	 * No point in marking the device as lost, if the device is already
	 * DEAD.
	 */
	if (atomic_read(&fcport->state) == FC_DEVICE_DEAD)
		return;

	/* Mark the device LOST */
	atomic_set(&fcport->state, FC_DEVICE_LOST);
#else
	/* 
	 * We may need to retry the login, so don't change the
	 * state of the port but do the retries.
	 */
	if (atomic_read(&fcport->state) != FC_DEVICE_DEAD)
		atomic_set(&fcport->state, FC_DEVICE_LOST);
#endif

#if defined(PORT_LOGIN_4xWAY)
	if (PORT_LOGIN_RETRY(fcport) > 0) {
		PORT_LOGIN_RETRY(fcport)--;
		DEBUG(printk("scsi%ld: Port login retry: "
				"%02x%02x%02x%02x%02x%02x%02x%02x, "
				"id = 0x%04x retry cnt=%d\n",
				ha->host_no,
				fcport->port_name[0],
				fcport->port_name[1],
				fcport->port_name[2],
				fcport->port_name[3],
				fcport->port_name[4],
				fcport->port_name[5],
				fcport->port_name[6],
				fcport->port_name[7],
				fcport->loop_id,
				PORT_LOGIN_RETRY(fcport));)
			
		set_bit(LOGIN_RETRY_NEEDED, &ha->dpc_flags);
	}
#else
	if (fcport->login_retry == 0) {
		fcport->login_retry = ha->login_retry_count;

		DEBUG(printk("scsi%ld: Port login retry: "
				"%02x%02x%02x%02x%02x%02x%02x%02x, "
				"id = 0x%04x retry cnt=%d\n",
				ha->host_no,
				fcport->port_name[0],
				fcport->port_name[1],
				fcport->port_name[2],
				fcport->port_name[3],
				fcport->port_name[4],
				fcport->port_name[5],
				fcport->port_name[6],
				fcport->port_name[7],
				fcport->loop_id,
				fcport->login_retry ); )
		set_bit(RELOGIN_NEEDED, &ha->dpc_flags);
	}
#endif
}

/*
 * qla2x00_mark_all_devices_lost
 *	Updates fcport state when device goes offline.
 *
 * Input:
 *	ha = adapter block pointer.
 *	fcport = port structure pointer.
 *
 * Return:
 *	None.
 *
 * Context:
 */
STATIC void
qla2x00_mark_all_devices_lost(scsi_qla_host_t *ha) 
{
	fc_port_t *fcport;

	for (fcport = ha->fcport; fcport != NULL; fcport = fcport->next) {
		/*
		 * No point in marking the device as lost, if the device is
		 * already DEAD.
		 */
		if (atomic_read(&fcport->state) == FC_DEVICE_DEAD)
			continue;

		atomic_set(&fcport->state, FC_DEVICE_LOST);
	}
}

/*
 * qla2x00_check_for_devices_online
 *
 *	Check fcport state of all devices to make sure online.
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Return:
 *	None.
 *
 * Context:
 */
STATIC uint8_t
qla2x00_check_for_devices_online(scsi_qla_host_t *ha) 
{
	fc_port_t	*fcport;
	int		found, cnt;

	found = 0;
	for (cnt = 0, fcport = ha->fcport; 
		fcport != NULL;
		fcport = fcport->next, cnt++) {

		if ((atomic_read(&fcport->state) == FC_ONLINE) ||
			(atomic_read(&fcport->state) == FC_DEVICE_DEAD))
			found++;
	}
	if (cnt == found) {
		DEBUG5(printk("%s(%ld): all online\n",
				__func__,
				ha->host_no);)
		return 1;
	} else
		return 0;
}

/*
 * qla2x00_update_fcport
 *	Updates device on list.
 *
 * Input:
 *	ha = adapter block pointer.
 *	fcport = port structure pointer.
 *
 * Return:
 *	0  - Success
 *  BIT_0 - error
 *
 * Context:
 *	Kernel context.
 */
static int
qla2x00_update_fcport(scsi_qla_host_t *ha, fc_port_t *fcport, int index) 
{
	DEBUG4(printk("%s(): entered, loop_id = %d\n",
			__func__,
			fcport->loop_id);)

	fcport->port_login_retry_count =
		ha->port_down_retry_count * PORT_RETRY_TIME;
	atomic_set(&fcport->state, FC_ONLINE);
	fcport->login_retry = 0;
	fcport->ha = ha;
	atomic_set(&fcport->port_down_timer,
			ha->port_down_retry_count * PORT_RETRY_TIME);

	/* Do LUN discovery. */
	return (qla2x00_lun_discovery(ha, fcport, index));
}

/*
 * qla2x00_lun_discovery
 *	Issue SCSI inquiry command for LUN discovery.
 *
 * Input:
 *	ha = adapter block pointer.
 *	fcport = FC port structure pointer.
 *
 * Return:
 *	0  - Success
 *  BIT_0 - error
 *
 * Context:
 *	Kernel context.
 */
static int
qla2x00_lun_discovery(scsi_qla_host_t *ha, fc_port_t *fcport, int index) 
{
	inq_cmd_rsp_t	*pkt;
	int		rval;
	uint16_t	lun;
	fc_lun_t	*fclun;
	dma_addr_t	phys_address = 0;
	int		disconnected;
	int		retry;
	fcdev_t		dev;
	int		rlc_succeeded;
	uint16_t	comp_status;
	uint16_t	scsi_status;

	ENTER(__func__);

	/* 
	 * Immediately issue a RLC to the fcport
	 */
	rlc_succeeded = 0;
	if (qla2x00_rpt_lun_discovery(ha, fcport) == QLA2X00_SUCCESS) {
		/* 
		 * We always need at least LUN 0 to be present in our fclun
		 * list if RLC succeeds.
		 */
		qla2x00_cfg_lun(fcport, 0);
		/* 
		 * At least do an inquiry on LUN 0 to determine peripheral
		 * qualifier type.
		 */
		rlc_succeeded = 1;
	}

	/*
	 * RLC failed for some reason, try basic inquiries
	 */
	pkt = pci_alloc_consistent(ha->pdev,
				sizeof(inq_cmd_rsp_t), &phys_address);

	if (pkt == NULL) {
		printk(KERN_WARNING
			"scsi(%ld): Memory Allocation failed - INQ\n",
			ha->host_no);
		ha->mem_err++;
		return BIT_0;
	}

	for (lun = 0; lun < ha->max_probe_luns; lun++) {
		retry = 2;
		do {
			// FIXME: dma_addr_t could be 64bits in length!
			memset(pkt, 0, sizeof(inq_cmd_rsp_t));
			pkt->p.cmd.entry_type = COMMAND_TYPE;
			pkt->p.cmd.entry_count = 1;
			pkt->p.cmd.lun = cpu_to_le16(lun);
			pkt->p.cmd.target = (uint8_t)fcport->loop_id;
			pkt->p.cmd.control_flags =
				__constant_cpu_to_le16(CF_READ | CF_SIMPLE_TAG);
			pkt->p.cmd.scsi_cdb[0] = INQ_SCSI_OPCODE;
			pkt->p.cmd.scsi_cdb[4] = INQ_DATA_SIZE;
			pkt->p.cmd.dseg_count = __constant_cpu_to_le16(1);
			pkt->p.cmd.timeout = __constant_cpu_to_le16(10);
			pkt->p.cmd.byte_count =
				__constant_cpu_to_le32(INQ_DATA_SIZE);
			pkt->p.cmd.dseg_0_address = cpu_to_le32(
				phys_address + sizeof(sts_entry_t));
			pkt->p.cmd.dseg_0_length =
				__constant_cpu_to_le32(INQ_DATA_SIZE);

			DEBUG5(printk("lun_discovery: Lun Inquiry - fcport=%p,"
					" lun (%d)\n", 
					fcport, 
					lun);)

			rval = qla2x00_issue_iocb(ha, pkt,
					phys_address, sizeof(inq_cmd_rsp_t));

			comp_status = le16_to_cpu(pkt->p.rsp.comp_status);
			scsi_status = le16_to_cpu(pkt->p.rsp.scsi_status);

			DEBUG5(printk("lun_discovery: lun (%d) inquiry - "
					"inq[0]= 0x%x, comp status 0x%x, "
					"scsi status 0x%x, rval=%d\n",
					lun, pkt->inq[0], 
					comp_status,
					scsi_status, 
					rval);)

			/* if port not logged in then try and login */
			if (lun == 0 && comp_status == CS_PORT_LOGGED_OUT) {
				memset(&dev, 0, sizeof (dev));
				dev.d_id.b24 = ha->fc_db[index].d_id.b24;

				/* login and update database */
				if (qla2x00_fabric_login(ha, &dev) == 0)
					ha->fc_db[index].loop_id = dev.loop_id;
			}
		} while ((rval != QLA2X00_SUCCESS ||
				comp_status != CS_COMPLETE) && 
				retry--);

		if (rval != QLA2X00_SUCCESS ||
			comp_status != CS_COMPLETE ||
			(scsi_status & SS_CHECK_CONDITION)) {

			DEBUG(printk("lun_discovery: Failed lun inquiry - "
					"inq[0]= 0x%x, comp status 0x%x, "
					"scsi status 0x%x. loop_id=%d\n",
					pkt->inq[0], 
					comp_status,
					scsi_status, 
					fcport->loop_id);)

			break;
		}

		disconnected = 0;

		/*
		 * We only need to issue an inquiry on LUN 0 to determine the
		 * port's peripheral qualifier type
		 */
		if (rlc_succeeded == 1) {
			if (pkt->inq[0] == 0 || pkt->inq[0] == 0xc) {
				fcport->flags &= ~(FC_TAPE_DEVICE);
				ha->fc_db[index].flag &= ~DEV_TAPE_DEVICE;
			} else if (pkt->inq[0] == 1 || pkt->inq[0] == 8) {
				fcport->flags |= FC_TAPE_DEVICE;
				ha->fc_db[index].flag |= DEV_TAPE_DEVICE;
			}

			/* Stop the scan */
			break;
		}

		/* inq[0] ==:
		 *	 0x0- Hard Disk.
		 *	 0xc- is a processor device.	
		 *	 0x1- is a Tape Device.
		 *       0x8- is a medium changer device
		 * 	      which is basically a Tape device.
		 */
		if (pkt->inq[0] == 0 || pkt->inq[0] == 0xc) {
			fcport->flags &= ~(FC_TAPE_DEVICE);
			ha->fc_db[index].flag &= ~DEV_TAPE_DEVICE;
		} else if (pkt->inq[0] == 1 || pkt->inq[0] == 8) {
			fcport->flags |= FC_TAPE_DEVICE;
			ha->fc_db[index].flag |= DEV_TAPE_DEVICE;
		} else if (pkt->inq[0] == 0x20 || pkt->inq[0] == 0x7f) {
			disconnected++;
		} else {
			continue;
		}
		
		/* Allocate LUN if not already allocated. */
		for (fclun = fcport->fclun; 
			fclun != NULL; 
			fclun = fclun->next) {

			if (fclun->lun == lun)
				break;
		}

		if (fclun != NULL) {
			/* Found this lun already in our list */
			continue;
		}

		/* Add this lun to our list */
		fcport->lun_cnt++;

		fclun = kmalloc(sizeof(fc_lun_t), GFP_ATOMIC);
		if (fclun != NULL) {
			/* Setup LUN structure. */
			memset(fclun, 0, sizeof(fc_lun_t));

			DEBUG5(printk("lun_discovery: Allocated fclun %p, "
					"disconnected=%d\n", 
					fclun,
					disconnected);)

			fclun->fcport = fcport;
			fclun->lun = lun;

			if (disconnected)
				fclun->flags |= FC_DISCON_LUN;

			fclun->next = fcport->fclun;
			fcport->fclun = fclun;

	 	 	DEBUG5(printk("lun_discvery: Allocated fclun %p, "
					"fclun.lun=%d\n", 
					fclun, fclun->lun););
		} else {
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - FCLUN\n",
				ha->host_no);
			ha->mem_err++;
			pci_free_consistent(ha->pdev,
						 sizeof(inq_cmd_rsp_t),
						 pkt,
						 phys_address);
			return BIT_0;
		}

	}

	DEBUG(printk("lun_discovery(%ld): fcport lun count=%d, fcport= %p\n", 
			ha->host_no,
			fcport->lun_cnt, 
			fcport);)

	pci_free_consistent(ha->pdev, sizeof(inq_cmd_rsp_t), pkt, phys_address);

	LEAVE(__func__);

	return 0;
}

/*
 * qla2x00_rpt_lun_discovery
 *	Issue SCSI report LUN command for LUN discovery.
 *
 * Input:
 *	ha:		adapter state pointer.
 *	fcport:		FC port structure pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
static int
qla2x00_rpt_lun_discovery(scsi_qla_host_t *ha, fc_port_t *fcport) 
{
	rpt_lun_cmd_rsp_t	*pkt;
	dma_addr_t		phys_address = 0;
	int			rval;
	uint32_t		len, cnt;
	uint8_t			retries;
	uint16_t		lun;
	uint16_t		comp_status;
	uint16_t		scsi_status;

	ENTER(__func__);

	/* Assume a failed status */
	rval = QLA2X00_FAILED;

	/* No point in continuing if the device doesn't support RLC */
	if (!(fcport->flags & FC_SUPPORT_RPT_LUNS))
		return (rval);

	pkt = pci_alloc_consistent(ha->pdev,
			sizeof(rpt_lun_cmd_rsp_t),
			&phys_address);
	if (pkt == NULL) {
		printk(KERN_WARNING
			"scsi(%ld): Memory Allocation failed - RLC",
			ha->host_no);
		ha->mem_err++;
		return BIT_0;
	}

	for (retries = 4; retries; retries--) {
		// FIXME: dma_addr_t could be 64bits in length!
		memset(pkt, 0, sizeof(rpt_lun_cmd_rsp_t));
		pkt->p.cmd.entry_type = COMMAND_TYPE;
		pkt->p.cmd.entry_count = 1;
		pkt->p.cmd.target = (uint8_t)fcport->loop_id;
		pkt->p.cmd.control_flags =
			__constant_cpu_to_le16(CF_READ | CF_SIMPLE_TAG);
		pkt->p.cmd.scsi_cdb[0] = RPT_LUN_SCSI_OPCODE;
		pkt->p.cmd.scsi_cdb[8] = MSB(sizeof(rpt_lun_lst_t));
		pkt->p.cmd.scsi_cdb[9] = LSB(sizeof(rpt_lun_lst_t));
		pkt->p.cmd.dseg_count = __constant_cpu_to_le16(1);
		pkt->p.cmd.timeout = __constant_cpu_to_le16(10);
		pkt->p.cmd.byte_count = 
			__constant_cpu_to_le32(sizeof(rpt_lun_lst_t));
		pkt->p.cmd.dseg_0_address = cpu_to_le32(
			phys_address + sizeof(sts_entry_t));
		pkt->p.cmd.dseg_0_length =
			__constant_cpu_to_le32(sizeof(rpt_lun_lst_t));

		rval = qla2x00_issue_iocb(ha, pkt, phys_address,
				sizeof(rpt_lun_cmd_rsp_t));

		comp_status = le16_to_cpu(pkt->p.rsp.comp_status);
		scsi_status = le16_to_cpu(pkt->p.rsp.scsi_status);

		if (rval != QLA2X00_SUCCESS ||
			comp_status != CS_COMPLETE ||
			scsi_status & SS_CHECK_CONDITION) {

			/* Device underrun, treat as OK. */
			if (comp_status == CS_DATA_UNDERRUN &&
				scsi_status & SS_RESIDUAL_UNDER) {

				rval = QLA2X00_SUCCESS;
				break;
			}

			DEBUG(printk("%s(%ld): FAILED, issue_iocb fcport = %p "
					"rval = %x cs = %x ss = %x\n",
					__func__,
					ha->host_no,
					fcport,
					rval,
					comp_status,
					scsi_status);)

			rval = QLA2X00_FAILED;
			if (scsi_status & SS_CHECK_CONDITION) {
				DEBUG2(printk("%s(%ld): SS_CHECK_CONDITION "
						"Sense Data "
						"%02x %02x %02x %02x "
						"%02x %02x %02x %02x\n",
						__func__,
						ha->host_no,
						pkt->p.rsp.req_sense_data[0],
						pkt->p.rsp.req_sense_data[1],
						pkt->p.rsp.req_sense_data[2],
						pkt->p.rsp.req_sense_data[3],
						pkt->p.rsp.req_sense_data[4],
						pkt->p.rsp.req_sense_data[5],
						pkt->p.rsp.req_sense_data[6],
						pkt->p.rsp.req_sense_data[7]);)
				/* No point in retrying if ILLEGAL REQUEST */
				if (pkt->p.rsp.req_sense_data[2] ==
							ILLEGAL_REQUEST) {
					/* Clear RLC support flag */
					fcport->flags &= ~(FC_SUPPORT_RPT_LUNS);
					break;
				}
			}
		} else {
			break;
		}
	}

	/* Test for report LUN failure. */
	if (rval == QLA2X00_SUCCESS) {
		/* Configure LUN list. */
		len = be32_to_cpu(pkt->list.hdr.len);
		len /= 8;
		if (len == 0) {
			rval = QLA2X00_FAILED;
		} else {
			for (cnt = 0; cnt < len; cnt++) {
				lun = CHAR_TO_SHORT(pkt->list.lst[cnt].lsb,
						pkt->list.lst[cnt].msb.b);

				DEBUG3(printk("%s(%ld): lun = (%d)\n",
						__func__,
						ha->host_no,
						lun);)

				/* We only support 0 through MAX_LUNS-1 range */
				if (lun < MAX_LUNS) {
					qla2x00_cfg_lun(fcport, lun);
				}
			}
			rval = QLA2X00_SUCCESS;
		}
	} else {
		rval = QLA2X00_FAILED;
	}

	pci_free_consistent(ha->pdev, sizeof(rpt_lun_cmd_rsp_t),
			pkt, phys_address);


	LEAVE(__func__);

	return (rval);
}

/*
 * qla2x00_cfg_lun
 *	Configures LUN into fcport LUN list.
 *
 * Input:
 *	fcport:		FC port structure pointer.
 *	lun:		LUN number.
 *
 * Context:
 *	Kernel context.
 */
static void
qla2x00_cfg_lun(fc_port_t *fcport, uint16_t lun) 
{
	fc_lun_t		*fclun;

	/* Allocate LUN if not already allocated. */
	for (fclun = fcport->fclun; fclun != NULL; fclun = fclun->next) {
		if (fclun->lun == lun) {
			break;
		}
	}
	if (fclun == NULL) {
		fclun = kmalloc(sizeof(fc_lun_t), GFP_ATOMIC);
		if (fclun != NULL) {
			/* Setup LUN structure. */
			memset(fclun, 0, sizeof(fc_lun_t));
			fcport->lun_cnt++;
			fclun->fcport = fcport;
			/* How dow we assign the following */
			/*  fclun->state = FCS_ONLINE; */
			fclun->lun = lun;
			fclun->next = fcport->fclun;
			fcport->fclun = fclun;
		} else {
			printk(KERN_WARNING
				"%s(): Memory Allocation failed - FCLUN\n",
				__func__);
		}
	}
}

/*
 * qla2x00_configure_local_loop
 *	Updates Fibre Channel Device Database with local loop devices.
 *
 * Input:
 *	ha = adapter block pointer.
 *	enable_slot_reuse = allows the use of PORT_AVAILABLE slots.
 *
 * Returns:
 *	0 = success.
 *	BIT_0 = error.
 *	BIT_1 = database was full and a device was not configured.
 */
static uint8_t
qla2x00_configure_local_loop(scsi_qla_host_t *ha, uint8_t enable_slot_reuse) 
{
	uint8_t  status = 0;
	uint8_t  rval;
	uint8_t  port_name[8];
	uint8_t  update_status = 0;
	uint16_t index, size;
	dma_addr_t phys_address = 0;
	fcdev_t device;
	port_list_entry_t *gn_list, *port_entry;
	uint16_t localdevices = 0;

	ENTER(__func__);

	/*
	 * No point in continuing if the loop is in a volatile state -- 
	 * reschedule LOCAL_LOOP_UPDATE for later processing
	 */
	if (test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags)) {
		set_bit(LOCAL_LOOP_UPDATE, &ha->dpc_flags);
		return (0);
	}

	gn_list = pci_alloc_consistent(ha->pdev,
			sizeof(GN_LIST_LENGTH), &phys_address);
	if (gn_list == NULL) {
		printk(KERN_WARNING
			"scsi(%ld): Memory Allocation failed - port_list",
			ha->host_no);
		ha->mem_err++;

		DEBUG2(printk("%s(%ld): Failed to allocate memory, No "
				"local loop\n",
				__func__,
				ha->host_no);)

		return (BIT_0);
	}
	memset(gn_list, 0, sizeof(GN_LIST_LENGTH));

	/* Mark all local devices PORT_LOST_ID first */
	for (index = 0; index < MAX_FIBRE_DEVICES; index++) {
		if (ha->fc_db[index].loop_id <= LAST_SNS_LOOP_ID &&
			!(ha->fc_db[index].flag & DEV_PUBLIC)) {

			DEBUG(printk("%s(%ld): port lost @ slot %d %06x\n", 
					__func__,
					ha->host_no,
					index, 
					ha->fc_db[index].d_id.b24);)

			ha->fc_db[index].loop_id |= PORT_LOST_ID;
		}
	}

	DEBUG3(printk("%s(%ld): Getting FCAL position map\n",
		__func__, ha->host_no));
	DEBUG3(qla2x00_get_fcal_position_map(ha, NULL));

	/* Get port name list.*/
#if defined(FC_IP_SUPPORT)
	if (ha->flags.enable_ip == FALSE)
		rval = qla2x00_get_port_list(ha,
				gn_list, phys_address, BIT_0, &size);
	else
		/*
		 * Bit 0 - return node names,
		 * Bit 1 - loop IDs 0-255
		 */
		rval = qla2x00_get_port_list(ha,
				gn_list, phys_address, BIT_0|BIT_1, &size);
#else
	rval = qla2x00_get_port_list(ha, gn_list, phys_address, BIT_0, &size);
#endif
	if (rval) {
		status = BIT_0;
		goto cleanup_allocation;
	}

	DEBUG3(printk("%s(%ld): port list size (%d)\n",
		__func__, ha->host_no, size));
	DEBUG3(qla2x00_dump_buffer((uint8_t *)gn_list, size));

	/* Any valid entries returned? */
	/* dg: 10/29/99 for an empty list */
	if (size / sizeof(port_list_entry_t) == 0)
		goto cleanup_allocation;

	port_entry = gn_list;
	for ( ; size >= sizeof(port_list_entry_t);
			size -= sizeof(port_list_entry_t),
			port_entry++) {
		device.loop_id = le16_to_cpu(port_entry->loop_id);

#if defined(FC_IP_SUPPORT)
		device.loop_id &= LOOP_ID_MASK; 
#endif

		/* Skip any non-local loop-ids - this includes 'known ports' */
		if (device.loop_id > LAST_LOCAL_LOOP_ID) 
			continue;
#if NOT_NEEDED
		/* Skip the known ports. */
		if ((device.loop_id == SNS_FL_PORT) ||
			(device.loop_id == FABRIC_CONTROLLER) ||
			(device.loop_id == SIMPLE_NAME_SERVER))
			continue;
#endif

		/* Get port name */
		rval = qla2x00_get_port_name(ha, device.loop_id, port_name, 0);
		if (rval || qla2x00_is_wwn_zero(port_name)) {
			DEBUG2(printk("%s(%ld): get_port_name error.\n",
					__func__,
					ha->host_no);)
			status = BIT_0;
			break;
		}
		memcpy(device.wwn, port_name, WWN_SIZE);
		DEBUG3(printk("%s(%ld): found portname -> "
				"%02x%02x%02x%02x%02x%02x%02x%02x\n",
				__func__,
				ha->host_no,
				port_name[0], port_name[1],
				port_name[2], port_name[3],
				port_name[4], port_name[5],
				port_name[6], port_name[7]);)
	
		/* Now get node name -- big-endian format */
		*((u64 *)device.name) = be64_to_cpup((u64 *)port_entry->name);
		DEBUG3(printk("%s(%ld): found nodename -> "
				"%02x%02x%02x%02x%02x%02x%02x%02x\n",
				__func__,
				ha->host_no,
				device.name[0], device.name[1],
				device.name[2], device.name[3],
				device.name[4], device.name[5],
				device.name[6], device.name[7]);)

		device.flag = 0;

		/* Derive portid from alpa table */
		device.d_id.b24 = 0;
		device.d_id.b.al_pa = alpa_table[device.loop_id];

#if defined(FC_IP_SUPPORT)
		if (!(list_entry_loop_id & PLE_NOT_SCSI_DEVICE)) {
#endif
			/* SCSI type device */
			update_status = qla2x00_update_fc_database(ha,
					&device, enable_slot_reuse);

			if (update_status)
				status |= update_status;
			else
				localdevices++;

#if defined(FC_IP_SUPPORT)
		} else if (ha->flags.enable_ip == TRUE) {
			/* SCSI login failed, assume it is IP device */
			DEBUG12(printk("qla%ld: IP local WWN:"
					"%02x%02x%02x%02x%02x%02x%02x%02x "
					"DID:%06x\n",
					ha->instance,
					device.name[0], device.name[1],
					device.name[2], device.name[3],
					device.name[4], device.name[5],
					device.name[6], device.name[7],
					device.d_id.b24);)

			update_status = qla2x00_update_ip_device_data(ha,
					&device);

			if (update_status == QL_STATUS_SUCCESS)
				localdevices++;
			else if (update_status == QL_STATUS_RESOURCE_ERROR)
				status |= BIT_1;
			else
				status |= BIT_0;
		}
#endif
	} /* for each port entry */

cleanup_allocation:

	pci_free_consistent(ha->pdev,
			sizeof(GN_LIST_LENGTH), gn_list, phys_address);

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status & BIT_0)
		printk(KERN_WARNING
			"%s(%ld): *** FAILED ***\n",
			__func__,
			ha->host_no);
#endif

	if (localdevices > 0) {
		ha->device_flags |= DFLG_LOCAL_DEVICES;
		ha->device_flags &= ~DFLG_RETRY_LOCAL_DEVICES;
	}

	LEAVE(__func__);

	return (status);
}


/*
 * qla2x00_tgt_alloc
 *	Allocate and pre-initialize target queue.
 *
 * Input:
 *	ha = adapter block pointer.
 *	t = SCSI target number.
 *
 * Returns:
 *	NULL = failure
 *
 * Context:
 *	Kernel context.
 */
os_tgt_t *
qla2x00_tgt_alloc(scsi_qla_host_t *ha, uint16_t t) 
{
	os_tgt_t	*tq;

	ENTER(__func__);

	/*
	 * If SCSI addressing OK, allocate TGT queue and lock.
	 */
	if (t >= MAX_TARGETS) {
		DEBUG2(printk("%s(%ld): *** Invalid target number, exiting ***",
				__func__,
				ha->host_no);)
		return (NULL);
	}

	tq = TGT_Q(ha, t);
	if (tq == NULL) {
		tq = kmalloc(sizeof(os_tgt_t), GFP_ATOMIC);
		if (tq != NULL) {
			DEBUG(printk("Alloc Target %d @ %p\n", t, tq);)

			memset(tq, 0, sizeof(os_tgt_t));
			tq->flags = TGT_TAGGED_QUEUE;
			tq->ha = ha;

			TGT_Q(ha, t) = tq;
		}
	}
	if (tq != NULL) {
		tq->port_down_retry_count = ha->port_down_retry_count;
	} else {
		printk(KERN_WARNING
			"%s(%ld): Failed to allocate target\n",
			__func__,
			ha->host_no);
		ha->mem_err++;
	}

	LEAVE(__func__);

	return (tq);
}

/*
 * qla2x00_tgt_free
 *	Frees target and LUN queues.
 *
 * Input:
 *	ha = adapter block pointer.
 *	t = SCSI target number.
 *
 * Context:
 *	Kernel context.
 */
void
qla2x00_tgt_free(scsi_qla_host_t *ha, uint16_t t) 
{
	os_tgt_t	*tq;
	uint16_t	l;

	ENTER(__func__);

	/*
	 * If SCSI addressing OK, allocate TGT queue and lock.
	 */
	if (t >= MAX_TARGETS) {
		DEBUG2(printk("%s(): **** FAILED exiting ****", __func__);)

		return;
	}

	tq = TGT_Q(ha, t);
	if (tq != NULL) {
		TGT_Q(ha, t) = NULL;
		DEBUG(printk("Dealloc target @ %p -- deleted\n", tq);)

		/* Free LUN structures. */
		for (l = 0; l < MAX_LUNS; l++)
			qla2x00_lun_free(ha, t, l);

		kfree(tq);
	}

	LEAVE(__func__);

	return;
}

/*
 * qla2x00_lun_alloc
 *	Allocate and initialize LUN queue.
 *
 * Input:
 *	ha = adapter block pointer.
 *	t = SCSI target number.
 *	l = LUN number.
 *
 * Returns:
 *	NULL = failure
 *
 * Context:
 *	Kernel context.
 */
os_lun_t *
qla2x00_lun_alloc(scsi_qla_host_t *ha, uint16_t t, uint16_t l) 
{
	os_lun_t	*lq;

	ENTER(__func__);

	/*
	 * If SCSI addressing OK, allocate LUN queue.
	 */
	if (t >= MAX_TARGETS || 
		l >= MAX_LUNS || 
		TGT_Q(ha, t) == NULL) {

		DEBUG2(printk("%s(): tgt=%d, tgt_q= %p, lun=%d, "
				"instance=%ld **** FAILED exiting ****\n",
				__func__,
				t,
				TGT_Q(ha,t),
				l,
				ha->instance);)

		return (NULL);
	}

	lq = LUN_Q(ha, t, l);
	if (lq == NULL) {
		lq = kmalloc(sizeof(os_lun_t), GFP_ATOMIC);
		if (lq != NULL) {

			DEBUG5(printk("Alloc Lun %d @ %p \n",l,lq);)

			memset(lq, 0, sizeof (os_lun_t));
			LUN_Q(ha, t, l) = lq;
			/*
			 * The following lun queue initialization code
			 * must be duplicated in alloc_ioctl_mem function
			 * for ioctl_lq.
			 */
			lq->q_state = LUN_STATE_READY;
			spin_lock_init(&lq->q_lock);
		} else {
			/*EMPTY*/
			DEBUG2(printk("%s(): Failed to allocate lun %d ***\n",
					__func__,
					l);)
			printk(KERN_WARNING
				"scsi(%ld): Memory Allocation failed - FCLUN\n",
				ha->host_no);
			ha->mem_err++;
		}
	}

	if (lq == NULL) {
		DEBUG2(printk("%s(): **** FAILED exiting ****\n", __func__);)
	} else {
		LEAVE(__func__);
	}

	return (lq);
}

/*
 * qla2x00_lun_free
 *	Frees LUN queue.
 *
 * Input:
 *	ha = adapter block pointer.
 *	t = SCSI target number.
 *
 * Context:
 *	Kernel context.
 */
static void
qla2x00_lun_free(scsi_qla_host_t *ha, uint16_t t, uint16_t l) 
{
	os_lun_t	*lq;

	ENTER(__func__);

	/*
	 * If SCSI addressing OK, allocate TGT queue and lock.
	 */
	if (t >= MAX_TARGETS || l >= MAX_LUNS) {
		DEBUG2(printk("%s(): **** FAILED exiting ****", __func__);)

		return;
	}

	if (TGT_Q(ha, t) != NULL && 
		(lq = LUN_Q(ha, t, l)) != NULL) {

		LUN_Q(ha, t, l) = NULL;
#ifdef __VMWARE__
		spin_lock_destroy(&lq->q_lock);
#endif
		kfree(lq);

		DEBUG3(printk("Dealloc lun @ %p -- deleted\n", lq);)
	}

	LEAVE(__func__);

	return;
}


/*
 * qla2x00_next
 *	Retrieve and process next job in the LUN queue.
 *
 * Input:
 *	tq = SCSI target queue pointer.
 *	lq = SCSI LUN queue pointer.
 *	TGT_LOCK must be already obtained.
 *
 * Output:
 *	Releases TGT_LOCK upon exit.
 *
 * Context:
 *	Kernel/Interrupt context.
 * 
 * Note: This routine will always try to start I/O from visible HBA.
 */
void
qla2x00_next(scsi_qla_host_t *vis_ha) 
{
	scsi_qla_host_t *dest_ha;
	fc_port_t	*fcport;
	srb_t		*sp;
	int		rval;
	unsigned long   flags;

	ENTER(__func__);

	spin_lock_irqsave(&vis_ha->list_lock, flags);
	while (!list_empty(&vis_ha->pending_queue)) {
		sp = list_entry(vis_ha->pending_queue.next, srb_t, list);

		fcport = sp->fclun->fcport;
		dest_ha = fcport->ha;

		/* Check if command can be started, exit if not. */
		if (LOOP_TRANSITION(dest_ha)) {
			break;
		}

		__del_from_pending_queue(vis_ha, sp);

		/* If device is dead then send request back to OS */
		if ((dest_ha->flags.link_down_error_enable &&
			atomic_read(&fcport->state) == FC_DEVICE_DEAD)) {

			CMD_RESULT(sp->cmd) = DID_NO_CONNECT << 16;

			if (!atomic_read(&dest_ha->loop_down_timer) && 
				dest_ha->loop_state == LOOP_DOWN) {
				sp->err_id = 2;

			} else {
				sp->err_id = 1;
			}
			DEBUG3(printk("scsi(%ld): loop/port is down - "
					"pid=%ld, sp=%p loopid=0x%x queued "
					"to dest HBA scsi%ld.\n", 
					dest_ha->host_no,
					sp->cmd->serial_number,
					sp,
					fcport->loop_id,
					dest_ha->host_no);)
			/* 
			 * Initiate a failover - done routine will initiate.
			 */
			__add_to_done_queue(vis_ha, sp);

			continue;
		}

		/*
		 * SCSI Kluge: Whenever, we need to wait for an event such as
		 * loop down (i.e. loop_down_timer ) or port down (i.e.  LUN
		 * request qeueue is suspended) then we will recycle new
		 * commands back to the SCSI layer.  We do this because this is
		 * normally a temporary condition and we don't want the
		 * mid-level scsi.c driver to get upset and start aborting
		 * commands.  The timeout value is extracted from the command
		 * minus 1-second and put on a retry queue (watchdog). Once the
		 * command timeout it is returned to the mid-level with a BUSY
		 * status, so the mid-level will retry it. This process
		 * continues until the LOOP DOWN time expires or the condition
		 * goes away.
		 */
	 	if (!(sp->flags & SRB_IOCTL) &&
			(atomic_read(&fcport->state) != FC_ONLINE ||
			 test_bit(ABORT_ISP_ACTIVE, &dest_ha->dpc_flags) ||
			 (dest_ha->loop_state != LOOP_READY)
			 || (sp->flags & SRB_FAILOVER)
			 )) {

			DEBUG3(printk("scsi(%ld): port=(0x%x) retry_q(%d) loop "
					"state = %d, loop counter = 0x%x"
					" dpc flags = 0x%lx\n",
					dest_ha->host_no,
					fcport->loop_id,
					atomic_read(&fcport->state),
					dest_ha->loop_state,
					atomic_read(&dest_ha->loop_down_timer),
					dest_ha->dpc_flags);)

			qla2x00_extend_timeout(sp->cmd, EXTEND_CMD_TIMEOUT);
			__add_to_retry_queue(vis_ha, sp);
			continue;
		} 

		/*
		 * if this request's lun is suspended then put the request on
		 * the  scsi_retry queue. 
		 */
	 	if (!(sp->flags & SRB_IOCTL) &&
			sp->lun_queue->q_state == LUN_STATE_WAIT) {
			DEBUG3(printk("%s(): lun wait state - pid=%ld, "
					"opcode=%d, allowed=%d, retries=%d\n",
					__func__,
					sp->cmd->serial_number,
					sp->cmd->cmnd[0],
					sp->cmd->allowed,
					sp->cmd->retries);)
				
			__add_to_scsi_retry_queue(vis_ha, sp);
			continue;
		}

		sp->lun_queue->io_cnt++;

		/* Release target queue lock */
		spin_unlock_irqrestore(&vis_ha->list_lock, flags);

		if (dest_ha->flags.enable_64bit_addressing)
			rval = qla2x00_64bit_start_scsi(sp);
		else
			rval = qla2x00_32bit_start_scsi(sp);

		spin_lock_irqsave(&vis_ha->list_lock, flags);

		if (rval != QLA2X00_SUCCESS) {
			/* Place request back on top of device queue */
			/* add to the top of queue */
			__add_to_pending_queue_head(vis_ha, sp);

			sp->lun_queue->io_cnt--;
			break;
		}
	}
	spin_unlock_irqrestore(&vis_ha->list_lock, flags);

	LEAVE(__func__);
}

/*
 * qla2x00_is_wwn_zero
 *
 * Input:
 *      wwn = Pointer to WW name to check
 *
 * Returns:
 *      TRUE if name is 0 else FALSE
 *
 * Context:
 *      Kernel context.
 */
static inline int
qla2x00_is_wwn_zero(uint8_t *wwn) 
{
	int cnt;

	/* Check for zero node name */
	for (cnt = 0; cnt < WWN_SIZE ; cnt++, wwn++) {
		if (*wwn != 0)
			break;
	}
	/* if zero return TRUE */
	if (cnt == WWN_SIZE)
		return (TRUE);
	else
		return (FALSE);
}

/*
 * qla2x00_get_lun_mask_from_config
 *      Get lun mask from the configuration parameters.
 *      Bit order is little endian.
 *
 * Input:
 * ha  -- Host adapter
 * tgt  -- target/device number
 * port -- pointer to port
 */
void
qla2x00_get_lun_mask_from_config(scsi_qla_host_t *ha, 
		fc_port_t *port, uint16_t tgt, uint16_t dev_no) 
{
	char		propbuf[60]; /* size of search string */
	int		rval, lun, l;
	lun_bit_mask_t	lun_mask, *mask_ptr = &lun_mask;

	/* Get "target-N-device-N-lun-mask" as a 256 bit lun_mask*/
	sprintf(propbuf, "scsi-qla%ld-tgt-%d-di-%d-lun-disabled",
			ha->instance, tgt, dev_no);

	rval = qla2x00_get_prop_xstr(ha, propbuf, (uint8_t *)&lun_mask,
			sizeof(lun_bit_mask_t));
	if (rval != -1 && 
		(rval == sizeof(lun_bit_mask_t))) {

		DEBUG3(printk("%s(%ld): lun mask for port %p from file:\n",
				__func__,
				ha->host_no, 
				port);)
		DEBUG3(qla2x00_dump_buffer((uint8_t *)&port->lun_mask,
					sizeof(lun_bit_mask_t));)

		for (lun = 8 * sizeof(lun_bit_mask_t) - 1, l = 0; 
			lun >= 0; 
			lun--, l++) {

			if (EXT_IS_LUN_BIT_SET(mask_ptr, lun))
				EXT_SET_LUN_BIT((&port->lun_mask),l);
			else
				EXT_CLR_LUN_BIT((&port->lun_mask),l);
		}

		DEBUG3(printk("%s(%ld): returning lun mask for port "
				"%02x%02x%02x%02x%02x%02x%02x%02x:\n",
				__func__,
				ha->host_no, 
				port->port_name[0], port->port_name[1],
				port->port_name[2], port->port_name[3],
				port->port_name[4], port->port_name[5],
				port->port_name[6], port->port_name[7]);)
		DEBUG3(qla2x00_dump_buffer((uint8_t *)&port->lun_mask,
				sizeof(lun_bit_mask_t));)
	}
}

/*
 * qla2x00_bstr_to_hex
 *	Convert hex byte string to number.
 *
 * Input:
 *	s = byte string pointer.
 *	bp = byte pointer for number.
 *	size = number of bytes.
 *
 * Context:
 *	Kernel/Interrupt context.
 */
static int
qla2x00_bstr_to_hex(char *s, uint8_t *bp, int size) 
{
	int		cnt;
	uint8_t		n;

	ENTER(__func__);

	for (cnt = 0; *s != '\0' && cnt / 2 < size; cnt++) {
		if (*s >= 'A' && *s <= 'F') {
			n = (*s++ - 'A') + 10;
		} else if (*s >= 'a' && *s <= 'f') {
			n = (*s++ - 'a') + 10;
		} else if (*s >= '0' && *s <= '9') {
			n = *s++ - '0';
		} else {
			cnt = 0;
			break;
		}

		if (cnt & BIT_0)
			*bp++ |= n;
		else
			*bp = n << 4;
	}
	/* fixme(dg) Need to swap data little endian */

	LEAVE(__func__);

	return (cnt / 2);
}

/*
 * qla2x00_get_prop_xstr
 *      Get a string property value for the specified property name and
 *      convert from the property string found in the configuration file,
 *      which are ASCII characters representing nibbles, 2 characters represent
 *      the hexdecimal value for a byte in the byte array.
 *      The byte array is initialized to zero.
 *      The resulting converted value is in big endian format (MSB at byte0).
 *
 * Input:
 *      ha = adapter state pointer.
 *      propname = property name pointer.
 *      propval  = pointer where to store converted property val.
 *      size = max or expected size of 'propval' array.
 *
 * Returns:
 *      0 = empty value string or invalid character in string
 *      >0 = count of characters converted
 *      -1 = property not found
 *
 * Context:
 *      Kernel context.
 */
int
qla2x00_get_prop_xstr(scsi_qla_host_t *ha, 
		char *propname, uint8_t *propval, int size) 
{
	char		*propstr;
	int		rval = -1;
	static char	buf[LINESIZE];

	ENTER(__func__);

	/* Get the requested property string */
	rval = qla2x00_find_propname(ha, propname, buf, ha->cmdline, size*2);
	DEBUG3(printk("%s(): Ret rval from find propname = %d\n",
			__func__,
			rval);)

	propstr = &buf[0];
	if (*propstr == '=')
		propstr++;   /* ignore equal sign */

	if (rval == 0) {  /* not found */
		LEAVE(__func__);
		return (-1);
	}

	rval = qla2x00_bstr_to_hex(propstr, (uint8_t *)propval, size);
	if (rval == 0) {
		/* Invalid character in value string */
		printk(KERN_INFO
			"%s(): %s Invalid hex string for property\n",
			__func__,
			propname);
		printk(KERN_INFO
			" Invalid string - %s\n", 
			propstr);
	}

	LEAVE(__func__);

	return (rval);
}

/*
 * qla2x00_chg_endian
 *	Change endianess of byte array.
 *
 * Input:
 *	buf = array pointer.
 *	size = size of array in bytes.
 *
 * Context:
 *	Kernel context.
 */
void
qla2x00_chg_endian(uint8_t buf[], size_t size) 
{
	uint8_t byte;
	size_t cnt1;
	size_t cnt;

	cnt1 = size - 1;
	for (cnt = 0; cnt < size / 2; cnt++) {
		byte = buf[cnt1];
		buf[cnt1] = buf[cnt];
		buf[cnt] = byte;
		cnt1--;
	}
}

/*
 * qla2x00_allocate_sp_pool
 * 	 This routine is called during initialization to allocate
 *  	 memory for local srb_t.
 *
 * Input:
 *	 ha   = adapter block pointer.
 *
 * Context:
 *      Kernel context.
 * 
 * Note: Sets the ref_count for non Null sp to one.
 */
uint8_t
qla2x00_allocate_sp_pool(scsi_qla_host_t *ha) 
{
	srb_t   *sp;
	int  i;
	uint8_t      status = QL_STATUS_SUCCESS;

	ENTER(__func__);
	
	DEBUG4(printk("%s(): Entered.\n", __func__);)

	/*
	 * Note: Need to alloacte each SRB as Kernel 2.4 seems to have error
	 * when allocating large amount of memory.
	 */
	/*
	 * FIXME(dg) - Need to allocated the SRBs by pages instead of each SRB
	 * object.
	 */
	INIT_LIST_HEAD(&ha->free_queue);
	ha->srb_alloc_cnt = 0;
	for (i=0; i < max_srbs; i++) {
		sp =  kmalloc(sizeof(srb_t), GFP_KERNEL);
		if (sp == NULL) {
			printk("%s(%ld): failed to allocate memory, "
				"count = %d\n", 
				__func__,
				ha->host_no, 
				i);
		} else {
			memset(sp, 0, sizeof(srb_t));
			__add_to_free_queue (ha, sp);
			sp->magic = SRB_MAGIC;
			sp->ref_num = ha->srb_alloc_cnt;
			sp->host_no = ha->host_no;
			ha->srb_alloc_cnt++;
			atomic_set(&sp->ref_count, 0);
		}
	}
	/*
	 * If we fail to allocte memory return an error
	 */
	if (ha->srb_alloc_cnt == 0)
		status = QL_STATUS_ERROR;

	printk(KERN_INFO
		"scsi(%ld): Allocated %d SRB(s).\n",
		ha->host_no,
		ha->srb_alloc_cnt);

	LEAVE(__func__);

	return( status );
}

/*
 *  This routine frees all adapter allocated memory.
 *  
 */
void
qla2x00_free_sp_pool( scsi_qla_host_t *ha) 
{
	struct list_head *list, *temp;
	srb_t         *sp;
	int cnt_free_srbs = 0;

	list_for_each_safe(list, temp, &ha->free_queue) {
		sp = list_entry(list, srb_t, list);
		/* Remove srb from LUN queue. */
		__del_from_free_queue(ha,sp);
		kfree(sp);
		cnt_free_srbs++;
	}

	if (cnt_free_srbs != ha->srb_alloc_cnt ) {
		DEBUG(printk("qla2x00 (%ld): Did not free all srbs,"
				" Free count = %d, Alloc Count = %d\n",
				ha->host_no, 
				cnt_free_srbs, 
				ha->srb_alloc_cnt);)
		printk(KERN_INFO
			"qla2x00 (%ld): Did not free all srbs, Free count = "
			"%d, Alloc Count = %d\n",
			ha->host_no, 
			cnt_free_srbs, 
			ha->srb_alloc_cnt);
	}
}

/* Flash support routines */

/**
 * qla2x00_flash_enable() - Setup flash for reading and writing.
 * @ha: HA context
 */
STATIC void
qla2x00_flash_enable(scsi_qla_host_t *ha)
{
	uint16_t	data;
	device_reg_t	*reg = ha->iobase;

	data = RD_REG_WORD(&reg->ctrl_status);
	data |= CSR_FLASH_ENABLE;
	WRT_REG_WORD(&reg->ctrl_status, data);
}

/**
 * qla2x00_flash_disable() - Disable flash and allow RISC to run.
 * @ha: HA context
 */
STATIC void
qla2x00_flash_disable(scsi_qla_host_t *ha)
{
	uint16_t	data;
	device_reg_t	*reg = ha->iobase;

	data = RD_REG_WORD(&reg->ctrl_status);
	data &= ~(CSR_FLASH_ENABLE);
	WRT_REG_WORD(&reg->ctrl_status, data);
}

/**
 * qla2x00_read_flash_byte() - Reads a byte from flash
 * @ha: HA context
 * @addr: Address in flash to read
 *
 * A word is read from the chip, but, only the lower byte is valid.
 *
 * Returns the byte read from flash @addr.
 */
STATIC uint8_t
qla2x00_read_flash_byte(scsi_qla_host_t *ha, uint32_t addr)
{
	uint16_t	data;
	uint16_t	bank_select;
	device_reg_t	*reg = ha->iobase;

	/* Setup bit 16 of flash address. */
	bank_select = RD_REG_WORD(&reg->ctrl_status);
	if ((addr & BIT_16) && ((bank_select & CSR_FLASH_64K_BANK) == 0)) {
		bank_select |= CSR_FLASH_64K_BANK;
		WRT_REG_WORD(&reg->ctrl_status, bank_select);
	} else if (((addr & BIT_16) == 0) &&
			(bank_select & CSR_FLASH_64K_BANK)) {
		bank_select &= ~(CSR_FLASH_64K_BANK);
		WRT_REG_WORD(&reg->ctrl_status, bank_select);
	}
	WRT_REG_WORD(&reg->flash_address, (uint16_t)addr);
	data = qla2x00_debounce_register(&reg->flash_data);

	return ((uint8_t)data);
}

/**
 * qla2x00_write_flash_byte() - Write a byte to flash
 * @ha: HA context
 * @addr: Address in flash to write
 * @data: Data to write
 */
STATIC void
qla2x00_write_flash_byte(scsi_qla_host_t *ha, uint32_t addr, uint8_t data)
{
	uint16_t	bank_select;
	device_reg_t	*reg = ha->iobase;

	/* Setup bit 16 of flash address. */
	bank_select = RD_REG_WORD(&reg->ctrl_status);
	if ((addr & BIT_16) && ((bank_select & CSR_FLASH_64K_BANK) == 0)) {
		bank_select |= CSR_FLASH_64K_BANK;
		WRT_REG_WORD(&reg->ctrl_status, bank_select);
	} else if (((addr & BIT_16) == 0) &&
			(bank_select & CSR_FLASH_64K_BANK)) {
		bank_select &= ~(CSR_FLASH_64K_BANK);
		WRT_REG_WORD(&reg->ctrl_status, bank_select);
	}
	WRT_REG_WORD(&reg->flash_address, (uint16_t)addr);
	WRT_REG_WORD(&reg->flash_data, (uint16_t)data);
}

/**
 * qla2x00_poll_flash() - Polls flash for completion.
 * @ha: HA context
 * @addr: Address in flash to poll
 * @poll_data: Data to be polled
 * @mid: Flash manufacturer ID
 *
 * This function polls the device until bit 7 of what is read matches data
 * bit 7 or until data bit 5 becomes a 1.  If that hapens, the flash ROM timed
 * out (a fatal error).  The flash book recommeds reading bit 7 again after
 * reading bit 5 as a 1.
 *
 * Returns 0 on success, else non-zero.
 */
STATIC uint8_t
qla2x00_poll_flash(scsi_qla_host_t *ha,
		uint32_t addr, uint8_t poll_data, uint8_t mid)
{
	uint8_t		status;
	uint8_t		flash_data;
	uint32_t	cnt;
	int		failed_pass;

	status = 1;
	failed_pass = 1;

	/* Wait for 30 seconds for command to finish. */
	poll_data &= BIT_7;
	for (cnt = 3000000; cnt; cnt--) {
		flash_data = qla2x00_read_flash_byte(ha, addr);
		if ((flash_data & BIT_7) == poll_data) {
			status = 0;
			break;
		}

		if (mid != 0x40 && mid != 0xda) {
			if (flash_data & BIT_5)
				failed_pass--;
			if (failed_pass < 0)
				break;
		}
		udelay(10);
		barrier();
	}
	return (status);
}

/**
 * qla2x00_program_flash_address() - Programs a flash address
 * @ha: HA context
 * @addr: Address in flash to program
 * @data: Data to be written in flash
 * @mid: Flash manufacturer ID
 *
 * Returns 0 on success, else non-zero.
 */
STATIC uint8_t
qla2x00_program_flash_address(scsi_qla_host_t *ha,
		uint32_t addr, uint8_t data, uint8_t mid)
{
	/* Write Program Command Sequence */
	qla2x00_write_flash_byte(ha, 0x5555, 0xaa);
	qla2x00_write_flash_byte(ha, 0x2aaa, 0x55);
	qla2x00_write_flash_byte(ha, 0x5555, 0xa0);
	qla2x00_write_flash_byte(ha, addr, data);

	/* Wait for write to complete. */
	return (qla2x00_poll_flash(ha, addr, data, mid));
}

/**
 * qla2x00_erase_flash_sector() - Erase a flash sector.
 * @ha: HA context
 * @addr: Flash sector to erase
 * @sec_mask: Sector address mask
 * @mid: Flash manufacturer ID
 *
 * Returns 0 on success, else non-zero.
 */
STATIC uint8_t
qla2x00_erase_flash_sector(scsi_qla_host_t *ha,
		uint32_t addr, uint32_t sec_mask, uint8_t mid)
{
	/* Individual Sector Erase Command Sequence */
	qla2x00_write_flash_byte(ha, 0x5555, 0xaa);
	qla2x00_write_flash_byte(ha, 0x2aaa, 0x55);
	qla2x00_write_flash_byte(ha, 0x5555, 0x80);
	qla2x00_write_flash_byte(ha, 0x5555, 0xaa);
	qla2x00_write_flash_byte(ha, 0x2aaa, 0x55);

	if (mid == 0xda)
		qla2x00_write_flash_byte(ha, addr & sec_mask, 0x10);
	else
		qla2x00_write_flash_byte(ha, addr & sec_mask, 0x30);

	udelay(150);

	/* Wait for erase to complete. */
	return (qla2x00_poll_flash(ha, addr, 0x80, mid));
}

/**
 * qla2x00_get_flash_manufacturer() - Read manufacturer ID from flash chip.
 * @ha: HA context
 *
 * Returns the manufacturer's ID read from the flash chip.
 */
STATIC uint8_t
qla2x00_get_flash_manufacturer(scsi_qla_host_t *ha)
{
	uint8_t	manuf_id;

	qla2x00_write_flash_byte(ha, 0x5555, 0xaa);
	qla2x00_write_flash_byte(ha, 0x2aaa, 0x55);
	qla2x00_write_flash_byte(ha, 0x5555, 0x90);
	manuf_id = qla2x00_read_flash_byte(ha, 0x0001);

	return (manuf_id);
}

/**
 * qla2x00_get_flash_version() - Read version information from flash.
 * @ha: HA context
 *
 * Returns QL_STATUS_SUCCESS on successful retrieval of flash version.
 */
STATIC uint16_t
qla2x00_get_flash_version(scsi_qla_host_t *ha)
{
	uint16_t	ret = QL_STATUS_SUCCESS;
	uint32_t	loop_cnt = 1;  /* this is for error exit only */
	uint32_t	pcir_adr;

	ENTER(__func__);

	qla2x00_flash_enable(ha);
	do {	/* Loop once to provide quick error exit */
		/* Match signature */
		if (!(qla2x00_read_flash_byte(ha, 0) == 0x55 &&
			qla2x00_read_flash_byte(ha, 1) == 0xaa)) {
			/* No signature */
			DEBUG2(printk("%s(): No matching signature.\n",
					__func__);)
			ret = QL_STATUS_ERROR;
			break;
		}

		pcir_adr = qla2x00_read_flash_byte(ha, 0x18) & 0xff;

		/* validate signature of PCI data structure */
		if ((qla2x00_read_flash_byte(ha, pcir_adr)) == 'P' &&
			(qla2x00_read_flash_byte(ha, pcir_adr + 1)) == 'C' &&
			(qla2x00_read_flash_byte(ha, pcir_adr + 2)) == 'I' &&
			(qla2x00_read_flash_byte(ha, pcir_adr + 3)) == 'R') {

			/* Read version */
			ha->optrom_minor = qla2x00_read_flash_byte(ha,
					pcir_adr + 0x12);
			ha->optrom_major = qla2x00_read_flash_byte(ha,
					pcir_adr + 0x13);
			DEBUG3(printk("%s(): got %d.%d.\n",
					__func__, 
					ha->optrom_major, ha->optrom_minor);)
		} else {
			/* error */
			DEBUG2(printk("%s(): PCI data struct not found. "
					"pcir_adr=%x.\n",
					__func__, pcir_adr);)
			ret = QL_STATUS_ERROR;
			break;
		}

	} while (--loop_cnt);
	qla2x00_flash_disable(ha);

	LEAVE(__func__);

	return (ret);
}

#if defined(NOT_USED_FUNCTION)
/**
 * qla2x00_get_flash_image() - Read image from flash chip.
 * @ha: HA context
 * @image: Buffer to receive flash image
 *
 * Returns 0 on success, else non-zero.
 */
STATIC uint16_t
qla2x00_get_flash_image(scsi_qla_host_t *ha, uint8_t *image)
{
	uint32_t	addr;
	uint32_t	midpoint;
	uint8_t		*data;
	device_reg_t	*reg = ha->iobase;

	midpoint = FLASH_IMAGE_SIZE / 2;

	qla2x00_flash_enable(ha);
	WRT_REG_WORD(&reg->nvram, 0);
	for (addr = 0, data = image; addr < FLASH_IMAGE_SIZE; addr++, data++) {
		if (addr == midpoint)
			WRT_REG_WORD(&reg->nvram, NV_SELECT);

		*data = qla2x00_read_flash_byte(ha, addr);
	}
	qla2x00_flash_disable(ha);

	return (0);
}
#endif

/**
 * qla2x00_set_flash_image() - Write image to flash chip.
 * @ha: HA context
 * @image: Source image to write to flash
 *
 * Returns 0 on success, else non-zero.
 */
STATIC uint16_t
qla2x00_set_flash_image(scsi_qla_host_t *ha, uint8_t *image)
{
	uint16_t	status;
	uint32_t	addr;
	uint32_t	midpoint;
	uint32_t	sec_mask;
	uint32_t	rest_addr;
	uint8_t		mid;
	uint8_t		sec_number;
	uint8_t		data;
	device_reg_t	*reg = ha->iobase;

	status = 0;
	sec_number = 0;

	/* Reset ISP chip. */
	WRT_REG_WORD(&reg->ctrl_status, CSR_ISP_SOFT_RESET);

	qla2x00_flash_enable(ha);
	do {	/* Loop once to provide quick error exit */
		/* Structure of flash memory based on manufacturer */
		mid = qla2x00_get_flash_manufacturer(ha);
		if (mid == 0x6d) {
			// Am29LV001 part
			rest_addr = 0x1fff;
			sec_mask = 0x1e000;
		}
		else if (mid == 0x40) {
			// Mostel v29c51001 part
			rest_addr = 0x1ff;
			sec_mask = 0x1fe00;
		}
		else if (mid == 0xbf) {
			// SST39sf10 part
			rest_addr = 0xfff;
			sec_mask = 0x1f000;
		}
		else if (mid == 0xda) {
			// Winbond W29EE011 part
			rest_addr = 0x7f;
			sec_mask = 0x1ff80;
			addr = 0;
			if (qla2x00_erase_flash_sector(ha,
						addr, sec_mask, mid)) {
				status = 1;
				break;
			}
		}
		else {
			// Am29F010 part
			rest_addr = 0x3fff;
			sec_mask = 0x1c000;
		}

		midpoint = FLASH_IMAGE_SIZE / 2;
		for (addr = 0; addr < FLASH_IMAGE_SIZE; addr++)
		{
			data = *image++;
			/* Are we at the beginning of a sector? */
			if(!(addr & rest_addr)) {
				if (addr == midpoint)
					WRT_REG_WORD(&reg->nvram, NV_SELECT);

				/* Then erase it */
				if (qla2x00_erase_flash_sector(ha,
							addr, sec_mask, mid)) {
					status = 1;
					break;
				}

				sec_number++;
			}
			if (mid == 0x6d) {
				if (sec_number == 1 &&
						(addr == (rest_addr - 1))) {
					rest_addr = 0x0fff;
					sec_mask   = 0x1f000;
				}
				else if (sec_number == 3 && (addr & 0x7ffe)) {
					rest_addr = 0x3fff;
					sec_mask   = 0x1c000;
				}
			}

			if (qla2x00_program_flash_address(ha,
						addr, data, mid)) {
				status = 1;
				break;
			}
		}
	} while (0);
	qla2x00_flash_disable(ha);

	return (status);
}

#if USE_FLASH_DATABASE
#error Do not use FLASH DATABASE!!!!

/*
* qla2x00_flash_enable_database
*      Setup flash for reading/writing.
*
* Input:
*      ha = adapter block pointer.
*/
STATIC void
qla2x00_flash_enable_database(scsi_qla_host_t *ha)
{
	device_reg_t *reg = ha->iobase;

	/* Setup bit 16 of flash address. */
	WRT_REG_WORD(&reg->nvram, NV_SELECT);

	/* Enable Flash Read/Write. */
	WRT_REG_WORD(&reg->ctrl_status, CSR_FLASH_ENABLE);

	/* Read/Reset Command Sequence */
	qla2x00_write_flash_byte(ha, 0x5555, 0xaa);
	qla2x00_write_flash_byte(ha, 0x2aaa, 0x55);
	qla2x00_write_flash_byte(ha, 0x5555, 0xf0);
	qla2x00_read_flash_byte(ha, FLASH_DATABASE_0);
}

/*
* qla2x00_flash_disable_database
*      Disable flash and allow RISC to run.
*
* Input:
*      ha = adapter block pointer.
*/
STATIC void
qla2x00_flash_disable_database(scsi_qla_host_t *ha)
{
	device_reg_t *reg = ha->iobase;

	/* Restore chip registers. */
	WRT_REG_WORD(&reg->ctrl_status, 0);
	WRT_REG_WORD(&reg->nvram, 0);
}


/*
* qla2x00_get_database
*      Copies and converts flash database to driver database.
*      (may sleep)
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_get_database(scsi_qla_host_t *ha)
{
	flash_database_t *fptr;
	uint8_t          status = 1;
	uint32_t         addr;
	uint16_t         cnt;
	uint8_t          *bptr;
	uint8_t          checksum;
	uint32_t         b, t;

	ENTER("qla2x00_get_database");

	/* Default setup. */
	ha->flash_db = FLASH_DATABASE_0;
	ha->flash_seq = 0;

	fptr = kmalloc(sizeof(flash_database_t), GFP_ATOMIC);
	if (!fptr) {
		printk(KERN_WARNING
			"scsi(%d): Memory Allocation failed - flash mem",
			(int)ha->host_no);
		ha->mem_err++;
		return (status);
	}

	/* Enable Flash Read/Write. */
	qla2x00_flash_enable_database(ha);

	/* 
	 * Start with flash database with the highest sequence number. 
	 */
	b = qla2x00_read_flash_byte(ha, FLASH_DATABASE_0);
	b |= qla2x00_read_flash_byte(ha, FLASH_DATABASE_0 + 1) << 8;
	b |= qla2x00_read_flash_byte(ha, FLASH_DATABASE_0 + 1) << 16;
	b |= qla2x00_read_flash_byte(ha, FLASH_DATABASE_0 + 1) << 24;
	t = qla2x00_read_flash_byte(ha, FLASH_DATABASE_1);
	t |= qla2x00_read_flash_byte(ha, FLASH_DATABASE_1 + 1) << 8;
	t |= qla2x00_read_flash_byte(ha, FLASH_DATABASE_1 + 1) << 16;
	t |= qla2x00_read_flash_byte(ha, FLASH_DATABASE_1 + 1) << 24;
	if (t > b) {
		ha->flash_db = FLASH_DATABASE_1;
	}

	/* Select the flash database with the good checksum. */
	for (t = 0; t < 2; t++) {
		checksum = 0;
		addr = ha->flash_db;
		bptr = (uint8_t *)fptr;
		fptr->hdr.size = sizeof(flash_database_t);

		/* Read flash database to driver. */
		for (cnt = 0; cnt < fptr->hdr.size; cnt++) {
			*bptr = (uint8_t)qla2x00_read_flash_byte(ha, addr++);
			checksum += *bptr++;
			if (bptr == &fptr->hdr.spares[0] &&
				(fptr->hdr.size > sizeof(flash_database_t) ||
				 fptr->hdr.size < sizeof(flash_hdr_t) ||
				 !fptr->hdr.version) ) {

				checksum = 1;
				break;
			}
		}

		if (!checksum) {
			status = 0;
			break;
		}
		/* trying other database */
		if (ha->flash_db == FLASH_DATABASE_0) {
			ha->flash_db = FLASH_DATABASE_1;
		} else {
			ha->flash_db = FLASH_DATABASE_0;
		}
	}

	if (!status) {
		ha->flash_seq = fptr->hdr.seq;

		/* Convert flash database to driver database format. */
		if (fptr->hdr.size -= sizeof(flash_hdr_t)) {
			for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
				ha->fc_db[cnt].name[0] =
						fptr->node[cnt].name[0];
				ha->fc_db[cnt].name[1] =
						fptr->node[cnt].name[1];
				/* UNKNOWN CODE!!! 
				cnt,
				ha->fc_db[cnt].name[1],
				ha->fc_db[cnt].name[0]);
				*/

				ha->fc_db[cnt].loop_id = PORT_AVAILABLE;
				ha->fc_db[cnt].flag = 0;  /* v2.19.05b3 */
				if(!(fptr->hdr.size -= sizeof(flash_node_t)))
					break;
			}
		}
	}

	qla2x00_flash_disable_database(ha);

	kfree(fptr);

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status)
		printk("qla2x00_get_database: **** FAILED ****\n");
#endif

	LEAVE("qla2x00_get_database");

	return(status);
}

/*
* qla2x00_save_database
*      Copies and converts driver database to flash database.
*      (may sleep)
*
* Input:
*      ha = adapter block pointer.
*
* Returns:
*      0 = success.
*/
STATIC uint8_t
qla2x00_save_database(scsi_qla_host_t *ha)
{
	flash_database_t *fptr;
	uint8_t          status = 1;
	uint32_t         addr;
	uint16_t         cnt;
	uint8_t          *bptr;
	uint8_t          checksum;

	ENTER("qla2x00_save_database");

	fptr = kmalloc(sizeof(flash_database_t), GFP_ATOMIC);
	if (!fptr) {
		printk(KERN_WARNING
			"scsi(%d): Memory Allocation failed - flash mem",
			(int)ha->host_no);
		ha->mem_err++;
		return (status);
	}

	/* Enable Flash Read/Write. */
	qla2x00_flash_enable_database(ha);

	fptr->hdr.seq = ++ha->flash_seq;
	fptr->hdr.version = FLASH_DATABASE_VERSION;
	fptr->hdr.size = sizeof(flash_hdr_t);

	/* Copy and convert driver database to flash database. */
	for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
		if (ha->fc_db[cnt].loop_id == PORT_UNUSED)
			break;
		else {
			fptr->node[cnt].name[0] = ha->fc_db[cnt].name[0];
			fptr->node[cnt].name[1] = ha->fc_db[cnt].name[1];
			fptr->hdr.size += sizeof(flash_node_t);
		}
	}

	/* Calculate checksum. */
	checksum = 0;
	bptr = (uint8_t *)fptr;
	for (cnt = 0; cnt < fptr->hdr.size; cnt++)
		checksum += *bptr++;
	fptr->hdr.checksum = ~checksum + 1;

	/* Setup next sector address for flash */
	if (ha->flash_db == FLASH_DATABASE_0)
		addr = FLASH_DATABASE_1;
	else
		addr = FLASH_DATABASE_0;
	ha->flash_db = addr;

	/* Erase flash sector prior to write. */
	status = qla2x00_erase_flash_sector(ha, addr);

	/* Write database to flash. */
	bptr = (uint8_t *)fptr;
	for (cnt = 0; cnt < fptr->hdr.size && !status; cnt++)
		status = qla2x00_program_flash_address(ha, addr++, *bptr++);

	qla2x00_flash_disable_database(ha);

	kfree(fptr);

#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_3)
	if (status)
		printk("qla2x00_save_database: **** FAILED ****\n");
#endif

	LEAVE("qla2x00_save_database");

	return(status);
}

#endif


static int
qla2x00_add_initiator_device(scsi_qla_host_t *ha, fcdev_t *device)
{
	int	ret;
	fc_initiator_t	*fcinitiator;

	ret = 1;
	fcinitiator = kmalloc(sizeof(fc_initiator_t), GFP_ATOMIC);
	if (fcinitiator != NULL) {
		/* Setup initiator structure. */
		memset(fcinitiator, 0, sizeof(fc_initiator_t));
	
		memcpy(fcinitiator->node_name, device->name, WWN_SIZE);
		memcpy(fcinitiator->port_name, device->wwn, WWN_SIZE);
		fcinitiator->d_id.b24 = device->d_id.b24;
		fcinitiator->loop_id = device->loop_id;
		list_add_tail(&fcinitiator->list, &ha->fcinitiators);
		ret = 0;
	} else {
		printk(KERN_WARNING
			"%s(): Memory Allocation failed - FCINITIATOR\n",
			__func__);
	}

	return (ret);
}


/*
* Declarations for load module
*/
static  Scsi_Host_Template driver_template = QLA2100_LINUX_TEMPLATE;
#include "scsi_module.c"

/****************************************************************************/
/*                         Driver Debug Functions.                          */
/****************************************************************************/

#if defined(QL_DEBUG_ROUTINES)
static void
qla2x00_dump_buffer(uint8_t * b, uint32_t size) 
{
	uint32_t cnt;
	uint8_t c;

	printk(" 0   1   2   3   4   5   6   7   8   9 "
			"	Ah  Bh  Ch  Dh  Eh  Fh\n");
	printk("---------------------------------------"
			"------------------------\n");

	for (cnt = 0; cnt < size;) {
		c = *b++;
		printk("%02x",(uint32_t) c);
		cnt++;
		if (!(cnt % 16))
			printk("\n");
		else
			printk("  ");
	}
	if (cnt % 16)
		printk("\n");
}
#endif

/**************************************************************************
 *   qla2x00_print_scsi_cmd
 *	 Dumps out info about the scsi cmd and srb.
 *   Input	 
 *	 cmd : Scsi_Cmnd
 **************************************************************************/
void
qla2x00_print_scsi_cmd(Scsi_Cmnd * cmd) 
{
	struct scsi_qla_host *ha;
	struct Scsi_Host *host = cmd->host;
	srb_t *sp;
	struct os_lun *lq;
	fc_port_t *fcport;

	int i;
	ha = (struct scsi_qla_host *) host->hostdata;

	sp = (srb_t *) CMD_SP(cmd);
	printk("SCSI Command @= 0x%p, Handle=0x%08lx\n", 
			cmd, (u_long) CMD_HANDLE(cmd));
	printk("  chan=%d, target = 0x%02x, lun = 0x%02x, cmd_len = 0x%02x\n",
			cmd->channel, cmd->target, cmd->lun, cmd->cmd_len);
	printk(" CDB = ");
	for (i = 0; i < cmd->cmd_len; i++) {
		printk("0x%02x ", cmd->cmnd[i]);
	}
	printk("\n  seg_cnt =%d, retries=%d, serial_number_at_timeout=0x%lx\n",
			cmd->use_sg,
			cmd->retries, cmd->serial_number_at_timeout);
	printk("  request buffer=0x%p, request buffer len=0x%x\n", 
			cmd->request_buffer,
			cmd->request_bufflen);
	printk("  tag=%d, flags=0x%x, transfersize=0x%x \n", 
			cmd->tag, cmd->flags, cmd->transfersize);
	printk("  serial_number=%d, SP=%p\n", (int) cmd->serial_number,sp); 
	printk("  data direction=%d\n", cmd->sc_data_direction);
	if (sp) {
		printk("  sp flags=0x%x\n", sp->flags);
		printk("  r_start=0x%lx, u_start=0x%lx, "
				"f_start=0x%lx, state=%d\n", 
				sp->r_start, sp->u_start,
				sp->f_start, sp->state);

		lq = sp->lun_queue;
		fcport = lq->fclun->fcport;
		printk(" e_start= 0x%lx, ext_history= %d, "
				"fo retry=%d, loopid =%x, port path=%d\n", 
				sp->e_start, sp->ext_history,
				sp->fo_retry_cnt,
				fcport->loop_id, 
				fcport->cur_path);
	}
}

/*
 * qla2x00_print_q_info
 * 	 Prints queue info
 * Input
 *      q: lun queue	 
 */ 
void 
qla2x00_print_q_info(struct os_lun *q) 
{
	printk("Queue info: flags=0x%lx\n", q->q_flag);
}

#if defined(QL_DEBUG_ROUTINES)
/*
 * qla2x00_formatted_dump_buffer
 *       Prints string plus buffer.
 *
 * Input:
 *       string  = Null terminated string (no newline at end).
 *       buffer  = buffer address.
 *       wd_size = word size 8, 16, 32 or 64 bits
 *       count   = number of words.
 */
void
qla2x00_formatted_dump_buffer(char *string, uint8_t * buffer, 
				uint8_t wd_size, uint32_t count) 
{
	uint32_t cnt;
	uint16_t *buf16;
	uint32_t *buf32;

	if (ql2x_debug_print != TRUE)
		return;

	if (strcmp(string, "") != 0)
		printk("%s\n",string);

	switch (wd_size) {
		case 8:
			printk(" 0    1    2    3    4    5    6    7    "
				"8    9    Ah   Bh   Ch   Dh   Eh   Fh\n");
			printk("-----------------------------------------"
				"-------------------------------------\n");

			for (cnt = 1; cnt <= count; cnt++, buffer++) {
				printk("%02x",*buffer);
				if (cnt % 16 == 0)
					printk("\n");
				else
					printk("  ");
			}
			if (cnt % 16 != 0)
				printk("\n");
			break;
		case 16:
			printk("   0      2      4      6      8      Ah "
				"	Ch     Eh\n");
			printk("-----------------------------------------"
				"-------------\n");

			buf16 = (uint16_t *) buffer;
			for (cnt = 1; cnt <= count; cnt++, buf16++) {
				printk("%4x",*buf16);

				if (cnt % 8 == 0)
					printk("\n");
				else if (*buf16 < 10)
					printk("   ");
				else
					printk("  ");
			}
			if (cnt % 8 != 0)
				printk("\n");
			break;
		case 32:
			printk("       0          4          8          Ch\n");
			printk("------------------------------------------\n");

			buf32 = (uint32_t *) buffer;
			for (cnt = 1; cnt <= count; cnt++, buf32++) {
				printk("%8x", *buf32);

				if (cnt % 4 == 0)
					printk("\n");
				else if (*buf32 < 10)
					printk("   ");
				else
					printk("  ");
			}
			if (cnt % 4 != 0)
				printk("\n");
			break;
		default:
			break;
	}
}

#endif
/**************************************************************************
*   qla2x00_dump_regs
**************************************************************************/
static void 
qla2x00_dump_regs(struct Scsi_Host *host) 
{
	printk("Mailbox registers:\n");
	printk("qla2x00 : mbox 0 0x%04x \n", inw(host->io_port + 0x10));
	printk("qla2x00 : mbox 1 0x%04x \n", inw(host->io_port + 0x12));
	printk("qla2x00 : mbox 2 0x%04x \n", inw(host->io_port + 0x14));
	printk("qla2x00 : mbox 3 0x%04x \n", inw(host->io_port + 0x16));
	printk("qla2x00 : mbox 4 0x%04x \n", inw(host->io_port + 0x18));
	printk("qla2x00 : mbox 5 0x%04x \n", inw(host->io_port + 0x1a));
}


#if STOP_ON_ERROR
/**************************************************************************
*   qla2x00_panic
*
**************************************************************************/
static void 
qla2x00_panic(char *cp, struct Scsi_Host *host) 
{
	struct scsi_qla_host *ha;
	long *fp;

	ha = (struct scsi_qla_host *) host->hostdata;
	DEBUG2(ql2x_debug_print = 1;);
	printk("qla2100 - PANIC:  %s\n", cp);
	printk("Current time=0x%lx\n", jiffies);
	printk("Number of pending commands =0x%lx\n", ha->actthreads);
	printk("Number of queued commands =0x%lx\n", ha->qthreads);
	printk("Number of free entries = (%d)\n", ha->req_q_cnt);
	printk("Request Queue @ 0x%lx, Response Queue @ 0x%lx\n",
			       ha->request_dma, ha->response_dma);
	printk("Request In Ptr %d\n", ha->req_ring_index);
	fp = (long *) &ha->flags;
	printk("HA flags =0x%lx\n", *fp);
	qla2x00_dump_requests(ha);
	qla2x00_dump_regs(host);
	cli();
	for (;;) {
		udelay(2);
		barrier();
		/* cpu_relax();*/
	}
	sti();
}

#endif

/**************************************************************************
*   qla2x00_dump_requests
*
**************************************************************************/
void
qla2x00_dump_requests(scsi_qla_host_t *ha) 
{

	Scsi_Cmnd       *cp;
	srb_t           *sp;
	int i;

	printk("Outstanding Commands on controller:\n");

	for (i = 1; i < MAX_OUTSTANDING_COMMANDS; i++) {
		if ((sp = ha->outstanding_cmds[i]) == NULL)
			continue;
		if ((cp = sp->cmd) == NULL)
			continue;

		printk("(%d): Pid=%d, sp flags=0x%lx, cmd=0x%p\n", 
			i, 
			(int)sp->cmd->serial_number, 
			(long)sp->flags,CMD_SP(sp->cmd));
	}
}


/**************************************************************************
*   qla2x00_setup
*
*   Handle Linux boot parameters. This routine allows for assigning a value
*   to a parameter with a ';' between the parameter and the value.
*   ie. qla2x00=arg0;arg1;...;argN;<properties .... properties>  OR
*   via the command line.
*   ie. qla2x00 ql2xopts=arg0;arg1;...;argN;<properties .... properties>
**************************************************************************/
#if !defined(MODULE)
static int __init
qla2x00_setup (char *s)
#else
void 
qla2x00_setup(char *s)
#endif	
{
	char		*cp, *np;
	char		*slots[MAXARGS];
	char		**argv = &slots[0];
	static char	buf[LINESIZE];
	int		argc, opts;

#if !defined(MODULE)
	        if (s == NULL || *s == '\0')
			return 0;
#endif

	/*
	 * Determine if we have any properties.
	 */
	cp = s;
	opts = 1;
	while (*cp && (np = qla2x00_get_line(cp, buf)) != NULL) {
		if (strncmp("scsi-qla",buf,8) == 0) {
			DEBUG(printk("qla2100: devconf=%s\n",cp);)

			ql2xdevconf = cp;
			(opts > 0)? opts-- : 0;
			break;
		}
		opts++;
		cp = np;
	}
	/*
	 * Parse the args before the properties
	 */
	if (opts) {
		opts = (opts > MAXARGS-1)? MAXARGS-1: opts;
		argc = qla2x00_get_tokens(s, argv, opts);
		while (argc > 0) {
			cp = *argv;
			DEBUG(printk("scsi: found cmd arg =[%s]\n", cp);)

			if (strcmp(cp, "verbose") == 0) {
				DEBUG(printk("qla2100: verbose\n");)
				qla2x00_verbose++;
			} else if (strcmp(cp, "quiet") == 0) {
				qla2x00_quiet = 1;
			} else if (strcmp(cp, "reinit_on_loopdown") == 0) {
				qla2x00_reinit++;
				DEBUG(printk("qla2100: reinit_on_loopdown\n");)
			}
			argc--, argv++;
		}
	}
#if !defined(MODULE)
	if (ql2xdevconf)
		return 1;
	else
		return 0;
#endif

}

#if !defined(MODULE)
__setup("ql2xopts=", qla2x00_setup);
#endif

/********************** qla2x00_get_line *********************
* qla2x00_get_line
* Copy a substring from the specified string. The substring
* consists of any number of chars seperated by white spaces (i.e. spaces)
* and ending with a newline '\n' or a semicolon ';'.
*
* Enter:
* str - orig string
* line - substring
*
* Returns:
*   cp - pointer to next string
*     or
*   null - End of string
*************************************************************/
static char *
qla2x00_get_line(char *str, char *line) 
{
	register	char 	*cp = str;
	register	char 	*sp = line;

	/* skip preceeding spaces */
	while (*cp && *cp == ' ')
		++cp;
	while ((*cp) && *cp != '\n' && *cp != ';')   /* end of line */
		*sp++ = *cp++;

	*sp = '\0';

	DEBUG5(printk("%s(): %s\n", __func__, line);)

	if( (*cp) ) {
		cp++;
		return (cp);
	}

	return (NULL);
}


/**************************** get_tokens *********************
* Parse command line into argv1, argv2, ... argvX
* Arguments are seperated by white spaces and colons and end
* with a NULL.
*************************************************************/
static int 
qla2x00_get_tokens(char *line, char **argv, int maxargs ) 
{
	register	char 	*cp = line;
	int	count = 0;

	while (*cp && count < maxargs) {
		/* skip preceeding spaces */
		while ((*cp) && *cp == ' ')
			++cp;
		/* symbol starts here */
		argv[count++] = cp;
		/* skip symbols */
		while ((*cp) && !(*cp == ' ' || *cp == ';' || *cp == ':'))
			cp++;
		/* replace comma or space with a null */
		if((*cp) && (*cp ==' ' ) && argv[count-1] != cp)
			*cp++ = '\0';
	}
	return (count);
}

#if VSA
/*
 * qla2x00_get_vsa_opt_from_config
 *      Get VSA option from the configuration parameters.
 *      Bit order is little endian.
 *
 * Input:
 * ha  -- Host adapter
 * tgt  -- target/device number
 */
void
qla2x00_get_vsa_opt_from_config(scsi_qla_host_t *ha,
				uint16_t tgt, uint16_t dev_no) 
{

	char		propbuf[60]; /* size of search string */
	int		rval;
	char		vsa;

	/* Get "target-N-device-N-vsa" as a 1 bit value */
	sprintf(propbuf, "scsi-qla%ld-tgt-%d-di-%d-vsa",
			ha->instance, tgt, dev_no);

	rval = qla2x00_get_prop_xstr(ha, propbuf, (uint8_t *)&vsa,1);
	if (rval != -1 && rval == 1) {
		ha->fc_db[tgt].flag |= DEV_FLAG_VSA;

		DEBUG(printk("cfg: scsi-qla%d-target-%d-vsa=1\n",
				(int) ha->instance,  tgt);)
	}
}
#endif

/*
 * qla2x00_cfg_persistent_binding
 *	Get driver configuration file target persistent binding entries.
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Context:
 *	Kernel context.
 */
STATIC void
qla2x00_cfg_persistent_binding(scsi_qla_host_t *ha) 
{
	int		rval;
	static char	propbuf[LINESIZE];
	uint16_t	tgt;
	uint16_t	dev_no = 0; /* not used */
	char		*cmdline = ha->cmdline;
	port_id_t	d_id, *pd_id;
	uint8_t		portid[3];
	uint8_t		node_name[8], *pnn;
	uint8_t		port_name[8], *ppn;
	os_tgt_t	*tq;

	ENTER(__func__);

	/* FC name for devices */
	for (tgt = 0; tgt < MAX_FIBRE_DEVICES; tgt++) {

		/*
		 * Retrive as much information as possible (PN/PID/NN).
		 *
		 * Based on binding type, skip incomplete entries.
		 */
		ppn = port_name;
		sprintf(propbuf, "scsi-qla%d-tgt-%d-di-%d-port",
				(int)ha->instance, tgt, dev_no);
		rval = qla2x00_get_prop_16chars(ha, propbuf, ppn, cmdline);
		if (rval != 0)
			ppn = NULL;
		if (ha->binding_type == BIND_BY_PORT_NAME && rval != 0)
			continue;

		pd_id = &d_id;
		sprintf(propbuf, "scsi-qla%d-tgt-%d-di-%d-pid",
				(int)ha->instance, tgt, dev_no);
		rval = qla2x00_get_prop_xstr(ha,
				propbuf, portid, sizeof(portid));
		if (rval == -1 || rval != sizeof(portid))
			pd_id = NULL;
		if (ha->binding_type == BIND_BY_PORT_ID &&
			(rval == -1 || rval != sizeof(portid)))
			continue;

		pnn = node_name;
		sprintf(propbuf, "scsi-qla%d-tgt-%d-di-%d-node",
				(int)ha->instance, tgt, dev_no);
		rval = qla2x00_get_prop_16chars(ha, propbuf, pnn, cmdline);
		if (rval != 0)
			pnn = NULL;
		if (ha->binding_type == BIND_BY_NODE_NAME && rval != 0)
			continue;

		tq = qla2x00_tgt_alloc(ha, tgt);
		if (tq == NULL) {
			printk(KERN_WARNING
				"%s(): Unable to allocate memory for target\n",
				__func__);
			continue;
		}

		ha->fc_db[tgt].loop_id = PORT_AVAILABLE;
		ha->fc_db[tgt].flag = 0;  /* v2.19.05b3 */
		ha->fc_db[tgt].flag |= DEV_CONFIGURED;

		if (ppn != NULL) {
			memcpy(tq->port_name, ppn, WWN_SIZE);
			memcpy(ha->fc_db[tgt].wwn, ppn, WWN_SIZE);
		}
		if (pd_id != NULL) {
			/*
			 * The portid is read in big-endian format, convert 
			 * before updating information
			 */
			pd_id->r.d_id[0] = portid[2];
			pd_id->r.d_id[1] = portid[1];
			pd_id->r.d_id[2] = portid[0];
			tq->d_id.b24 = pd_id->b24;
			ha->fc_db[tgt].d_id.b24 = pd_id->b24;
		}
		if (pnn != NULL) {
			memcpy(tq->node_name, pnn, WWN_SIZE);
			memcpy(ha->fc_db[tgt].name, pnn, WWN_SIZE);
		}

		DEBUG(printk("Target %03d - configured by user: ",tgt);)
		switch (ha->binding_type) {
			case BIND_BY_PORT_NAME:
				DEBUG(printk("tgt-%03d="
					"%02x%02x%02x%02x%02x%02x%02x%02x\n",
					tgt,
					ppn[0], ppn[1], ppn[2], ppn[3],
					ppn[4], ppn[5], ppn[6], ppn[7]);)
				break;

			case BIND_BY_PORT_ID:
				DEBUG(printk("tgt-%03d=%06x\n",
					tgt,
					pd_id->b24);)
				break;

			case BIND_BY_NODE_NAME:
				DEBUG(printk("tgt-%03d="
					"%02x%02x%02x%02x%02x%02x%02x%02x\n",
					tgt,
					pnn[0], pnn[1], pnn[2], pnn[3],
					pnn[4], pnn[5], pnn[6], pnn[7]);)
				break;
		}
		/* look for VSA */
#if VSA
		qla2x00_get_vsa_opt_from_config(ha, tgt, dev_no);
#endif

	}

	LEAVE(__func__);
}


/*
 * qla2x00_kmem_zalloc
 * Allocate and zero out the block of memory
 */
inline void *
qla2x00_kmem_zalloc( int siz, int code, int id) 
{
	uint8_t *bp;

	if ((bp = kmalloc(siz, code)) != NULL) {
		memset(bp, 0, siz);
	}
#if QL_TRACE_MEMORY
	if (mem_trace_ptr == 1000)
		mem_trace_ptr = 0;
	mem_trace[mem_trace_ptr] = (u_long ) bp;
	mem_id[mem_trace_ptr++] = (u_long ) id;
#endif

	return ((void *)bp);
}

#if 0
/*
 * kmem_free
 * Deallocate the block of memory
 */
inline void 
kmem_free(void *ptr) 
{
#if QL_TRACE_MEMORY
	int	i;

	for (i =0; i < 1000; i++)
		if (mem_trace[i] == (unsigned long) ptr) {
			mem_trace[i]  = (unsigned long) NULL;
			break;
		}
#endif
	kfree(ptr);
}
#endif

#if defined(FC_IP_SUPPORT)
/* Include routines for supporting IP */
#include "qla_ip.c"
#endif /* FC_IP_SUPPORT */

/*
 * Declarations for failover
 */
#include "qla_cfg.c"
#include "qla_fo.c"

#if APIDEV
/****************************************************************************/
/* Create character driver "HbaApiDev" w dynamically allocated major number */
/* and create "/proc/scsi/qla2x00/HbaApiNode" as the device node associated */
/* with the major number.                                                   */
/****************************************************************************/

#define APIDEV_NODE  "HbaApiNode"
#define APIDEV_NAME  "HbaApiDev"

static int apidev_major = 0;
static struct Scsi_Host *apidev_host = 0;

static int 
apidev_open(struct inode *inode, struct file *file) 
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	DEBUG9(printk(KERN_INFO
			"%s(): open MAJOR number = %d, MINOR number = %d\n",
			__func__,
			MAJOR(inode->i_rdev), MINOR(inode->i_rdev));)
#else
	DEBUG9(printk(KERN_INFO 
			"%s(): open MAJOR number = %d, MINOR number = %d\n", 
			__func__,
			major(inode->i_rdev), minor(inode->i_rdev));)
#endif

	return 0;
}

static int 
apidev_close(struct inode *inode, struct file *file) 
{
	DEBUG9(printk(KERN_INFO
			"%s(): closed\n", __func__);)

	return 0;
}

static int 
apidev_ioctl(struct inode *inode, struct file *fp, 
		unsigned int cmd, unsigned long arg) 
{
	/* Since this var is not really used, use static type to
	 * conserve stack space.
	 */
	static Scsi_Device dummy_scsi_device;

	dummy_scsi_device.host = apidev_host;

	return (qla2x00_ioctl(&dummy_scsi_device, (int)cmd, (void*)arg));
}

static struct file_operations apidev_fops = {
	 ioctl:
		 apidev_ioctl,
	 open:
		 apidev_open,
	 release:
		 apidev_close
};

static int 
apidev_init(struct Scsi_Host *host) 
{

	if (apidev_host) {
		return 0;
	}

	apidev_major = register_chrdev(0, APIDEV_NAME, &apidev_fops);
	if (0 > apidev_major) {
		DEBUG(printk("%s(): register_chrdev rc=%d\n",
				__func__,
				apidev_major);)

		return apidev_major;
	}

	apidev_host = host;

	DEBUG(printk("%s(): Creating (%s) %s/%s major=%d\n",
			__func__,
			host->hostt->proc_name,
			host->hostt->proc_dir->name, 
			APIDEV_NODE, apidev_major);)

#ifndef __VMWARE__
           // XXX: Fix this when proc_mknod works again on main!!!
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	proc_mknod(APIDEV_NODE, 0777+S_IFCHR, host->hostt->proc_dir,
			(kdev_t)MKDEV(apidev_major, 0));
#else
	proc_mknod(APIDEV_NODE, 0777+S_IFCHR, host->hostt->proc_dir,
			(kdev_t)mk_kdev(apidev_major, 0));
#endif
#endif //__VMWARE__

	return 0;
}

static int apidev_cleanup() 
{
	if (!apidev_host)
		return 0;

	unregister_chrdev(apidev_major,APIDEV_NAME);
	remove_proc_entry(APIDEV_NODE,apidev_host->hostt->proc_dir);
	apidev_host = 0;

	return 0;
}
#endif /* APIDEV */

#if defined(QL_DEBUG_ROUTINES)
#if DEBUG_GET_FW_DUMP
#include  "x2300dbg.c"
#endif
#endif

EXPORT_NO_SYMBOLS;
