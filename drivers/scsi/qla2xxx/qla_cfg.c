/******************************************************************************
 *                  QLOGIC LINUX SOFTWARE
 *
 * QLogic ISP2x00 device driver for Linux 2.4.x
 * Copyright (C) 2003 Qlogic Corporation
 * (www.qlogic.com)
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

/*
 * QLogic ISP2x00 Multi-path LUN Support Driver
 *
 */

#include "qlfo.h"
#include "qla_cfg.h"
#include "qla_gbl.h"


#if defined(LINUX)
#include "qla_cfgln.c"
#endif


/*
 *  Local Function Prototypes.
 */

static uint32_t qla2x00_add_portname_to_mp_dev(mp_device_t *, uint8_t *);

static mp_device_t * qla2x00_allocate_mp_dev(uint8_t *, uint8_t *);
static mp_path_t * qla2x00_allocate_path(mp_host_t *, uint16_t, fc_port_t *,
    uint16_t);
static mp_path_list_t * qla2x00_allocate_path_list(void);

mp_host_t * qla2x00_cfg_find_host(scsi_qla_host_t *);
static mp_host_t * qla2x00_find_host_by_name(uint8_t *);

static mp_device_t * qla2x00_find_or_allocate_mp_dev (mp_host_t *, uint16_t,
    fc_port_t *);
static mp_path_t * qla2x00_find_or_allocate_path(mp_host_t *, mp_device_t *,
    uint16_t, fc_port_t *);

static uint32_t qla2x00_cfg_register_failover_lun(mp_device_t *,srb_t *,
    fc_lun_t *);
static uint32_t qla2x00_send_failover_notify(mp_device_t *, uint8_t,
    mp_path_t *, mp_path_t *);
static mp_path_t * qla2x00_select_next_path(mp_host_t *, mp_device_t *,
    uint8_t);

static BOOL qla2x00_update_mp_host(mp_host_t  *);
static uint32_t qla2x00_update_mp_tree (void);

static fc_lun_t *qla2x00_find_matching_lun(uint8_t , mp_path_t *);
static mp_path_t *qla2x00_find_path_by_id(mp_device_t *, uint8_t);
static mp_device_t *qla2x00_find_mp_dev_by_id(mp_host_t *, uint8_t);
static mp_device_t *qla2x00_find_mp_dev_by_name(mp_host_t *, uint8_t *);

static mp_path_t *qla2x00_get_visible_path(mp_device_t *dp);
static void qla2x00_map_os_targets(mp_host_t *);
static void qla2x00_map_os_luns(mp_host_t *, mp_device_t *, uint16_t);
static BOOL qla2x00_map_a_oslun(mp_host_t *, mp_device_t *, uint16_t, uint16_t);

static BOOL qla2x00_is_ww_name_zero(uint8_t *);
static void qla2x00_add_path(mp_path_list_t *, mp_path_t *);
static BOOL qla2x00_is_portname_in_device(mp_device_t *, uint8_t *);
static void qla2x00_failback_single_lun(mp_device_t *, uint8_t, uint8_t);
static void qla2x00_failback_luns(mp_host_t *);
static void qla2x00_setup_new_path(mp_device_t *, mp_path_t *);

/*
 * Global data items
 */
mp_host_t  *mp_hosts_base = NULL;
BOOL   mp_config_required = FALSE;
static int    mp_num_hosts = 0;
static BOOL   mp_initialized = FALSE;


/*
 * ENTRY ROUTINES
 */

/*
 * qla2x00_cfg_init
 *      Initialize configuration structures to handle an instance of
 *      an HBA, QLA2x000 card.
 *
 * Input:
 *      ha = adapter state pointer.
 *
 * Returns:
 *      qla2x00 local function return status code.
 *
 * Context:
 *      Kernel context.
 */
int
qla2x00_cfg_init(scsi_qla_host_t *ha)
{
	int	rval;

	ENTER("qla2x00_cfg_init");
	set_bit(CFG_ACTIVE, &ha->cfg_flags);
	if (!mp_initialized) {
		/* First HBA, initialize the failover global properties */
		qla2x00_fo_init_params(ha);

		/* If the user specified a device configuration then
		 * it is use as the configuration. Otherwise, we wait
		 * for path discovery.
		 */
		if ( mp_config_required )
			qla2x00_cfg_build_path_tree(ha);
	}
	rval = qla2x00_cfg_path_discovery(ha);
	clear_bit(CFG_ACTIVE, &ha->cfg_flags);
	LEAVE("qla2x00_cfg_init");
	return rval;
}

/*
 * qla2x00_cfg_path_discovery
 *      Discover the path configuration from the device configuration
 *      for the specified host adapter and build the path search tree.
 *      This function is called after the lower level driver has
 *      completed its port and lun discovery.
 *
 * Input:
 *      ha = adapter state pointer.
 *
 * Returns:
 *      qla2x00 local function return status code.
 *
 * Context:
 *      Kernel context.
 */
int
qla2x00_cfg_path_discovery(scsi_qla_host_t *ha)
{
	int		rval = QLA2X00_SUCCESS;
	mp_host_t	*host;
	uint8_t		*name;

	ENTER("qla2x00_cfg_path_discovery");

	name = 	&ha->init_cb->node_name[0];

	set_bit(CFG_ACTIVE, &ha->cfg_flags);
	/* Initialize the path tree for this adapter */
	host = qla2x00_find_host_by_name(name);
	if ( mp_config_required ) {
		if (host == NULL ) {
			DEBUG4(printk("cfg_path_discovery: host not found, "
				"node name = "
				"%02x%02x%02x%02x%02x%02x%02x%02x\n",
				name[0], name[1], name[2], name[3],
				name[4], name[5], name[6], name[7]);)
			rval = QLA2X00_FUNCTION_FAILED;
		} else if (ha->instance != host->instance) {
			DEBUG4(printk("cfg_path_discovery: host instance "
				"don't match - instance=%ld.\n",
				ha->instance);)
			rval = QLA2X00_FUNCTION_FAILED;
		}
	} else if ( host == NULL ) {
		/* New host adapter so allocate it */
		if ( (host = qla2x00_alloc_host(ha)) == NULL ) {
			printk(KERN_INFO
				"qla2x00(%d): Couldn't allocate "
				"host - ha = %p.\n",
				(int)ha->instance, ha);
			rval = QLA2X00_FUNCTION_FAILED;
		}
	}

	/* Fill in information about host */
	if (host != NULL ) {
		host->flags |= MP_HOST_FLAG_NEEDS_UPDATE;
		host->flags |= MP_HOST_FLAG_LUN_FO_ENABLED;
		host->fcport = ha->fcport;

		/* Check if multipath is enabled */
		if (!qla2x00_update_mp_host(host)) {
			rval = QLA2X00_FUNCTION_FAILED;
		}
		host->flags &= ~MP_HOST_FLAG_LUN_FO_ENABLED;
	}

	if (rval != QLA2X00_SUCCESS) {
		/* EMPTY */
		DEBUG4(printk("qla2x00_path_discovery: Exiting FAILED\n");)
	} else {
		LEAVE("qla2x00_cfg_path_discovery");
	}
	clear_bit(CFG_ACTIVE, &ha->cfg_flags);

	return rval;
}

/*
 * qla2x00_cfg_event_notifiy
 *      Callback for host driver to notify us of configuration changes.
 *
 * Input:
 *      ha = adapter state pointer.
 *      i_type = event type
 *
 * Returns:
 *
 * Context:
 *      Kernel context.
 */
int
qla2x00_cfg_event_notify(scsi_qla_host_t *ha, uint32_t i_type)
{
	mp_host_t	*host;			/* host adapter pointer */

	ENTER("qla2x00_cfg_event_notify");

	set_bit(CFG_ACTIVE, &ha->cfg_flags);
	switch (i_type) {
		case MP_NOTIFY_RESET_DETECTED:
			DEBUG(printk("scsi%ld: MP_NOTIFY_RESET_DETECTED "
					"- no action\n",
					ha->host_no);)
				break;
		case MP_NOTIFY_PWR_LOSS:
			DEBUG(printk("scsi%ld: MP_NOTIFY_PWR_LOSS - "
					"update tree\n",
					ha->host_no);)
			/*
			 * Update our path tree in case we are
			 * losing the adapter
			 */
			qla2x00_update_mp_tree();
			/* Free our resources for adapter */
			break;
		case MP_NOTIFY_LOOP_UP:
			DEBUG(printk("scsi%ld: MP_NOTIFY_LOOP_UP - "
					"update host tree\n",
					ha->host_no);)
			/* Adapter is back up with new configuration */
			if ((host = qla2x00_cfg_find_host(ha)) != NULL) {
				host->flags |= MP_HOST_FLAG_NEEDS_UPDATE;
				host->fcport = ha->fcport;
				qla2x00_update_mp_tree();
			}
			break;
		case MP_NOTIFY_LOOP_DOWN:
		case MP_NOTIFY_BUS_RESET:
			DEBUG(printk("scsi%ld: MP_NOTIFY_OTHERS - "
					"no action\n",
					ha->host_no);)
			break;
		default:
			break;

	}
	clear_bit(CFG_ACTIVE, &ha->cfg_flags);

	LEAVE("qla2x00_cfg_event_notify");

	return QLA2X00_SUCCESS;
}

/*
 * qla2x00_cfg_failover
 *      A problem has been detected with the current path for this
 *      lun.  Select the next available path as the current path
 *      for this device.
 *
 * Inputs:
 *      ha = pointer to host adapter
 *      fp - pointer to failed fc_lun (failback lun)
 *      tgt - pointer to target
 *
 * Returns:
 *      pointer to new fc_lun_t, or NULL if failover fails.
 */
fc_lun_t *
qla2x00_cfg_failover(scsi_qla_host_t *ha, fc_lun_t *fp,
    os_tgt_t *tgt, srb_t *sp)
{
	mp_host_t	*host;			/* host adapter pointer */
	mp_device_t	*dp;			/* virtual device pointer */
	mp_path_t	*new_path;		/* new path pointer */
	fc_lun_t	*new_fp = NULL;

	ENTER("qla2x00_cfg_failover");
	set_bit(CFG_ACTIVE, &ha->cfg_flags);
	if ((host = qla2x00_cfg_find_host(ha)) != NULL) {
		if ((dp = qla2x00_find_mp_dev_by_name(
					host, tgt->node_name)) != NULL ) {

			DEBUG3(printk("qla2x00_cfg_failover: dp = %p\n", dp);)
			/*
			 * Point at the next path in the path list if there is
			 * one, and if it hasn't already been failed over by
			 * another I/O. If there is only one path continuer
			 * to point at it.
			 */
			new_path = qla2x00_select_next_path(host, dp, fp->lun);
			DEBUG3(printk("cfg_failover: new path @ %p\n",
						new_path);)
			new_fp = qla2x00_find_matching_lun(fp->lun, new_path);
			DEBUG3(printk("cfg_failover: new fp lun @ %p\n",
						new_fp);)

			qla2x00_cfg_register_failover_lun(dp, sp, new_fp);
		} else {
			printk(KERN_INFO
				"qla2x00(%d): Couldn't find device "
				"to failover\n",
				host->instance);
		}
	}
	clear_bit(CFG_ACTIVE, &ha->cfg_flags);

	LEAVE("qla2x00_cfg_failover");

	return new_fp;
}

/*
 * IOCTL support
 */
#define CFG_IOCTL
#if defined(CFG_IOCTL)
/*
 * qla2x00_cfg_get_paths
 *      Get list of paths EXT_FO_GET_PATHS.
 *
 * Input:
 *      ha = pointer to adapter
 *      bp = pointer to buffer
 *      cmd = Pointer to kernel copy of EXT_IOCTL.
 *
 * Return;
 *      0 on success or errno.
 *	driver ioctl errors are returned via cmd->Status.
 *
 * Context:
 *      Kernel context.
 */
int
qla2x00_cfg_get_paths(EXT_IOCTL *cmd, FO_GET_PATHS *bp, int mode)
{
	FO_PATHS_INFO	*paths,	*u_paths;
	FO_PATH_ENTRY	*entry;
	EXT_DEST_ADDR   *sap = &bp->HbaAddr;
	mp_host_t	*host = NULL;	/* host adapter pointer */
	mp_device_t	*dp;		/* virtual device pointer */
	mp_path_t	*path;		/* path pointer */
	mp_path_list_t	*path_list;	/* path list pointer */
	int	cnt;
	int	rval = 0;
	scsi_qla_host_t *ha;

	DEBUG9(printk("%s: entered.\n", __func__);)

	u_paths = (FO_PATHS_INFO *) cmd->ResponseAdr;
	ha = qla2x00_get_hba((int)bp->HbaInstance);

	if (!ha) {
		DEBUG2_9_10(printk("%s: no ha matching inst %d.\n",
		    __func__, bp->HbaInstance);)

		cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (rval);
	}

	if (ha->flags.failover_enabled)
		if ((host = qla2x00_cfg_find_host(ha)) == NULL) {
			cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
			cmd->DetailStatus = EXT_DSTATUS_HBA_INST;
			DEBUG4(printk("%s: cannot find target (%ld)\n",
			    __func__, ha->instance);)
			DEBUG9_10(printk("%s: cannot find host inst(%ld).\n",
			    __func__, ha->instance);)

			return rval;
		}

	if ((paths = (FO_PATHS_INFO *)qla2x00_kmem_zalloc(sizeof(FO_PATHS_INFO),
	    GFP_ATOMIC,20)) == NULL) {

		DEBUG4(printk("%s: failed to allocate memory of size (%d)\n",
		    __func__, (int)sizeof(FO_PATHS_INFO));)
		DEBUG9_10(printk("%s: failed allocate memory size(%d).\n",
		    __func__, (int)sizeof(FO_PATHS_INFO));)

		cmd->Status = EXT_STATUS_NO_MEMORY;

		return -ENOMEM;
	}

	if (!ha->flags.failover_enabled) {
		/* non-fo case. There's only one path. */

		mp_path_list_t	*ptmp_plist;
#define STD_MAX_PATH_CNT	1
#define STD_VISIBLE_INDEX	0
		fc_port_t		*pfcport = NULL;

		DEBUG9(printk("%s: non-fo case.\n", __func__);)

		if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_plist,
		    sizeof(mp_path_list_t))) {
			/* not enough memory */
			DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
			    "lun_mask requested=%d.\n",
			    __func__, ha->host_no, ha->instance,
			    sizeof(mp_path_list_t));)
			cmd->Status = EXT_STATUS_NO_MEMORY;

			return -ENOMEM;
		}

		for (pfcport = ha->fcport; pfcport != NULL;
		    pfcport = pfcport->next) {

			if (memcmp(pfcport->node_name, sap->DestAddr.WWNN,
			    EXT_DEF_WWN_NAME_SIZE) == 0) {
				break;
			}
		}

		if (pfcport) {
			DEBUG9(printk("%s: found fcport:"
			    "(%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x)\n.",
			    __func__,
			    sap->DestAddr.WWNN[0],
			    sap->DestAddr.WWNN[1],
			    sap->DestAddr.WWNN[2],
			    sap->DestAddr.WWNN[3],
			    sap->DestAddr.WWNN[4],
			    sap->DestAddr.WWNN[5],
			    sap->DestAddr.WWNN[6],
			    sap->DestAddr.WWNN[7]);)

			paths->HbaInstance         = bp->HbaInstance;
			paths->PathCount           = STD_MAX_PATH_CNT;
			paths->VisiblePathIndex    = STD_VISIBLE_INDEX;

			/* Copy current path, which is the first one (0). */
			memcpy(paths->CurrentPathIndex, ptmp_plist->current_path,
			    sizeof(paths->CurrentPathIndex));

			entry = &(paths->PathEntry[STD_VISIBLE_INDEX]);

			entry->Visible     = TRUE;
			entry->HbaInstance = bp->HbaInstance;

			memcpy(entry->PortName, pfcport->port_name,
			    EXT_DEF_WWP_NAME_SIZE);

			rval = verify_area(VERIFY_WRITE, (void *)u_paths,
			    cmd->ResponseLen);
			if (rval) {
				/* error */
				DEBUG9_10(printk("%s: u_paths %p verify write"
				    " error. paths->PathCount=%d.\n",
				    __func__, u_paths, paths->PathCount);)
			}

			/* Copy data to user */
			if (rval == 0)
			 	rval = copy_to_user(&u_paths->PathCount,
			 	    &paths->PathCount, 4);
			if (rval == 0)
				rval = copy_to_user(&u_paths->CurrentPathIndex,
				    &paths->CurrentPathIndex,
				    sizeof(paths->CurrentPathIndex));
			if (rval == 0)
				rval = copy_to_user(&u_paths->PathEntry,
				    &paths->PathEntry,
				    sizeof(paths->PathEntry));

			if (rval) { /* if any of the above failed */
				DEBUG9_10(printk("%s: data copy failed.\n",
				    __func__);)

				cmd->Status = EXT_STATUS_COPY_ERR;
			}
		} else {
			cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
			cmd->DetailStatus = EXT_DSTATUS_TARGET;

			DEBUG10(printk("%s: cannot find fcport "
			    "(%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x)\n.",
			    __func__,
			    sap->DestAddr.WWNN[0],
			    sap->DestAddr.WWNN[1],
			    sap->DestAddr.WWNN[2],
			    sap->DestAddr.WWNN[3],
			    sap->DestAddr.WWNN[4],
			    sap->DestAddr.WWNN[5],
			    sap->DestAddr.WWNN[6],
			    sap->DestAddr.WWNN[7]);)
			DEBUG4(printk("%s: cannot find fcport "
			    "(%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x)\n.",
			    __func__,
			    sap->DestAddr.WWNN[0],
			    sap->DestAddr.WWNN[1],
			    sap->DestAddr.WWNN[2],
			    sap->DestAddr.WWNN[3],
			    sap->DestAddr.WWNN[4],
			    sap->DestAddr.WWNN[5],
			    sap->DestAddr.WWNN[6],
			    sap->DestAddr.WWNN[7]);)
		}

		qla2x00_free_ioctl_scrap_mem(ha);
		/* end of non-fo case. */

	} else if (sap->DestType != EXT_DEF_DESTTYPE_WWNN) {
		/* Scan for mp_dev by nodename *ONLY* */

		cmd->Status = EXT_STATUS_INVALID_PARAM;
		cmd->DetailStatus = EXT_DSTATUS_TARGET;

		DEBUG4(printk("%s: target can be accessed by NodeName only.",
		    __func__);)
		DEBUG10(printk("%s: target can be accessed by NodeName only. "
		    "got type %d.\n",
		    __func__, sap->DestType);)

	} else if ((dp = qla2x00_find_mp_dev_by_name(host,
	    sap->DestAddr.WWNN)) != NULL) {
		DEBUG9(printk("%s: Found mp_dev:"
		    "(%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x)\n.",
		    __func__,
		    sap->DestAddr.WWNN[0],
		    sap->DestAddr.WWNN[1],
		    sap->DestAddr.WWNN[2],
		    sap->DestAddr.WWNN[3],
		    sap->DestAddr.WWNN[4],
		    sap->DestAddr.WWNN[5],
		    sap->DestAddr.WWNN[6],
		    sap->DestAddr.WWNN[7]);)

		path_list = dp->path_list;

		paths->HbaInstance = bp->HbaInstance;
		paths->PathCount           = path_list->path_cnt;
		paths->VisiblePathIndex    = path_list->visible;

		/* copy current paths */
		memcpy(paths->CurrentPathIndex,
		    path_list->current_path,
		    sizeof(paths->CurrentPathIndex));

		path = path_list->last;
		for (cnt = 0; cnt < path_list->path_cnt; cnt++) {
			entry = &(paths->PathEntry[path->id]);

			entry->Visible    = (path->id == path_list->visible);
			entry->HbaInstance = path->host->instance;

			memcpy(entry->PortName,
			    path->portname,
			    EXT_DEF_WWP_NAME_SIZE);

			path = path->next;
		}

		rval = verify_area(VERIFY_WRITE, (void *)u_paths,
		    cmd->ResponseLen);
		if (rval) {
			/* error */
			DEBUG9_10(printk("%s: u_paths %p verify write"
			    " error. paths->PathCount=%d.\n",
			    __func__, u_paths, paths->PathCount);)
		}

		/* copy data to user */
		if (rval == 0)
			rval = copy_to_user(&u_paths->PathCount,
			    &paths->PathCount, 4);
		if (rval == 0)
			rval = copy_to_user(&u_paths->CurrentPathIndex,
			    &paths->CurrentPathIndex,
			    sizeof(paths->CurrentPathIndex));
		if (rval == 0)
			rval = copy_to_user(&u_paths->PathEntry,
			    &paths->PathEntry,
			    sizeof(paths->PathEntry));

		if (rval != 0) {  /* if any of the above failed */
			DEBUG9_10(printk("%s: u_paths %p copy"
			    " error. paths->PathCount=%d.\n",
			    __func__, u_paths, paths->PathCount);)
			cmd->Status = EXT_STATUS_COPY_ERR;
		}

	} else {

		cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
		cmd->DetailStatus = EXT_DSTATUS_TARGET;

		DEBUG4(printk("%s: cannot find device "
		    "(%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x)\n.",
		    __func__,
		    sap->DestAddr.WWNN[0],
		    sap->DestAddr.WWNN[1],
		    sap->DestAddr.WWNN[2],
		    sap->DestAddr.WWNN[3],
		    sap->DestAddr.WWNN[4],
		    sap->DestAddr.WWNN[5],
		    sap->DestAddr.WWNN[6],
		    sap->DestAddr.WWNN[7]);)
	}

	KMEM_FREE(paths, sizeof(FO_PATHS_INFO));

	DEBUG9(printk("%s: exiting. rval=%d.\n", __func__, rval);)

	return rval;

} /* qla2x00_cfg_get_paths */

/*
 * qla2x00_cfg_set_current_path
 *      Set the current failover path EXT_FO_GET_PATHS IOCTL call.
 *
 * Input:
 *      ha = pointer to adapter
 *      bp = pointer to buffer
 *      cmd = Pointer to kernel copy of EXT_IOCTL.
 *
 * Return;
 *      0 on success or errno.
 *
 * Context:
 *      Kernel context.
 */
int
qla2x00_cfg_set_current_path(EXT_IOCTL *cmd, FO_SET_CURRENT_PATH *bp, int mode )
{
	uint8_t         orig_id, new_id;
	mp_host_t       *host, *new_host;
	mp_device_t     *dp;
	mp_path_list_t  *path_list;
	EXT_DEST_ADDR   *sap = &bp->HbaAddr;
	uint32_t        rval = 0;
	scsi_qla_host_t *ha;
	mp_path_t       *new_path, *old_path;

	DEBUG9(printk("%s: entered.\n", __func__);)

	/* First find the adapter with the instance number. */
	ha = qla2x00_get_hba((int)bp->HbaInstance);
	if (!ha) {
		DEBUG2_9_10(printk("%s: no ha matching inst %d.\n",
		    __func__, bp->HbaInstance);)

		cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (rval);
	}

	if (!ha->flags.failover_enabled)
		/* non-failover mode. nothing to be done. */
		return 0;

	if ((host = qla2x00_cfg_find_host(ha)) == NULL) {
		cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
		cmd->DetailStatus = EXT_DSTATUS_HBA_INST;
		DEBUG4(printk("%s: cannot find adapter.\n",
		    __func__);)
		return (rval);
	}

	set_bit(CFG_ACTIVE, &ha->cfg_flags);
	sap = &bp->HbaAddr;
	/* Scan for mp_dev by nodename *ONLY* */
	if (sap->DestType != EXT_DEF_DESTTYPE_WWNN) {
		cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
		cmd->DetailStatus = EXT_DSTATUS_TARGET;
		DEBUG4(printk("%s: target can be accessed by NodeName only.",
		    __func__);)
		DEBUG9_10(printk("%s: target can be accessed by NodeName only.",
		    __func__);)
	} else if ((dp = qla2x00_find_mp_dev_by_name(
	    host, sap->DestAddr.WWNN)) != NULL) {

		path_list = dp->path_list;

		if (bp->NewCurrentPathIndex < MAX_PATHS_PER_DEVICE &&
		    sap->Lun < MAX_LUNS &&
		    bp->NewCurrentPathIndex < path_list->path_cnt) {

			orig_id = path_list->current_path[sap->Lun];

			DEBUG(printk("%s: dev no  %d, lun %d, "
			    "newindex %d, oldindex %d "
			    "nn=%02x%02x%02x%02x%02x%02x%02x%02x\n",
			    __func__, dp->dev_id, sap->Lun,
			    bp->NewCurrentPathIndex, orig_id,
			    host->nodename[0], host->nodename[1],
			    host->nodename[2], host->nodename[3],
			    host->nodename[4], host->nodename[5],
			    host->nodename[6], host->nodename[7]);)

			if (bp->NewCurrentPathIndex != orig_id) {
				/* Acquire the update spinlock. */

				/* Set the new current path. */
				new_id = path_list-> current_path[sap->Lun] =
				    bp->NewCurrentPathIndex;

				/* Release the update spinlock. */
				old_path = qla2x00_find_path_by_id(
				    dp, orig_id);
				new_path = qla2x00_find_path_by_id(dp, new_id);
				new_host = new_path->host;

				/* remap the lun */
				qla2x00_map_a_oslun(new_host, dp,
				    dp->dev_id, sap->Lun);

				qla2x00_send_failover_notify(dp,
				    sap->Lun, old_path, new_path);
			} else {
				/* EMPTY */
				DEBUG4(printk("%s: path index not changed.\n",
				    __func__);)
			}
		} else {
			cmd->Status = EXT_STATUS_INVALID_PARAM;
			cmd->DetailStatus = EXT_DSTATUS_PATH_INDEX;
			DEBUG4(printk("%s: invalid index for device.\n",
			    __func__);)
			DEBUG9_10(printk("%s: invalid index for device.\n",
			    __func__);)
		}
	} else {
		cmd->Status = EXT_STATUS_DEV_NOT_FOUND;
		cmd->DetailStatus = EXT_DSTATUS_TARGET;
		DEBUG4(printk("%s: cannot find device.\n",
		    __func__);)
		DEBUG9_10(printk("%s: cannot find device.\n",
		    __func__);)
	}
	clear_bit(CFG_ACTIVE, &ha->cfg_flags);

	DEBUG9(printk("%s: exiting. rval = %d.\n", __func__, rval);)

	return rval;
}
#endif

/*
 * MP SUPPORT ROUTINES
 */

/*
 * qla2x00_add_mp_host
 *	Add the specified host the host list.
 *
 * Input:
 *	node_name = pointer to node name
 *
 * Returns:
 *
 * Context:
 *	Kernel context.
 */
mp_host_t *
qla2x00_add_mp_host(uint8_t *node_name)
{
	mp_host_t   *host, *temp;

	host = (mp_host_t *) KMEM_ZALLOC(sizeof(mp_host_t), 1);
	if (host != NULL) {
		memcpy(host->nodename, node_name, WWN_SIZE);
		host->next = NULL;
		/* add to list */
		if (mp_hosts_base == NULL) {
			mp_hosts_base = host;
		} else {
			temp = mp_hosts_base;
			while (temp->next != NULL)
				temp = temp->next;
			temp->next = host;
		}
		mp_num_hosts++;
	}
	return host;
}

/*
 * qla2x00_alloc_host
 *      Allocate and initialize an mp host structure.
 *
 * Input:
 *      ha = pointer to base driver's adapter structure.
 *
 * Returns:
 *      Pointer to host structure or null on error.
 *
 * Context:
 *      Kernel context.
 */
mp_host_t   *
qla2x00_alloc_host(scsi_qla_host_t *ha)
{
	mp_host_t	*host, *temp;
	uint8_t		*name, *portname;

	name = 	&ha->init_cb->node_name[0];
	portname = &ha->init_cb->port_name[0];

	ENTER("qla2x00_alloc_host");

	host = (mp_host_t *) KMEM_ZALLOC(sizeof(mp_host_t), 2);

	if (host != NULL) {
		host->ha = ha;
		memcpy(host->nodename, name, WWN_SIZE);
		memcpy(host->portname, portname, WWN_SIZE);
		host->next = NULL;
		host->flags = MP_HOST_FLAG_NEEDS_UPDATE;
		host->instance = ha->instance;
		/* host->MaxLunsPerTarget = qla_fo_params.MaxLunsPerTarget; */

		if (qla2x00_fo_enabled(host->ha, host->instance)) {
			host->flags |= MP_HOST_FLAG_FO_ENABLED;
			DEBUG4(printk("%s: Failover enabled.\n",
			    __func__);)
		} else {
			/* EMPTY */
			DEBUG4(printk("%s: Failover disabled.\n",
			    __func__);)
		}
		/* add to list */
		if (mp_hosts_base == NULL) {
			mp_hosts_base = host;
		} else {
			temp = mp_hosts_base;
			while (temp->next != NULL)
				temp = temp->next;
			temp->next = host;
		}
		mp_num_hosts++;

		DEBUG4(printk("%s: Alloc host @ %p\n", __func__, host);)
	} else {
		/* EMPTY */
		DEBUG4(printk("%s: Failed\n", __func__);)
	}

	return host;
}

/*
 * qla2x00_add_portname_to_mp_dev
 *      Add the specific port name to the list of port names for a
 *      multi-path device.
 *
 * Input:
 *      dp = pointer ti virtual device
 *      portname = Port name to add to device
 *
 * Returns:
 *      qla2x00 local function return status code.
 *
 * Context:
 *      Kernel context.
 */
static uint32_t
qla2x00_add_portname_to_mp_dev(mp_device_t *dp, uint8_t *portname)
{
	uint8_t		index;
	uint32_t	rval = QLA2X00_SUCCESS;

	ENTER("qla2x00_add_portname_to_mp_dev");

	/* Look for an empty slot and add the specified portname.   */
	for (index = 0; index < MAX_NUMBER_PATHS; index++) {
		if (qla2x00_is_ww_name_zero(&dp->portnames[index][0])) {
			DEBUG4(printk("%s: adding portname to dp = "
			    "%p at index = %d\n",
			    __func__, dp, index);)
			memcpy(&dp->portnames[index][0], portname, WWN_SIZE);
			break;
		}
	}
	if (index == MAX_NUMBER_PATHS) {
		rval = QLA2X00_FUNCTION_FAILED;
		DEBUG4(printk("%s: Fail no room\n", __func__);)
	} else {
		/* EMPTY */
		DEBUG4(printk("%s: Exit OK\n", __func__);)
	}

	LEAVE("qla2x00_add_portname_to_mp_dev");

	return rval;
}


/*
 *  qla2x00_allocate_mp_dev
 *      Allocate an fc_mp_dev, clear the memory, and log a system
 *      error if the allocation fails. After fc_mp_dev is allocated
 *
 *  Inputs:
 *      nodename  = pointer to nodename of new device
 *      portname  = pointer to portname of new device
 *
 *  Returns:
 *      Pointer to new mp_device_t, or NULL if the allocation fails.
 *
 * Context:
 *      Kernel context.
 */
static mp_device_t *
qla2x00_allocate_mp_dev(uint8_t  *nodename, uint8_t *portname)
{
	mp_device_t   *dp;            /* Virtual device pointer */

	ENTER("qla2x00_allocate_mp_dev");

	dp = (mp_device_t *)KMEM_ZALLOC(sizeof(mp_device_t), 3);

	if (dp != NULL) {
		DEBUG3(printk("%s: mp_device_t allocated at %p\n",
		    __func__, dp);)

		/*
		 * Copy node name into the mp_device_t.
		 */
		if (nodename)
			memcpy(dp->nodename, nodename, WWN_SIZE);

		/*
		 * Since this is the first port, it goes at
		 * index zero.
		 */
		if (portname)
			memcpy(&dp->portnames[0][0], portname, PORT_NAME_SIZE);

		/* Allocate an PATH_LIST for the fc_mp_dev. */
		if ((dp->path_list = qla2x00_allocate_path_list()) == NULL) {
			DEBUG4(printk("%s: allocate path_list Failed.\n",
			    __func__);)
			KMEM_FREE(dp, sizeof(mp_device_t));
			dp = NULL;
		} else {
			DEBUG4(printk("%s: mp_path_list_t allocated at %p\n",
			    __func__, dp->path_list);)
			/* EMPTY */
			DEBUG4(printk("qla2x00_allocate_mp_dev: Exit Okay\n");)
		}
	} else {
		/* EMPTY */
		DEBUG4(printk("%s: Allocate failed.\n", __func__);)
	}

	LEAVE("qla2x00_allocate_mp_dev");

	return dp;
}

/*
 *  qla2x00_allocate_path
 *      Allocate a PATH.
 *
 *  Inputs:
 *     host   Host adapter for the device.
 *     path_id  path number
 *     port   port for device.
 *      dev_id  device number
 *
 *  Returns:
 *      Pointer to new PATH, or NULL if the allocation failed.
 *
 * Context:
 *      Kernel context.
 */
static mp_path_t *
qla2x00_allocate_path(mp_host_t *host, uint16_t path_id,
    fc_port_t *port, uint16_t dev_id)
{
	mp_path_t	*path;
	uint16_t	lun;

	ENTER("qla2x00_allocate_path");

	path = (mp_path_t *) KMEM_ZALLOC(sizeof(mp_path_t), 4);
	if (path != NULL) {
		DEBUG3(printk("%s: mp_path_t allocated at %p\n",
		    __func__, path); )

		/* Copy the supplied information into the MP_PATH.  */
		path->host = host;
		if (!(port->flags & FC_CONFIG) ||
			port->loop_id != FC_NO_LOOP_ID) {

			path->port = port;
		}
		path->id   = path_id;
		port->cur_path = path->id;
		path->mp_byte  = port->mp_byte;
		path->next  = NULL;
		memcpy(path->portname, port->port_name, WWN_SIZE);
		for (lun = 0; lun < MAX_LUNS; lun++) {
			path->lun_data.data[lun] |= LUN_DATA_ENABLED;
		}
	} else {
		/* EMPTY */
		DEBUG4(printk("%s: Failed\n", __func__);)
	}

	return path;
}


/*
 *  qla2x00_allocate_path_list
 *      Allocate a PATH_LIST
 *
 *  Input:
 * 		None
 *
 *  Returns:
 *      Pointer to new PATH_LIST, or NULL if the allocation fails.
 *
 * Context:
 *      Kernel context.
 */
static mp_path_list_t *
qla2x00_allocate_path_list( void )
{
	mp_path_list_t	*path_list;
	uint16_t		i;
	uint8_t			l;

	path_list = (mp_path_list_t *) KMEM_ZALLOC(sizeof(mp_path_list_t), 5);

	if (path_list != NULL) {
		DEBUG4(printk("%s: allocated at %p\n",
		    __func__, path_list);)

		path_list->visible = PATH_INDEX_INVALID;
		/* Initialized current path */
		for (i = 0; i < MAX_LUNS_PER_DEVICE; i++) {
			l = (uint8_t)(i & 0xFF);
			path_list->current_path[l] = PATH_INDEX_INVALID;
		}
		path_list->last = NULL;

	} else {
		/* EMPTY */
		DEBUG4(printk("%s: Alloc pool failed for MP_PATH_LIST.\n",
		    __func__);)
	}

	return path_list;
}

/*
 *  qla2x00_cfg_find_host
 *      Look through the existing multipath tree, and find
 *      a host adapter to match the specified ha.
 *
 *  Input:
 *      ha = pointer to host adapter
 *
 *  Return:
 *      Pointer to new host, or NULL if no match found.
 *
 * Context:
 *      Kernel context.
 */
mp_host_t *
qla2x00_cfg_find_host(scsi_qla_host_t *ha)
{
	mp_host_t     *host = NULL;	/* Host found and null if not */
	mp_host_t     *tmp_host;

	ENTER("qla2x00_cfg_find_host");

	for (tmp_host = mp_hosts_base; (tmp_host); tmp_host = tmp_host->next) {
		if (tmp_host->ha == ha) {
			host = tmp_host;
			DEBUG3(printk("%s: Found host =%p, instance %d\n",
			    __func__, host, host->instance);)
			break;
		}
	}

	LEAVE("qla2x00_cfg_find_host");

	return host;
}

/*
 *  qla2x00_find_host_by_name
 *      Look through the existing multipath tree, and find
 *      a host adapter to match the specified name.
 *
 *  Input:
 *      name = node name to match.
 *
 *  Return:
 *      Pointer to new host, or NULL if no match found.
 *
 * Context:
 *      Kernel context.
 */
mp_host_t *
qla2x00_find_host_by_name(uint8_t   *name)
{
	mp_host_t     *host;		/* Host found and null if not */

	for (host = mp_hosts_base; (host); host = host->next) {
		if (memcmp(host->nodename, name, WWN_SIZE) == 0)
			break;
	}
	return host;
}


/*
 *  qla2x00_find_or_allocate_mp_dev
 *      Look through the existing multipath control tree, and find
 *      an mp_device_t with the supplied world-wide node name.  If
 *      one cannot be found, allocate one.
 *
 *  Input:
 *      host      Adapter to add device to.
 *      dev_id    Index of device on adapter.
 *      port      port database information.
 *
 *  Returns:
 *      Pointer to new mp_device_t, or NULL if the allocation fails.
 *
 *  Side Effects:
 *      If the MP HOST does not already point to the mp_device_t,
 *      a pointer is added at the proper port offset.
 *
 * Context:
 *      Kernel context.
 */
static mp_device_t *
qla2x00_find_or_allocate_mp_dev(mp_host_t *host, uint16_t dev_id,
    fc_port_t *port)
{
	mp_device_t	*dp = NULL;  /* pointer to multi-path device   */
	BOOL		node_found;  /* Found matching node name. */
	BOOL		port_found;  /* Found matching port name. */
	BOOL		names_valid; /* Node name and port name are not zero */ 
	mp_host_t	*temp_host;  /* pointer to temporary host */

	uint16_t	j;
	mp_device_t	*temp_dp;

	ENTER("qla2x00_find_or_allocate_mp_dev");

	DEBUG3(printk("%s: host =%p, port =%p, id = %d\n",
	    __func__, host, port, dev_id);)

	temp_dp = qla2x00_find_mp_dev_by_id(host,dev_id);

	DEBUG3(printk("%s: temp dp =%p\n", __func__, temp_dp);)
	/* if Device already known at this port. */
	if (temp_dp != NULL) {
		node_found = qla2x00_is_nodename_equal(temp_dp->nodename,
					port->node_name);
		port_found = qla2x00_is_portname_in_device(temp_dp,
					port->port_name);

		if (node_found && port_found) {
			DEBUG3(printk("%s: port exists in device %p\n",
			    __func__, temp_dp);)
			dp = temp_dp;

			/*
			 * Copy the LUN configuration data
			 * into the mp_device_t.
			 */
		}
	}


	/* Sanity check the port information  */
	names_valid = (!qla2x00_is_ww_name_zero(port->node_name) &&
			!qla2x00_is_ww_name_zero(port->port_name));

	/*
	 * If the optimized check failed, loop through each known
	 * device on each known adapter looking for the node name.
	 */
	if (dp == NULL && names_valid) {
		DEBUG3(printk("%s: Searching each adapter for the device...\n",
		    __func__);)

		for (temp_host = mp_hosts_base; (temp_host);
		    temp_host = temp_host->next) {

			/* Loop through each potential device on adapter. */
			for (j = 0; j < MAX_MP_DEVICES; j++) {
				temp_dp = temp_host->mp_devs[j];

				if (temp_dp == NULL)
					continue;

				node_found = qla2x00_is_nodename_equal(
						temp_dp->nodename,
						port->node_name);
				port_found = qla2x00_is_portname_in_device(
						temp_dp,
						port->port_name);

				if (node_found || port_found) {
					DEBUG3(printk("%s: Matching device "
					    "found at %p @ %d\n",
					    __func__, temp_dp, j);)
					/*
					 * If the node name matches but
					 * the port name was not found,
					 * add the port name to the list
					 * of port names.
					 */
					if (!port_found) {
						qla2x00_add_portname_to_mp_dev(
								temp_dp,
								port->port_name);
					}

					/*
					 * Set the flag that we have
					 * found the device.
					 */
					dp = temp_dp;
					host->mp_devs[j] = dp;
					dp->use_cnt++;

					/* Fixme(dg)
					 * Copy the LUN info into
					 * the mp_device_t
					 */
					break;
				}
			}

			/* Break outer loop if inner loop succeeded. */
			if (dp != NULL)
				break;
		}

	}

	/* If we couldn't find one, allocate one. */
	if (dp == NULL &&
		((port->flags & FC_CONFIG) || !mp_config_required ) ) {

		dp = qla2x00_allocate_mp_dev(port->node_name, port->port_name);
		host->mp_devs[dev_id] = dp;
		dp->dev_id = dev_id;
		dp->use_cnt++;
	}

	LEAVE("qla2x00_allocate_mp_dev");

	return dp;
}


/*
 *  qla2x00_find_or_allocate_path
 *      Look through the path list for the supplied device, and either
 *      find the supplied adapter (path) for the adapter, or create
 *      a new one and add it to the path list.
 *
 *  Input:
 *      host      Adapter (path) for the device.
 *      dp       Device and path list for the device.
 *      dev_id    Index of device on adapter.
 *      port     Device data from port database.
 *
 *  Returns:
 *      Pointer to new PATH, or NULL if the allocation fails.
 *
 *  Side Effects:
 *      1. If the PATH_LIST does not already point to the PATH,
 *         a new PATH is added to the PATH_LIST.
 *      2. If the new path is found to be a second visible path, it is
 *         marked as hidden, and the device database is updated to be
 *         hidden as well, to keep the miniport synchronized.
 *
 * Context:
 *      Kernel context.
 */
/* ARGSUSED */
static mp_path_t *
qla2x00_find_or_allocate_path(mp_host_t *host, mp_device_t *dp,
    uint16_t dev_id, fc_port_t *port)
{
	mp_path_list_t	*path_list = dp->path_list;
	mp_path_t		*path;
	uint8_t			id;


	ENTER("qla2x00_find_or_allocate_path");

	DEBUG4(printk("%s: host =%p, port =%p, dp=%p, dev id = %d\n",
	    __func__, host, port, dp, dev_id);)
	/*
	 * Loop through each known path in the path list.  Look for
	 * a PATH that matches both the adapter and the port name.
	 */
	path = qla2x00_find_path_by_name(host, path_list, port->port_name);


	if (path != NULL ) {
		DEBUG3(printk("%s: Found an existing "
		    "path -  host =%p, port =%p, path id = %d\n",
		    __func__, host, path->port, path->id);)
		DEBUG3(printk("%s: Luns for path_id %d, instance %d\n",
		    __func__, path->id, host->instance);)
		DEBUG3(qla2x00_dump_buffer(
		    (char *)&path->lun_data.data[0], 64);)

		/* If we found an existing path, look for any changes to it. */
		if (path->port == NULL) {
			DEBUG3(printk("%s: update path %p, path id= %d, "
			    "mp_byte=0x%x port=%p\n",
			    __func__, path, path->id,
			    path->mp_byte, path->port);)
			path->port = port;
			port->mp_byte = path->mp_byte;
		} else {
			if ((path->mp_byte & MP_MASK_HIDDEN) &&
				!(port->mp_byte & MP_MASK_HIDDEN)) {

				DEBUG3(printk("%s: Adapter(%p) "
				    "Device (%p) Path (%d) "
				    "has become visible.\n",
				    __func__, host, dp, path->id);)

				path->mp_byte &= ~MP_MASK_HIDDEN;
			}

			if (!(path->mp_byte & MP_MASK_HIDDEN) &&
					(port->mp_byte & MP_MASK_HIDDEN)) {

				DEBUG3(printk("%s: Adapter(%p) "
				    "Device (%p) Path (%d) "
				    "has become hidden.\n",
				    __func__, host, dp, path->id);)

				path->mp_byte |= MP_MASK_HIDDEN;
			}
		}

	} else {
		/*
		 * If we couldn't find an existing path, and there is still
		 * room to add one, allocate one and put it in the list.
		 */
		if (path_list->path_cnt < MAX_PATHS_PER_DEVICE &&
			path_list->path_cnt < qla_fo_params.MaxPathsPerDevice) {

			id = path_list->path_cnt;

			/* Update port with bitmask info */
			path = qla2x00_allocate_path(host, id, port, dev_id);
			DEBUG3(printk("%s: new path %p, path id= %d, "
			    "mp_byte=0x%x port=%p\n",
			    __func__, path, id,
			    path->mp_byte, path->port);)
			qla2x00_add_path(path_list, path);

			/* Reconcile the new path against the existing ones. */
			qla2x00_setup_new_path(dp, path);
		} else {
			/* EMPTY */
			DEBUG4(printk("%s: Err exit, no space to add path.\n",
			    __func__);)
		}

	}

	LEAVE("qla2x00_find_or_allocate_path");

	return path;
}

static uint32_t
qla2x00_cfg_register_failover_lun(mp_device_t *dp, srb_t *sp, fc_lun_t *new_lp)
{
	uint32_t	status = QLA2X00_SUCCESS;
	os_tgt_t	*tq;
	os_lun_t	*lq;
	fc_lun_t 	*old_lp;

	DEBUG2(printk("%s: NEW fclun = %p, sp = %p\n",
	    __func__, new_lp, sp);)

	/*
	 * Fix lun descriptors to point to new fclun which is a new fcport.
	 */
	if (new_lp == NULL) {
		DEBUG2(printk("%s: Failed new lun %p\n",
		    __func__, new_lp);)
		return QLA2X00_FUNCTION_FAILED;
	}

	tq = sp->tgt_queue;
	lq = sp->lun_queue;
	if (tq == NULL) {
		DEBUG2(printk("%s: Failed to get old tq %p\n",
		    __func__, tq);)
		return QLA2X00_FUNCTION_FAILED;
	}
	if (lq == NULL) {
		DEBUG2(printk("%s: Failed to get old lq %p\n",
		    __func__, lq);)
		return QLA2X00_FUNCTION_FAILED;
	}
	old_lp = lq->fclun;
	lq->fclun = new_lp;

	/* Log the failover to console */
	printk(KERN_INFO
		"qla2x00: FAILOVER device %d from "
		"%02x%02x%02x%02x%02x%02x%02x%02x -> "
		"%02x%02x%02x%02x%02x%02x%02x%02x - "
		"LUN %02x, reason=0x%x\n",
		dp->dev_id,
		old_lp->fcport->port_name[0], old_lp->fcport->port_name[1],
		old_lp->fcport->port_name[2], old_lp->fcport->port_name[3],
		old_lp->fcport->port_name[4], old_lp->fcport->port_name[5],
		old_lp->fcport->port_name[6], old_lp->fcport->port_name[7],
		new_lp->fcport->port_name[0], new_lp->fcport->port_name[1],
		new_lp->fcport->port_name[2], new_lp->fcport->port_name[3],
		new_lp->fcport->port_name[4], new_lp->fcport->port_name[5],
		new_lp->fcport->port_name[6], new_lp->fcport->port_name[7],
		new_lp->lun, sp->err_id);
	printk(KERN_INFO
		"qla2x00: FROM HBA %d to HBA %d\n",
		(int)old_lp->fcport->ha->instance,
		(int)new_lp->fcport->ha->instance);

	DEBUG3(printk("%s: NEW fclun = %p , port =%p, "
	    "loop_id =0x%x, instance %ld\n",
	    __func__,
	    new_lp, new_lp->fcport,
	    new_lp->fcport->loop_id,
	    new_lp->fcport->ha->instance);)

	return status;
}


/*
 * qla2x00_send_failover_notify
 *      A failover operation has just been done from an old path
 *      index to a new index.  Call lower level driver
 *      to perform the failover notification.
 *
 * Inputs:
 *      device           Device being failed over.
 *      lun                LUN being failed over.
 *      newpath           path that was failed over too.
 *      oldpath           path that was failed over from.
 *
 * Return:
 *      Local function status code.
 *
 * Context:
 *      Kernel context.
 */
/* ARGSUSED */
static uint32_t
qla2x00_send_failover_notify(mp_device_t *dp,
    uint8_t lun, mp_path_t *newpath, mp_path_t *oldpath)
{
	fc_lun_t	*old_lp, *new_lp;
	uint32_t	status = QLA2X00_SUCCESS;

	ENTER("qla2x00_send_failover_notify");

	old_lp = qla2x00_find_matching_lun(lun, oldpath);
	new_lp = qla2x00_find_matching_lun(lun, newpath);

	/*
	 * If the target is the same target, but a new HBA has been selected,
	 * send a third party logout if required.
	 */
	if ((qla_fo_params.FailoverNotifyType &
			 FO_NOTIFY_TYPE_LOGOUT_OR_LUN_RESET ||
			qla_fo_params.FailoverNotifyType &
			 FO_NOTIFY_TYPE_LOGOUT_OR_CDB) &&
			qla2x00_is_portname_equal(
				oldpath->portname, newpath->portname)) {

		status =  qla2x00_send_fo_notification(old_lp, new_lp);
		if (status == QLA2X00_SUCCESS) {
			/* EMPTY */
			DEBUG4(printk("%s: Logout succeded\n",
			    __func__);)
		} else {
			/* EMPTY */
			DEBUG4(printk("%s: Logout Failed\n",
			    __func__);)
		}
	} else if ((qla_fo_params.FailoverNotifyType &
			 FO_NOTIFY_TYPE_LUN_RESET) ||
			(qla_fo_params.FailoverNotifyType &
			 FO_NOTIFY_TYPE_LOGOUT_OR_LUN_RESET)) {

		/*
		 * If desired, send a LUN reset as the
		 * failover notification type.
		 */
		if (newpath->lun_data.data[lun] & LUN_DATA_ENABLED) {
			status = qla2x00_send_fo_notification(old_lp, new_lp);
			if (status == QLA2X00_SUCCESS) {
				/* EMPTY */
				DEBUG4(printk("%s: LUN reset succeeded.\n",
				    __func__);)
			} else {
				/* EMPTY */
				DEBUG4(printk("%s: Failed reset LUN.\n",
				    __func__);)
			}
		}

	} else if (qla_fo_params.FailoverNotifyType == FO_NOTIFY_TYPE_CDB ||
			qla_fo_params.FailoverNotifyType ==
			 FO_NOTIFY_TYPE_LOGOUT_OR_CDB) {

		if (newpath->lun_data.data[lun] & LUN_DATA_ENABLED) {
			status = qla2x00_send_fo_notification(old_lp, new_lp);
			if (status == QLA2X00_SUCCESS) {
				/* EMPTY */
				DEBUG4(printk("%s: Send CDB succeeded.\n",
				    __func__);)
			} else {
				/* EMPTY */
				DEBUG4(printk("%s: Send CDB Error "
				    "lun=(%d).\n", __func__, lun);)
			}
		}
	} else {
		/* EMPTY */
		DEBUG4(printk("%s: failover disabled or no notify routine "
		    "defined.\n", __func__);)
	}

	return status;
}

/*
 *  qla2x00_select_next_path
 *      A problem has been detected with the current path for this
 *      device.  Try to select the next available path as the current
 *      path for this device.  If there are no more paths, the same
 *      path will still be selected.
 *
 *  Inputs:
 *      dp           pointer of device structure.
 *      lun                LUN to failover.
 *
 *  Return Value:
 *      	new path or same path
 *
 * Context:
 *      Kernel context.
 */
static mp_path_t *
qla2x00_select_next_path(mp_host_t *host, mp_device_t *dp, uint8_t lun)
{
	mp_path_t	*path = NULL;
	mp_path_list_t	*path_list;
	mp_path_t	*orig_path;
	int		id;
	uint32_t	status;
	mp_host_t *new_host;

	ENTER("qla2x00_select_next_path:");

	path_list = dp->path_list;
	if (path_list == NULL)
		return NULL;

	/* Get current path */
	id = path_list->current_path[lun];

	/* Get path for current path id  */
	if ((orig_path = qla2x00_find_path_by_id(dp, id)) != NULL) {

		/* select next path */
		path = orig_path->next;
		new_host = path->host;

		/* FIXME may need to check for HBA being reset */
		DEBUG3(printk("%s: orig path = %p new path = %p " 
		    "curr idx = %d, new idx = %d\n",
		    __func__, orig_path, path, orig_path->id, path->id);)
		DEBUG3(printk("  FAILOVER: device nodename: "
		    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		    dp->nodename[0], dp->nodename[1],
		    dp->nodename[2], dp->nodename[3],
		    dp->nodename[4], dp->nodename[5],
		    dp->nodename[6], dp->nodename[7]);)
		DEBUG3(printk(" Original  - host nodename: "
		    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		    orig_path->host->nodename[0],
		    orig_path->host->nodename[1],
		    orig_path->host->nodename[2],
		    orig_path->host->nodename[3],
		    orig_path->host->nodename[4],
		    orig_path->host->nodename[5],
		    orig_path->host->nodename[6],
		    orig_path->host->nodename[7]);)
		DEBUG3(printk("   portname: "
		    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		    orig_path->port->port_name[0],
		    orig_path->port->port_name[1],
		    orig_path->port->port_name[2],
		    orig_path->port->port_name[3],
		    orig_path->port->port_name[4],
		    orig_path->port->port_name[5],
		    orig_path->port->port_name[6],
		    orig_path->port->port_name[7]);)
		DEBUG3(printk(" New  - host nodename: "
		    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		    new_host->nodename[0], new_host->nodename[1],
		    new_host->nodename[2], new_host->nodename[3],
		    new_host->nodename[4], new_host->nodename[5],
		    new_host->nodename[6], new_host->nodename[7]);)
		DEBUG3(printk("   portname: "
		    "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		    path->port->port_name[0],
		    path->port->port_name[1],
		    path->port->port_name[2],
		    path->port->port_name[3],
		    path->port->port_name[4],
		    path->port->port_name[5],
		    path->port->port_name[6],
		    path->port->port_name[7]);)

		path_list->current_path[lun] = path->id;

		/* If we selected a new path, do failover notification. */
		if (path != orig_path) {
			status = qla2x00_send_failover_notify(
					dp, lun, path, orig_path);

			/*
			 * Currently we ignore the returned status from
			 * the notify. however, if failover notify fails
			 */
		}
	}

	LEAVE("qla2x00_select_next_path:");

	return  path ;
}



/*
 *  qla2x00_update_mp_host
 *      Update the multipath control information from the port
 *      database for that adapter.
 *
 *  Input:
 *      host      Adapter to update. Devices that are new are
 *                      known to be attached to this adapter.
 *
 *  Returns:
 *      TRUE if updated successfully; FALSE if error.
 *
 */
static BOOL
qla2x00_update_mp_host(mp_host_t  *host)
{
	BOOL		success = TRUE;
	uint16_t	dev_id;
	fc_port_t 	*port;
	scsi_qla_host_t *ha = host->ha;

	ENTER("qla2x00_update_mp_host");

	/*
	 * We make sure each port is attached to some virtual device.
	 */
	for (dev_id = 0, port = ha->fcport;
			(port);
			port = port->next, dev_id++) {

		success |= qla2x00_update_mp_device(host, port, dev_id);
	}
	if (success) {
		DEBUG2(printk("%s: Exit OK\n", __func__);)
		qla2x00_map_os_targets(host);
	} else {
		/* EMPTY */
		DEBUG2(printk("%s: Exit FAILED\n", __func__);)
	}

	LEAVE("qla2x00_update_mp_host");

	return success;
}

/*
 *  qla2x00_update_mp_device
 *      Update the multipath control information from the port
 *      database for that adapter.
 *
 *  Inputs:
 *		host   Host adapter structure
 *      port   Device to add to the path tree.
 *		dev_id  Device id
 *
 *  Synchronization:
 *      The Adapter Lock should have already been acquired
 *      before calling this routine.
 *
 *  Return
 *      TRUE if updated successfully; FALSE if error.
 *
 */
BOOL
qla2x00_update_mp_device(mp_host_t *host, fc_port_t *port, uint16_t dev_id)
{
	BOOL		success = TRUE;
	mp_device_t *dp;
	mp_path_t  *path;

	ENTER("qla2x00_update_mp_device");

	DEBUG3(printk("%s: host =%p, port =%p, id = %d\n",
	    __func__, host, port, dev_id);)

	if (!qla2x00_is_ww_name_zero(port->port_name)) {

		/*
		 * Search for a device with a matching node name,
		 * or create one.
		 */
		dp = qla2x00_find_or_allocate_mp_dev(host, dev_id, port);

		/*
		 * We either have found or created a path list. Find this
		 * host's path in the path list or allocate a new one
		 * and add it to the list.
		 */
		if (dp == NULL) {
			DEBUG4(printk("%s: Device NOT found or created at.\n",
			    __func__);)
			return FALSE;
		}

		/*
		 * Find the path in the current path list, or allocate
		 * a new one and put it in the list if it doesn't exist.
		 * Note that we do NOT set bSuccess to FALSE in the case
		 * of failure here.  We must tolerate the situation where
		 * the customer has more paths to a device than he can
		 * get into a PATH_LIST.
		 */

		path = qla2x00_find_or_allocate_path(host, dp, dev_id, port);
		if (path == NULL) {
			DEBUG4(printk("%s:Path NOT found or created.\n",
			    __func__);)
			return FALSE;
		}

		/* Set the PATH flag to match the device flag
		 * of whether this device needs a relogin.  If any
		 * device needs relogin, set the relogin countdown.
		 */
		if (port->flags & FC_CONFIG)
			path->config = TRUE;

		if (atomic_read(&port->state) != FC_ONLINE) {
			path->relogin = TRUE;
			if (host->relogin_countdown == 0)
				host->relogin_countdown = 30;
		} else {
			path->relogin = FALSE;
		}

	} else {
		/* EMPTY */
		DEBUG4(printk("%s: Failed portname empty.\n",
		    __func__);)
	}

	LEAVE("qla2x00_update_mp_device");

	return success;
}

/*
 * qla2x00_update_mp_tree
 *      Get port information from each adapter, and build or rebuild
 *      the multipath control tree from this data.  This is called
 *      from init and during port database notification.
 *
 * Input:
 *      None
 *
 * Return:
 *      Local function return code.
 *
 */
static uint32_t
qla2x00_update_mp_tree(void)
{
	mp_host_t	*host;
	uint32_t	rval = QLA2X00_SUCCESS;

	ENTER("qla2x00_update_mp_tree:");

	/* Loop through each adapter and see what needs updating. */
	for (host = mp_hosts_base; (host) ; host = host->next) {

		DEBUG4(printk("%s: hba(%d) flags (%x)\n",
		    __func__, host->instance, host->flags);)
		/* Clear the countdown; it may be reset in the update. */
		host->relogin_countdown = 0;

		/* Override the NEEDS_UPDATE flag if disabled. */
		if (host->flags & MP_HOST_FLAG_DISABLE ||
		    host->fcport == NULL)
			host->flags &= ~MP_HOST_FLAG_NEEDS_UPDATE;

		if (host->flags & MP_HOST_FLAG_NEEDS_UPDATE) {

			/*
			 * Perform the actual updates.  If this succeeds, clear
			 * the flag that an update is needed, and failback all
			 * devices that are visible on this path to use this
			 * path.  If the update fails, leave set the flag that
			 * an update is needed, and it will be picked back up
			 * during the next timer routine.
			 */
			if (qla2x00_update_mp_host(host)) {
				host->flags &= ~MP_HOST_FLAG_NEEDS_UPDATE;

				qla2x00_failback_luns(host);
			} else
				rval = QLA2X00_FUNCTION_FAILED;

		}

	}

	if (rval != QLA2X00_SUCCESS) {
		/* EMPTY */
		DEBUG4(printk("%s: Exit FAILED.\n", __func__);)

	} else {
		/* EMPTY */
		DEBUG4(printk("%s: Exit OK.\n", __func__);)
	}
	return rval;
}



/*
 * qla2x00_find_matching_lun
 *      Find the lun in the path that matches the
 *  specified lun number.
 *
 * Input:
 *      lun  = lun number
 *      newpath = path to search for lun
 *
 * Returns:
 *      NULL or pointer to lun
 *
 * Context:
 *      Kernel context.
 * (dg)
 */
static fc_lun_t  *
qla2x00_find_matching_lun(uint8_t lun, mp_path_t *newpath)
{
	fc_lun_t *lp = NULL;	/* lun ptr */
	fc_lun_t *nlp;			/* Next lun ptr */
	fc_port_t *port;		/* port ptr */

	if ((port = newpath->port) != NULL) {
		for (nlp = port->fclun; (nlp); nlp = nlp->next) {
			if (lun == nlp->lun) {
				lp = nlp;
				break;
			}
		}
	}
	return lp;
}

/*
 * qla2x00_find_path_by_name
 *      Find the path specified portname from the pathlist
 *
 * Input:
 *      host = host adapter pointer.
 * 	pathlist =  multi-path path list
 *      portname  	portname to search for
 *
 * Returns:
 * pointer to the path or NULL
 *
 * Context:
 *      Kernel context.
 */
mp_path_t *
qla2x00_find_path_by_name(mp_host_t *host, mp_path_list_t *plp,
    uint8_t *portname)
{
	mp_path_t  *path = NULL;		/* match if not NULL */
	mp_path_t  *tmp_path;
	int cnt;

	if ((tmp_path = plp->last) != NULL) {
		for (cnt = 0; cnt < plp->path_cnt; cnt++) {
			if (tmp_path->host == host &&
				qla2x00_is_portname_equal(
					tmp_path->portname, portname)) {

				path = tmp_path;
				break;
			}
			tmp_path = tmp_path->next;
		}
	}
	return path ;
}

/*
 * qla2x00_find_path_by_id
 *      Find the path for the specified path id.
 *
 * Input:
 * 	dp 		multi-path device
 * 	id 		path id
 *
 * Returns:
 *      pointer to the path or NULL
 *
 * Context:
 *      Kernel context.
 */
static mp_path_t *
qla2x00_find_path_by_id(mp_device_t *dp, uint8_t id)
{
	mp_path_t  *path = NULL;
	mp_path_t  *tmp_path;
	mp_path_list_t		*path_list;
	int cnt;

	path_list = dp->path_list;
	tmp_path = path_list->last;
	for (cnt = 0; (tmp_path) && cnt < path_list->path_cnt; cnt++) {
		if (tmp_path->id == id) {
			path = tmp_path;
			break;
		}
		tmp_path = tmp_path->next;
	}
	return path ;
}

/*
 * qla2x00_find_mp_dev_by_id
 *      Find the mp_dev for the specified target id.
 *
 * Input:
 *      host = host adapter pointer.
 *      tgt  = Target id
 *
 * Returns:
 *
 * Context:
 *      Kernel context.
 */
static mp_device_t  *
qla2x00_find_mp_dev_by_id(mp_host_t *host, uint8_t id )
{
	if (id < MAX_MP_DEVICES)
		return host->mp_devs[id];
	else
		return NULL;
}

/*
 * qla2x00_find_mp_dev_by_name
 *      Find the mp_dev for the specified target name.
 *
 * Input:
 *      host = host adapter pointer.
 *      name  = Target name
 *
 * Returns:
 *
 * Context:
 *      Kernel context.
 */
static mp_device_t  *
qla2x00_find_mp_dev_by_name(mp_host_t *host, uint8_t *name )
{
	int id;
	mp_device_t *dp;

	ENTER("qla2x00_find_mp_dev_by_name");

	for (id= 0; id < MAX_MP_DEVICES; id++) {
		if ((dp = host->mp_devs[id] ) == NULL)
			continue;

		if (qla2x00_is_nodename_equal(dp->nodename, name)) {
			DEBUG3(printk("%s: Found matching device @ index %d:\n",
			    __func__, id);)
			return dp;
		}
	}

	LEAVE("qla2x00_find_mp_dev_by_name");

	return NULL;
}

/*
 * qla2x00_get_visible_path
 * Find the the visible path for the specified device.
 *
 * Input:
 *      dp = device pointer
 *
 * Returns:
 *      NULL or path
 *
 * Context:
 *      Kernel context.
 */
static mp_path_t *
qla2x00_get_visible_path(mp_device_t *dp)
{
	uint16_t	id;
	mp_path_list_t	*path_list;
	mp_path_t	*path;

	path_list = dp->path_list;
	/* if we don't have a visible path skip it */
	if ((id = path_list->visible) == PATH_INDEX_INVALID) {
		return NULL;
	}

	if ((path = qla2x00_find_path_by_id(dp,id))== NULL)
		return NULL;

	return path ;
}

/*
 * qla2x00_map_os_targets
 * Allocate the luns and setup the OS target.
 *
 * Input:
 *      host = host adapter pointer.
 *
 * Returns:
 *      None
 *
 * Context:
 *      Kernel context.
 */
static void
qla2x00_map_os_targets(mp_host_t *host)
{
	scsi_qla_host_t *ha = host->ha;
	mp_path_t	*path;
	mp_device_t 	*dp;
	os_tgt_t	*tgt;
	int		t;

	ENTER("qla2x00_map_os_targets ");

	for (t = 0; t < MAX_TARGETS; t++ ) {
		dp = host->mp_devs[t];
		if (dp != NULL) {
			DEBUG3(printk("%s: (%d) found a dp=%p, "
			    "host=%p, ha=%p\n",
			    __func__, t, dp, host,ha);)

			if ((path = qla2x00_get_visible_path(dp)) == NULL) {
				printk(KERN_INFO
				    "qla_cfg(%d): No visible path "
				    "for target %d, dp = %p\n",
				    host->instance, t, dp);
				continue;
			}

			/* if not the visible path skip it */
			if (path->host == host) {
				if (TGT_Q(ha, t) == NULL) {
					tgt = qla2x00_tgt_alloc(ha, t);
					memcpy(tgt->node_name,
							dp->nodename,
							WWN_SIZE);
					tgt->vis_port = path->port;
				}
				DEBUG3(printk("%s(%ld): host=%d, "
				    "device= %p has VISIBLE "
				    "path=%p, path id=%d\n",
				    __func__, ha->host_no,
				    host->instance,
				    dp, path, path->id);)
			} else {
			/* EMPTY */
				DEBUG3(printk("%s(%ld): host=%d, "
				    "device= %p has HIDDEN "
				    "path=%p, path id=%d\n",
				    __func__, ha->host_no,
				    host->instance, dp, path,path->id);)
			}
			qla2x00_map_os_luns(host, dp, t);
		} else {
			if ((tgt= TGT_Q(ha,t)) != NULL) {
				qla2x00_tgt_free(ha,t);
			}
		}
	}

	LEAVE("qla2x00_map_os_targets ");
}

/*
 * qla2x00_map_os_luns
 *      Allocate the luns for the OS target.
 *
 * Input:
 *      dp = pointer to device
 *      t  = OS target number.
 *
 * Returns:
 *      None
 *
 * Context:
 *	Kernel context.
 */
static void
qla2x00_map_os_luns(mp_host_t *host, mp_device_t *dp, uint16_t t)
{
	uint16_t lun;
	int	i;

	for (lun = 0; lun < MAX_LUNS; lun++ ) {
		if ( qla2x00_map_a_oslun(host, dp, t, lun) &&
			(host->flags & MP_HOST_FLAG_LUN_FO_ENABLED) ){
			/* find a path for us to use */
			for ( i = 0; i < dp->path_list->path_cnt; i++ ){
				qla2x00_select_next_path(host, dp, lun);
				if( !qla2x00_map_a_oslun(host, dp, t, lun))
					break;
			}
		}
	}
}

/*
 * qla2x00_map_a_osluns
 *      Map the OS lun to the current path
 *
 * Input:
 *      host = pointer to host
 *      dp = pointer to device
 *      lun  = OS lun number.
 *
 * Returns:
 *      None
 *
 * Context:
 *	Kernel context.
 */

static BOOL
qla2x00_map_a_oslun(mp_host_t *host, mp_device_t *dp, uint16_t t, uint16_t lun)
{
	fc_port_t	*fcport;
	fc_lun_t	*fclun;
	os_lun_t	*lq;
	uint16_t	id;
	mp_path_t	*path, *vis_path;
	mp_host_t 	*vis_host;
	BOOL		status = FALSE;

	if ((id = dp->path_list->current_path[lun]) != PATH_INDEX_INVALID) {
		path = qla2x00_find_path_by_id(dp,id);
		if (path) {
			fcport = path->port;
			if (fcport) {
				/* dg 04/26/02 */
			 	fcport->cur_path = id;
				fclun = qla2x00_find_matching_lun(lun,path);

				/* Always map all luns if they are enabled */
				if (fclun &&
					(path->lun_data.data[lun] &
					 LUN_DATA_ENABLED) ) {

					/*
					 * Mapped lun on the visible path
					 */
					if ((vis_path =
					    qla2x00_get_visible_path(dp)) ==
					    NULL ) {

						printk(KERN_INFO
						    "qla2x00(%d): No visible "
						    "path for target %d, "
						    "dp = %p\n",
						    host->instance,
						    t, dp);

						return FALSE;
					}

					vis_host = vis_path->host;

					/* ra 11/30/01 */
					/*
					 * Always alloc LUN 0 so kernel
					 * will scan past LUN 0.
					 */
					if (lun != 0 &&
					    (EXT_IS_LUN_BIT_SET(
						&(fcport->lun_mask), lun))) {

						/* mask this LUN */
						return FALSE;
					}

					if ((lq = qla2x00_lun_alloc(
							vis_host->ha,
							t, lun)) != NULL) {

						lq->fclun = fclun;
					}
				}
			}
			else
				status = TRUE;
		}
	}
	return status;
}

/*
 * qla2x00_is_ww_name_zero
 *
 * Input:
 *      ww_name = Pointer to WW name to check
 *
 * Returns:
 *      TRUE if name is 0 else FALSE
 *
 * Context:
 *      Kernel context.
 */
static BOOL
qla2x00_is_ww_name_zero(uint8_t *nn)
{
	int cnt;

	/* Check for zero node name */
	for (cnt = 0; cnt < WWN_SIZE ; cnt++, nn++) {
		if (*nn != 0)
			break;
	}
	/* if zero return TRUE */
	if (cnt == WWN_SIZE)
		return TRUE;
	else
		return FALSE;
}

/*
 * qla2x00_add_path
 * Add a path to the pathlist
 *
 * Input:
 * pathlist -- path list of paths
 * path -- path to be added to list
 *
 * Returns:
 *      None
 *
 * Context:
 *      Kernel context.
 */
static void
qla2x00_add_path( mp_path_list_t *pathlist, mp_path_t *path )
{
	mp_path_t *last = pathlist->last;

	ENTER("qla2x00_add_path");

	DEBUG3(printk("%s: pathlist =%p, path =%p, cnt = %d\n",
	    __func__, pathlist, path, pathlist->path_cnt);)
	if (last == NULL) {
		last = path;
	} else {
		path->next = last->next;
	}

	last->next = path;
	pathlist->last = path;
	pathlist->path_cnt++;

	LEAVE("qla2x00_add_path");
}


/*
 * qla2x00_is_portname_in_device
 *	Search for the specified "portname" in the device list.
 *
 * Input:
 *	dp = device pointer
 *	portname = portname to searched for in device
 *
 * Returns:
 *      qla2x00 local function return status code.
 *
 * Context:
 *      Kernel context.
 */
static BOOL
qla2x00_is_portname_in_device(mp_device_t *dp, uint8_t *portname)
{
	int idx;

	for (idx = 0; idx < MAX_PATHS_PER_DEVICE; idx++) {
		if (memcmp(&dp->portnames[idx][0], portname, WWN_SIZE) == 0)
			return TRUE;
	}
	return FALSE;
}


/*
 *  qla2x00_set_lun_data_from_bitmask
 *      Set or clear the LUN_DATA_ENABLED bits in the LUN_DATA from
 *      a LUN bitmask provided from the miniport driver.
 *
 *  Inputs:
 *      lun_data = Extended LUN_DATA buffer to set.
 *      lun_mask = Pointer to lun bit mask union.
 *
 *  Return Value: none.
 */
void
qla2x00_set_lun_data_from_bitmask(mp_lun_data_t *lun_data,
    lun_bit_mask_t *lun_mask)
{
	int16_t	lun;

	ENTER("qla2x00_set_lun_data_from_bitmask");

	for (lun = 0; lun < MAX_LUNS; lun++) {
		/* our bit mask is inverted */
		if (!(EXT_IS_LUN_BIT_SET(lun_mask,lun)))
			lun_data->data[lun] |= LUN_DATA_ENABLED;
		else
			lun_data->data[lun] &= ~LUN_DATA_ENABLED;

		DEBUG5(printk("%s: lun data[%d] = 0x%x\n",
		    __func__, lun, lun_data->data[lun]);)
	}

	LEAVE("qla2x00_set_lun_data_from_bitmask");

	return;
}

static void
qla2x00_failback_single_lun(mp_device_t *dp, uint8_t lun, uint8_t new)
{
	mp_path_list_t   *pathlist;
	mp_path_t        *new_path, *old_path;
	uint8_t 	old;
	mp_host_t  *host;
	os_lun_t *lq;
	mp_path_t	*vis_path;
	mp_host_t 	*vis_host;

	/* Failback and update statistics. */
	if ((pathlist = dp->path_list) == NULL)
		return;

	old = pathlist->current_path[lun];
	pathlist->current_path[lun] = new;

	if ((new_path = qla2x00_find_path_by_id(dp, new)) == NULL)
		return;
	if ((old_path = qla2x00_find_path_by_id(dp, old)) == NULL)
		return;

	/* An fclun should exist for the failbacked lun */
	if (qla2x00_find_matching_lun(lun, new_path) == NULL)
		return;
	if (qla2x00_find_matching_lun(lun, old_path) == NULL)
		return;

	/* Log to console and to event log. */
	printk(KERN_INFO
		"qla2x00: FAILBACK device %d -> "
		"%02x%02x%02x%02x%02x%02x%02x%02x LUN %02x\n",
		dp->dev_id,
		dp->nodename[0], dp->nodename[1],
		dp->nodename[2], dp->nodename[3],
		dp->nodename[4], dp->nodename[5],
		dp->nodename[6], dp->nodename[7],
		lun);

	printk(KERN_INFO
		"qla2x00: FROM HBA %d to HBA %d \n",
		old_path->host->instance,
		new_path->host->instance);


	/* Send a failover notification. */
#if 0
	qla2x00_send_failover_notify(dp, lun, new_path, old_path);
#endif

	host = 	new_path->host;

	/* remap the lun */
	qla2x00_map_a_oslun(host, dp, dp->dev_id, lun);

	/* 7/16
	 * Reset counts on the visible path
	 */
	if ((vis_path = qla2x00_get_visible_path(dp)) == NULL) {
		printk(KERN_INFO
			"qla2x00(%d): No visible path for "
			"target %d, dp = %p\n",
			host->instance,
			dp->dev_id, dp);
		return;
	}

	vis_host = vis_path->host;
	if ((lq = qla2x00_lun_alloc(vis_host->ha, dp->dev_id, lun)) != NULL) {
		qla2x00_delay_lun(vis_host->ha, lq, recoveryTime);
		qla2x00_flush_failover_q(vis_host->ha, lq);
		qla2x00_reset_lun_fo_counts(vis_host->ha, lq);
	}
}

/*
*  qla2x00_failback_luns
*      This routine looks through the devices on an adapter, and
*      for each device that has this adapter as the visible path,
*      it forces that path to be the current path.  This allows us
*      to keep some semblance of static load balancing even after
*      an adapter goes away and comes back.
*
*  Arguments:
*      host          Adapter that has just come back online.
*
*  Return:
*	None.
*/
static void
qla2x00_failback_luns( mp_host_t  *host)
{
	uint16_t          dev_no;
	uint8_t           l;
	uint16_t          lun;
	int i;
	mp_device_t      *dp;
	mp_path_list_t   *path_list;
	mp_path_t        *path;
	fc_lun_t	*new_fp;

	ENTER("qla2x00_failback_luns");

	for (dev_no = 0; dev_no < MAX_MP_DEVICES; dev_no++) {
		dp = host->mp_devs[dev_no];

		if (dp == NULL)
			continue;

		path_list = dp->path_list;
		for (path = path_list->last, i= 0;
			i < path_list->path_cnt;
			i++, path = path->next) {

			if (path->host != host )
				continue;

			if (path->port == NULL)
				continue;

			if (atomic_read(&path->port->state) == FC_DEVICE_DEAD)
				continue;

			/* 
			 * Failback all the paths for this host,
			 * the luns could be preferred across all paths 
			 */
			DEBUG2(printk("%s(%d): Lun Data for device %p, "
			    "id=%d, path id=%d\n",
			    __func__, host->instance, dp, dp->dev_id,
			    path->id);)
			DEBUG4(qla2x00_dump_buffer(
			    (char *)&path->lun_data.data[0], 64);)
			DEBUG4(printk("%s(%d): Perferrred Path data:\n",
			    __func__, host->instance);)
			DEBUG4(qla2x00_dump_buffer(
			    (char *)&path_list->current_path[0], 64);)

			for (lun = 0; lun < MAX_LUNS_PER_DEVICE; lun++) {
				l = (uint8_t)(lun & 0xFF);

				/*
				 * if this is the preferred lun and not
				 * the current path then failback lun.
				 */
				DEBUG4(printk("%s: target=%d, cur path id =%d, "
				    "lun data[%d] = %d)\n",
				    __func__, dp->dev_id, path->id,
				    lun, path->lun_data.data[lun]);)

				if ((path->lun_data.data[l] &
						LUN_DATA_PREFERRED_PATH) &&
					/* !path->relogin && */
					path_list->current_path[l] !=
						path->id) {
					/* No point in failing back a
					   disconnected lun */
					new_fp = qla2x00_find_matching_lun(
							l, path);

					if (new_fp == NULL)
						continue;
					if (new_fp->flags & FC_DISCON_LUN)
						continue;

					qla2x00_failback_single_lun(
							dp, l, path->id);
				}
			}
		}

	}

	LEAVE("qla2x00_failback_luns");

	return;
}

/*
 *  qla2x00_setup_new_path
 *      Checks the path against the existing paths to see if there
 *      are any incompatibilities.  It then checks and sets up the
 *      current path indices.
 *
 *  Inputs:
 *      dp   =  pointer to device
 *      path = new path
 *
 *  Returns:
 *      None
 */
static void
qla2x00_setup_new_path( mp_device_t *dp, mp_path_t *path)
{
	mp_path_list_t  *path_list = dp->path_list;
	mp_path_t       *tmp_path, *first_path;
	mp_host_t       *first_host;
	mp_host_t       *tmp_host;

	uint16_t	lun;
	uint8_t		l;
	int		i;

	ENTER("qla2x00_setup_new_path");

	/* If this is a visible path, and there is not already a
	 * visible path, save it as the visible path.  If there
	 * is already a visible path, log an error and make this
	 * path invisible.
	 */
	if (!(path->mp_byte & (MP_MASK_HIDDEN | MP_MASK_UNCONFIGURED))) {

		/* No known visible path */
		if (path_list->visible == PATH_INDEX_INVALID) {
			DEBUG3(printk("%s: No know visible path - make this "
			    "path visible\n",
			    __func__);)
				
			path_list->visible = path->id;
			path->mp_byte &= ~MP_MASK_HIDDEN;
		} else {
			DEBUG3(printk("%s: Second visible path found- make "
			    "this one hidden\n",
			    __func__);)

			path->mp_byte |= MP_MASK_HIDDEN;
		}
	}

	/*
	 * If this is not the first path added, and the setting for
	 * MaxLunsPerTarget does not match that of the first path
	 * then disable qla_cfg for all adapters.
	 */
	first_path = qla2x00_find_path_by_id(dp, 0);

	if (first_path != NULL) {
		first_host = first_path->host;
		if ((path->id != 0) &&
			(first_host->MaxLunsPerTarget !=
			 path->host->MaxLunsPerTarget)) {

			for (tmp_path = path_list->last, i = 0;
				(tmp_path) && i <= path->id; i++) {

				tmp_host = tmp_path->host;
				if (!(tmp_host->flags &
						MP_HOST_FLAG_DISABLE)) {

					DEBUG4(printk("%s: 2nd visible "
					    "path (%p)\n",
					    __func__, tmp_host);)

					tmp_host->flags |= MP_HOST_FLAG_DISABLE;
				}
			}
		}
	}

	/*
	 * For each LUN, evaluate whether the new path that is added
	 * is better than the existing path.  If it is, make it the
	 * current path for the LUN.
	 */
	for (lun = 0; lun < MAX_LUNS_PER_DEVICE; lun++) {
		l = (uint8_t)(lun & 0xFF);

		/* If this is the first path added, it is the only
		 * available path, so make it the current path.
		 */

		DEBUG4(printk("%s: lun_data 0x%x, LUN %d\n",
		    __func__, path->lun_data.data[l], lun);)

		if (first_path == path) {
			path_list->current_path[l] = 0;
			path->lun_data.data[l] |=  LUN_DATA_PREFERRED_PATH;
		} else if (path->lun_data.data[l] & LUN_DATA_PREFERRED_PATH) {
			/*
			 * If this is not the first path added, if this is
			 * the preferred path, make it the current path.
			 */
			path_list->current_path[l] = path->id;
		}
	}

	LEAVE("qla2x00_setup_new_path");

	return;
}

/*
 * qla2x00_cfg_mem_free
 *     Free all configuration structures.
 *
 * Input:
 *      ha = adapter state pointer.
 *
 * Context:
 *      Kernel context.
 */
void
qla2x00_cfg_mem_free(scsi_qla_host_t *ha)
{
	mp_device_t *dp;
	mp_path_list_t  *path_list;
	mp_path_t       *tmp_path, *path;
	mp_host_t       *host, *temp;
	int	id, cnt;

	if ((host = qla2x00_cfg_find_host(ha)) != NULL) {
		if( mp_num_hosts == 0 )
			return;

		for (id= 0; id < MAX_MP_DEVICES; id++) {
			if ((dp = host->mp_devs[id]) == NULL)
				continue;
			if ((path_list = dp->path_list) == NULL)
				continue;
			if ((tmp_path = path_list->last) == NULL)
				continue;
			for (cnt = 0; cnt < path_list->path_cnt; cnt++) {
				path = tmp_path;
				tmp_path = tmp_path->next;
				DEBUG(printk(KERN_INFO
						"host%d - Removing path[%d] "
						"= %p\n",
						host->instance,
						cnt, path);)
				KMEM_FREE(path,sizeof(mp_path_t));
			}
			KMEM_FREE(path_list, sizeof(mp_path_list_t));
			host->mp_devs[id] = NULL;
			/* remove dp from other hosts */
			for (temp = mp_hosts_base; (temp); temp = temp->next) {
				if (temp->mp_devs[id] == dp) {
					DEBUG(printk(KERN_INFO
						"host%d - Removing host[%d] = "
						"%p\n",
						host->instance,
						temp->instance,temp);)
					temp->mp_devs[id] = NULL;
				}
			}
			KMEM_FREE(dp, sizeof(mp_device_t));
		}

		/* remove this host from host list */
		temp = mp_hosts_base;
		if (temp != NULL) {
			/* Remove from top of queue */
			if (temp == host) {
				mp_hosts_base = host->next;
			} else {
				/*
				 * Remove from middle of queue
				 * or bottom of queue
				 */
				for (temp = mp_hosts_base;
						temp != NULL;
						temp = temp->next) {

					if (temp->next == host) {
						temp->next = host->next;
						break;
					}
				}
			}
		}
		KMEM_FREE(host, sizeof(mp_host_t));
		mp_num_hosts--;
	}
}
