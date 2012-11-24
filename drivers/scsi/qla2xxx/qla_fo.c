/********************************************************************************
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
******************************************************************************
* Failover include file
******************************************************************************/
#include "qla2x00.h"
#include "qla_gbl.h"

#include "exioct.h"
#include "qlfo.h"
#include "qla_fo.h"
#include "qlfolimits.h"

/*
 * Global variables
 */
SysFoParams_t qla_fo_params;

/*
 * Local routines
 */
#if !defined(linux)
static int qla2x00_sdm_setup(EXT_IOCTL *cmd_stp, void *arg, int mode);
#endif
static uint32_t qla2x00_fo_get_params(PFO_PARAMS pp);
static uint32_t qla2x00_fo_set_params(PFO_PARAMS pp);
static BOOL qla2x00_fo_count_retries(scsi_qla_host_t *ha, srb_t *sp);
static int qla2x00_fo_get_lun_data(EXT_IOCTL *pext,
		FO_LUN_DATA_INPUT *bp, int mode);
static int qla2x00_fo_set_lun_data(EXT_IOCTL *pext,
		FO_LUN_DATA_INPUT *bp, int mode);
static uint32_t qla2x00_fo_stats(FO_HBA_STAT *stat_p, BOOL reset);
static int qla2x00_fo_set_target_data(EXT_IOCTL *pext,
		FO_TARGET_DATA_INPUT *bp, int mode);
static int qla2x00_fo_get_target_data(EXT_IOCTL *pext,
		FO_TARGET_DATA_INPUT *bp, int mode);

/*
 * qla2x00_get_hba
 *	Searches the hba structure chain for the requested instance
 *      aquires the mutex and returns a pointer to the hba structure.
 *
 * Input:
 *	inst = adapter instance number.
 *
 * Returns:
 *	Return value is a pointer to the adapter structure or
 *      NULL if instance not found.
 *
 * Context:
 *	Kernel context.
 */
scsi_qla_host_t *
qla2x00_get_hba(int instance)
{
	scsi_qla_host_t * hbap;

	hbap = (scsi_qla_host_t *) qla2x00_hostlist;

	while (hbap != NULL) {
		if (hbap->instance == instance) {
			break;
		}
		hbap = (scsi_qla_host_t *)hbap->next;
	}
	return hbap;
}

/*
 * qla2x00_fo_stats
 *	Searches the hba structure chan for the requested instance
 *      aquires the mutex and returns a pointer to the hba structure.
 *
 * Input:
 *	stat_p = Pointer to FO_HBA_STAT union.
 *      reset  = Flag, TRUE = reset statistics.
 *                     FALSE = return statistics values.
 *
 * Returns:
 *	0 = success
 *
 * Context:
 *	Kernel context.
 */
static uint32_t
qla2x00_fo_stats(FO_HBA_STAT *stat_p, BOOL reset)
{
	int32_t	inst, idx;
	uint32_t rval = 0;
	scsi_qla_host_t *hbap;

	DEBUG9(printk("%s: entered.\n", __func__);)

	inst = stat_p->input.HbaInstance;
	stat_p->info.HbaCount = 0;

	hbap = (scsi_qla_host_t *) qla2x00_hostlist;

	while (hbap != NULL) {
		if (inst == FO_ADAPTER_ALL) {
			stat_p->info.HbaCount++;
			idx = hbap->instance;
		} else if (hbap->instance == inst) {
			stat_p->info.HbaCount = 1;
			idx = inst;
		}
		if (reset == TRUE) {
			DEBUG9(printk("%s: reset stats.\n", __func__);)
			hbap->IosRequested = 0;
			hbap->BytesRequested = 0;
			hbap->IosExecuted = 0;
			hbap->BytesExecuted = 0;
		} else {
			DEBUG9(printk("%s: get stats for inst %d.\n",
			    __func__, inst);)

#if 0
			stat_p->info.StatEntry[idx].IosRequested =
				hbap->IosRequested;
			stat_p->info.StatEntry[idx].BytesRequested =
				hbap->BytesRequested;
			stat_p->info.StatEntry[idx].IosExecuted =
				hbap->IosExecuted;
			stat_p->info.StatEntry[idx].BytesExecuted =
				hbap->BytesExecuted;
#endif
		}
		if (inst != FO_ADAPTER_ALL)
			break;
		else
			hbap = (scsi_qla_host_t *)hbap->next;
	}

	DEBUG9(printk("%s: exiting.\n", __func__);)

	return rval;
}

/*
 * qla2x00_fo_get_lun_data
 *      Get lun data from all devices attached to a HBA (FO_GET_LUN_DATA).
 *      Gets lun mask if failover not enabled.
 *
 * Input:
 *      ha = pointer to adapter
 *      bp = pointer to buffer
 *
 * Return;
 *      0 on success or errno.
 *
 * Context:
 *      Kernel context.
 */
static int
qla2x00_fo_get_lun_data(EXT_IOCTL *pext, FO_LUN_DATA_INPUT *bp, int mode)
{
	scsi_qla_host_t  *ha;
	fc_port_t        *fcport;
	int              ret = 0;
	mp_host_t        *host = NULL;
	mp_device_t      *dp;
	mp_path_t        *path;
	mp_path_list_t   *pathlist;
	os_tgt_t         *ostgt;
	uint8_t          path_id;
	uint16_t         dev_no;
	uint16_t         cnt;
	uint16_t         lun;
	FO_EXTERNAL_LUN_DATA_ENTRY *u_entry, *entry;
	FO_LUN_DATA_LIST *u_list, *list;


	DEBUG9(printk("%s: entered.\n", __func__);)

	ha = qla2x00_get_hba((int)bp->HbaInstance);

	if (!ha) {
		DEBUG2_9_10(printk("%s: no ha matching inst %d.\n",
		    __func__, bp->HbaInstance);)

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (ret);
	}

	DEBUG9(printk("%s: ha inst %ld, buff %p.\n",
	    __func__, ha->instance, bp);)
	DEBUG4(printk("%s: hba %p, buff %p bp->HbaInstance(%x).\n",
	    __func__, ha, bp, (int)bp->HbaInstance));

	if (ha->flags.failover_enabled)
		if ((host = qla2x00_cfg_find_host(ha)) == NULL) {
			DEBUG2_9_10(printk("%s: no HOST for ha inst %ld.\n",
			    __func__, ha->instance);)
			pext->Status = EXT_STATUS_DEV_NOT_FOUND;
			return (ret);
		}

	if ((list = (FO_LUN_DATA_LIST *)qla2x00_kmem_zalloc(sizeof(FO_LUN_DATA_LIST),
	    GFP_ATOMIC, 12)) == NULL) {
		DEBUG2_9_10(printk("%s: failed to alloc memory of size (%d)\n",
		    __func__, (int)sizeof(FO_LUN_DATA_LIST));)
		pext->Status = EXT_STATUS_NO_MEMORY;
		return (-ENOMEM);
	}

	entry = &list->DataEntry[0];

	u_list = (FO_LUN_DATA_LIST *)pext->ResponseAdr;
	u_entry = &u_list->DataEntry[0];

	/* find the correct fcport list */
	if (!ha->flags.failover_enabled)
		fcport = ha->fcport;
	else
		fcport = host->fcport;

	/* Check thru this adapter's fcport list */
	for ( ; (fcport); fcport = fcport->next) {

		memcpy(entry->NodeName,
		    fcport->node_name, EXT_DEF_WWN_NAME_SIZE);
		memcpy(entry->PortName,
		    fcport->port_name, EXT_DEF_WWN_NAME_SIZE);

		if (!ha->flags.failover_enabled) {
			/*
			 * Failover disabled. Just return LUN mask info
			 * in lun data entry of this port.
			 */
			entry->TargetId = 0;
			for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
				if (!(ostgt = ha->otgt[cnt])) {
					continue;
				}

				if (ostgt->vis_port == fcport) {
					entry->TargetId = cnt;
					break;
				}
			}
			if (cnt == MAX_FIBRE_DEVICES) {
				/* Not found?  For now just go to next port. */
#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_10)
				uint8_t          *tmp_name;
#if USE_PORTNAME
				tmp_name = fcport->port_name;
#else
				tmp_name = fcport->node_name;
#endif

				printk("%s(%ld): ERROR - port "
				    "%02x%02x%02x%02x%02x%02x%02x%02x "
				    "not configured.\n",
				    __func__, ha->host_no,
				    tmp_name[0], tmp_name[1], tmp_name[2],
				    tmp_name[3], tmp_name[4], tmp_name[5],
				    tmp_name[6], tmp_name[7]);
#endif /* DEBUG */

				continue;
			}

			/* Got a valid port */
			list->EntryCount++;

			for (lun = 0; lun < MAX_LUNS; lun++) {
				/* set MSB if masked */
				entry->Data[lun] = LUN_DATA_PREFERRED_PATH;
				if (!EXT_IS_LUN_BIT_SET(&(fcport->lun_mask),
				    lun)) {
					entry->Data[lun] |= LUN_DATA_ENABLED;
				}
			}

			DEBUG9(printk("%s: got lun_mask for tgt %d\n",
			    __func__, cnt);)
			DEBUG9(qla2x00_dump_buffer((char *)&(fcport->lun_mask),
			    sizeof(lun_bit_mask_t));)

			ret = verify_area(VERIFY_WRITE, (void *)u_entry,
			    sizeof(FO_EXTERNAL_LUN_DATA_ENTRY));
			if (ret) {
				/* error */
				DEBUG9_10(printk("%s: u_entry %p verify write"
				    " error. list->EntryCount=%d.\n",
				    __func__, u_entry, list->EntryCount);)
				pext->Status = EXT_STATUS_COPY_ERR;
				break;
			}

			ret = copy_to_user(u_entry, entry,
			    sizeof(FO_EXTERNAL_LUN_DATA_ENTRY));

			if (ret) {
				/* error */
				DEBUG9_10(printk("%s: u_entry %p copy "
				    "error. list->EntryCount=%d.\n",
				    __func__, u_entry, list->EntryCount);)
				pext->Status = EXT_STATUS_COPY_ERR;
				break;
			}

			/* Go to next port */
			u_entry++;
			continue;
		}

		/*
		 * Failover is enabled. Go through the mp_devs list and return
		 * lun data in configured path.
		 */
		for (dev_no = 0; dev_no < MAX_MP_DEVICES; dev_no++) {
			dp = host->mp_devs[dev_no];

			if (dp == NULL)
				continue;

			/* Lookup entry name */
			if (!qla2x00_is_portname_in_device(dp, entry->PortName))
				continue;

			if ((pathlist = dp->path_list) == NULL)
				continue;

			path = pathlist->last;
			for (path_id = 0; path_id < pathlist->path_cnt;
			    path_id++, path = path->next) {

				if (path->host != host)
					continue;

				if (!qla2x00_is_portname_equal(path->portname,
				    entry->PortName))
					continue;

				/* Got an entry */
				entry->TargetId = dp->dev_id;
				entry->Dev_No = path->id;
				list->EntryCount++;

				for (lun = 0; lun < MAX_LUNS; lun++) {
					entry->Data[lun] =
					    path->lun_data.data[lun];
				}

				ret = verify_area(VERIFY_WRITE, (void *)u_entry,
				    sizeof(FO_EXTERNAL_LUN_DATA_ENTRY));
				if (ret) {
					/* error */
					DEBUG2_9_10(printk("%s: u_entry %p "
					    "verify wrt err. EntryCount=%d.\n",
					    __func__, u_entry, list->EntryCount);)
					pext->Status = EXT_STATUS_COPY_ERR;
					break;
				}

				ret = copy_to_user(u_entry, entry,
				    sizeof(FO_EXTERNAL_LUN_DATA_ENTRY));
				if (ret) {
					/* error */
					DEBUG2_9_10(printk("%s: u_entry %p "
					    "copy out err. EntryCount=%d.\n",
					    __func__, u_entry, list->EntryCount);)
					pext->Status = EXT_STATUS_COPY_ERR;
					break;
				}

				u_entry++;

				DEBUG9(printk("%s: (output) get_lun_data - "
				    "u_entry(%p) - lun entry[%d] :\n",
				    __func__, u_entry,list->EntryCount - 1);)

				DEBUG9(qla2x00_dump_buffer((void *)entry, 64);)

				/*
				 * We found the right path for this port.
				 * Continue with next port.
				 */
				break;
			}

			/* Continue with next port. */
			break;
		}
	}

	DEBUG9(printk("%s: get_lun_data - entry count = [%d]\n",
	    __func__, list->EntryCount);)
	DEBUG4(printk("%s: get_lun_data - entry count = [%d]\n",
	    __func__, list->EntryCount);)

	if (ret == 0) {
		ret = verify_area(VERIFY_WRITE, (void *)&u_list->EntryCount,
		    sizeof(list->EntryCount));
		if (ret) {
			/* error */
			DEBUG2_9_10(printk("%s: u_list->EntryCount %p verify "
			    " write error. list->EntryCount=%d.\n",
			    __func__, u_entry, list->EntryCount);)
			pext->Status = EXT_STATUS_COPY_ERR;
		} else {
			/* copy number of entries */
			ret = copy_to_user(&u_list->EntryCount, &list->EntryCount,
			    sizeof(list->EntryCount));
			pext->ResponseLen = FO_LUN_DATA_LIST_MAX_SIZE;
		}
	}

	KMEM_FREE(list, sizeof(FO_LUN_DATA_LIST));

	DEBUG9(printk("%s: exiting. ret=%d.\n", __func__, ret);)

	return ret;
}

/*
 * qla2x00_fo_set_lun_data
 *      Set lun data for the specified device on the attached hba
 *      (FO_SET_LUN_DATA).
 *      Sets lun mask if failover not enabled.
 *
 * Input:
 *      bp = pointer to buffer
 *
 * Return;
 *      0 on success or errno.
 *
 * Context:
 *      Kernel context.
 */
static int
qla2x00_fo_set_lun_data(EXT_IOCTL *pext, FO_LUN_DATA_INPUT  *bp, int mode)
{
	scsi_qla_host_t  *ha;
	fc_port_t        *fcport;
	int              i;
	int              ret = 0;
	mp_host_t        *host = NULL;
	mp_device_t      *dp;
	mp_path_t        *path;
	mp_path_list_t   *pathlist;
	os_tgt_t         *ostgt;
	uint8_t	         path_id;
	uint16_t         dev_no;
	uint16_t         lun;
	FO_LUN_DATA_LIST *u_list, *list;
	FO_EXTERNAL_LUN_DATA_ENTRY *u_entry, *entry;

	typedef struct _tagStruct {
		FO_LUN_DATA_INPUT   foLunDataInput;
		FO_LUN_DATA_LIST    foLunDataList;
	}
	com_struc;
	com_struc *com_iter;


	DEBUG9(printk("%s: entered.\n", __func__);)

	ha = qla2x00_get_hba((int)bp->HbaInstance);

	if (!ha) {
		DEBUG2_9_10(printk("%s: no ha matching inst %d.\n",
		    __func__, bp->HbaInstance);)

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (ret);
	}

	DEBUG9(printk("%s: ha inst %ld, buff %p.\n",
	    __func__, ha->instance, bp);)

	if (ha->flags.failover_enabled)
		if ((host = qla2x00_cfg_find_host(ha)) == NULL) {
			DEBUG2_9_10(printk("%s: no HOST for ha inst %ld.\n",
			    __func__, ha->instance);)
			pext->Status = EXT_STATUS_DEV_NOT_FOUND;
			return (ret);
		}

	if ((list = (FO_LUN_DATA_LIST *)qla2x00_kmem_zalloc(sizeof(FO_LUN_DATA_LIST),
	    GFP_ATOMIC, 13)) == NULL) {

		DEBUG2_9_10(printk("%s: failed to alloc memory of size (%d)\n",
		    __func__, (int)sizeof(FO_LUN_DATA_LIST));)
		pext->Status = EXT_STATUS_NO_MEMORY;
		return (-ENOMEM);
	}

	entry = &list->DataEntry[0];

	/* get lun data list from user */
	com_iter = (com_struc *)pext->RequestAdr;
	u_list = &(com_iter->foLunDataList);
	u_entry = &u_list->DataEntry[0];

	ret = verify_area(VERIFY_READ, (void *)u_list,
	    sizeof(FO_LUN_DATA_LIST));
	if (ret) {
		/* error */
		DEBUG2_9_10(printk("%s: u_list %p verify read error.\n",
		    __func__, u_list);)
		pext->Status = EXT_STATUS_COPY_ERR;
		KMEM_FREE(list, FO_LUN_DATA_LIST);
		return (ret);
	}

	ret = copy_from_user(list, u_list, sizeof(FO_LUN_DATA_LIST));
	if (ret) {
		/* error */
		DEBUG2_9_10(printk("%s: u_list %p copy error.\n",
		    __func__, u_list);)
		pext->Status = EXT_STATUS_COPY_ERR;
		KMEM_FREE(list, FO_LUN_DATA_LIST);
		return (ret);
	}

	DEBUG2(printk("qla_fo_set_lun_data: pext->RequestAdr(%p) u_list (%p) "
			"sizeof(FO_LUN_DATA_INPUT) =(%d) and 64 bytes...\n",
			pext->RequestAdr, u_list,
			(int)sizeof(FO_LUN_DATA_INPUT));)
	DEBUG2(qla2x00_dump_buffer((void *)u_list, 64);)

	for (i = 0; i < list->EntryCount; i++, u_entry++) {

		ret = verify_area(VERIFY_READ, (void *)u_entry,
		    sizeof(FO_EXTERNAL_LUN_DATA_ENTRY));
		if (ret) {
			/* error */
			DEBUG2_9_10(printk("%s: u_entry %p verify "
			    " read error.\n",
			    __func__, u_entry);)
			pext->Status = EXT_STATUS_COPY_ERR;
			break;
		}
		ret = copy_from_user(entry, u_entry,
		    sizeof(FO_EXTERNAL_LUN_DATA_ENTRY));
		if (ret) {
			/* error */
			DEBUG2_9_10(printk("%s: u_entry %p copy error.\n",
			    __func__, u_entry);)
			pext->Status = EXT_STATUS_COPY_ERR;
			break;
		}

		if (!ha->flags.failover_enabled) {
			/*
			 * Failover disabled. Just find the port and set
			 * LUN mask values in lun_mask field of this port.
			 */

			if (entry->TargetId >= MAX_FIBRE_DEVICES)
				/* ERROR */
				continue;

			if (!(ostgt = ha->otgt[entry->TargetId]))
				/* ERROR */
				continue;

			if (!(fcport = ostgt->vis_port))
				/* ERROR */
				continue;

			for (lun = 0; lun < MAX_LUNS; lun++) {
				/* set MSB if masked */
				if (entry->Data[lun] | LUN_DATA_ENABLED) {
					EXT_CLR_LUN_BIT(&(fcport->lun_mask),
								lun);
				} else {
					EXT_SET_LUN_BIT(&(fcport->lun_mask),
								lun);
				}
			}

			/* Go to next entry */
			continue;
		}

		/*
		 * Failover is enabled. Go through the mp_devs list and set lun
		 * data in configured path.
		 */
		for (dev_no = 0; dev_no < MAX_MP_DEVICES; dev_no++) {
			dp = host->mp_devs[dev_no];

			if (dp == NULL)
				continue;

			/* Lookup entry name */
			if (!qla2x00_is_portname_in_device(dp, entry->PortName))
					continue;

			if ((pathlist = dp->path_list) == NULL)
					continue;

			path = pathlist->last;
			for (path_id = 0; path_id < pathlist->path_cnt;
			    path_id++, path = path->next) {

				if (path->host != host)
					continue;

				if (!qla2x00_is_portname_equal(path->portname,
				    entry->PortName))
					continue;

				for (lun = 0; lun < MAX_LUNS; lun++) {
					path->lun_data.data[lun] =
					    entry->Data[lun];
					DEBUG4(printk("cfg_set_lun_data: lun "
					    "data[%d] = 0x%x \n", lun,
					    path->lun_data.data[lun]);)
				}

				break;
			}
			break;
		}
	}

	KMEM_FREE(list, FO_LUN_DATA_LIST);

	DEBUG9(printk("%s: exiting. ret = %d.\n", __func__, ret);)

	return ret;
}

/*
 * qla2x00_fo_get_target_data
 *      Get the target control byte for all devices attached to a HBA.
 *
 * Input:
 *      bp = pointer to buffer
 *
 * Return;
 *      0 on success or errno.
 *
 * Context:
 *      Kernel context.
 */
static int
qla2x00_fo_get_target_data(EXT_IOCTL *pext, FO_TARGET_DATA_INPUT *bp, int mode)
{
	scsi_qla_host_t  *ha;
	fc_port_t        *fcport;
	int              ret = 0;
	mp_host_t        *host = NULL;
	mp_device_t      *dp;
	mp_path_t        *path;
	mp_path_list_t   *pathlist;
	os_tgt_t         *ostgt;
	uint8_t          i, cnt;
	uint8_t          path_id;
	uint16_t         dev_no;
	FO_DEVICE_DATA   *entry, *u_entry;
	uint32_t	b;


	DEBUG9(printk("%s: entered.\n", __func__);)

	ha = qla2x00_get_hba((int)bp->HbaInstance);

	if (!ha) {
		DEBUG2_9_10(printk("%s: no ha matching inst %d.\n",
		    __func__, bp->HbaInstance);)

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (ret);
	}

	DEBUG9(printk("%s: ha inst %ld, buff %p.\n",
	    __func__, ha->instance, bp);)

	if (ha->flags.failover_enabled)
		if ((host = qla2x00_cfg_find_host(ha)) == NULL) {
			DEBUG2_9_10(printk("%s: no HOST for ha inst %ld.\n",
			    __func__, ha->instance);)
			pext->Status = EXT_STATUS_DEV_NOT_FOUND;
			return (ret);
		}

	if ((entry = (FO_DEVICE_DATA *)qla2x00_kmem_zalloc(sizeof(FO_DEVICE_DATA),
	    GFP_ATOMIC,14)) == NULL) {
		DEBUG2_9_10(printk("%s: failed to alloc memory of size (%d)\n",
		    __func__, (int)sizeof(FO_DEVICE_DATA));)
		pext->Status = EXT_STATUS_NO_MEMORY;
		return (-ENOMEM);
	}

	u_entry = (FO_DEVICE_DATA *) pext->ResponseAdr;

	/* find the correct fcport list */
	if (!ha->flags.failover_enabled)
		fcport = ha->fcport;
	else
		fcport = host->fcport;

	/* Check thru this adapter's fcport list */
	for (i = 0; fcport && i < MAX_TARGETS; i++, fcport = fcport->next) {

		memcpy(entry->WorldWideName,
		    fcport->node_name, EXT_DEF_WWN_NAME_SIZE);
		memcpy(entry->PortName,
		    fcport->port_name, EXT_DEF_WWN_NAME_SIZE);

		for (b = 0; b < 3 ; b++)
			entry->PortId[b] = fcport->d_id.r.d_id[2-b];

		if (!ha->flags.failover_enabled) {
			/*
			 * Failover disabled. Just find the port and return
			 * target info.
			 */
			for (cnt = 0; cnt < MAX_FIBRE_DEVICES; cnt++) {
				if (!(ostgt = ha->otgt[cnt])) {
					continue;
				}

				if (ostgt->vis_port == fcport) {
					entry->TargetId = cnt;
					break;
				}
			}
			if (cnt == MAX_FIBRE_DEVICES) {
				/* Not found?  For now just go to next port. */
#if defined(QL_DEBUG_LEVEL_2) || defined(QL_DEBUG_LEVEL_10)
				uint8_t          *tmp_name;
#if USE_PORTNAME
				tmp_name = fcport->port_name;
#else
				tmp_name = fcport->node_name;
#endif

				printk("fo_get_target_data(%ld): ERROR "
				    "port %02x%02x%02x%02x%02x%02x%02x%02x "
				    "not configured.\n", ha->host_no,
				    tmp_name[0], tmp_name[1], tmp_name[2],
				    tmp_name[3], tmp_name[4], tmp_name[5],
				    tmp_name[6], tmp_name[7]);
#endif /* DEBUG */

				continue;
			}

			entry->MultipathControl = 0; /* always configured */

			ret = verify_area(VERIFY_WRITE, (void *)u_entry,
			    sizeof(FO_DEVICE_DATA));
			if (ret) {
				/* error */
				DEBUG2_9_10(printk("%s: u_entry %p verify "
				    " wrt err. tgt id=%d.\n",
				    __func__, u_entry, cnt);)
				pext->Status = EXT_STATUS_COPY_ERR;
				break;
			}

			ret = copy_to_user(u_entry, entry,
			    sizeof(FO_DEVICE_DATA));
			if (ret) {
				/* error */
				DEBUG2_9_10(printk("%s: u_entry %p copy "
				    "out err. tgt id=%d.\n",
				    __func__, u_entry, cnt);)
				pext->Status = EXT_STATUS_COPY_ERR;
				break;
			}

			u_entry++;

			continue;
		}

		/*
		 * Failover is enabled. Go through the mp_devs list and
		 * get target data in configured path.
		 */
		for (dev_no = 0; dev_no < MAX_MP_DEVICES; dev_no++) {
			dp = host->mp_devs[dev_no];

			if (dp == NULL)
				continue;

			/* Lookup entry name */
			if (!qla2x00_is_portname_in_device(dp, entry->PortName))
				continue;

			if ((pathlist = dp->path_list) == NULL)
				continue;

			path = pathlist->last;
			for (path_id = 0; path_id < pathlist->path_cnt;
			    path_id++, path= path->next) {

				if (path->host != host)
					continue;

				if (!qla2x00_is_portname_equal(path->portname,
				    entry->PortName))
					continue;

				entry->TargetId = dp->dev_id;
				entry->Dev_No = path->id;
				entry->MultipathControl = path->mp_byte;

				DEBUG9(printk("cfg_get_target_data: path->id "
				    "= %d, target data = 0x%x \n",
				    path->id, path->mp_byte);)

				ret = verify_area(VERIFY_WRITE, (void *)u_entry,
				    sizeof(FO_DEVICE_DATA));
				if (ret) {
					/* error */
					DEBUG2_9_10(printk("%s: u_entry %p "
					    "verify wrt err. tgt id=%d.\n",
					    __func__, u_entry, dp->dev_id);)
					pext->Status = EXT_STATUS_COPY_ERR;
					break;
				}

				ret = copy_to_user(u_entry, entry,
				    sizeof(FO_DEVICE_DATA));
				if (ret) {
					/* error */
					DEBUG2_9_10(printk("%s: u_entry %p "
					    "copy out err. tgt id=%d.\n",
					    __func__, u_entry, dp->dev_id);)
					pext->Status = EXT_STATUS_COPY_ERR;
					break;
				}

				u_entry++;

				/* Path found. Continue with next fcport */
				break;
			}
			break;
		}
	}

	if (ret == 0) {
		pext->ResponseLen = sizeof(FO_DEVICE_DATABASE);
	}

	KMEM_FREE(entry, sizeof(FO_DEVICE_DATA));

	DEBUG9(printk("%s: exiting. ret = %d.\n", __func__, ret);)

	return (ret);
}

/*
 * qla2x00_fo_set_target_data
 *      Set multipath control byte for all devices on the attached hba
 *
 * Input:
 *      bp = pointer to buffer
 *
 * Return;
 *      0 on success or errno.
 *
 * Context:
 *      Kernel context.
 */
static int
qla2x00_fo_set_target_data(EXT_IOCTL *pext, FO_TARGET_DATA_INPUT  *bp, int mode)
{
	scsi_qla_host_t  *ha;
	int              i;
	int              ret = 0;
	mp_host_t        *host;
	mp_device_t      *dp;
	mp_path_t        *path;
	mp_path_list_t   *pathlist;
	uint16_t         dev_no;
	uint8_t	         path_id;
	FO_DEVICE_DATA *entry, *u_entry;

	DEBUG9(printk("%s: entered.\n", __func__);)

	ha = qla2x00_get_hba((int)bp->HbaInstance);

	if (!ha) {
		DEBUG2_9_10(printk("%s: no ha matching inst %d.\n",
		    __func__, bp->HbaInstance);)

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (ret);
	}

	DEBUG9(printk("%s: ha inst %ld, buff %p.\n",
	    __func__, ha->instance, bp);)

	if (!ha->flags.failover_enabled)
		/* non-failover mode. nothing to be done. */
		return 0;

	if ((host = qla2x00_cfg_find_host(ha)) == NULL) {
		DEBUG2_9_10(printk("%s: no HOST for ha inst %ld.\n",
		    __func__, ha->instance);)
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (ret);
	}

	if ((entry = (FO_DEVICE_DATA *)qla2x00_kmem_zalloc(sizeof(FO_DEVICE_DATA),
	    GFP_ATOMIC,15)) == NULL) {
		DEBUG2_9_10(printk("%s: failed to alloc memory of size (%d)\n",
		    __func__, (int)sizeof(FO_DEVICE_DATA));)
		pext->Status = EXT_STATUS_NO_MEMORY;
		return (-ENOMEM);
	}

	u_entry = (FO_DEVICE_DATA *)(pext->RequestAdr +
	    sizeof(FO_TARGET_DATA_INPUT));

	for (i = 0; i < MAX_TARGETS; i++, u_entry++) {
		ret = verify_area(VERIFY_READ, (void *)u_entry,
		    sizeof(FO_DEVICE_DATA));
		if (ret) {
			/* error */
			DEBUG2_9_10(printk("%s: u_entry %p verify read err.\n",
			    __func__, u_entry);)
			pext->Status = EXT_STATUS_COPY_ERR;
			break;
		}

		ret = copy_from_user(entry, u_entry, sizeof(FO_DEVICE_DATA));

		if (ret) {
			/* error */
			DEBUG2_9_10(printk("%s: u_entry %p copy error.\n",
			    __func__, u_entry);)
			pext->Status = EXT_STATUS_COPY_ERR;
			break;
		}

		for (dev_no = 0; dev_no < MAX_MP_DEVICES; dev_no++) {
			dp = host->mp_devs[dev_no];

			if (dp == NULL)
				continue;

			/* Lookup entry name */
			if (!qla2x00_is_portname_in_device(dp, entry->PortName))
				continue;

			if ((pathlist = dp->path_list) == NULL)
				continue;

			path = pathlist->last;
			for (path_id = 0; path_id < pathlist->path_cnt;
			    path_id++, path= path->next) {

				if (path->host != host)
					continue;

				if (!qla2x00_is_portname_equal(path->portname,
				    entry->PortName))
					continue;

				path->mp_byte = entry->MultipathControl;

				DEBUG9(printk("cfg_set_target_data: %d target "
				    "data = 0x%x \n",
				    path->id,path->mp_byte);)

				/*
				 * If this is the visible path, then make it
				 * available on next reboot.
				 */
				if (!((path->mp_byte & MP_MASK_HIDDEN) ||
				    (path->mp_byte & MP_MASK_UNCONFIGURED))) {
					pathlist->visible = path->id;
				}

				/* Found path. Go to next entry. */
				break;
			}
			break;
		}
	}

	KMEM_FREE(entry, sizeof(FO_DEVICE_DATA));

	DEBUG9(printk("%s: exiting. ret = %d.\n", __func__, ret);)

	return (ret);

}

/*
 * qla2x00_fo_ioctl
 *	Provides functions for failover ioctl() calls.
 *
 * Input:
 *	ha = adapter state pointer.
 *	ioctl_code = ioctl function to perform
 *	arg = Address of application EXT_IOCTL cmd data
 *	mode = flags
 *
 * Returns:
 *	Return value is the ioctl rval_p return value.
 *	0 = success
 *
 * Context:
 *	Kernel context.
 */
/* ARGSUSED */
int
qla2x00_fo_ioctl(scsi_qla_host_t *ha, int ioctl_code, void *ret_arg, int mode)
{
	static EXT_IOCTL cmd_struct;
	int    rval = 0;
	size_t	in_size, out_size;
	static	union {
		FO_PARAMS params;
		FO_GET_PATHS path;
		FO_SET_CURRENT_PATH set_path;
		/* FO_HBA_STAT_INPUT stat; */
		FO_HBA_STAT stat;
		FO_LUN_DATA_INPUT lun_data;
		FO_TARGET_DATA_INPUT target_data;
	} buff;


	ENTER("qla2x00_fo_ioctl");
	DEBUG9(printk("%s: entered. arg (%p):\n", __func__, ret_arg);)

	memcpy(&cmd_struct, ret_arg, sizeof(cmd_struct));

	/*
	 * default case for this switch not needed,
	 * ioctl_code validated by caller.
	 */
	in_size = out_size = 0;
	switch (ioctl_code) {
		case FO_CC_GET_PARAMS:
			out_size = sizeof(FO_PARAMS);
			break;
		case FO_CC_SET_PARAMS:
			in_size = sizeof(FO_PARAMS);
			break;
		case FO_CC_GET_PATHS:
			in_size = sizeof(FO_GET_PATHS);
			break;
		case FO_CC_SET_CURRENT_PATH:
			in_size = sizeof(FO_SET_CURRENT_PATH);
			break;
		case FO_CC_GET_HBA_STAT:
		case FO_CC_RESET_HBA_STAT:
			in_size = sizeof(FO_HBA_STAT_INPUT);
			break;
		case FO_CC_GET_LUN_DATA:
			in_size = sizeof(FO_LUN_DATA_INPUT);
			break;
		case FO_CC_SET_LUN_DATA:
			in_size = sizeof(FO_LUN_DATA_INPUT);
			break;
		case FO_CC_GET_TARGET_DATA:
			in_size = sizeof(FO_TARGET_DATA_INPUT);
			break;
		case FO_CC_SET_TARGET_DATA:
			in_size = sizeof(FO_TARGET_DATA_INPUT);
			break;

	}
	if (in_size != 0) {
		if ((int)cmd_struct.RequestLen < in_size) {
			cmd_struct.Status = EXT_STATUS_INVALID_PARAM;
			cmd_struct.DetailStatus = EXT_DSTATUS_REQUEST_LEN;
			DEBUG10(printk("%s: got invalie req len (%d).\n",
			    __func__, cmd_struct.RequestLen);)

		} else {

			rval = verify_area(VERIFY_READ,
			    (void *)cmd_struct.RequestAdr, in_size);
			if (rval) {
				/* error */
				DEBUG2_9_10(printk("%s: req buf verify read "
				    "error. size=%d.\n",
				    __func__, in_size);)
				cmd_struct.Status = EXT_STATUS_COPY_ERR;
			}
			rval = copy_from_user(&buff,
			    (void *)cmd_struct.RequestAdr, in_size);

			if (rval) {
				DEBUG2_9_10(printk("%s: req buf copy error. "
				    "size=%d.\n",
				    __func__, in_size);)

				cmd_struct.Status = EXT_STATUS_COPY_ERR;
			} else {
				DEBUG9(printk("qla2x00_fo_ioctl: req buf "
				    "copied ok.\n"));
			}
		}
	} else if (out_size != 0 && (int)cmd_struct.ResponseLen < out_size) {
		cmd_struct.Status = EXT_STATUS_BUFFER_TOO_SMALL;
		cmd_struct.DetailStatus = out_size;
		DEBUG10(printk("%s: got invalie resp len (%d).\n",
		    __func__, cmd_struct.ResponseLen);)
	}

	if (rval != 0 || cmd_struct.Status != 0)
		goto done_fo_ioctl;

	cmd_struct.Status = EXT_STATUS_OK;
	cmd_struct.DetailStatus = EXT_STATUS_OK;

	switch (ioctl_code) {
		case FO_CC_GET_PARAMS:
			rval = qla2x00_fo_get_params(&buff.params);
			break;
		case FO_CC_SET_PARAMS:
			rval = qla2x00_fo_set_params(&buff.params);
			break;
		case FO_CC_GET_PATHS:
			rval = qla2x00_cfg_get_paths(&cmd_struct,
					&buff.path,mode);
			if (rval != 0)
				out_size = 0;
			break;
		case FO_CC_SET_CURRENT_PATH:
			rval = qla2x00_cfg_set_current_path(&cmd_struct,
					&buff.set_path,mode);
			break;
		case FO_CC_RESET_HBA_STAT:
			rval = qla2x00_fo_stats(&buff.stat, TRUE);
			break;
		case FO_CC_GET_HBA_STAT:
			rval = qla2x00_fo_stats(&buff.stat, FALSE);
			break;
		case FO_CC_GET_LUN_DATA:

			DEBUG4(printk("calling qla2x00_fo_get_lun_data\n");)
			DEBUG4(printk("cmd_struct.RequestAdr (%p):\n",
					cmd_struct.RequestAdr);)

			rval = qla2x00_fo_get_lun_data(&cmd_struct,
			    &buff.lun_data, mode);

			if (rval != 0)
				out_size = 0;
			break;
		case FO_CC_SET_LUN_DATA:

			DEBUG4(printk("calling qla2x00_fo_set_lun_data\n");)
			DEBUG4(printk("	cmd_struct.RequestAdr (%p):\n",
			    cmd_struct.RequestAdr);)

			rval = qla2x00_fo_set_lun_data(&cmd_struct,
			    &buff.lun_data, mode);
			break;
		case FO_CC_GET_TARGET_DATA:
			DEBUG4(printk("calling qla2x00_fo_get_target_data\n");)
			DEBUG4(printk("cmd_struct.RequestAdr (%p):\n",
			    cmd_struct.RequestAdr);)

			rval = qla2x00_fo_get_target_data(&cmd_struct,
			    &buff.target_data, mode);

			if (rval != 0) {
				out_size = 0;
			}
			break;
		case FO_CC_SET_TARGET_DATA:
			DEBUG4(printk("calling qla2x00_fo_set_target_data\n");)
			DEBUG4(printk("	cmd_struct.RequestAdr (%p):\n",
			    cmd_struct.RequestAdr);)
			rval = qla2x00_fo_set_target_data(&cmd_struct,
			    &buff.target_data, mode);
			break;

	}

	if (rval == 0 && (cmd_struct.ResponseLen = out_size) != 0) {
		rval = verify_area(VERIFY_WRITE, (void *)cmd_struct.ResponseAdr,
		    out_size);
		if (rval != 0) {
			DEBUG10(printk("%s: resp buf very write error.\n",
			    __func__);)
			cmd_struct.Status = EXT_STATUS_COPY_ERR;
		}
	}

	if (rval == 0) {
		rval = copy_to_user((void *)cmd_struct.ResponseAdr,
		    &buff, out_size);

		if (rval != 0) {
			DEBUG10(printk("%s: resp buf copy error. size=%d.\n",
			    __func__, out_size);)
			cmd_struct.Status = EXT_STATUS_COPY_ERR;
		}
	}

done_fo_ioctl:

	/* Set Status and DetailStatus fields in application EXT_IOCTL */
	(((EXT_IOCTL*)ret_arg)->Status) 	= cmd_struct.Status;
	(((EXT_IOCTL*)ret_arg)->DetailStatus)	= cmd_struct.DetailStatus;
	(((EXT_IOCTL*)ret_arg)->ResponseLen)	= cmd_struct.ResponseLen;

	if (rval != 0) {
		/*EMPTY*/
		DEBUG10(printk("%s: **** FAILED ****\n", __func__);)
	} else {
		/*EMPTY*/
		DEBUG9(printk("%s: exiting normally\n", __func__);)
	}

	return rval;
}


/*
 * qla2x00_fo_count_retries
 *	Increment the retry counter for the command.
 *      Set or reset the SRB_RETRY flag.
 *
 * Input:
 *	sp = Pointer to command.
 *
 * Returns:
 *	TRUE -- retry
 * 	FALSE -- don't retry
 *
 * Context:
 *	Kernel context.
 */
static BOOL
qla2x00_fo_count_retries(scsi_qla_host_t *ha, srb_t *sp)
{
	BOOL		retry = TRUE;
	os_lun_t	*lq;
	os_tgt_t	*tq;

	DEBUG9(printk("%s: entered.\n", __func__);)

	if (++sp->fo_retry_cnt >  qla_fo_params.MaxRetriesPerIo) {
		/* no more failovers for this request */
		retry = FALSE;
		sp->fo_retry_cnt = 0;
		printk(KERN_INFO
		    "qla2x00: no more failovers for request - "
		    "pid= %ld\n", sp->cmd->serial_number);
	} else {
		/*
		 * We haven't exceeded the max retries for this request, check
		 * max retries this path
		 */
		if ((sp->fo_retry_cnt % qla_fo_params.MaxRetriesPerPath) == 0) {
			DEBUG(printk(" qla2x00_fo_count_retries: FAILOVER - "
			    "queuing ha=%ld, sp=%p, pid =%ld, "
			    "fo retry= %d \n",
			    ha->host_no,
			    sp, sp->cmd->serial_number,
			    sp->fo_retry_cnt);)

			/*
			 * Note: we don't want it to timeout, so it is
			 * recycling on the retry queue and the fialover queue.
			 */
			lq = sp->lun_queue;
			tq = sp->tgt_queue;
			set_bit(LUN_MPIO_BUSY, &lq->q_flag);

			/*
			 * ??? We can get a path error on any ha, but always
			 * queue failover on originating ha. This will allow us
			 * to syncronized the requests for a given lun.
			 */
			sp->f_start=jiffies;/*ra 10/29/01*/
			/* Now queue it on to be failover */
			sp->ha = ha;
			add_to_failover_queue(ha,sp);
		}
	}

	DEBUG9(printk("%s: exiting. retry = %d.\n", __func__, retry);)

	return retry ;
}


/*
 * qla2x00_fo_check
 *	This function is called from the done routine to see if
 *  the SRB requires a failover.
 *
 *	This function examines the available os returned status and
 *  if meets condition, the command(srb) is placed ont the failover
 *  queue for processing.
 *
 * Input:
 *	sp  = Pointer to the SCSI Request Block
 *
 * Output:
 *      sp->flags SRB_RETRY bit id command is to
 *      be retried otherwise bit is reset.
 *
 * Returns:
 *      None.
 *
 * Context:
 *	Kernel/Interrupt context.
 */
BOOL
qla2x00_fo_check(scsi_qla_host_t *ha, srb_t *sp)
{
	BOOL		retry = FALSE;
	int host_status;
#if DEBUG_QLA2100
	STATIC char *reason[] = {
		"DID_OK",
		"DID_NO_CONNECT",
		"DID_BUS_BUSY",
		"DID_TIME_OUT",
		"DID_BAD_TARGET",
		"DID_ABORT",
		"DID_PARITY",
		"DID_ERROR",
		"DID_RESET",
		"DID_BAD_INTR"
	};
#endif

	DEBUG9(printk("%s: entered.\n", __func__);)

	/* we failover on selction timeouts only */
	host_status = CMD_RESULT(sp->cmd) >>16;
	if( host_status == DID_NO_CONNECT) {
		if( qla2x00_fo_count_retries(ha,sp) ) {
			/* Force a retry  on this request, it will
			 * cause the LINUX timer to get reset, while we
			 * we are processing the failover.
			 */
			CMD_RESULT(sp->cmd) = DID_BUS_BUSY << 16;
			retry = TRUE;
		}
		DEBUG(printk("qla2x00_fo_check: pid= %ld sp %p "
				"retry count=%d, retry flag = %d, "
				"host status (%s)\n\r",
				sp->cmd->serial_number,
				sp, sp->fo_retry_cnt,
				retry, reason[host_status]);)
	}

	DEBUG9(printk("%s: exiting. retry = %d.\n", __func__, retry);)

	return retry;
}

/*
 * qla2x00_fo_path_change
 *	This function is called from configuration mgr to notify
 *	of a path change.
 *
 * Input:
 *      type    = Failover notify type, FO_NOTIFY_LUN_RESET or FO_NOTIFY_LOGOUT
 *      newlunp = Pointer to the fc_lun struct for current path.
 *      oldlunp = Pointer to fc_lun struct for previous path.
 *
 * Returns:
 *
 * Context:
 *	Kernel context.
 */
uint32_t
qla2x00_fo_path_change(uint32_t type, fc_lun_t *newlunp, fc_lun_t *oldlunp)
{
	uint32_t	ret = QLA2X00_SUCCESS;

	newlunp->max_path_retries = 0;
	return ret;
}

/*
 * qla2x00_fo_get_params
 *	Process an ioctl request to get system wide failover parameters.
 *
 * Input:
 *	pp = Pointer to FO_PARAMS structure.
 *
 * Returns:
 *	EXT_STATUS code.
 *
 * Context:
 *	Kernel context.
 */
static uint32_t
qla2x00_fo_get_params(PFO_PARAMS pp)
{
	DEBUG9(printk("%s: entered.\n", __func__);)

	pp->MaxPathsPerDevice = qla_fo_params.MaxPathsPerDevice;
	pp->MaxRetriesPerPath = qla_fo_params.MaxRetriesPerPath;
	pp->MaxRetriesPerIo = qla_fo_params.MaxRetriesPerIo;
	pp->Flags = qla_fo_params.Flags;
	pp->FailoverNotifyType = qla_fo_params.FailoverNotifyType;
	pp->FailoverNotifyCdbLength = qla_fo_params.FailoverNotifyCdbLength;
	memset(pp->FailoverNotifyCdb, 0, sizeof(pp->FailoverNotifyCdb));
	memcpy(pp->FailoverNotifyCdb,
	    &qla_fo_params.FailoverNotifyCdb[0], sizeof(pp->FailoverNotifyCdb));

	DEBUG9(printk("%s: exiting.\n", __func__);)

	return EXT_STATUS_OK;
}

/*
 * qla2x00_fo_set_params
 *	Process an ioctl request to set system wide failover parameters.
 *
 * Input:
 *	pp = Pointer to FO_PARAMS structure.
 *
 * Returns:
 *	EXT_STATUS code.
 *
 * Context:
 *	Kernel context.
 */
static uint32_t
qla2x00_fo_set_params(PFO_PARAMS pp)
{
	DEBUG9(printk("%s: entered.\n", __func__);)

	/* Check values for defined MIN and MAX */
	if ((pp->MaxPathsPerDevice > SDM_DEF_MAX_PATHS_PER_DEVICE) ||
	    (pp->MaxRetriesPerPath < FO_MAX_RETRIES_PER_PATH_MIN) ||
	    (pp->MaxRetriesPerPath > FO_MAX_RETRIES_PER_PATH_MAX) ||
	    (pp->MaxRetriesPerIo < FO_MAX_RETRIES_PER_IO_MIN) ||
	    (pp->MaxRetriesPerPath > FO_MAX_RETRIES_PER_IO_MAX)) {
		DEBUG2_9_10(printk("%s: got invalid params.\n", __func__);)
		return EXT_STATUS_INVALID_PARAM;
	}

	/* Update the global structure. */
	qla_fo_params.MaxPathsPerDevice = pp->MaxPathsPerDevice;
	qla_fo_params.MaxRetriesPerPath = pp->MaxRetriesPerPath;
	qla_fo_params.MaxRetriesPerIo = pp->MaxRetriesPerIo;
	qla_fo_params.Flags = pp->Flags;
	qla_fo_params.FailoverNotifyType = pp->FailoverNotifyType;
	qla_fo_params.FailoverNotifyCdbLength = pp->FailoverNotifyCdbLength;
	if (pp->FailoverNotifyType & FO_NOTIFY_TYPE_CDB) {
		if (pp->FailoverNotifyCdbLength >
		    sizeof(qla_fo_params.FailoverNotifyCdb)) {
			DEBUG2_9_10(printk("%s: got invalid cdb length.\n",
			    __func__);)
			return EXT_STATUS_INVALID_PARAM;
		}

		memcpy(qla_fo_params.FailoverNotifyCdb,
		    pp->FailoverNotifyCdb,
		    sizeof(qla_fo_params.FailoverNotifyCdb));
	}

	DEBUG9(printk("%s: exiting.\n", __func__);)

	return EXT_STATUS_OK;
}


/*
 * qla2x00_fo_init_params
 *	Gets driver configuration file failover properties to initalize
 *	the global failover parameters structure.
 *
 * Input:
 *	ha = adapter block pointer.
 *
 * Context:
 *	Kernel context.
 */
void
qla2x00_fo_init_params(scsi_qla_host_t *ha)
{
	DEBUG3(printk("%s: entered.\n", __func__);)

	/* For parameters that are not completely implemented yet, */

	memset(&qla_fo_params, 0, sizeof(qla_fo_params));

	if(MaxPathsPerDevice) {
		qla_fo_params.MaxPathsPerDevice = MaxPathsPerDevice;
	} else
		qla_fo_params.MaxPathsPerDevice =FO_MAX_PATHS_PER_DEVICE_DEF ;
	if(MaxRetriesPerPath) {
		qla_fo_params.MaxRetriesPerPath = MaxRetriesPerPath;
	} else
		qla_fo_params.MaxRetriesPerPath =FO_MAX_RETRIES_PER_PATH_DEF;
	if(MaxRetriesPerIo) {
		qla_fo_params.MaxRetriesPerIo =MaxRetriesPerIo;
	} else
		qla_fo_params.MaxRetriesPerIo =FO_MAX_RETRIES_PER_IO_DEF;

	qla_fo_params.Flags =  0;
	qla_fo_params.FailoverNotifyType = FO_NOTIFY_TYPE_NONE;

	DEBUG3(printk("%s: exiting.\n", __func__);)

}

/*
 * qla2x00_send_fo_notification
 *      Sends failover notification if needed.  Change the fc_lun pointer
 *      in the old path lun queue.
 *
 * Input:
 *      old_lp = Pointer to old fc_lun.
 *      new_lp = Pointer to new fc_lun.
 *
 * Returns:
 *      Local function status code.
 *
 * Context:
 *      Kernel context.
 */
uint32_t
qla2x00_send_fo_notification(fc_lun_t *old_lp, fc_lun_t *new_lp)
{
	scsi_qla_host_t	*old_ha = old_lp->fcport->ha;
	int		rval = QLA2X00_SUCCESS;
	inq_cmd_rsp_t	*pkt;
	uint16_t	loop_id, lun;
	dma_addr_t	phys_address;


	ENTER("qla2x00_send_fo_notification");
	DEBUG3(printk("%s: entered.\n", __func__);)

	loop_id = old_lp->fcport->loop_id;
	lun = old_lp->lun;

	if (qla_fo_params.FailoverNotifyType == FO_NOTIFY_TYPE_LUN_RESET) {
		rval = qla2x00_lun_reset(old_ha, loop_id, lun);
		if (rval == QLA2X00_SUCCESS) {
			DEBUG4(printk("qla2x00_send_fo_notification: LUN "
			    "reset succeded\n");)
		} else {
			DEBUG4(printk("qla2x00_send_fo_notification: LUN "
			    "reset failed\n");)
		}

	}
	if ( (qla_fo_params.FailoverNotifyType ==
	     FO_NOTIFY_TYPE_LOGOUT_OR_LUN_RESET) ||
	    (qla_fo_params.FailoverNotifyType ==
	     FO_NOTIFY_TYPE_LOGOUT_OR_CDB) )  {

		rval = qla2x00_fabric_logout(old_ha, loop_id);
		if (rval == QLA2X00_SUCCESS) {
			DEBUG4(printk("qla2x00_send_fo_failover_notify: "
			    "logout succeded\n");)
		} else {
			DEBUG4(printk("qla2x00_send_fo_failover_notify: "
			    "logout failed\n");)
		}

	}

	if (qla_fo_params.FailoverNotifyType == FO_NOTIFY_TYPE_CDB) {
		pkt = pci_alloc_consistent(old_ha->pdev,
		    sizeof(inq_cmd_rsp_t), &phys_address);
		if (pkt == NULL) {
			DEBUG4(printk("qla2x00_send_fo_failover_notify: "
			    "memory allocation failed\n");)

			return(QLA2X00_FUNCTION_FAILED);
		}

		memset(pkt,0, sizeof(inq_cmd_rsp_t));
		/* FIXME: COMMAND_A64_TYPE ??? */
		pkt->p.cmd.entry_type = COMMAND_TYPE;
		pkt->p.cmd.entry_count = 1;
		pkt->p.cmd.lun = lun;
		pkt->p.cmd.target = (uint8_t)loop_id;
		pkt->p.cmd.control_flags = CF_SIMPLE_TAG;
		memcpy(pkt->p.cmd.scsi_cdb,
		    qla_fo_params.FailoverNotifyCdb,
		    qla_fo_params.FailoverNotifyCdbLength);
		/* FIXME This setup needs to be verified with Dennis. */
		pkt->p.cmd.dseg_count = __constant_cpu_to_le16(1);
		pkt->p.cmd.byte_count = __constant_cpu_to_le32(0);
		pkt->p.cmd.dseg_0_address = cpu_to_le32(
			phys_address + sizeof (sts_entry_t));
		pkt->p.cmd.dseg_0_length = __constant_cpu_to_le32(0);

		rval = qla2x00_issue_iocb(old_ha, pkt, phys_address,
		    sizeof (inq_cmd_rsp_t));
		if (rval != QLA2X00_SUCCESS ||
		    pkt->p.rsp.comp_status != CS_COMPLETE ||
		    pkt->p.rsp.scsi_status & SS_CHECK_CONDITION ||
		    pkt->inq[0] == 0x7f) {

			DEBUG4(printk("qla2x00_fo_notification: send CDB "
			    "failed: comp_status = %x"
			    "scsi_status = %x inq[0] = %x\n",
			    pkt->p.rsp.comp_status,
			    pkt->p.rsp.scsi_status,
			    pkt->inq[0]);)
		}

		pci_free_consistent(old_ha->pdev,
		    sizeof(inq_cmd_rsp_t), pkt, phys_address);
	}

	DEBUG3(printk("%s: exiting. rval = %d.\n", __func__, rval);)

	return rval;
}


/*
 * qla2100_fo_enabled
 *      Reads and validates the failover enabled property.
 *
 * Input:
 *      ha = adapter state pointer.
 *      instance = HBA number.
 *
 * Returns:
 *      TRUE when failover is authorized else FALSE
 *
 * Context:
 *      Kernel context.
 */
BOOL
qla2x00_fo_enabled(scsi_qla_host_t *ha, int instance)
{
	BOOL enable = FALSE;

	if (ha->flags.failover_enabled)
		enable = TRUE;

	return enable;
}
