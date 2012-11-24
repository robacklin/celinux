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


#define	QLA_PT_CMD_TOV			(60) /* firmware timeout */
#define QLA_PT_CMD_DRV_TOV		(QLA_PT_CMD_TOV + 1) /* drvr timeout */
#define QLA_IOCTL_ACCESS_WAIT_TIME	(QLA_PT_CMD_DRV_TOV + 2) /* wait_q tov */
#define QLA_INITIAL_IOCTLMEM_SIZE	(2 * PAGE_SIZE)
#define QLA_IOCTL_SCRAP_SIZE		2048 /* scrap memory for local use. */

/* ELS related defines */
#define FC_HEADER_LEN		24
#define ELS_RJT_LENGTH		0x08	/* 8  */
#define ELS_RPS_ACC_LENGTH	0x40	/* 64 */
#define ELS_RLS_ACC_LENGTH	0x1C	/* 28 */

/* ELS cmd Reply Codes */
#define ELS_STAT_LS_RJT		0x01
#define ELS_STAT_LS_ACC		0x02

#define IOCTL_INVALID_STATUS    0xffff

#if defined(INTAPI)
#include "inioct.h"
/* from qla_inioct.c */
extern int qla2x00_read_nvram(scsi_qla_host_t *, EXT_IOCTL *, int);
extern int qla2x00_update_nvram(scsi_qla_host_t *, EXT_IOCTL *, int);
extern int qla2x00_write_nvram_word(scsi_qla_host_t *, uint8_t, uint16_t);
extern int qla2x00_send_loopback(scsi_qla_host_t *, EXT_IOCTL *, int);
extern int qla2x00_read_option_rom(scsi_qla_host_t *, EXT_IOCTL *, int);
extern int qla2x00_update_option_rom(scsi_qla_host_t *, EXT_IOCTL *, int);
#endif


/*
 * Local prototypes
 */
STATIC int qla2x00_alloc_ioctl_mem(scsi_qla_host_t *);
STATIC int qla2x00_get_new_ioctl_dma_mem(scsi_qla_host_t *, uint32_t);
STATIC void qla2x00_free_ioctl_mem(scsi_qla_host_t *);

STATIC int qla2x00_get_ioctl_scrap_mem(scsi_qla_host_t *, void **, uint32_t);
STATIC void qla2x00_free_ioctl_scrap_mem(scsi_qla_host_t *);

#if defined(ISP2300)
STATIC uint8_t qla2x00_get_next_free_pub_id(scsi_qla_host_t *, uint16_t *);
STATIC uint8_t qla2x00_host_relogin(scsi_qla_host_t *, fcdev_t *);
#endif

STATIC int qla2x00_find_curr_ha(uint16_t, scsi_qla_host_t **);

STATIC int qla2x00_get_driver_specifics(EXT_IOCTL *);

STATIC int qla2x00_aen_reg(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_aen_get(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC void qla2x00_enqueue_aen(scsi_qla_host_t *, uint16_t, void *);

STATIC int qla2x00_query(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_query_hba_node(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_query_hba_port(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_query_disc_port(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_query_disc_tgt(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_query_chip(scsi_qla_host_t *, EXT_IOCTL *, int);

STATIC int qla2x00_get_data(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_get_statistics(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_get_fc_statistics(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_get_port_summary(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_query_driver(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_query_fw(scsi_qla_host_t *, EXT_IOCTL *, int);

#if defined(ISP2300)
STATIC int qla2x00_send_els_passthru(scsi_qla_host_t *, EXT_IOCTL *, int);
#endif
STATIC int qla2x00_send_fcct(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_start_ms_cmd(scsi_qla_host_t *, EXT_IOCTL *, srb_t *,
    EXT_ELS_PT_REQ *);

STATIC int qla2x00_wwpn_to_scsiaddr(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_scsi_passthru(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_sc_scsi_passthru(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_sc_fc_scsi_passthru(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_sc_scsi3_passthru(scsi_qla_host_t *, EXT_IOCTL *, int);

STATIC int qla2x00_send_els_rnid(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_get_rnid_params(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_set_host_data(scsi_qla_host_t *, EXT_IOCTL *, int);
STATIC int qla2x00_set_rnid_params(scsi_qla_host_t *, EXT_IOCTL *, int);

STATIC void qla2x00_waitq_sem_timeout(unsigned long);
STATIC uint8_t qla2x00_get_ioctl_access(scsi_qla_host_t *, uint32_t);
STATIC uint8_t qla2x00_release_ioctl_access(scsi_qla_host_t *);

STATIC void qla2x00_wait_q_memb_alloc(scsi_qla_host_t *, wait_q_t **);
STATIC void qla2x00_wait_q_memb_free(scsi_qla_host_t *, wait_q_t *);
STATIC uint8_t qla2x00_wait_q_add(scsi_qla_host_t *, wait_q_t **);
STATIC void qla2x00_wait_q_get_next(scsi_qla_host_t *, wait_q_t **);
STATIC void qla2x00_wait_q_remove(scsi_qla_host_t *, wait_q_t *);


/*
 * qla2x00_ioctl_sleep_done
 *
 * Description:
 *   This is the callback function to wakeup ioctl completion semaphore
 *   for the ioctl request that is waiting.
 *
 * Input:
 *   sem - pointer to the ioctl completion semaphore.
 *
 * Returns:
 */
STATIC void
qla2x00_ioctl_sleep_done(struct semaphore * sem)
{
	DEBUG9(printk("%s: entered.\n", __func__);)

	if (sem != NULL){
		DEBUG9(printk("ioctl_sleep: wake up sem.\n");)
		up(sem);
	}

	DEBUG9(printk("%s: exiting.\n", __func__);)
}

/*
 * qla2x00_ioctl_sem_init
 *
 * Description:
 *   Initialize the ioctl timer and semaphore used to wait for passthru
 *   completion.
 *
 * Input:
 *   ha - pointer to scsi_qla_host_t structure used for initialization.
 *
 * Returns:
 *   None.
 */
STATIC void
qla2x00_ioctl_sem_init(scsi_qla_host_t *ha)
{
	init_MUTEX_LOCKED(&ha->ioctl->cmpl_sem);
	init_timer(&(ha->ioctl->cmpl_timer));
	ha->ioctl->cmpl_timer.data = (unsigned long)&ha->ioctl->cmpl_sem;
	ha->ioctl->cmpl_timer.function =
	    (void (*)(unsigned long))qla2x00_ioctl_sleep_done;
}

/*
 * qla2x00_scsi_pt_done
 *
 * Description:
 *   Resets ioctl progress flag and wakes up the ioctl completion semaphore.
 *
 * Input:
 *   pscsi_cmd - pointer to the passthru Scsi cmd structure which has completed.
 *
 * Returns:
 */
STATIC void
qla2x00_scsi_pt_done(Scsi_Cmnd *pscsi_cmd)
{
	struct Scsi_Host *host;
	scsi_qla_host_t  *ha;

	host = pscsi_cmd->host;
	ha = (scsi_qla_host_t *) host->hostdata;

	DEBUG9(printk("%s post function called OK\n", __func__);)

	/* save detail status for IOCTL reporting */
	ha->ioctl->SCSIPT_InProgress = 0;
	ha->ioctl->ioctl_tov = 0;

	up(&ha->ioctl->cmpl_sem);

	DEBUG9(printk("%s: exiting.\n", __func__);)

	return;
}

/*
 * qla2x00_msiocb_done
 *
 * Description:
 *   Resets MSIOCB ioctl progress flag and wakes up the ioctl completion
 *   semaphore.
 *
 * Input:
 *   cmd - pointer to the passthru Scsi cmd structure which has completed.
 *
 * Returns:
 */
STATIC void
qla2x00_msiocb_done(Scsi_Cmnd *pscsi_cmd)
{
	struct Scsi_Host *host;
	scsi_qla_host_t  *ha;

	host = pscsi_cmd->host;
	ha = (scsi_qla_host_t *) host->hostdata;

	DEBUG9(printk("%s post function called OK\n", __func__);)

	ha->ioctl->MSIOCB_InProgress = 0;
	ha->ioctl->ioctl_tov = 0;

	up(&ha->ioctl->cmpl_sem);

	DEBUG9(printk("%s: exiting.\n", __func__);)
		
	return;
}

/*************************************************************************
 * qla2x00_ioctl
 *
 * Description:
 *   Performs additional ioctl requests not satisfied by the upper levels.
 *
 * Returns:
 *   ret  = 0    Success
 *   ret != 0    Failed; detailed status copied to EXT_IOCTL structure
 *               if possible
 *************************************************************************/
int
qla2x00_ioctl(Scsi_Device *dev, int cmd, void *arg)
{
	int		mode = 0;
	int		tmp_rval = 0;
	int		ret = -EINVAL;

	uint8_t		*temp;
	uint8_t		tempbuf[8];
	uint32_t	i;
	uint32_t	status;

	EXT_IOCTL	*pext;

	scsi_qla_host_t	*ha;


	DEBUG9(printk("%s: entry to command (%x), arg (%p)\n",
	    __func__, cmd, arg);)

	/* Catch any non-exioct ioctls */
	if (_IOC_TYPE(cmd) != QLMULTIPATH_MAGIC) {
		return (ret);
	}

	ret = verify_area(VERIFY_READ, (void *)arg, sizeof(EXT_IOCTL));
	if (ret) {
		DEBUG9_10(printk("%s: ERROR VERIFY_READ EXT_IOCTL "
		    "sturct. cmd=%d arg=%p.\n", __func__, cmd, arg);)
		return (ret);
	}

	/* Allocate ioctl structure buffer to support multiple concurrent
	 * entries.
	 */
	pext = KMEM_ZALLOC(sizeof(EXT_IOCTL), 16);
	if (pext == NULL) {
		/* error */
		printk(KERN_WARNING
		    "qla2x00: ERROR in main ioctl buffer allocation.\n");
		return (-ENOMEM);
	}

	/* copy in application layer EXT_IOCTL */
	ret = copy_from_user(pext, arg, sizeof(EXT_IOCTL));
	if (ret) {
		DEBUG9_10(printk("%s: ERROR COPY_FROM_USER "
		    "EXT_IOCTL sturct. cmd=%d arg=%p.\n",
		    __func__, cmd, arg);)

		KMEM_FREE(pext, sizeof(EXT_IOCTL));
		return (ret);
	}

	/* Verify before update status fields in EXT_IOCTL struct. */
	ret = verify_area(VERIFY_WRITE, (void *)arg, sizeof(EXT_IOCTL));
	if (ret) {
		DEBUG9_10(printk("%s: ERROR VERIFY_WRITE EXT_IOCTL "
		    "sturct. cmd=%d arg=%p.\n", __func__, cmd, arg);)

		KMEM_FREE(pext, sizeof(EXT_IOCTL));
		return (ret);
	}

	/* check signature of this ioctl */
	temp = (uint8_t *) &pext->Signature;

	for (i = 0; i < 4; i++, temp++)
		tempbuf[i] = *temp;

	if ((tempbuf[0] == 'Q') && (tempbuf[1] == 'L') &&
	    (tempbuf[2] == 'O') && (tempbuf[3] == 'G'))
		status = 0;
	else
		status = 1;

	if (status != 0) {
		DEBUG9_10(printk("%s: signature did not match. "
		    "cmd=%d arg=%p.\n", __func__, cmd, arg);)
		pext->Status = EXT_STATUS_INVALID_PARAM;
		copy_to_user((void *)arg, (void *)pext, sizeof(EXT_IOCTL));

		KMEM_FREE(pext, sizeof(EXT_IOCTL));
		return (-EINVAL);
	}

	/* check version of this ioctl */
	if (pext->Version > EXT_VERSION) {
		printk(KERN_WARNING
		    "qla2x00: ioctl interface version not supported = %d.\n",
		    pext->Version);
		pext->Status = EXT_STATUS_UNSUPPORTED_VERSION;
		copy_to_user((void *)arg, (void *)pext, sizeof(EXT_IOCTL));

		KMEM_FREE(pext, sizeof(EXT_IOCTL));
		return (-EINVAL);
	}

	/* check for special cmds used during application's setup time. */
	switch (cmd) {
	case EXT_CC_STARTIOCTL:
		DEBUG9(printk("%s: got startioctl command.\n", __func__);)

		pext->Instance = num_hosts;
		pext->Status = EXT_STATUS_OK;
		ret = copy_to_user((void *)arg, (void *)pext,
		    sizeof(EXT_IOCTL));

		KMEM_FREE(pext, sizeof(EXT_IOCTL));
		return (ret);

	case EXT_CC_SETINSTANCE:
		/* This call is used to return the HBA's host number to
		 * ioctl caller.  All subsequent ioctl commands will put
		 * the host number in HbaSelect field to tell us which
		 * HBA is the destination.
		 */
		if (pext->Instance < num_hosts) {
			if (!((uint32_t)pext->VendorSpecificData &
			    EXT_DEF_USE_HBASELECT)) {
				/* Backward compatible code. */
				apiHBAInstance = pext->Instance;
			}

			/*
			 * Return host number via pext->HbaSelect for
			 * specified API instance number.
			 */
			if (qla2x00_find_curr_ha(pext->Instance, &ha) != 0) {
				pext->Status = EXT_STATUS_DEV_NOT_FOUND;
				ret = copy_to_user(arg, pext, sizeof(EXT_IOCTL));
				DEBUG9_10(printk("%s: SETINSTANCE invalid inst "
				    "%d. num_hosts=%d ha=%p ret=%d.\n",
				    __func__, pext->Instance, num_hosts, ha,
				    ret);)

				KMEM_FREE(pext, sizeof(EXT_IOCTL));
				return (ret); /* ioctl completed ok */
			}

			pext->HbaSelect = ha->host_no;
			pext->Status = EXT_STATUS_OK;

			DEBUG9(printk("%s: Matching instance %d to hba "
			    "%ld.\n", __func__, pext->Instance, ha->host_no);)
		} else {
			DEBUG9_10(printk("%s: ERROR EXT_SETINSTANCE."
			    " Instance=%d num_hosts=%d ha=%p.\n",
			    __func__, pext->Instance, num_hosts, ha);)

			pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		}
		ret = copy_to_user(arg, pext, sizeof(EXT_IOCTL));
		KMEM_FREE(pext, sizeof(EXT_IOCTL));

		DEBUG9(printk("%s: SETINSTANCE exiting. ret=%d.\n",
		    __func__, ret);)

		return (ret);

	case EXT_CC_DRIVER_SPECIFIC:
		ret = qla2x00_get_driver_specifics(pext);
		tmp_rval = copy_to_user(arg, (void *)pext, sizeof(EXT_IOCTL));

		if (ret == 0)
			ret = tmp_rval;

		KMEM_FREE(pext, sizeof(EXT_IOCTL));
		return (ret);

	default:
		break;
	}

	if (!((uint32_t)pext->VendorSpecificData & EXT_DEF_USE_HBASELECT)) {
		/* Backward compatible code. */
		/* Will phase out soon. */

		/* Check for valid apiHBAInstance (set previously by
		 * EXT_SETINSTANCE or default 0)  and set ha context
		 * for this IOCTL.
		 */
		if (qla2x00_find_curr_ha(apiHBAInstance, &ha) != 0) {

			DEBUG9_10(printk("%s: ERROR matching apiHBAInstance "
			    "%d to an HBA Instance.\n",
			    __func__, pext->HbaSelect);)

			pext->Status = EXT_STATUS_DEV_NOT_FOUND;
			copy_to_user(arg, pext, sizeof(EXT_IOCTL));

			KMEM_FREE(pext, sizeof(EXT_IOCTL));
			return (-EINVAL);
		}

		DEBUG9(printk("%s: active apiHBAInstance=%d host_no=%ld "
		    "CC=%x SC=%x.\n",
		    __func__, apiHBAInstance, ha->host_no, cmd, pext->SubCode);)

	} else {
		/* Use HbaSelect value to get a matching ha instance
		 * for this ioctl command.
		 */
		if (qla2x00_find_curr_ha(pext->HbaSelect, &ha) != 0) {

			DEBUG9_10(printk("%s: ERROR matching pext->HbaSelect "
			    "%d to an HBA Instance.\n",
			    __func__, pext->HbaSelect);)

			pext->Status = EXT_STATUS_DEV_NOT_FOUND;
			copy_to_user(arg, pext, sizeof(EXT_IOCTL));

			KMEM_FREE(pext, sizeof(EXT_IOCTL));
			return (-EINVAL);
		}

		DEBUG9(printk("%s: active host_inst=%ld CC=%x SC=%x.\n",
		    __func__, ha->instance, cmd, pext->SubCode);)
	}

	/*
	 * Get permission to process ioctl command. Only one will proceed
	 * at a time.
	 */
	if (qla2x00_get_ioctl_access(ha, QLA_IOCTL_ACCESS_WAIT_TIME) != 0) {
		/* error timed out */
		DEBUG9_10(printk("%s: ERROR timeout getting ioctl "
		    "access. host no=%d.\n", __func__, pext->HbaSelect);)

		pext->Status = EXT_STATUS_BUSY;
		copy_to_user(arg, pext, sizeof(EXT_IOCTL));

		KMEM_FREE(pext, sizeof(EXT_IOCTL));
		return (-EBUSY);
	}


	while (test_bit(CFG_ACTIVE, &ha->cfg_flags) || ha->dpc_active) {
		if( signal_pending(current) )
			break;   /* get out */

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
	}

	switch (cmd) { /* switch on EXT IOCTL COMMAND CODE */

	case EXT_CC_QUERY:
		DEBUG9(printk("%s: got query command.\n", __func__);)

		ret = qla2x00_query(ha, pext, 0);

		break;

	case EXT_CC_GET_DATA:
		DEBUG9(printk("%s: got get_data command.\n", __func__);)

		ret = qla2x00_get_data(ha, pext, 0);

		break;

	case EXT_CC_SEND_FCCT_PASSTHRU:
		DEBUG9(printk("%s: got CT passthru cmd.\n", __func__));

		ret = qla2x00_send_fcct(ha, pext, 0);

		break;

	case EXT_CC_SEND_SCSI_PASSTHRU:
		DEBUG9(printk("%s: got SCSI passthru cmd.\n", __func__));

		ret = qla2x00_scsi_passthru(ha, pext, mode);

		break;

	case EXT_CC_REG_AEN:
		ret = qla2x00_aen_reg(ha, pext, mode);

		break;

	case EXT_CC_GET_AEN:
		ret = qla2x00_aen_get(ha, pext, mode);

		break;

	case EXT_CC_WWPN_TO_SCSIADDR:
		ret = qla2x00_wwpn_to_scsiaddr(ha, pext, 0);
		break;

	case EXT_CC_SEND_ELS_RNID:
		ret = qla2x00_send_els_rnid(ha, pext, mode);
		break;

	case EXT_CC_SET_DATA:
		ret = qla2x00_set_host_data(ha, pext, mode);
		break;                                                          

#if defined(INTAPI)
	case INT_CC_READ_NVRAM:
		ret = qla2x00_read_nvram(ha, pext, mode);

		break;

	case INT_CC_UPDATE_NVRAM:
		ret = qla2x00_update_nvram(ha, pext, mode);

		break;

	case INT_CC_LOOPBACK:
		ret = qla2x00_send_loopback(ha, pext, mode);

		break;

	case INT_CC_READ_OPTION_ROM:
		ret = qla2x00_read_option_rom(ha, pext, mode);

		break;

	case INT_CC_UPDATE_OPTION_ROM:
		ret = qla2x00_update_option_rom(ha, pext, mode);

		break;
#endif /* INTAPI */

#if defined(ISP2300)
	case EXT_CC_SEND_ELS_PASSTHRU:
		DEBUG9(printk("%s: got ELS passthru cmd.\n", __func__));

		ret = qla2x00_send_els_passthru(ha, pext, 0);

		break;
#endif

	/* all others go here */
	/*
	   case EXT_CC_PLATFORM_REG:
	   break;
	 */

	/* Failover IOCTLs */
	case FO_CC_GET_PARAMS:
	case FO_CC_SET_PARAMS:
	case FO_CC_GET_PATHS:
	case FO_CC_SET_CURRENT_PATH:
	case FO_CC_RESET_HBA_STAT:
	case FO_CC_GET_HBA_STAT:
	case FO_CC_GET_LUN_DATA:
	case FO_CC_SET_LUN_DATA:
	case FO_CC_GET_TARGET_DATA:
	case FO_CC_SET_TARGET_DATA:
		DEBUG9(printk("%s: failover arg (%p):\n", __func__, arg);)

		qla2x00_fo_ioctl(ha, cmd, pext, mode);

		break;

	default:
		pext->Status = EXT_STATUS_INVALID_REQUEST;
		break;

	} /* end of CC decode switch */

	/* Always try to copy values back regardless what happened before. */
	tmp_rval = copy_to_user(arg, (void *)pext, sizeof(EXT_IOCTL));

	if (ret == 0)
		ret = tmp_rval;

	DEBUG9(printk("%s: exiting. tmp_rval(%d) ret(%d)\n",
	    __func__, tmp_rval, ret);)

	qla2x00_release_ioctl_access(ha);

	KMEM_FREE(pext, sizeof(EXT_IOCTL));
	return (ret);
}

/*
 * qla2x00_alloc_ioctl_mem
 *	Allocates memory needed by IOCTL code.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_alloc_ioctl_mem(scsi_qla_host_t *ha)
{
	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_new_ioctl_dma_mem(ha, QLA_INITIAL_IOCTLMEM_SIZE) !=
	    QL_STATUS_SUCCESS) {
		printk(KERN_WARNING
		    "qla2x00: ERROR in ioctl physical memory allocation\n");

		return QL_STATUS_RESOURCE_ERROR;
	}

	/* Allocate context memory buffer */
	ha->ioctl = KMEM_ZALLOC(sizeof(hba_ioctl_context), 11);
	if (ha->ioctl == NULL) {
		/* error */
		printk(KERN_WARNING
		    "qla2x00: ERROR in ioctl context allocation.\n");
		return QL_STATUS_RESOURCE_ERROR;
	}

	/* Allocate AEN tracking buffer */
	ha->ioctl->aen_tracking_queue =
	    KMEM_ZALLOC(EXT_DEF_MAX_AEN_QUEUE * sizeof(EXT_ASYNC_EVENT), 12);
	if (ha->ioctl->aen_tracking_queue == NULL) {
		printk(KERN_WARNING
		    "qla2x00: ERROR in ioctl aen_queue allocation.\n");
		return QL_STATUS_RESOURCE_ERROR;
	}

	ha->ioctl->ioctl_tq = KMEM_ZALLOC(sizeof(os_tgt_t), 13);
	if (ha->ioctl->ioctl_tq == NULL) {
		printk(KERN_WARNING
		    "qla2x00: ERROR in ioctl tgt queue allocation.\n");
		return QL_STATUS_RESOURCE_ERROR;
	}

	ha->ioctl->ioctl_lq = KMEM_ZALLOC(sizeof(os_lun_t), 14);
	if (ha->ioctl->ioctl_lq == NULL) {
		printk(KERN_WARNING
		    "qla2x00: ERROR in ioctl lun queue allocation.\n");
		return QL_STATUS_RESOURCE_ERROR;
	}
	/*INIT_LIST_HEAD(&(ha->ioctl->ioctl_lq->cmd));*/

	/* Pick the largest size we'll need per ha of all ioctl cmds.
	 * Use this size when freeing.
	 */
	ha->ioctl->scrap_mem = KMEM_ZALLOC(QLA_IOCTL_SCRAP_SIZE, 15);
	if (ha->ioctl->scrap_mem == NULL) {
		printk(KERN_WARNING
		    "qla2x00: ERROR in ioctl scrap_mem allocation.\n");
		return QL_STATUS_RESOURCE_ERROR;
	}
	ha->ioctl->scrap_mem_size = QLA_IOCTL_SCRAP_SIZE;
	ha->ioctl->scrap_mem_used = 0;
	DEBUG9(printk("%s(%ld): scrap_mem_size=%d.\n",
	    __func__, ha->host_no, ha->ioctl->scrap_mem_size);)

	ha->ioctl->ioctl_lq->q_state = LUN_STATE_READY;
#ifdef __VMWARE__
	spin_lock_init(&ha->ioctl->ioctl_lq->q_lock);
#else
	ha->ioctl->ioctl_lq->q_lock = SPIN_LOCK_UNLOCKED;
#endif

	/* Init wait_q fields */
#ifdef __VMWARE__
	spin_lock_init(&ha->ioctl->wait_q_lock);
#else
	ha->ioctl->wait_q_lock = SPIN_LOCK_UNLOCKED;
#endif

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return QL_STATUS_SUCCESS;
}

/*
 * qla2x00_get_new_ioctl_dma_mem
 *	Allocates dma memory of the specified size.
 *	This is done to replace any previously allocated ioctl dma buffer.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_get_new_ioctl_dma_mem(scsi_qla_host_t *ha, uint32_t size)
{
	DEBUG9(printk("%s entered.\n", __func__);)

	if (ha->ioctl_mem) {
		DEBUG9(printk("%s: ioctl_mem was previously allocated. "
		    "Dealloc old buffer.\n", __func__);)

	 	/* free the memory first */
	 	pci_free_consistent(ha->pdev, ha->ioctl_mem_size, ha->ioctl_mem,
		    ha->ioctl_mem_phys);
	}

	/* Get consistent memory allocated for ioctl I/O operations. */
	ha->ioctl_mem = pci_alloc_consistent(ha->pdev,
	    size, &ha->ioctl_mem_phys);

	if (ha->ioctl_mem == NULL) {
		printk(KERN_WARNING
		    "%s: ERROR in ioctl physical memory allocation. "
		    "Requested length=%x.\n", __func__, size);

		ha->ioctl_mem_size = 0;
		return QL_STATUS_RESOURCE_ERROR;
	}
	ha->ioctl_mem_size = size;

	DEBUG9(printk("%s exiting.\n", __func__);)

	return QL_STATUS_SUCCESS;
}

/*
 * qla2x00_free_ioctl_mem
 *	Frees memory used by IOCTL code for the specified ha.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Context:
 *	Kernel context.
 */
STATIC void
qla2x00_free_ioctl_mem(scsi_qla_host_t *ha)
{
	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (ha->ioctl != NULL) {

		if (ha->ioctl->scrap_mem != NULL) {
			/* The size here must match up to what we
			 * allocated before.
			 */
			KMEM_FREE(ha->ioctl->scrap_mem,
			    ha->ioctl->scrap_mem_size);
			ha->ioctl->scrap_mem = NULL;
			ha->ioctl->scrap_mem_size = 0;
		}

		if (ha->ioctl->ioctl_tq != NULL) {
			KMEM_FREE(ha->ioctl->ioctl_tq, sizeof(os_tgt_t));
			ha->ioctl->ioctl_tq = NULL;
		}

		if (ha->ioctl->ioctl_lq != NULL) {
#ifdef __VMWARE__
			spin_lock_destroy(&ha->ioctl->ioctl_lq->q_lock);
			spin_lock_destroy(&ha->ioctl->wait_q_lock);
#endif
			KMEM_FREE(ha->ioctl->ioctl_lq, sizeof(os_lun_t));
			ha->ioctl->ioctl_lq = NULL;
		}

		if (ha->ioctl->aen_tracking_queue != NULL) {
			KMEM_FREE(ha->ioctl->aen_tracking_queue,
			    EXT_DEF_MAX_AEN_QUEUE * sizeof(EXT_ASYNC_EVENT));
			ha->ioctl->aen_tracking_queue = NULL;
		}

		KMEM_FREE(ha->ioctl, sizeof(hba_ioctl_context));
		ha->ioctl = NULL;
	}

	/* free memory allocated for ioctl operations */
	pci_free_consistent(ha->pdev, ha->ioctl_mem_size, ha->ioctl_mem,
	    ha->ioctl_mem_phys);
	ha->ioctl_mem = NULL;

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

}

/*
 * qla2x00_get_ioctl_scrap_mem
 *	Returns pointer to memory of the specified size from the scrap buffer.
 *	This can be called multiple times before the free call as long
 *	as the memory is to be used by the same ioctl command and
 *	there's still memory left in the scrap buffer.
 *
 * Input:
 *	ha = adapter state pointer.
 *	ppmem = pointer to return a buffer pointer.
 *	size = size of buffer to return.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_get_ioctl_scrap_mem(scsi_qla_host_t *ha, void **ppmem, uint32_t size)
{
	int		ret = QL_STATUS_SUCCESS;
	uint32_t	free_mem;

	DEBUG9(printk("%s(%ld): inst=%ld entered. size=%d.\n",
	    __func__, ha->host_no, ha->instance, size);)

	free_mem = ha->ioctl->scrap_mem_size - ha->ioctl->scrap_mem_used;
	if (free_mem >= size) {
		*ppmem = ha->ioctl->scrap_mem + ha->ioctl->scrap_mem_used;
		ha->ioctl->scrap_mem_used += size;
	} else {
		DEBUG10(printk("%s(%ld): no more scrap memory.\n",
		    __func__, ha->host_no);)

		ret = QL_STATUS_ERROR;
	}

	DEBUG9(printk("%s(%ld): exiting. ret=%d.\n",
	    __func__, ha->host_no, ret);)

	return (ret);
}

/*
 * qla2x00_free_ioctl_scrap_mem
 *	Makes the entire scrap buffer free for use.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 */
STATIC void
qla2x00_free_ioctl_scrap_mem(scsi_qla_host_t *ha)
{
	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	memset(ha->ioctl->scrap_mem, 0, ha->ioctl->scrap_mem_size);
	ha->ioctl->scrap_mem_used = 0;

	DEBUG9(printk("%s(%ld): exiting.\n",
	    __func__, ha->host_no);)
}

#if defined(ISP2300)
/*
 * qla2x00_get_next_free_pub_id
 *	Find the next free public loop ID to use, starting from the old
 *	loop ID passed in.  If the old loop ID is invalid, this function
 *	will start the search from beginning.
 *
 * Input:
 *	ha		= adapter block pointer.
 *	ploop_id	= pointer to a 16bit var containing the old loop
 *			ID which is also to be used to get the new loop ID.
 *
 * Returns:
 *      QL_STATUS_SUCCESS - Found an usable loop ID
 *      QL_STATUS_RESOURCE_ERROR - No more free loop ID
 */
STATIC uint8_t
qla2x00_get_next_free_pub_id(scsi_qla_host_t *ha, uint16_t *ploop_id)
{
	uint8_t		retval = QL_STATUS_SUCCESS;
	uint16_t	index;
	uint16_t	old_id;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	old_id = *ploop_id;
	if (old_id >= LAST_SNS_LOOP_ID) {
		/* set a starting point */
		old_id = ha->min_external_loopid;
	}

	for (index = old_id; index < LAST_SNS_LOOP_ID; index++) {
		if (!ha->fabricid[index].in_use) {
			ha->fabricid[index].in_use = TRUE;
			*ploop_id = index;
			DEBUG9(printk("%s(%ld): found Lid %02x.\n",
			    __func__, ha->host_no, index);)
			break;
		}
	}
	if (index >= LAST_SNS_LOOP_ID) {
		/* no more free ID */
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR no more free LID "
		    "available.\n", __func__, ha->host_no, ha->instance);)

		retval = QL_STATUS_RESOURCE_ERROR;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting. retval=%d.\n",
	     __func__, ha->host_no, ha->instance, retval);)

	return retval;
}

/*
 * qla2x00_host_relogin
 *	Issue fabric login command to a host in the host_db which
 *	had somehow been lost before. All updates are passed back
 *	via pdevice. No update will be done to any of ha's database.
 *
 * Input:
 *	ha = adapter block pointer.
 *	pdevice = pointer to FC device type structure.
 *
 * Returns:
 *      QL_STATUS_SUCCESS - Login successfully
 *      QL_STATUS_ERROR - Login failed
 *      QL_STATUS_FATAL_ERROR - Fatal error
 */
STATIC uint8_t
qla2x00_host_relogin(scsi_qla_host_t *ha, fcdev_t *pdevice) 
{
	uint8_t		retval = QL_STATUS_SUCCESS;
	uint16_t	status[3];
	uint16_t	tmp_loop_id;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	/* pdevice->loop_id is assumed to be straight from the current
	 * database content.
	 */
	tmp_loop_id = pdevice->loop_id & 0xff;
	if (tmp_loop_id >= LAST_SNS_LOOP_ID) {
		/* Invalid value.  We need to find a valid ID to use. */
		if (qla2x00_get_next_free_pub_id(ha, &tmp_loop_id) != 0) {
			/* no more free IDs to use */
			DEBUG9_10(printk("%s(%ld): inst=%ld no free loop_id "
			    " available for login.\n",
			    __func__, ha->host_no, ha->instance);)

			return QL_STATUS_ERROR;
		}
	}

	for (;;) {
		DEBUG9(printk("%s(%ld): Login w/loop id 0x%02x for port %06x\n",
		    __func__, ha->host_no, pdevice->loop_id,
		    pdevice->d_id.b24);)

		/* Login device on switch. */
		qla2x00_login_fabric(ha,
		    tmp_loop_id, pdevice->d_id.b.domain,
		    pdevice->d_id.b.area, pdevice->d_id.b.al_pa, 
		    &status[0], 0);

		if (status[0] != MBS_CMD_CMP &&
		    status[0] != MBS_PORT_ID_IN_USE &&
		    status[0] != MBS_LOOP_ID_IN_USE) {

	 		DEBUG9_10(printk("%s(%ld): inst=%ld "
			    "ERROR login status[0]=%x status[1]=%x.\n",
			    __func__, ha->host_no, ha->instance, status[0],
			    status[1]);)

			retval = QL_STATUS_FATAL_ERROR;
			break;
		}

		if (status[0] == MBS_CMD_CMP) {
			DEBUG9(printk("%s(%ld): inst=%ld "
			    " host login success; loop_id=%x.\n",
			    __func__, ha->host_no, ha->instance, tmp_loop_id);)

			pdevice->loop_id = tmp_loop_id;
			retval = QL_STATUS_SUCCESS;
			break;

		} else if (status[0] == MBS_PORT_ID_IN_USE) {
			ha->fabricid[tmp_loop_id].in_use = FALSE;
			tmp_loop_id = status[1];

			DEBUG9(printk("%s(%ld): inst=%ld "
			    "port %06x already using loop id=0x%02x in "
			    "f/w database. Retrying.\n",
			    __func__, ha->host_no, ha->instance,
			    pdevice->d_id.b24, tmp_loop_id);)

			if (tmp_loop_id <= LAST_SNS_LOOP_ID) {
				ha->fabricid[tmp_loop_id].in_use = TRUE;
			} else {
				/* Error */
				DEBUG9_10(printk("%s(%ld): inst=%ld "
				    "PORT_ID_IN_USE - invalid loop id %02x "
				    "returned.\n",
				    __func__, ha->host_no, ha->instance,
				    pdevice->loop_id);)
				retval = QL_STATUS_ERROR;
				break;
			}

		} else if (status[0] == MBS_LOOP_ID_IN_USE) {
			/* loop id already used by others; try another one */
			DEBUG9_10(printk("%s(%ld): inst=%ld "
			    "loop id %02x already used.\n",
			    __func__, ha->host_no, ha->instance,
			    pdevice->loop_id);)

			/* Search for another usable loop_id */
			if (qla2x00_get_next_free_pub_id(ha,
			    &tmp_loop_id) == 0) {

				DEBUG9(printk("%s(%ld): previous loop "
				    "id in use. Retry with 0x%02x.\n",
				    __func__, ha->host_no, tmp_loop_id);)

				ha->fabricid[tmp_loop_id].in_use = TRUE;
			} else {
				/* Error */
				DEBUG9_10(printk("%s(%ld): inst=%ld loop id "
				    "in use; no more free loop id.\n",
				    __func__, ha->host_no, ha->instance);)

				retval = QL_STATUS_ERROR;
				break;
			}
		}
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting. retval=%d.\n",
	     __func__, ha->host_no, ha->instance, retval);)

	return (retval);
}
#endif

/*
 * qla2x00_find_curr_ha
 *	Searches and returns the pointer to the adapter host_no specified.
 *
 * Input:
 *	host_inst = driver internal adapter instance number to search.
 *	ha = adapter state pointer of the instance requested.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_find_curr_ha(uint16_t host_inst, scsi_qla_host_t **ret_ha)
{
	int	rval = QL_STATUS_SUCCESS;
	scsi_qla_host_t *search_ha = NULL;

	/*
	 * Set ha context for this IOCTL by matching host_no.
	 */
	for (search_ha = qla2x00_hostlist;
	    (search_ha != NULL) && (search_ha->instance != host_inst);
	    search_ha = search_ha->next)
		continue;

	if (search_ha == NULL) {
		DEBUG10(printk("%s: ERROR matching host_inst "
		    "%d to an HBA Instance.\n", __func__, host_inst);)
		rval = QL_STATUS_ERROR;
	} else {
		DEBUG9(printk("%s: found matching host_inst "
		    "%d to an HBA Instance.\n", __func__, host_inst);)
		*ret_ha = search_ha;
	}

	return rval;
}

/*
 * qla2x00_get_driver_specifics
 *	Returns driver specific data in the response buffer.
 *
 * Input:
 *	pext = pointer to EXT_IOCTL structure containing values from user.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_get_driver_specifics(EXT_IOCTL *pext)
{
	int			ret = 0;
	EXT_LN_DRIVER_DATA	data;

	DEBUG9(printk("%s: entered.\n",
	    __func__);)

	if (pext->ResponseLen < sizeof(EXT_LN_DRIVER_DATA)) {
		pext->Status = EXT_STATUS_BUFFER_TOO_SMALL;
		DEBUG9_10(printk("%s: ERROR ResponseLen too small.\n",
		    __func__);)

		return (ret);
	}

	data.DrvVer.Major = QLA_DRIVER_MAJOR_VER;
	data.DrvVer.Minor = QLA_DRIVER_MINOR_VER;
	data.DrvVer.Patch = QLA_DRIVER_PATCH_VER;
	data.DrvVer.Beta = QLA_DRIVER_BETA_VER;

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
	    sizeof(EXT_LN_DRIVER_DATA));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s: ERROR verify write resp buf\n",
		    __func__);)

		return (ret);
	}

	ret = copy_to_user(pext->ResponseAdr, &data, sizeof(EXT_LN_DRIVER_DATA));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s: ERROR copy resp buf\n",
		    __func__);)
	}

	DEBUG9(printk("%s: exiting. ret=%d.\n",
	    __func__, ret);)

	return (ret);
}

/*
 * qla2x00_aen_reg
 *	IOCTL management server Asynchronous Event Tracking Enable/Disable.
 *
 * Input:
 *	ha = pointer to the adapter struct of the adapter to register.
 *	cmd = pointer to EXT_IOCTL structure containing values from user.
 *	mode = flags. not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_aen_reg(scsi_qla_host_t *ha, EXT_IOCTL *cmd, int mode)
{
	int		rval = 0;
	EXT_REG_AEN	reg_struct;

	DEBUG9(printk("%s(%ld): inst %ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	rval = verify_area(VERIFY_READ, (void *)cmd->RequestAdr,
	    sizeof(EXT_REG_AEN));
	if (rval) {
		cmd->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst %ld ERROR verify read req buf\n",
		    __func__, ha->host_no, ha->instance);)

		return (rval);
	}

	rval = copy_from_user(&reg_struct, cmd->RequestAdr, sizeof(EXT_REG_AEN));
	if (rval == 0) {
		cmd->Status = EXT_STATUS_OK;
		if (reg_struct.Enable) {
			ha->ioctl->flags |= IOCTL_AEN_TRACKING_ENABLE;
		} else {
			ha->ioctl->flags &= ~IOCTL_AEN_TRACKING_ENABLE;
		}
	} else {
		cmd->Status = EXT_STATUS_COPY_ERR;
	}

	DEBUG9(printk("%s(%ld): inst %ld reg_struct.Enable(%d) "
	    "ha->ioctl_flag(%x) cmd->Status(%d).",
	    __func__, ha->host_no, ha->instance, reg_struct.Enable,
	    ha->ioctl->flags, cmd->Status);)

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (rval);
}

/*
 * qla2x00_aen_get
 *	Asynchronous Event Record Transfer to user.
 *	The entire queue will be emptied and transferred back.
 *
 * Input:
 *	ha = pointer to the adapter struct of the specified adapter.
 *	pext = pointer to EXT_IOCTL structure containing values from user.
 *	mode = flags.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 *
 * NOTE: Need to use hardware lock to protect the queues from updates
 *	 via isr/enqueue_aen after we get rid of io_request_lock.
 */
STATIC int
qla2x00_aen_get(scsi_qla_host_t *ha, EXT_IOCTL *cmd, int mode)
{
	int		rval = 0;
	EXT_ASYNC_EVENT	*tmp_q;
	EXT_ASYNC_EVENT	*paen;
	uint8_t		i;
	uint8_t		queue_cnt;
	uint8_t		request_cnt;
	uint32_t	stat = EXT_STATUS_OK;
	uint32_t	ret_len = 0;
	unsigned long   cpu_flags = 0;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	request_cnt = (uint8_t)(cmd->ResponseLen / sizeof(EXT_ASYNC_EVENT));

	if (request_cnt < EXT_DEF_MAX_AEN_QUEUE) {
		/* We require caller to alloc for the maximum request count */
		cmd->Status       = EXT_STATUS_BUFFER_TOO_SMALL;
		DEBUG9_10(printk("%s(%ld): inst=%ld Buffer too small. "
		    "Exiting normally.",
		    __func__, ha->host_no, ha->instance);)

		return (rval);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&paen,
	    sizeof(EXT_ASYNC_EVENT) * EXT_DEF_MAX_AEN_QUEUE)) {
		/* not enough memory */
		cmd->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_ASYNC_EVENT)*EXT_DEF_MAX_AEN_QUEUE);)
		return (rval);
	}

	/* 1st: Make a local copy of the entire queue content. */
	tmp_q = (EXT_ASYNC_EVENT *)ha->ioctl->aen_tracking_queue;
	queue_cnt = 0;

	spin_lock_irqsave(&ha->hardware_lock, cpu_flags);
	i = ha->ioctl->aen_q_head;

	for (; queue_cnt < EXT_DEF_MAX_AEN_QUEUE;) {
		if (tmp_q[i].AsyncEventCode != 0) {
			memcpy(&paen[queue_cnt], &tmp_q[i],
			    sizeof(EXT_ASYNC_EVENT));
			queue_cnt++;
			tmp_q[i].AsyncEventCode = 0; /* empty out the slot */
		}

		if (i == ha->ioctl->aen_q_tail) {
			/* done. */
			break;
		}

		i++;

		if (i == EXT_DEF_MAX_AEN_QUEUE) {
			i = 0;
		}
	}

	/* Empty the queue. */
	ha->ioctl->aen_q_head = 0;
	ha->ioctl->aen_q_tail = 0;

	spin_unlock_irqrestore(&ha->hardware_lock, cpu_flags);

	/* 2nd: Now transfer the queue content to user buffer */
	/* Copy the entire queue to user's buffer. */
	ret_len = (uint32_t)(queue_cnt * sizeof(EXT_ASYNC_EVENT));
	if (queue_cnt != 0) {
		rval = verify_area(VERIFY_WRITE, (void *)cmd->ResponseAdr,
		    ret_len);
		if (rval != 0) {
			cmd->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk(
			    "%s(%ld): inst=%ld ERROR verify write resp buf.\n",
			    __func__, ha->host_no, ha->instance);)

			qla2x00_free_ioctl_scrap_mem(ha);
			return (rval);
		}

		rval = copy_to_user(cmd->ResponseAdr, paen, ret_len);
	}
	cmd->ResponseLen = ret_len;

	if (rval != 0) {
		stat = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld FAILED. error = %d\n",
		    __func__, ha->host_no, ha->instance, stat);)
	} else {
		stat = EXT_STATUS_OK;
	}

	cmd->Status = stat;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting. rval=%d.\n",
	     __func__, ha->host_no, ha->instance, rval);)

	return (rval);
}

/*
 * qla2x00_enqueue_aen
 *
 * Input:
 *	ha = adapter state pointer.
 *	event_code = async event code of the event to add to queue.
 *	payload = event payload for the queue.
 *
 * Context:
 *	Interrupt context.
 * NOTE: Need to hold the hardware lock to protect the queues from
 *	 aen_get after we get rid of the io_request_lock.
 */
void
qla2x00_enqueue_aen(scsi_qla_host_t *ha, uint16_t event_code, void *payload)
{
	uint8_t			new_entry; /* index to current entry */
	uint16_t		*mbx;
	EXT_ASYNC_EVENT		*aen_queue;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	aen_queue = (EXT_ASYNC_EVENT *)ha->ioctl->aen_tracking_queue;
	if (aen_queue[ha->ioctl->aen_q_tail].AsyncEventCode != 0) {
		/* Need to change queue pointers to make room. */

		/* Increment tail for adding new entry. */
		ha->ioctl->aen_q_tail++;
		if (ha->ioctl->aen_q_tail == EXT_DEF_MAX_AEN_QUEUE) {
			ha->ioctl->aen_q_tail = 0;
		}

		if (ha->ioctl->aen_q_head == ha->ioctl->aen_q_tail) {
			/*
			 * We're overwriting the oldest entry, so need to
			 * update the head pointer.
			 */
			ha->ioctl->aen_q_head++;
			if (ha->ioctl->aen_q_head == EXT_DEF_MAX_AEN_QUEUE) {
				ha->ioctl->aen_q_head = 0;
			}
		}
	}

	DEBUG(printk("%s(%ld): inst=%ld Adding code 0x%x to aen_q %p @ %d\n",
	    __func__, ha->host_no, ha->instance, event_code, aen_queue,
	    ha->ioctl->aen_q_tail);)

	new_entry = ha->ioctl->aen_q_tail;
	aen_queue[new_entry].AsyncEventCode = event_code;

		/* Update payload */
	switch (event_code) {
	case MBA_LIP_OCCURRED:
	case MBA_LOOP_UP:
	case MBA_LOOP_DOWN:
	case MBA_LIP_RESET:
	case MBA_PORT_UPDATE:
		/* empty */
		break;

	case MBA_SCR_UPDATE:
		mbx = (uint16_t *)payload;
		/* al_pa */
		aen_queue[new_entry].Payload.RSCN.RSCNInfo[0] = LSB(mbx[2]);
		/* area */
		aen_queue[new_entry].Payload.RSCN.RSCNInfo[1] = MSB(mbx[2]);
		/* domain */
		aen_queue[new_entry].Payload.RSCN.RSCNInfo[2] = LSB(mbx[1]);
		/* save in big endian */
		BIG_ENDIAN_24(aen_queue[new_entry].Payload.RSCN.RSCNInfo[0]);

		aen_queue[new_entry].Payload.RSCN.AddrFormat = MSB(mbx[1]);

		break;

	default:
		/* Not supported */
		aen_queue[new_entry].AsyncEventCode = 0;
		break;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)
}

/*
 * qla2x00_query
 *	Handles all subcommands of the EXT_CC_QUERY command.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int rval = 0;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	/* All Query type ioctls are done here */
	switch(pext->SubCode) {

	case EXT_SC_QUERY_HBA_NODE:
		/* fill in HBA NODE Information */
		rval = qla2x00_query_hba_node(ha, pext, mode);
		break;

	case EXT_SC_QUERY_HBA_PORT:
		/* return HBA PORT related info */
		rval = qla2x00_query_hba_port(ha, pext, mode);
		break;

	case EXT_SC_QUERY_DISC_PORT:
		/* return discovered port information */
		rval = qla2x00_query_disc_port(ha, pext, mode);
		break;

	case EXT_SC_QUERY_DISC_TGT:
		/* return discovered target information */
		rval = qla2x00_query_disc_tgt(ha, pext, mode);
		break;

	case EXT_SC_QUERY_CHIP:
		rval = qla2x00_query_chip(ha, pext, mode);
		break;

	case EXT_SC_QUERY_DISC_LUN:
		pext->Status = EXT_STATUS_UNSUPPORTED_SUBCODE;
		break;

	default:
		DEBUG9_10(printk("%s(%ld): inst=%ld unknown SubCode %d.\n",
		    __func__, ha->host_no, ha->instance, pext->SubCode);)
		pext->Status = EXT_STATUS_UNSUPPORTED_SUBCODE;
		break;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return rval;
}

/*
 * qla2x00_query_hba_node
 *	Handles EXT_SC_QUERY_HBA_NODE subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query_hba_node(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint32_t	i, transfer_size;
	EXT_HBA_NODE	*ptmp_hba_node;
	qla_boards_t	*bdp;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_hba_node,
	    sizeof(EXT_HBA_NODE))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_HBA_NODE));)
		return (ret);
	}

	/* fill all available HBA NODE Information */
	bdp = &QLBoardTbl_fc[ha->devnum];
	for (i = 0; i < 8 ; i++)
		ptmp_hba_node->WWNN[i] = ha->node_name[i];

	sprintf((char *)(ptmp_hba_node->Manufacturer),"Qlogic Corp.");
	sprintf((char *)(ptmp_hba_node->Model),(char *)&bdp->bdName[0]);

	ptmp_hba_node->SerialNum[0] = ha->serial0;
	ptmp_hba_node->SerialNum[1] = ha->serial1;
	ptmp_hba_node->SerialNum[2] = ha->serial2;
	sprintf((char *)(ptmp_hba_node->DriverVersion),qla2x00_version_str);
	sprintf((char *)(ptmp_hba_node->FWVersion),"%2d.%02d.%02d",
	    bdp->fwver[0], bdp->fwver[1], bdp->fwver[2]);

	sprintf((char *)(ptmp_hba_node->OptRomVersion),"%d.%d",
	    ha->optrom_major, ha->optrom_minor);

	ptmp_hba_node->InterfaceType = EXT_DEF_FC_INTF_TYPE;
	ptmp_hba_node->PortCount = 1;


	ptmp_hba_node->DriverAttr = (ha->flags.failover_enabled) ?
	    DRVR_FO_ENABLED : 0;

	ret = verify_area(VERIFY_WRITE, (void  *)pext->ResponseAdr,
	    sizeof(EXT_HBA_NODE));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* now copy up the HBA_NODE to user */
	if (pext->ResponseLen < sizeof(EXT_HBA_NODE))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_HBA_NODE);

	ret = copy_to_user((uint8_t *)pext->ResponseAdr,
	    (uint8_t *)ptmp_hba_node, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}

/*
 * qla2x00_query_hba_port
 *	Handles EXT_SC_QUERY_HBA_PORT subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query_hba_port(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint32_t	tgt_cnt, tgt, transfer_size;
	uint32_t	port_cnt;
	fc_port_t	*fcport;
	EXT_HBA_PORT	*ptmp_hba_port;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_hba_port,
	    sizeof(EXT_HBA_PORT))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_HBA_PORT));)
		return (ret);
	}

	/* reflect all HBA PORT related info */
	ptmp_hba_port->WWPN[7] = ha->init_cb->port_name[7];
	ptmp_hba_port->WWPN[6] = ha->init_cb->port_name[6];
	ptmp_hba_port->WWPN[5] = ha->init_cb->port_name[5];
	ptmp_hba_port->WWPN[4] = ha->init_cb->port_name[4];
	ptmp_hba_port->WWPN[3] = ha->init_cb->port_name[3];
	ptmp_hba_port->WWPN[2] = ha->init_cb->port_name[2];
	ptmp_hba_port->WWPN[1] = ha->init_cb->port_name[1];
	ptmp_hba_port->WWPN[0] = ha->init_cb->port_name[0];
	ptmp_hba_port->Id[0] = 0;
	ptmp_hba_port->Id[1] = ha->d_id.r.d_id[2];
	ptmp_hba_port->Id[2] = ha->d_id.r.d_id[1];
	ptmp_hba_port->Id[3] = ha->d_id.r.d_id[0];
	ptmp_hba_port->Type =  EXT_DEF_INITIATOR_DEV;

	switch (ha->current_topology) {
	case ISP_CFG_NL:
	case ISP_CFG_FL:
		ptmp_hba_port->Mode = EXT_DEF_LOOP_MODE;
		break;

	case ISP_CFG_N:
	case ISP_CFG_F:
		ptmp_hba_port->Mode = EXT_DEF_P2P_MODE;
		break;

	default:
		ptmp_hba_port->Mode = EXT_DEF_UNKNOWN_MODE;
		break;
	}

	port_cnt = 0;
	for (fcport = ha->fcport; (fcport); fcport = fcport->next) {
		/* if removed or missing */
		if (atomic_read(&fcport->state) != FC_ONLINE) {
			DEBUG9_10(printk(
			    "%s(%ld): inst=%ld port %06x not online\n",
			    __func__, ha->host_no, ha->instance,
			    fcport->d_id.b24);)
			continue;
		}
		port_cnt++;
	}

	tgt_cnt  = 0;
	for (tgt = 0; tgt < MAX_TARGETS; tgt++) {
		if (ha->otgt[tgt] == NULL) {
			continue;
		}
		tgt_cnt++;
	}

	DEBUG9_10(printk("%s(%ld): inst=%ld disc_port cnt=%d, tgt cnt=%d.\n",
	    __func__, ha->host_no, ha->instance,
	    port_cnt, tgt_cnt);)
	ptmp_hba_port->DiscPortCount   = port_cnt;
	ptmp_hba_port->DiscTargetCount = tgt_cnt;

	if (ha->loop_state == LOOP_DOWN) {

		ptmp_hba_port->State = EXT_DEF_HBA_LOOP_DOWN;

	} else if (ha->loop_state != LOOP_READY ||
	    test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags) ||
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) || ABORTS_ACTIVE) {

		ptmp_hba_port->State = EXT_DEF_HBA_SUSPENDED;

	} else {

		ptmp_hba_port->State = EXT_DEF_HBA_OK;

	}

	ptmp_hba_port->DiscPortNameType = EXT_DEF_USE_PORT_NAME;

	/* Return supported FC4 type depending on driver support. */
	ptmp_hba_port->PortSupportedFC4Types = EXT_DEF_FC4_TYPE_SCSI;
#if defined(FC_IP_SUPPORT)

	ptmp_hba_port->PortSupportedFC4Types |= EXT_DEF_FC4_TYPE_IP;
#endif
#if defined(FC_SCTP_SUPPORT)

	ptmp_hba_port->PortSupportedFC4Types |= EXT_DEF_FC4_TYPE_SCTP;
#endif

	ptmp_hba_port->PortActiveFC4Types = ha->active_fc4_types;

	/* Return supported speed depending on adapter type */
#if defined(ISP2100)

	ptmp_hba_port->PortSupportedSpeed = EXT_DEF_PORTSPEED_1GBIT;
#elif defined(ISP2200)

	ptmp_hba_port->PortSupportedSpeed = EXT_DEF_PORTSPEED_1GBIT;
#elif defined(ISP2300)

	ptmp_hba_port->PortSupportedSpeed = EXT_DEF_PORTSPEED_2GBIT;
#else
	/* invalid */
	ptmp_hba_port->PortSupportedSpeed = 0;
#endif

	ptmp_hba_port->PortSpeed = ha->current_speed;

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr ,
	    sizeof(EXT_HBA_PORT));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* now copy up the HBA_PORT to user */
	if (pext->ResponseLen < sizeof(EXT_HBA_PORT))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_HBA_PORT);

	ret = copy_to_user((uint8_t *)pext->ResponseAdr,
	    (uint8_t *)ptmp_hba_port, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return ret;
}

/*
 * qla2x00_query_disc_port
 *	Handles EXT_SC_QUERY_DISC_PORT subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query_disc_port(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint32_t	tgt, transfer_size, inst;
	fc_port_t	*fcport;
	os_tgt_t	*tq;
	EXT_DISC_PORT	*ptmp_disc_port;

	DEBUG9(printk("%s(%ld): inst=%ld entered. Port inst=%02d.\n",
	    __func__, ha->host_no, ha->instance, pext->Instance);)

	inst = 0;
	for (fcport = ha->fcport; fcport != NULL; fcport = fcport->next) {
		if (atomic_read(&fcport->state) != FC_ONLINE) {
			/* port does not exist anymore */
			DEBUG9(printk("%s(%ld): fcport marked lost. "
			    "d_id=%06x loop_id=%02x not online.\n",
			    __func__, ha->host_no, fcport->d_id.b24,
			    fcport->loop_id);)

			continue;
		}

		if (inst != pext->Instance) {
			DEBUG9(printk("%s(%ld): found fcport %02d d_id=%06x. "
			    "Skipping.\n",
			    __func__, ha->host_no, inst, fcport->d_id.b24);)

			inst++;
			continue;
		}

		DEBUG9(printk("%s(%ld): inst=%ld found matching fcport %02d "
		    "online. d_id=%06x loop_id=%02x online.\n",
		    __func__, ha->host_no, ha->instance, inst, fcport->d_id.b24,
		    fcport->loop_id);)

		/* Found the matching port still connected. */
		break;
	}

	if (fcport == NULL) {
		DEBUG9_10(printk("%s(%ld): inst=%ld dev not found.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_disc_port,
	    sizeof(EXT_DISC_PORT))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_DISC_PORT));)
		return (ret);
	}

	memcpy(ptmp_disc_port->WWNN, fcport->node_name, WWN_SIZE);
	memcpy(ptmp_disc_port->WWPN, fcport->port_name, WWN_SIZE);

	ptmp_disc_port->Id[0] = 0;
	ptmp_disc_port->Id[1] = fcport->d_id.r.d_id[2];
	ptmp_disc_port->Id[2] = fcport->d_id.r.d_id[1];
	ptmp_disc_port->Id[3] = fcport->d_id.r.d_id[0];

	/* Currently all devices on fcport list are target capable devices */
	/* This default value may need to be changed after we add non target
	 * devices also to this list.
	 */
	ptmp_disc_port->Type = EXT_DEF_TARGET_DEV;

	if (fcport->flags & FC_FABRIC_DEVICE) {
		ptmp_disc_port->Type |= EXT_DEF_FABRIC_DEV;
	}
	if (fcport->flags & FC_TAPE_DEVICE) {
		ptmp_disc_port->Type |= EXT_DEF_TAPE_DEV;
	}
	if (fcport->flags & FC_INITIATOR_DEVICE) {
		ptmp_disc_port->Type |= EXT_DEF_INITIATOR_DEV;
	}

	ptmp_disc_port->LoopID = fcport->loop_id;
	ptmp_disc_port->Status = 0;
	ptmp_disc_port->Bus    = 0;

	for (tgt = 0; tgt < MAX_TARGETS; tgt++) {
		if ((tq = ha->otgt[tgt]) == NULL) {
			continue;
		}

		if (tq->vis_port == NULL)  /* dg 08/14/01 */
			continue;

		if (memcmp(fcport->port_name, tq->vis_port->port_name,
		    EXT_DEF_WWN_NAME_SIZE) == 0) {
			ptmp_disc_port->TargetId = tgt;
			break;
		}
	}

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr ,
	    sizeof(EXT_DISC_PORT));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* now copy up the DISC_PORT to user */
	if (pext->ResponseLen < sizeof(EXT_DISC_PORT))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_DISC_PORT);

	ret = copy_to_user((uint8_t *)pext->ResponseAdr,
	    (uint8_t *)ptmp_disc_port, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_query_disc_tgt
 *	Handles EXT_SC_QUERY_DISC_TGT subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query_disc_tgt(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint32_t	tgt, transfer_size, inst;
	uint32_t	cnt, i;
	fc_port_t	*tgt_fcport;
	os_tgt_t	*tq;
	EXT_DISC_TARGET	*ptmp_disc_target;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	tq = NULL;
	for (tgt = 0, inst = 0; tgt < MAX_TARGETS; tgt++) {
		if (ha->otgt[tgt] == NULL) {
			continue;
		}
		/* if wrong target id then skip to next entry */
		if (inst != pext->Instance) {
			inst++;
			continue;
		}
		tq = ha->otgt[tgt];
		break;
	}

	if (tq == NULL || tgt == MAX_TARGETS) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld target dev not found. "
		    "tq=%p, tgt=%d.\n",
		    __func__, ha->host_no, ha->instance, tq, tgt);)
		return (ret);
	}

	if (tq->vis_port == NULL) { 	/* dg 08/14/01 */
		pext->Status = EXT_STATUS_BUSY;
		DEBUG9_10(printk("%s(%ld): inst=%ld target dev not found. "
		    "tq=%p.\n",
		    __func__, ha->host_no, ha->instance, tq);)
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_disc_target,
	    sizeof(EXT_DISC_TARGET))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_DISC_TARGET));)
		return (ret);
	}

	tgt_fcport = tq->vis_port;
	memcpy(ptmp_disc_target->WWNN, tgt_fcport->node_name, WWN_SIZE);
	memcpy(ptmp_disc_target->WWPN, tgt_fcport->port_name, WWN_SIZE);

	ptmp_disc_target->Id[0] = 0;
	ptmp_disc_target->Id[1] = tgt_fcport->d_id.r.d_id[2];
	ptmp_disc_target->Id[2] = tgt_fcport->d_id.r.d_id[1];
	ptmp_disc_target->Id[3] = tgt_fcport->d_id.r.d_id[0];

	/* All devices on ha->otgt list are target capable devices. */
	ptmp_disc_target->Type = EXT_DEF_TARGET_DEV;

	if (tgt_fcport->flags & FC_FABRIC_DEVICE) {
		ptmp_disc_target->Type |= EXT_DEF_FABRIC_DEV;
	}
	if (tgt_fcport->flags & FC_TAPE_DEVICE) {
		ptmp_disc_target->Type |= EXT_DEF_TAPE_DEV;
	}
	if (tgt_fcport->flags & FC_INITIATOR_DEVICE) {
		ptmp_disc_target->Type |= EXT_DEF_INITIATOR_DEV;
	}

	ptmp_disc_target->LoopID   = tgt_fcport->loop_id;
	ptmp_disc_target->Status   = 0;
	ptmp_disc_target->Bus      = 0;
	ptmp_disc_target->TargetId = tgt;

	cnt = 0;
	/* enumerate available LUNs under this TGT (if any) */
	if (ha->otgt[tgt] != NULL) {
		for (i = 0; i < MAX_LUNS ; i++) {
			if ((ha->otgt[tgt])->olun[i] !=0)
				cnt++;
		}
	}

	ptmp_disc_target->LunCount = cnt;

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
	    sizeof(EXT_DISC_TARGET));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* now copy up the DISC_PORT to user */
	if (pext->ResponseLen < sizeof(EXT_DISC_PORT))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_DISC_TARGET);

	ret = copy_to_user((uint8_t *)pext->ResponseAdr,
	    (uint8_t *)ptmp_disc_target, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_query_chip
 *	Handles EXT_SC_QUERY_CHIP subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query_chip(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint32_t	transfer_size, i;
	EXT_CHIP		*ptmp_isp;
	struct Scsi_Host	*host;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_isp,
	    sizeof(EXT_CHIP))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_CHIP));)
		return (ret);
	}

	host = ha->host;
	ptmp_isp->VendorId       = QLA2X00_VENDOR_ID;
	ptmp_isp->DeviceId       = ha->device_id;
	ptmp_isp->SubVendorId    = ha->subsystem_vendor;
	ptmp_isp->SubSystemId    = ha->subsystem_device;
	ptmp_isp->PciBusNumber   = ha->pdev->bus->number;
	ptmp_isp->PciDevFunc     = ha->pdev->devfn;
	ptmp_isp->PciSlotNumber  = PCI_SLOT(ha->pdev->devfn);
	ptmp_isp->IoAddr         = host->io_port;
	ptmp_isp->IoAddrLen      = 512;
	ptmp_isp->MemAddr        = 0; /* ? */
	ptmp_isp->MemAddrLen     = 0; /* ? */
	ptmp_isp->ChipType       = 0; /* ? */
	ptmp_isp->InterruptLevel = host->irq;

	for (i = 0; i < 8; i++)
		ptmp_isp->OutMbx[i] = 0;

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
	    sizeof(EXT_CHIP));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* now copy up the ISP to user */
	if (pext->ResponseLen < sizeof(EXT_CHIP))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_CHIP);

	ret = copy_to_user((uint8_t *)pext->ResponseAdr, (uint8_t *)ptmp_isp,
	    transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_get_data
 *	Handles all subcommands of the EXT_CC_GET_DATA command.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_get_data(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int	tmp_rval = 0;

	switch(pext->SubCode) {
	case EXT_SC_GET_STATISTICS:
		tmp_rval = qla2x00_get_statistics(ha, pext, mode);
		break;

	case EXT_SC_GET_FC_STATISTICS:
		tmp_rval = qla2x00_get_fc_statistics(ha, pext, mode);
		break;

	case EXT_SC_GET_PORT_SUMMARY:
		tmp_rval = qla2x00_get_port_summary(ha, pext, mode);
		break;

	case EXT_SC_QUERY_DRIVER:
		tmp_rval = qla2x00_query_driver(ha, pext, mode);
		break;

	case EXT_SC_QUERY_FW:
		tmp_rval = qla2x00_query_fw(ha, pext, mode);
		break;

	case EXT_SC_GET_RNID:
		tmp_rval = qla2x00_get_rnid_params(ha, pext, mode);
		break;

	default:
		DEBUG10(printk("%s(%ld): inst=%ld unknown SubCode %d.\n",
		    __func__, ha->host_no, ha->instance, pext->SubCode);)
		pext->Status = EXT_STATUS_UNSUPPORTED_SUBCODE;
		break;
	 }

	return (tmp_rval);
}

/*
 * qla2x00_get_statistics
 *	Issues get_link_status mbx cmd and returns statistics
 *	relavent to the specified adapter.
 *
 * Input:
 *	ha = pointer to adapter struct of the specified adapter.
 *	pext = pointer to EXT_IOCTL structure containing values from user.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_get_statistics(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	EXT_HBA_PORT_STAT	*ptmp_stat;
	int		ret = 0;
	link_stat_t	stat_buf;
	uint8_t		rval;
	uint8_t		*usr_temp, *kernel_tmp;
	uint16_t	mb_stat[1];
	uint32_t	transfer_size;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
	    sizeof(EXT_HBA_PORT_STAT));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR VERIFY_WRITE "
		    "EXT_HBA_PORT_STAT.\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	/* check on loop down */
	if (ha->loop_state != LOOP_READY || 
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	    ABORTS_ACTIVE || ha->dpc_active) {

		pext->Status = EXT_STATUS_BUSY;
		DEBUG9_10(printk("%s(%ld): inst=%ld loop not ready.\n",
		    __func__, ha->host_no, ha->instance);)

		return (ret);
	}

	/* Send mailbox cmd to get more. */
	if ((rval = qla2x00_get_link_status(ha, ha->loop_id, &stat_buf,
	    mb_stat)) != QL_STATUS_SUCCESS) {

		if (rval == BIT_0) {
			pext->Status = EXT_STATUS_NO_MEMORY;
		} else if (rval == BIT_1) {
			pext->Status = EXT_STATUS_MAILBOX;
			pext->DetailStatus = EXT_DSTATUS_NOADNL_INFO;
		} else {
			pext->Status = EXT_STATUS_ERR;
		}

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR mailbox failed. "
		    "mb[0]=%x.\n",
		    __func__, ha->host_no, ha->instance, mb_stat[0]);)
		printk(KERN_WARNING
		     "%s(%ld): inst=%ld ERROR mailbox failed. mb[0]=%x.\n",
		    __func__, ha->host_no, ha->instance, mb_stat[0]);

		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_stat,
	    sizeof(EXT_HBA_PORT_STAT))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_HBA_PORT_STAT));)
		return (ret);
	}

	ptmp_stat->ControllerErrorCount   =  ha->total_isp_aborts;
	ptmp_stat->DeviceErrorCount       =  ha->total_dev_errs;
	ptmp_stat->TotalIoCount           =  ha->total_ios;
	ptmp_stat->TotalMBytes            =  ha->total_bytes >> 20;
	ptmp_stat->TotalLipResets         =  ha->total_lip_cnt;
	/*
	   ptmp_stat->TotalInterrupts        =  ha->total_isr_cnt;
	 */

	ptmp_stat->TotalLinkFailures = le32_to_cpu(stat_buf.link_fail_cnt);

	ptmp_stat->TotalLossOfSync = le32_to_cpu(stat_buf.loss_sync_cnt);

	ptmp_stat->TotalLossOfSignals = le32_to_cpu(stat_buf.loss_sig_cnt);

	ptmp_stat->PrimitiveSeqProtocolErrorCount =
				le32_to_cpu(stat_buf.prim_seq_err_cnt);

	ptmp_stat->InvalidTransmissionWordCount =
				le32_to_cpu(stat_buf.inval_xmit_word_cnt);

	ptmp_stat->InvalidCRCCount = le32_to_cpu(stat_buf.inval_crc_cnt);

	/* now copy up the STATISTICS to user */
	if (pext->ResponseLen < sizeof(EXT_HBA_PORT_STAT))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_HBA_PORT_STAT);

	usr_temp   = (uint8_t *)pext->ResponseAdr;
	kernel_tmp = (uint8_t *)ptmp_stat;
	ret = copy_to_user(usr_temp, kernel_tmp, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_get_fc_statistics
 *	Issues get_link_status mbx cmd to the target device with
 *	the specified WWN and returns statistics relavent to the
 *	device.
 *
 * Input:
 *	ha = pointer to adapter struct of the specified device.
 *	pext = pointer to EXT_IOCTL structure containing values from user.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_get_fc_statistics(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	EXT_HBA_PORT_STAT	*ptmp_stat;
	EXT_DEST_ADDR		addr_struct;
	int		ret = 0;
	link_stat_t	stat_buf;
	uint8_t		rval, tgt;
	uint8_t		*usr_temp, *kernel_tmp;
	uint8_t		*req_name;
	uint16_t	mb_stat[1];
	uint32_t	transfer_size;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
	    sizeof(EXT_HBA_PORT_STAT));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR VERIFY_WRITE.\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	ret = copy_from_user(&addr_struct, pext->RequestAdr, pext->RequestLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy req buf.\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	/* find the device's loop_id */
	switch (addr_struct.DestType) {
	case EXT_DEF_DESTTYPE_WWPN:
		req_name = addr_struct.DestAddr.WWPN;
		for (tgt = 0; tgt < MAX_FIBRE_DEVICES; tgt++) {
			if (memcmp(ha->fc_db[tgt].wwn, req_name,
			    EXT_DEF_WWN_NAME_SIZE) == 0) {
				break;
			}
		}
		break;

	case EXT_DEF_DESTTYPE_WWNN:
	case EXT_DEF_DESTTYPE_PORTID:
	case EXT_DEF_DESTTYPE_FABRIC:
	case EXT_DEF_DESTTYPE_SCSI:
	default:
		pext->Status = EXT_STATUS_INVALID_PARAM;
		pext->DetailStatus = EXT_DSTATUS_NOADNL_INFO;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR Unsupported subcode "
		    "address type.\n", __func__, ha->host_no, ha->instance);)
		return (ret);

		break;
	}

	if (tgt == MAX_FIBRE_DEVICES) {
		/* not found */
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		pext->DetailStatus = EXT_DSTATUS_TARGET;
		return (ret);
	}

	/* check for suspended/lost device */
	/*
	   if (ha->fcport is suspended/lost) {
	   pext->Status = EXT_STATUS_SUSPENDED;
	   pext->DetailStatus = EXT_DSTATUS_TARGET;
	   return pext->Status;
	   }
	 */

	/* check on loop down */
	if (ha->loop_state != LOOP_READY ||
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	    ABORTS_ACTIVE || ha->dpc_active) {

		pext->Status = EXT_STATUS_BUSY;
		DEBUG9_10(printk("%s(%ld): inst=%ld loop not ready.\n",
		     __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	/* Send mailbox cmd to get more. */
	if ((rval = qla2x00_get_link_status(ha, ha->fc_db[tgt].loop_id,
	    &stat_buf, mb_stat)) != QL_STATUS_SUCCESS) {
		if (rval == BIT_0) {
			pext->Status = EXT_STATUS_NO_MEMORY;
		} else if (rval == BIT_1) {
			pext->Status = EXT_STATUS_MAILBOX;
			pext->DetailStatus = EXT_DSTATUS_NOADNL_INFO;
		} else {
			pext->Status = EXT_STATUS_ERR;
		}

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR mailbox failed. "
		    "mb[0]=%x.\n",
		    __func__, ha->host_no, ha->instance, mb_stat[0]);)
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptmp_stat,
	    sizeof(EXT_HBA_PORT_STAT))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_HBA_PORT_STAT));)
		return (ret);
	}

	ptmp_stat->ControllerErrorCount   =  ha->total_isp_aborts;
	ptmp_stat->DeviceErrorCount       =  ha->total_dev_errs;
	ptmp_stat->TotalIoCount           =  ha->total_ios;
	ptmp_stat->TotalMBytes            =  ha->total_bytes >> 20;
	ptmp_stat->TotalLipResets         =  ha->total_lip_cnt;
	/*
	   ptmp_stat->TotalInterrupts        =  ha->total_isr_cnt;
	 */

	ptmp_stat->TotalLinkFailures = le32_to_cpu(stat_buf.link_fail_cnt);

	ptmp_stat->TotalLossOfSync = le32_to_cpu(stat_buf.loss_sync_cnt);

	ptmp_stat->TotalLossOfSignals = le32_to_cpu(stat_buf.loss_sig_cnt);

	ptmp_stat->PrimitiveSeqProtocolErrorCount =
				le32_to_cpu(stat_buf.prim_seq_err_cnt);

	ptmp_stat->InvalidTransmissionWordCount =
				le32_to_cpu(stat_buf.inval_xmit_word_cnt);

	ptmp_stat->InvalidCRCCount = le32_to_cpu(stat_buf.inval_crc_cnt);

	/* now copy up the STATISTICS to user */
	if (pext->ResponseLen < sizeof(EXT_HBA_PORT_STAT))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_HBA_PORT_STAT);

	usr_temp   = (uint8_t *)pext->ResponseAdr;
	kernel_tmp = (uint8_t *)ptmp_stat;
	ret = copy_to_user(usr_temp, kernel_tmp, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_get_port_summary
 *	Handles EXT_SC_GET_PORT_SUMMARY subcommand.
 *	Returns values of devicedata and dd_entry list.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_get_port_summary(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint8_t		*usr_temp, *kernel_tmp;
	uint32_t	b;
	uint32_t	i, port_cnt, entry;
	uint32_t	number_of_entries = 0;
	uint32_t	tgt_cnt, transfer_size;
	fc_port_t	*fcport;
	uint32_t	current_offset;
	void		*start_of_entry_list;

	EXT_DEVICEDATA		*pdevicedata;
	EXT_DEVICEDATAENTRY	*pdd_entry;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	port_cnt = 0;
	tgt_cnt  = 0;

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pdevicedata,
	    sizeof(EXT_DEVICEDATA))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "pdevicedata requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_DEVICEDATA));)
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pdd_entry,
	    sizeof(EXT_DEVICEDATAENTRY))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "pdd_entry requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_DEVICEDATAENTRY));)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	for (fcport = ha->fcport; fcport != NULL; fcport = fcport->next) {
		port_cnt++;
	}
	pdevicedata->TotalDevices = port_cnt;

	number_of_entries = pext->ResponseLen / sizeof(EXT_DEVICEDATAENTRY);

	/* we want the lesser of port_cnt and number_of_entries */
	if (number_of_entries > port_cnt)
		number_of_entries = port_cnt;
	pdevicedata->ReturnListEntryCount  = number_of_entries;

	DEBUG9(printk("%s(%ld): inst=%ld EXT_SC_GET_PORT_SUMMARY port_cnt=%x, "
	    "return entry cnt=%x.\n",
	    __func__, ha->host_no, ha->instance, port_cnt,
	    number_of_entries);)

	transfer_size = sizeof(pdevicedata->ReturnListEntryCount) +
	    sizeof(pdevicedata->TotalDevices);

	/* copy top of devicedata here */
	ret = verify_area(VERIFY_WRITE, (void *)(pext->ResponseAdr),
	    transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	usr_temp   = (uint8_t *)pext->ResponseAdr;
	kernel_tmp = (uint8_t *)pdevicedata;
	ret = copy_to_user(usr_temp, kernel_tmp, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp "
		    "devicedata buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	start_of_entry_list = (void *)(pext->ResponseAdr) + transfer_size;

	for (entry = 0, fcport = ha->fcport; (entry < number_of_entries) &&
	    (fcport); entry++, fcport = fcport->next) {

		/* copy from fc_db of this target (port) to dd_entry */

		memcpy(pdd_entry->NodeWWN, fcport->node_name, WWN_SIZE);
		memcpy(pdd_entry->PortWWN, fcport->port_name, WWN_SIZE);

		for (b = 0; b < 3 ; b++)
			pdd_entry->PortID[b] = fcport->d_id.r.d_id[2-b];

		if (fcport->flags & FC_FABRIC_DEVICE) {
			pdd_entry->ControlFlags = EXT_DEF_GET_FABRIC_DEVICE;
		} else {
			pdd_entry->ControlFlags = 0;
		}

		pdd_entry->TargetAddress.Bus    = 0;
		/* Retrieve 'Target' number for port via fc_db */
		for (i = 0; i < MAX_TARGETS; i++) {
			if (ha->fc_db[i].loop_id == PORT_UNUSED)
				continue;

			if (memcmp(fcport->port_name, ha->fc_db[i].wwn,
			    EXT_DEF_WWN_NAME_SIZE) == 0) {
				pdd_entry->TargetAddress.Target = i;
				break;
			}
		}
		pdd_entry->TargetAddress.Lun    = 0;
		pdd_entry->DeviceFlags          = 0;
		pdd_entry->LoopID               = fcport->loop_id;
		pdd_entry->BaseLunNumber        = 0;

		current_offset = entry * sizeof(EXT_DEVICEDATAENTRY);

		transfer_size = sizeof(EXT_DEVICEDATAENTRY);
		ret = verify_area(VERIFY_WRITE,
		    (void *)(start_of_entry_list + current_offset),
		    transfer_size);

		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG10(printk("%s(%ld): inst=%ld ERROR verify WRITE "
			    "rsp bufaddr=%p\n",
			    __func__, ha->host_no, ha->instance,
			    (void *)(start_of_entry_list + current_offset));)
			qla2x00_free_ioctl_scrap_mem(ha);
			return pext->Status;
		}

		/* now copy up this dd_entry to user */
		usr_temp = (uint8_t *)((u_long) start_of_entry_list +
		    current_offset);
		kernel_tmp = (uint8_t *)pdd_entry;
	 	ret = copy_to_user(usr_temp, kernel_tmp, transfer_size);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp "
			    "entry list buffer.\n",
			    __func__, ha->host_no, ha->instance);)
			qla2x00_free_ioctl_scrap_mem(ha);
			return (ret);
		}

	} /* for number_of_entries */

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_query_driver
 *	Handles EXT_SC_QUERY_DRIVER subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query_driver(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint8_t		*usr_temp, *kernel_tmp;
	uint32_t	transfer_size;
	EXT_DRIVER	*pdriver_prop;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pdriver_prop,
	    sizeof(EXT_DRIVER))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_DRIVER));)
		return (ret);
	}

	sprintf(pdriver_prop->Version, qla2x00_version_str);
	pdriver_prop->NumOfBus = MAX_BUSES;
	pdriver_prop->TargetsPerBus = MAX_FIBRE_DEVICES;
	pdriver_prop->LunsPerTarget = MAX_LUNS;
	pdriver_prop->MaxTransferLen  = 0xffffffff;
	pdriver_prop->MaxDataSegments = 0xffffffff;

	if (ha->flags.enable_64bit_addressing == 1)
		pdriver_prop->DmaBitAddresses = 64;
	else
		pdriver_prop->DmaBitAddresses = 32;

	if (pext->ResponseLen < sizeof(EXT_DRIVER))
		transfer_size = pext->ResponseLen;
	else
		transfer_size = sizeof(EXT_DRIVER);

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr ,
	    transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* now copy up the ISP to user */
	usr_temp   = (uint8_t *)pext->ResponseAdr;
	kernel_tmp = (uint8_t *)pdriver_prop;
	ret = copy_to_user(usr_temp, kernel_tmp, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_query_fw
 *	Handles EXT_SC_QUERY_FW subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_query_fw(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	qla_boards_t	*bdp;
	uint8_t		*usr_temp, *kernel_tmp;
	uint32_t	transfer_size;
	EXT_FW		*pfw_prop;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pfw_prop,
	    sizeof(EXT_FW))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_FW));)
		return (ret);
	}

	bdp = &QLBoardTbl_fc[ha->devnum];
	pfw_prop->Version[0] = bdp->fwver[0];
	pfw_prop->Version[1] = bdp->fwver[1];
	pfw_prop->Version[2] = bdp->fwver[2];

	transfer_size = sizeof(EXT_FW);

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr ,
	    transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp buf.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	usr_temp   = (uint8_t *)pext->ResponseAdr;
	kernel_tmp = (uint8_t *)pfw_prop;
	ret = copy_to_user(usr_temp, kernel_tmp, transfer_size);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	pext->Status       = EXT_STATUS_OK;
	pext->DetailStatus = EXT_STATUS_OK;
	qla2x00_free_ioctl_scrap_mem(ha);

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

#if defined(ISP2300)
/*
 * qla2x00_send_els_passthru
 *	Passes the ELS command down to firmware as MSIOCB and
 *	copies the response back when it completes.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_send_els_passthru(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	int		tmp_rval = 0;

	uint8_t		index;
	uint8_t		invalid_wwn = FALSE;
	uint8_t		port_found;
	uint8_t		*ptmp_stat;
	uint8_t		*pusr_req_buf;
	uint8_t		*presp_payload;
	uint32_t	payload_len;
	uint32_t	usr_req_len;

	fcdev_t		tmpdev;

	fc_lun_t	*ptemp_fclun;	/* buf from scrap mem */
	fc_port_t	*ptemp_fcport;	/* buf from scrap mem */

	fc_port_t	*pfcport;

	os_lun_t	*plq;
	os_tgt_t	*ptq;

	Scsi_Cmnd	*pscsi_cmd;

	srb_t		*sp = NULL;
	EXT_ELS_PT_REQ	*pels_pt_req;

	struct list_head *fcil;
	fc_initiator_t	*fcinitiator;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	/* check on current topology */
	if ((ha->current_topology != ISP_CFG_F) &&
	    (ha->current_topology != ISP_CFG_FL)) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR not in F or FL mode\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	if (ha->ioctl_mem_size <= 0) {
		if (qla2x00_get_new_ioctl_dma_mem(ha,
		    QLA_INITIAL_IOCTLMEM_SIZE) != QL_STATUS_SUCCESS) {

			DEBUG9_10(printk("%s: ERROR cannot alloc DMA "
			    "buffer size=%lx.\n",
			    __func__, QLA_INITIAL_IOCTLMEM_SIZE);)

			pext->Status = EXT_STATUS_NO_MEMORY;
			return (ret);
		}
	}

	if (pext->ResponseLen > ha->ioctl_mem_size) {
		if (qla2x00_get_new_ioctl_dma_mem(ha, pext->ResponseLen) !=
		    QL_STATUS_SUCCESS) {

			DEBUG9_10(printk("%s: ERROR cannot alloc requested"
			    "DMA buffer size %x.\n",
			    __func__, pext->ResponseLen);)

			pext->Status = EXT_STATUS_NO_MEMORY;
			return (ret);
		}

		DEBUG9(printk("%s(%ld): inst=%ld rsp buf length larger than "
		    "existing size. Additional mem alloc successful.\n",
		    __func__, ha->host_no, ha->instance);)
	}

	usr_req_len = pext->RequestLen - sizeof(EXT_ELS_PT_REQ);
	if (usr_req_len > ha->ioctl_mem_size) {
		pext->Status = EXT_STATUS_INVALID_PARAM;

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR ReqLen too big=%x.\n",
		    __func__, ha->host_no, ha->instance, pext->RequestLen);)

		return (ret);
	}

	/* Verify entire EXT IOCTL request buffer */
	
	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    pext->RequestLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR verify read req buf\n",
		    __func__, ha->host_no, ha->instance);)

		return (ret);
	}

	DEBUG9(printk("%s(%ld): inst=%ld req buf verified.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pscsi_cmd,
	    sizeof(Scsi_Cmnd))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Cmnd));)
		return (ret);
	}
	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pels_pt_req,
	    sizeof(EXT_ELS_PT_REQ))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "els_pt_req size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_ELS_PT_REQ));)
		goto els_passthru_done;
	}
	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptemp_fclun,
	    sizeof(fc_lun_t))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "fclun size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(fc_lun_t));)
		goto els_passthru_done;
	}
	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptemp_fcport,
	    sizeof(fc_port_t))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "fcport size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(fc_port_t));)
		goto els_passthru_done;
	}

	/* initialize */

	memset(ha->ioctl_mem, 0, ha->ioctl_mem_size);

	/* copy request buffer */
	
	ret = copy_from_user(pels_pt_req, pext->RequestAdr,
	    sizeof(EXT_ELS_PT_REQ));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR"
		    "copy_from_user() of struct failed (%d).\n",
		    __func__, ha->host_no, ha->instance, ret);)

		goto els_passthru_done;
	}

	pusr_req_buf = (uint8_t *)pext->RequestAdr + sizeof(EXT_ELS_PT_REQ);
	
	ret = copy_from_user(ha->ioctl_mem, pusr_req_buf, usr_req_len);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR"
		    "copy_from_user() of request buf failed (%d).\n",
		    __func__, ha->host_no, ha->instance, ret);)

		goto els_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld after copy request.\n",
	    __func__, ha->host_no, ha->instance);)
	
	/* check on loop down (1) */
	if (ha->loop_state != LOOP_READY || 
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) || ABORTS_ACTIVE) {

		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld before dest port validation- loop not "
		    "ready; cannot proceed.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_BUSY;

		goto els_passthru_done;
	}

	/*********************************/
	/* Validate the destination port */
	/*********************************/

	/* first: WWN cannot be zero if no PID is specified */
	invalid_wwn = qla2x00_is_wwn_zero(pels_pt_req->WWPN);
	if (invalid_wwn && !(pels_pt_req->ValidMask & EXT_DEF_PID_VALID)) {
		/* error: both are not set. */
		pext->Status = EXT_STATUS_INVALID_PARAM;

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR no valid WWPN/PID\n",
		    __func__, ha->host_no, ha->instance);)

		goto els_passthru_done;
	}

	/* second: it cannot be the local/current HBA itself */
	if (!invalid_wwn) {
		if (memcmp(ha->init_cb->port_name, pels_pt_req->WWPN,
		    EXT_DEF_WWN_NAME_SIZE) == 0) {

			/* local HBA specified. */

			pext->Status = EXT_STATUS_INVALID_PARAM;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR local HBA's "
			    "WWPN found.\n",
			    __func__, ha->host_no, ha->instance);)

			goto els_passthru_done;
		}
	} else { /* using PID */
		if (pels_pt_req->Id[1] == ha->d_id.r.d_id[2]
		    && pels_pt_req->Id[2] == ha->d_id.r.d_id[1]
		    && pels_pt_req->Id[3] == ha->d_id.r.d_id[0]) {

			/* local HBA specified. */

			pext->Status = EXT_STATUS_INVALID_PARAM;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR local HBA's "
			    "PID found.\n",
			    __func__, ha->host_no, ha->instance);)

			goto els_passthru_done;
		}
	}

	/************************/
	/* Now find the loop ID */
	/************************/

	/* 1st: scan thru our HBA database */
	index = 0;
	port_found = FALSE;
	fcinitiator = NULL;
	if (!invalid_wwn) {
		/* search with WWPN */
		list_for_each(fcil, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if (memcmp(pels_pt_req->WWPN, fcinitiator->port_name,
				 EXT_DEF_WWN_NAME_SIZE) == 0) {

				port_found = TRUE;
				pels_pt_req->Lid = fcinitiator->loop_id;

				DEBUG9(printk("%s(%ld): inst=%ld found host "
				    "w/ WWN. loop_id = %02x.\n",
				    __func__, ha->host_no, ha->instance,
				    pels_pt_req->Lid);)

				break;
			}
		}
	} else {
		/* search with PID */
		list_for_each(fcil, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if (pels_pt_req->Id[1] == fcinitiator->d_id.r.d_id[2]
			    && pels_pt_req->Id[2] == fcinitiator->d_id.r.d_id[1]
			    && pels_pt_req->Id[3] ==
				    fcinitiator->d_id.r.d_id[0]) {

				port_found = TRUE;
				pels_pt_req->Lid = fcinitiator->loop_id;

				DEBUG9(printk("%s(%ld): inst=%ld found host "
				    "w/ WWN. loop_id = %02x.\n",
				    __func__, ha->host_no, ha->instance,
				    pels_pt_req->Lid);)

				break;
			}
		}
	}

	/* If this is for a host device, check if we need to perform login */
	if (port_found && (fcinitiator->loop_id >= LAST_SNS_LOOP_ID)) {

		DEBUG9_10(printk("%s(%ld): inst=%ld need to relogin to "
		    "dest host.\n",
		    __func__, ha->host_no, ha->instance);)

		if (fcinitiator->d_id.b24 == 0) {
			/* Either RSCN hasn't been processed yet or
			 * this host is no longer connected to us.
			 */
			pext->Status = EXT_STATUS_DEV_NOT_FOUND;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR dest host "
			    "port lost.\n",
			    __func__, ha->host_no, ha->instance);)

			goto els_passthru_done;
		}

		/* login and update database */
		tmpdev.d_id.b24 = fcinitiator->d_id.b24;
		tmpdev.loop_id = fcinitiator->loop_id;

		if (qla2x00_host_relogin(ha, &tmpdev) != 0) {
			/* login failed. */
			pext->Status = EXT_STATUS_DEV_NOT_FOUND;

			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR login to "
			    "host port failed. loop_id=%02x pid=%06x ret=%d.\n",
			    __func__, ha->host_no, ha->instance, tmpdev.loop_id,
			    tmpdev.d_id.b24, ret);)

			goto els_passthru_done;
		} else {
			fcinitiator->loop_id = tmpdev.loop_id;
			pels_pt_req->Lid = tmpdev.loop_id;

			DEBUG9(printk("%s(%ld): inst=%ld success login to "
			    "remote host; Lid=%02x.\n",
			    __func__, ha->host_no, ha->instance,
			    fcinitiator->loop_id);)
		}
	}
	
	/* 2nd: scan thru our fcport database */
	if (!invalid_wwn) {
		/* search with WWPN */
		for (pfcport = ha->fcport;
		    (!port_found) && pfcport != NULL; pfcport = pfcport->next) {

			if (memcmp(pfcport->port_name, pels_pt_req->WWPN,
			    EXT_DEF_WWN_NAME_SIZE) == 0) {

				port_found = TRUE;
				pels_pt_req->Lid = pfcport->loop_id;

				DEBUG9(printk("%s(%ld): inst=%ld found fcport "
				    "w/ WWN. loop_id = %02x.\n",
				    __func__, ha->host_no, ha->instance,
				    pels_pt_req->Lid);)
				break;
			}
		}
	} else {
		/* search with PID */
		for (pfcport = ha->fcport;
		    (!port_found) && pfcport != NULL; pfcport = pfcport->next) {

			if (pels_pt_req->Id[1] == pfcport->d_id.r.d_id[2]
			    && pels_pt_req->Id[2] == pfcport->d_id.r.d_id[1]
			    && pels_pt_req->Id[3] == pfcport->d_id.r.d_id[0]) {

				port_found = TRUE;
				pels_pt_req->Lid = pfcport->loop_id;

				DEBUG9(printk("%s(%ld): inst=%ld found fcport "
				    "w/ PID. loop_id = %02x.\n",
				    __func__, ha->host_no, ha->instance,
				    pels_pt_req->Lid);)

				break;
			}
		}
	}
	
	if (!port_found) {
		/* invalid WWN or PID specified */
		pext->Status = EXT_STATUS_INVALID_PARAM;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR WWPN/PID invalid.\n",
		    __func__, ha->host_no, ha->instance);)

		goto els_passthru_done;
	}

	/* alloc sp */
	if ((sp = qla2x00_get_new_sp(ha)) == NULL) {

		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s: ERROR cannot alloc sp %p.\n",
		    __func__, sp);)

		goto els_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld after alloc sp.\n",
	    __func__, ha->host_no, ha->instance);)

	/* setup sp for this command */
	ptq = ha->ioctl->ioctl_tq;
	plq = ha->ioctl->ioctl_lq;
	sp->cmd = pscsi_cmd;
	sp->flags = SRB_IOCTL;
	sp->lun_queue = plq;
	sp->tgt_queue = ptq;
	ptemp_fclun->fcport = ptemp_fcport;
	ptemp_fclun->lun = 0;
	ptemp_fclun->flags = 0;
	ptemp_fclun->next = NULL;
	plq->fclun = ptemp_fclun;
	plq->fclun->fcport->ha = ha;

	/* init scsi_cmd */
	pscsi_cmd->host = ha->host;
	pscsi_cmd->scsi_done = qla2x00_msiocb_done;

	/* check on loop down (2)- check again just before sending cmd out. */
	if (ha->loop_state != LOOP_READY || 
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	    ABORTS_ACTIVE) {

		DEBUG9_10(printk("%s(%ld): inst=%ld before issue cmd- loop "
		    "not ready.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_BUSY;

		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto els_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld going to issue command.\n",
	    __func__, ha->host_no, ha->instance);)

	tmp_rval = qla2x00_start_ms_cmd(ha, pext, sp, pels_pt_req);

	DEBUG9(printk("%s(%ld): inst=%ld after issue command.\n",
	    __func__, ha->host_no, ha->instance);)

	if (tmp_rval != 0) {
		/* We waited and post function did not get called */
		DEBUG9_10(printk("%s(%ld): inst=%ld command timed out.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_MS_NO_RESPONSE;

		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto els_passthru_done;
	}

	/* check on data returned */
	ptmp_stat = (uint8_t *)ha->ioctl_mem + FC_HEADER_LEN;

	if (*ptmp_stat == ELS_STAT_LS_RJT) {
		payload_len = FC_HEADER_LEN + ELS_RJT_LENGTH;

	} else if (*ptmp_stat == ELS_STAT_LS_ACC) {
		payload_len = pext->ResponseLen - sizeof(EXT_ELS_PT_REQ);

	} else {
		/* invalid. just copy the status word. */
		DEBUG9_10(printk("%s(%ld): inst=%ld invalid stat "
		    "returned =0x%x.\n",
		    __func__, ha->host_no, ha->instance, *ptmp_stat);)

		payload_len = FC_HEADER_LEN + 4;
	}

	DEBUG9(printk("%s(%ld): inst=%ld data dump-\n",
	    __func__, ha->host_no, ha->instance);)
	DEBUG9(qla2x00_dump_buffer((uint8_t *)ptmp_stat,
	    pext->ResponseLen - sizeof(EXT_ELS_PT_REQ) - FC_HEADER_LEN);)
	
	/* Verify response buffer to be written */
	/* The data returned include FC frame header */
	presp_payload = (uint8_t *)pext->ResponseAdr + sizeof(EXT_ELS_PT_REQ);

	ret = verify_area(VERIFY_WRITE, (void *)presp_payload, payload_len);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp "
		    "buffer. ha=%p.\n",
		    __func__, ha->host_no, ha->instance, ha);)

		goto els_passthru_done;
	}

	/* copy back data returned to response buffer */
	ret = copy_to_user(presp_payload, (uint8_t *)ha->ioctl_mem,
	    payload_len);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		goto els_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting normally.\n",
	    __func__, ha->host_no, ha->instance);)

els_passthru_done:

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}
#endif

/*
 * qla2x00_send_fcct
 *	Passes the FC CT command down to firmware as MSIOCB and
 *	copies the response back when it completes.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_send_fcct(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	int		tmp_rval = 0;
	uint16_t	mb[MAILBOX_REGISTER_COUNT];

	fc_lun_t	*ptemp_fclun;
	fc_port_t	*ptemp_fcport;

	os_lun_t	*lq;
	os_tgt_t	*tq;

	Scsi_Cmnd	*pscsi_cmd;
	srb_t		*sp = NULL;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	/* check on current topology */
	if ((ha->current_topology != ISP_CFG_F) &&
	    (ha->current_topology != ISP_CFG_FL)) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR not in F or FL mode\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	if (ha->ioctl_mem_size <= 0) {
		if (qla2x00_get_new_ioctl_dma_mem(ha,
		    QLA_INITIAL_IOCTLMEM_SIZE) != QL_STATUS_SUCCESS) {

			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc "
			    "DMA buffer size=%lx.\n",
			    __func__, ha->host_no, ha->instance,
			    QLA_INITIAL_IOCTLMEM_SIZE);)

			pext->Status = EXT_STATUS_NO_MEMORY;
			return (ret);
		}
	}

	if (pext->ResponseLen > ha->ioctl_mem_size) {
		if (qla2x00_get_new_ioctl_dma_mem(ha, pext->ResponseLen) !=
		    QL_STATUS_SUCCESS) {
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc "
			    "requested DMA buffer size %x.\n",
			    __func__, ha->host_no, ha->instance,
			    pext->ResponseLen);)
			pext->Status = EXT_STATUS_NO_MEMORY;
			return (ret);
		}

		DEBUG9(printk("%s(%ld): inst=%ld rsp buf len larger than "
		    "existing size. Additional mem alloc successful.\n",
		    __func__, ha->host_no, ha->instance);)
	}

	if (pext->RequestLen > ha->ioctl_mem_size) {
		pext->Status = EXT_STATUS_INVALID_PARAM;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR ReqLen too big=%x.\n",
		    __func__, ha->host_no, ha->instance, pext->RequestLen);)

		return (ret);
	}

	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    pext->RequestLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR verify read req buf\n",
		    __func__, ha->host_no, ha->instance);)

		return (ret);
	}

	DEBUG9(printk("%s(%ld): inst=%ld req buf verified.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pscsi_cmd,
	    sizeof(Scsi_Cmnd))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Cmnd));)
		return (ret);
	}
	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptemp_fclun,
	    sizeof(fc_lun_t))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "fclun size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(fc_lun_t));)
		goto fcct_passthru_done;
	}
	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&ptemp_fcport,
	    sizeof(fc_port_t))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "fcport size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(fc_port_t));)
		goto fcct_passthru_done;
	}

	/* initialize */
	memset(ha->ioctl_mem, 0, ha->ioctl_mem_size);

	/* copy request buffer */
	ret = copy_from_user(ha->ioctl_mem, pext->RequestAdr, pext->RequestLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR copy req buf. ret=%d\n",
		    __func__, ha->host_no, ha->instance, ret);)


		goto fcct_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld after copy request.\n",
	    __func__, ha->host_no, ha->instance);)

	/* alloc sp */
	if ((sp = qla2x00_get_new_sp(ha)) == NULL) {

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc "
		    "sp %p.\n",
		    __func__, ha->host_no, ha->instance, sp);)

		pext->Status = EXT_STATUS_NO_MEMORY;
		goto fcct_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld after alloc sp.\n",
	    __func__, ha->host_no, ha->instance);)

	/* setup sp for this command */
	tq = ha->ioctl->ioctl_tq;
	lq = ha->ioctl->ioctl_lq;
	sp->cmd = pscsi_cmd;
	sp->flags = SRB_IOCTL;
	sp->lun_queue = lq;
	sp->tgt_queue = tq;
	ptemp_fclun->fcport = ptemp_fcport;
	ptemp_fclun->lun = 0;
	ptemp_fclun->flags = 0;
	ptemp_fclun->next = NULL;
	lq->fclun = ptemp_fclun;
	lq->fclun->fcport->ha = ha;

	/* init scsi_cmd */
	pscsi_cmd->host = ha->host;
	pscsi_cmd->scsi_done = qla2x00_msiocb_done;

	/* check on management server login status */
	if (ha->flags.management_server_logged_in == 0) {
		/* login to management server device */

		tmp_rval = qla2x00_login_fabric(ha, MANAGEMENT_SERVER,
		    0xff, 0xff, 0xfa, &mb[0], BIT_1);

		if (tmp_rval != 0 || mb[0] != 0x4000) {
			pext->Status = EXT_STATUS_DEV_NOT_FOUND;

	 		DEBUG10(printk("%s(%ld): inst=%ld ERROR login to MS.\n",
			    __func__, ha->host_no, ha->instance);)

			atomic_set(&sp->ref_count, 0);
			add_to_free_queue (ha, sp);

			goto fcct_passthru_done;
		}

		ha->flags.management_server_logged_in = 1;
	}

	DEBUG9(printk("%s(%ld): success login to MS.\n",
	    __func__, ha->host_no);)

	/* check on loop down */
	if (ha->loop_state != LOOP_READY || 
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	    ABORTS_ACTIVE) {

		DEBUG10(printk("%s(%ld): inst=%ld loop not ready.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_BUSY;

		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto fcct_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld going to issue command.\n",
	    __func__, ha->host_no, ha->instance);)

	tmp_rval = qla2x00_start_ms_cmd(ha, pext, sp, NULL);

	DEBUG9(printk("%s(%ld): inst=%ld after issue command.\n",
	    __func__, ha->host_no, ha->instance);)

	if (tmp_rval != 0) {
		/* We waited and post function did not get called */
		DEBUG9_10(printk("%s(%ld): inst=%ld command timed out.\n",
		    __func__, ha->host_no, ha->instance);)
		pext->Status = EXT_STATUS_MS_NO_RESPONSE;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto fcct_passthru_done;
	}

	if (CMD_COMPL_STATUS(pscsi_cmd) != 0 ||
	    CMD_ENTRY_STATUS(pscsi_cmd) != 0) {
		DEBUG9_10(printk("%s(%ld): inst=%ld cmd returned error=%x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pscsi_cmd));)
		pext->Status = EXT_STATUS_ERR;
		goto fcct_passthru_done;
	}

	/* getting device data and putting in pext->ResponseAdr */
	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr ,
	    pext->ResponseLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify wrt rsp "
		    "buffer. ha=%p.\n",
		    __func__, ha->host_no, ha->instance, ha);)
		goto fcct_passthru_done;
	}

	/* sending back data returned from Management Server */
	ret = copy_to_user((uint8_t *)pext->ResponseAdr,
	    (uint8_t *)ha->ioctl_mem, pext->ResponseLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		goto fcct_passthru_done;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

fcct_passthru_done:

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}

/*
 * qla2x00_start_ms_cmd
 *	Allocates an MSIOCB request pkt and sends out the passthru cmd.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_start_ms_cmd(scsi_qla_host_t *ha, EXT_IOCTL *pext, srb_t *sp,
    EXT_ELS_PT_REQ *pels_pt_req)
{
#define	ELS_REQUEST_RCTL	0x22
#define ELS_REPLY_RCTL		0x23

	uint32_t	usr_req_len;
	uint32_t	usr_resp_len;
	uint64_t	usr_reqbuf_addr;

	ms_iocb_entry_t		*pkt;
	unsigned long		cpu_flags = 0;


	/* get spin lock for this operation */
	spin_lock_irqsave(&ha->hardware_lock, cpu_flags);

	/* Get MS request packet. */
	pkt = (ms_iocb_entry_t *)qla2x00_ms_req_pkt(ha, sp);
	if (pkt == NULL) {
		/* release spin lock and return error. */
		spin_unlock_irqrestore(&ha->hardware_lock, cpu_flags);

		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG10(printk("%s(%ld): inst=%ld MSIOCB PT - could not get "
		    "Request Packet.\n", __func__, ha->host_no, ha->instance);)
		return (QL_STATUS_RESOURCE_ERROR);
	}

	pkt->entry_type  = MS_IOCB_TYPE;
	pkt->entry_count = 1;
	
	if (pels_pt_req != NULL) {
		/* process ELS passthru command */
		usr_reqbuf_addr = (uint64_t)ha->ioctl_mem_phys +
		    sizeof(EXT_ELS_PT_REQ);
		usr_req_len = pext->RequestLen - sizeof(EXT_ELS_PT_REQ);
		usr_resp_len = pext->ResponseLen - sizeof(EXT_ELS_PT_REQ);
	
		pkt->control_flags = BIT_15; /* ELS passthru enabled */
		pkt->loop_id = pels_pt_req->Lid;
		pkt->type    = 1; /* ELS frame */
		
		if (pext->ResponseLen != 0) {
			pkt->r_ctl    = ELS_REQUEST_RCTL;
			pkt->rx_id    = 0;
		} else {
			pkt->r_ctl    = ELS_REPLY_RCTL;
			pkt->rx_id    = pels_pt_req->Rxid;
		}
	} else {
		usr_reqbuf_addr = (uint64_t)ha->ioctl_mem_phys;
		usr_req_len = pext->RequestLen;
		usr_resp_len = pext->ResponseLen;
		pkt->loop_id     = MANAGEMENT_SERVER;
	}

	DEBUG9_10(printk("%s(%ld): inst=%ld using loop_id=%02x req_len=%d, "
	    "resp_len=%d.\n", __func__, ha->host_no, ha->instance,
	    pkt->loop_id, usr_req_len, usr_resp_len);)

	pkt->timeout = QLA_PT_CMD_TOV;
	pkt->cmd_dsd_count = 1;
	pkt->total_dsd_count = 2; /* no continuation */
	pkt->rsp_bytecount = usr_resp_len;
	pkt->req_bytecount = usr_req_len;

	/* loading command payload address */
	pkt->dseg_req_address[0] = LS_64BITS(usr_reqbuf_addr);
	pkt->dseg_req_address[1] = MS_64BITS(usr_reqbuf_addr);
	pkt->dseg_req_length = usr_req_len;

	/* loading response payload address */
	pkt->dseg_rsp_address[0] = LS_64BITS(ha->ioctl_mem_phys);
	pkt->dseg_rsp_address[1] = MS_64BITS(ha->ioctl_mem_phys);
	pkt->dseg_rsp_length = usr_resp_len;

	/* set flag to indicate IOCTL MSIOCB cmd in progress */
	ha->ioctl->MSIOCB_InProgress = 1;
	ha->ioctl->ioctl_tov = pkt->timeout + 1; /* 1 second more */

	/* prepare for receiving completion. */
	qla2x00_ioctl_sem_init(ha);

	/* Issue command to ISP */
	qla2x00_isp_cmd(ha);

	ha->ioctl->cmpl_timer.expires = jiffies + ha->ioctl->ioctl_tov * HZ;
	add_timer(&ha->ioctl->cmpl_timer);

	DEBUG9(printk("%s(%ld): inst=%ld releasing hardware_lock.\n",
	    __func__, ha->host_no, ha->instance);)
	spin_unlock_irqrestore(&ha->hardware_lock, cpu_flags);

	DEBUG9(printk("%s(%ld): inst=%ld sleep for completion.\n",
	    __func__, ha->host_no, ha->instance);)

	down(&ha->ioctl->cmpl_sem);

	del_timer(&ha->ioctl->cmpl_timer);

	if (ha->ioctl->MSIOCB_InProgress == 1) {
	 	DEBUG9_10(printk("%s(%ld): inst=%ld timed out. exiting.\n",
		    __func__, ha->host_no, ha->instance);)
		return QL_STATUS_ERROR;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return QL_STATUS_SUCCESS;
}

/*
 * qla2x00_wwpn_to_scsiaddr
 *	Handles the EXT_CC_WWPN_TO_SCSIADDR command.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_wwpn_to_scsiaddr(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	fc_port_t	*tgt_fcport;
	os_tgt_t	*tq;
	uint8_t		tmp_wwpn[EXT_DEF_WWN_NAME_SIZE];
	uint32_t	b, tgt, l;
	EXT_SCSI_ADDR	tmp_addr;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (pext->RequestLen != EXT_DEF_WWN_NAME_SIZE ||
	    pext->ResponseLen < sizeof(EXT_SCSI_ADDR)) {
		/* error */
		DEBUG9_10(printk("%s(%ld): inst=%ld invalid WWN buffer size %d "
		    "received.\n",
		    __func__, ha->host_no, ha->instance, pext->ResponseLen);)
		pext->Status = EXT_STATUS_INVALID_PARAM;

		return (ret);
	}

	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    pext->RequestLen);
	if (ret) {
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR VERIFY_READ req buf\n",
		    __func__, ha->host_no, ha->instance);)
		pext->Status = EXT_STATUS_COPY_ERR;
		return (ret);
	}

	ret = copy_from_user(tmp_wwpn, pext->RequestAdr, pext->RequestLen);
	if (ret) {
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy_from_user "
		    "failed(%d) on request buf.\n",
		    __func__, ha->host_no, ha->instance, ret);)
		pext->Status = EXT_STATUS_COPY_ERR;
		return (ret);
	}

	tq = NULL;
	for (tgt = 0; tgt < MAX_TARGETS; tgt++) {
		if (ha->otgt[tgt] == NULL) {
			continue;
		}

		tq = ha->otgt[tgt];
		if (tq->vis_port == NULL) {
			break;
		}

		tgt_fcport = tq->vis_port;
		if (memcmp(tmp_wwpn, tgt_fcport->port_name,
		    EXT_DEF_WWN_NAME_SIZE) == 0) {
			break;
		}
	}

	if (tq == NULL || tgt >= MAX_TARGETS) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld target dev not found. "
		    "tq=%p, tgt=%x.\n", __func__, ha->host_no, ha->instance,
		    tq, tgt);)
		return (ret);
	}

	if (tq->vis_port == NULL) { 	/* dg 08/14/01 */
		pext->Status = EXT_STATUS_BUSY;
		DEBUG9_10(printk("%s(%ld): inst=%ld target port not found. "
		    "tq=%p, tgt=%x.\n",
		    __func__, ha->host_no, ha->instance, tq, tgt);)
		return (ret);
	}	

	/* Currently we only have bus 0 and no translation on LUN */
	b = 0;
	l = 0;

	/*
	 * Return SCSI address. Currently no translation is done for
	 * LUN.
	 */
	tmp_addr.Bus = b;
	tmp_addr.Target = tgt;
	tmp_addr.Lun = l;
	if (pext->ResponseLen > sizeof(EXT_SCSI_ADDR))
		pext->ResponseLen = sizeof(EXT_SCSI_ADDR);

	ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
	    pext->ResponseLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR VERIFY wrt rsp buf\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	ret = copy_to_user((uint8_t *)pext->ResponseAdr, &tmp_addr,
	    pext->ResponseLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buffer.\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	DEBUG9(printk(KERN_INFO
	    "%s(%ld): Found t%d l%d for %02x%02x%02x%02x%02x%02x%02x%02x.\n",
	    __func__, ha->host_no,
	    tmp_addr.Target, tmp_addr.Lun,
	    tmp_wwpn[0], tmp_wwpn[1], tmp_wwpn[2], tmp_wwpn[3],
	    tmp_wwpn[4], tmp_wwpn[5], tmp_wwpn[6], tmp_wwpn[7]);)

	pext->Status = EXT_STATUS_OK;

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_scsi_passthru
 *	Handles all subcommands of the EXT_CC_SEND_SCSI_PASSTHRU command.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_scsi_passthru(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int tmp_rval = 0;

	switch(pext->SubCode) {
	case EXT_SC_SEND_SCSI_PASSTHRU:
		tmp_rval = qla2x00_sc_scsi_passthru(ha, pext, mode);
		break;
	case EXT_SC_SEND_FC_SCSI_PASSTHRU:
		tmp_rval = qla2x00_sc_fc_scsi_passthru(ha, pext, mode);
		break;
	case EXT_SC_SCSI3_PASSTHRU:
		tmp_rval = qla2x00_sc_scsi3_passthru(ha, pext, mode);
		break;
	default:
		break;
	}

	return tmp_rval;
}

/*
 * qla2x00_sc_scsi_passthru
 *	Handles EXT_SC_SEND_SCSI_PASSTHRU subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_sc_scsi_passthru(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	uint8_t		*usr_temp, *kernel_tmp;
	uint8_t		scsi_direction;
	uint32_t	i;

#if defined(QL_DEBUG_LEVEL_9)
	uint32_t	b, t, l;
#endif
	uint32_t	transfer_len;

	EXT_SCSI_PASSTHRU	*pscsi_pass;

	Scsi_Cmnd		*pscsi_cmd;
	Scsi_Device		*pscsi_device;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    sizeof(EXT_SCSI_PASSTHRU));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify READ "
		    "req buf.\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	if (pext->ResponseLen > ha->ioctl_mem_size) {
		if (qla2x00_get_new_ioctl_dma_mem(ha, pext->ResponseLen) !=
		    QL_STATUS_SUCCESS) {
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc "
			    "requested DMA buffer size %x.\n",
			    __func__, ha->host_no, ha->instance,
			    pext->ResponseLen);)
			pext->Status = EXT_STATUS_NO_MEMORY;
			return (ret);
		}
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pscsi_pass,
	    sizeof(EXT_SCSI_PASSTHRU))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_SCSI_PASSTHRU));)
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pscsi_cmd,
	    sizeof(Scsi_Cmnd))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "pscsi_cmd requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Cmnd));)
		goto scsi_passthru_done;
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pscsi_device,
	    sizeof(Scsi_Device))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "pscsi_device requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Device));)
		goto scsi_passthru_done;
	}

	/* clear ioctl_mem to be used */
	memset(ha->ioctl_mem, 0, ha->ioctl_mem_size);

	/* Copy request buffer */
	usr_temp = (uint8_t *)pext->RequestAdr;
	kernel_tmp = (uint8_t *)pscsi_pass;
	ret = copy_from_user(kernel_tmp, usr_temp, sizeof(EXT_SCSI_PASSTHRU));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR copy req buf ret=%d\n",
		    __func__, ha->host_no, ha->instance, ret);)
		goto scsi_passthru_done;
	}

	/* set target coordinates */
	pscsi_cmd->target = pscsi_pass->TargetAddr.Target;
	pscsi_cmd->lun    = pscsi_pass->TargetAddr.Lun;

	/* Verify target exists */
	if (TGT_Q(ha, pscsi_cmd->target) == NULL) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR tgt %d not found.\n",
		    __func__, ha->host_no, ha->instance, pscsi_cmd->target);)
		goto scsi_passthru_done;
	}

	/* Copy over cdb */

	if (pscsi_pass->CdbLength == 6) {
		pscsi_cmd->cmd_len = 6;

	} else if (pscsi_pass->CdbLength == 10) {
		pscsi_cmd->cmd_len = 0x0A;

	} else if (pscsi_pass->CdbLength == 12) {
		pscsi_cmd->cmd_len = 0x0C;

	} else {
		printk(KERN_WARNING
		    "%s: Unsupported Cdb Length=%x.\n",
		    __func__, pscsi_pass->CdbLength);

		pext->Status = EXT_STATUS_INVALID_PARAM;
		goto scsi_passthru_done;
	}

	memcpy(pscsi_cmd->data_cmnd, pscsi_pass->Cdb, pscsi_cmd->cmd_len);
	memcpy(pscsi_cmd->cmnd, pscsi_pass->Cdb, pscsi_cmd->cmd_len);

	DEBUG9(printk("%s Dump of cdb buffer:\n", __func__);)
	DEBUG9(qla2x00_dump_buffer((uint8_t *)&pscsi_cmd->data_cmnd[0],
	    pscsi_cmd->cmd_len);)

	pscsi_cmd->host    = ha->host;

	/* mark this as a special delivery and collection command */
	pscsi_cmd->flags     = 0;
	pscsi_cmd->scsi_done = qla2x00_scsi_pt_done;

	pscsi_cmd->device               = pscsi_device;
	pscsi_cmd->device->tagged_queue = 0;
	pscsi_cmd->use_sg               = 0; /* no ScatterGather */
	pscsi_cmd->request_bufflen      = pext->ResponseLen;
	pscsi_cmd->request_buffer       = ha->ioctl_mem;
	pscsi_cmd->timeout_per_command  = QLA_PT_CMD_TOV * HZ;
	CMD_RESID_LEN(pscsi_cmd) = SRB_IOCTL; /* Used to set sp->flags later */

	if (pscsi_pass->Direction == EXT_DEF_SCSI_PASSTHRU_DATA_OUT) {
		/* sending user data from pext->ResponseAdr to device */
		ret = verify_area(VERIFY_READ, (void *)pext->ResponseAdr,
		    pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify READ "
			    "rsp buf.\n", __func__, ha->host_no, ha->instance);)
			goto scsi_passthru_done;
		}

		pscsi_cmd->sc_data_direction = SCSI_DATA_WRITE;
		usr_temp   = (uint8_t *)pext->ResponseAdr;
		kernel_tmp = (uint8_t *)ha->ioctl_mem;
		ret = copy_from_user(kernel_tmp, usr_temp, pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buf "
			    "failed(%d).\n",
			    __func__, ha->host_no, ha->instance, ret);)
			goto scsi_passthru_done;
		}
	} else {
		pscsi_cmd->sc_data_direction = SCSI_DATA_READ;
	}

	DEBUG9({
		b = SCSI_BUS_32(pscsi_cmd);
		t = SCSI_TCN_32(pscsi_cmd);
		l = SCSI_LUN_32(pscsi_cmd);
	})
	DEBUG9(printk("%s(%ld): CDB=%02x %02x %02x %02x; b=%x t=%x l=%x.\n",
	    __func__, ha->host_no, pscsi_cmd->cmnd[0], pscsi_cmd->cmnd[1],
	    pscsi_cmd->cmnd[2], pscsi_cmd->cmnd[3], b, t, l);)

	/*
	 * Check the status of the port
	 */
	if (qla2x00_check_tgt_status(ha, pscsi_cmd) != QL_STATUS_SUCCESS) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld check_tgt_status "
		    "failed.\n",
		    __func__, ha->host_no, ha->instance);)
		goto scsi_passthru_done;
	}

	/* set flag to indicate IOCTL SCSI PassThru in progress */
	ha->ioctl->SCSIPT_InProgress = 1;
	ha->ioctl->ioctl_tov = (int)QLA_PT_CMD_DRV_TOV;

	/* prepare for receiving completion. */
	qla2x00_ioctl_sem_init(ha);
	CMD_COMPL_STATUS(pscsi_cmd) = (int) IOCTL_INVALID_STATUS;

	/* send command to adapter */
	DEBUG9(printk("%s(%ld): inst=%ld sending command.\n",
	    __func__, ha->host_no, ha->instance);)

	/* get spin lock for this operation */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	spin_lock_irqsave(&io_request_lock, ha->cpu_flags);
#else
	spin_lock_irqsave(ha->host->host_lock, ha->cpu_flags);
#endif

	qla2x00_queuecommand(pscsi_cmd, (void *) qla2x00_scsi_pt_done);

	ha->ioctl->cmpl_timer.expires = jiffies + ha->ioctl->ioctl_tov * HZ;
	add_timer(&ha->ioctl->cmpl_timer);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	spin_unlock_irqrestore(&io_request_lock, ha->cpu_flags);
#else
	spin_unlock_irqrestore(ha->host->host_lock, ha->cpu_flags);
#endif

	DEBUG9(printk("%s(%ld): inst=%ld waiting for completion.\n",
	    __func__, ha->host_no, ha->instance);)

	down(&ha->ioctl->cmpl_sem);

	del_timer(&ha->ioctl->cmpl_timer);

	DEBUG9(printk("%s(%ld): inst=%ld completed.\n",
	    __func__, ha->host_no, ha->instance);)

	if (ha->ioctl->SCSIPT_InProgress == 1) {

		printk(KERN_WARNING
		    "qla2x00: scsi%ld ERROR passthru command timeout.\n",
		    ha->host_no);

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		goto scsi_passthru_done;
	}

	if (CMD_COMPL_STATUS(pscsi_cmd) == (int)IOCTL_INVALID_STATUS) {

		DEBUG9(printk("%s(%ld): inst=%ld ERROR - cmd not completed.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_ERR;
		goto scsi_passthru_done;
	}

	switch (CMD_COMPL_STATUS(pscsi_cmd)) {
	case CS_INCOMPLETE:
	case CS_ABORTED:
	case CS_PORT_UNAVAILABLE:
	case CS_PORT_LOGGED_OUT:
	case CS_PORT_CONFIG_CHG:
	case CS_PORT_BUSY:
		DEBUG9_10(printk("%s(%ld): inst=%ld cs err = %x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pscsi_cmd));)
		pext->Status = EXT_STATUS_BUSY;

		goto scsi_passthru_done;
	}

	if ((CMD_COMPL_STATUS(pscsi_cmd) == CS_DATA_UNDERRUN) ||
	    (CMD_SCSI_STATUS(pscsi_cmd) != 0)) {

		/* have done the post function */
		pext->Status       = EXT_STATUS_SCSI_STATUS;
		pext->DetailStatus = CMD_SCSI_STATUS(pscsi_cmd) & 0xff;
		DEBUG9_10(printk("%s(%ld): inst=%ld data underrun or scsi err. "
		    "host status =0x%x, scsi status = 0x%x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pscsi_cmd), CMD_SCSI_STATUS(pscsi_cmd));)

	} else if (CMD_COMPL_STATUS(pscsi_cmd) != 0) {
		DEBUG9_10(printk("%s(%ld): inst=%ld cs err=%x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pscsi_cmd));)
		pext->Status = EXT_STATUS_ERR;
		goto scsi_passthru_done;
	}

	/* copy up structure to make sense data available to user */
	pscsi_pass->SenseLength = CMD_ACTUAL_SNSLEN(pscsi_cmd);
	if (CMD_ACTUAL_SNSLEN(pscsi_cmd)) {
		for (i = 0; i < CMD_ACTUAL_SNSLEN(pscsi_cmd); i++)
			pscsi_pass->SenseData[i] = pscsi_cmd->sense_buffer[i];

		DEBUG10(printk("%s Dump of sense buffer:\n", __func__);)
		DEBUG10(qla2x00_dump_buffer(
		    (uint8_t *)&pscsi_pass->SenseData[0],
		    CMD_ACTUAL_SNSLEN(pscsi_cmd));)

		ret = verify_area(VERIFY_WRITE, (void *)pext->RequestAdr,
		    sizeof(EXT_SCSI_PASSTHRU));
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify WRITE "
			    "req buf.\n", __func__, ha->host_no, ha->instance);)
			goto scsi_passthru_done;
		}

		usr_temp   = (uint8_t *)pext->RequestAdr;
		kernel_tmp = (uint8_t *)pscsi_pass;
		ret = copy_to_user(usr_temp, kernel_tmp,
		    sizeof(EXT_SCSI_PASSTHRU));
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy sense "
			    "buffer.\n",
			    __func__, ha->host_no, ha->instance);)
			goto scsi_passthru_done;
		}
	}

	scsi_direction = pscsi_pass->Direction;

	if (scsi_direction == EXT_DEF_SCSI_PASSTHRU_DATA_IN) {
		DEBUG9(printk("%s(%ld): inst=%ld copying data.\n",
		    __func__, ha->host_no, ha->instance);)

		/* getting device data and putting in pext->ResponseAdr */
		ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr ,
		    pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify write "
			    "ResponseAdr.\n",
			    __func__, ha->host_no, ha->instance);)
			goto scsi_passthru_done;
		}

		/* now copy up the READ data to user */
		if ((CMD_COMPL_STATUS(pscsi_cmd) == CS_DATA_UNDERRUN) &&
		    (CMD_RESID_LEN(pscsi_cmd))) {

			transfer_len = pext->ResponseLen -
			    CMD_RESID_LEN(pscsi_cmd);

			pext->ResponseLen = transfer_len;
		} else {
			transfer_len = pext->ResponseLen;
		}

		usr_temp   = (uint8_t *)pext->ResponseAdr;
		kernel_tmp = (uint8_t *)ha->ioctl_mem;
		ret = copy_to_user(usr_temp, kernel_tmp, transfer_len);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk(
			    "%s(%ld): inst=%ld ERROR copy rsp buf\n",
			    __func__, ha->host_no, ha->instance);)
			goto scsi_passthru_done;
		}
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

scsi_passthru_done:

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}

/*
 * qla2x00_sc_fc_scsi_passthru
 *	Handles EXT_SC_SEND_FC_SCSI_PASSTHRU subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_sc_fc_scsi_passthru(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int			ret = 0;
	fc_lun_t		temp_fclun;
	fc_lun_t		*fclun = NULL;
	fc_port_t		*fcport;
	os_lun_t		*lq;
	os_tgt_t		*tq;
	srb_t			*sp = NULL;
	uint8_t			*usr_temp, *kernel_tmp;
	uint32_t		i;

#if defined(QL_DEBUG_LEVEL_9)
	uint32_t		b, t, l;
#endif
	uint32_t		transfer_len;
	uint8_t			scsi_direction;

	EXT_FC_SCSI_PASSTHRU	*pfc_scsi_pass;

	Scsi_Cmnd		*pfc_scsi_cmd;
	Scsi_Device		*pfc_scsi_device;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if ((sp = qla2x00_get_new_sp(ha)) == NULL) {

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc sp.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_NO_MEMORY;
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pfc_scsi_pass,
	    sizeof(EXT_FC_SCSI_PASSTHRU))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_FC_SCSI_PASSTHRU));)
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pfc_scsi_cmd,
	    sizeof(Scsi_Cmnd))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Cmnd));)
		goto fc_scsi_passthru_done;
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pfc_scsi_device,
	    sizeof(Scsi_Device))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Device));)
		goto fc_scsi_passthru_done;
	}

	/* clear ioctl_mem to be used */
	memset(ha->ioctl_mem, 0, ha->ioctl_mem_size);

	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    sizeof(EXT_FC_SCSI_PASSTHRU));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR verify READ req buf.\n",
		    __func__, ha->host_no, ha->instance);)
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto fc_scsi_passthru_done;
	}

	if (pext->ResponseLen > ha->ioctl_mem_size) {
		if (qla2x00_get_new_ioctl_dma_mem(ha, pext->ResponseLen) !=
		    QL_STATUS_SUCCESS) {

			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc "
			    "requested DMA buffer size %x.\n",
			    __func__, ha->host_no, ha->instance,
			    pext->ResponseLen);)

			pext->Status = EXT_STATUS_NO_MEMORY;
			atomic_set(&sp->ref_count, 0);
			add_to_free_queue (ha, sp);
			goto fc_scsi_passthru_done;
		}
	}

	/* Copy request buffer */
	usr_temp   = (uint8_t *)pext->RequestAdr;
	kernel_tmp = (uint8_t *)pfc_scsi_pass;
	ret = copy_from_user(kernel_tmp, usr_temp,
	    sizeof(EXT_FC_SCSI_PASSTHRU));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR copy req buf ret=%d\n",
		    __func__, ha->host_no, ha->instance, ret);)
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto fc_scsi_passthru_done;
	}

	if (pfc_scsi_pass->FCScsiAddr.DestType != EXT_DEF_DESTTYPE_WWPN) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR -wrong Dest type. \n",
		    __func__, ha->host_no, ha->instance);)
		ret = EXT_STATUS_ERR;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto fc_scsi_passthru_done;
	}

	fclun = NULL;
	for (fcport = ha->fcport; (fcport); fcport = fcport->next) {
		if (memcmp(fcport->port_name,
		    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN, 8) != 0) {
			continue;

		}

		for (fclun = fcport->fclun; fclun; fclun = fclun->next) {
			if (fclun->lun == pfc_scsi_pass->FCScsiAddr.Lun) {
				/* Found the right LUN */
				break;
			}
		}
		break;
	}

	if (fcport == NULL) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld FC AddrFormat - DID NOT "
		    "FIND Port matching WWPN.\n",
		    __func__, ha->host_no, ha->instance);)
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto fc_scsi_passthru_done;
	}

	/* v5.21b9 - use a temporary fclun */
	if (fclun == NULL) {
		fclun = &temp_fclun;
		fclun->fcport = fcport;
		fclun->lun = pfc_scsi_pass->FCScsiAddr.Lun;
		fclun->flags = 0;
		fclun->next = NULL;
	}

	/* set target coordinates */
	pfc_scsi_cmd->target = 0xff; /* not used. just put something there. */
	pfc_scsi_cmd->lun    = pfc_scsi_pass->FCScsiAddr.Lun;

	DEBUG9(printk("%s(%ld): inst=%ld cmd for loopid=%04x L=%04x "
	    "WWPN=%02x%02x%02x%02x%02x%02x%02x%02x.\n",
	    __func__, ha->host_no, ha->instance, fclun->fcport->loop_id,
	    pfc_scsi_cmd->lun,
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[0],
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[1],
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[2],
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[3],
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[4],
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[5],
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[6],
	    pfc_scsi_pass->FCScsiAddr.DestAddr.WWPN[7]);)

	if (pfc_scsi_pass->CdbLength == 6) {
		sp->cmd_length = 6;
		pfc_scsi_cmd->cmd_len = 6;

	} else if (pfc_scsi_pass->CdbLength == 0x0A) {
		sp->cmd_length = 0x0A;
		pfc_scsi_cmd->cmd_len = 0x0A;

	} else if (pfc_scsi_pass->CdbLength == 0x0C) {
		sp->cmd_length = 0x0C;
		pfc_scsi_cmd->cmd_len = 0x0C;

	} else if (pfc_scsi_pass->CdbLength == 0x10) {
		sp->cmd_length = 0x10;
		pfc_scsi_cmd->cmd_len = 0x10;

		/* Does the kernel support 16byte CDBs? */
		if (sp->cmd_length > MAX_COMMAND_SIZE) {
			/* No, use workaround method */
			pfc_scsi_cmd->cmd_len = 0x0C;

			sp->more_cdb[0] = pfc_scsi_pass->Cdb[12];
			sp->more_cdb[1] = pfc_scsi_pass->Cdb[13];
			sp->more_cdb[2] = pfc_scsi_pass->Cdb[14];
			sp->more_cdb[3] = pfc_scsi_pass->Cdb[15];
		}
	} else {
		printk(KERN_WARNING
		    "qla2x00_ioctl: FC_SCSI_PASSTHRU Unknown Cdb Length=%x.\n",
		    pfc_scsi_pass->CdbLength);
		pext->Status = EXT_STATUS_INVALID_PARAM;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto fc_scsi_passthru_done;
	}

	memcpy(pfc_scsi_cmd->data_cmnd, pfc_scsi_pass->Cdb,
	    pfc_scsi_cmd->cmd_len);
	memcpy(pfc_scsi_cmd->cmnd, pfc_scsi_pass->Cdb,
	    pfc_scsi_cmd->cmd_len);

	DEBUG9(printk("%s Dump of cdb buffer:\n", __func__);)
	DEBUG9(qla2x00_dump_buffer((uint8_t *)&pfc_scsi_cmd->data_cmnd[0], 16);)

	pfc_scsi_cmd->host    = ha->host;
	sp->ha                = ha;
	sp->cmd               = pfc_scsi_cmd;
	sp->flags             = SRB_IOCTL;

	/* set local fc_scsi_cmd's sp pointer to sp */
	CMD_SP(pfc_scsi_cmd)  = (void *) sp;

	/* mark this as a special delivery and collection command */
	pfc_scsi_cmd->flags     = 0;
	pfc_scsi_cmd->scsi_done = qla2x00_scsi_pt_done;

	pfc_scsi_cmd->device               = pfc_scsi_device;
	pfc_scsi_cmd->device->tagged_queue = 0;
	pfc_scsi_cmd->use_sg               = 0; /* no ScatterGather */
	pfc_scsi_cmd->request_bufflen      = pext->ResponseLen;
	pfc_scsi_cmd->request_buffer       = ha->ioctl_mem;
	pfc_scsi_cmd->timeout_per_command  = QLA_PT_CMD_TOV * HZ;

	if (pfc_scsi_pass->Direction == EXT_DEF_SCSI_PASSTHRU_DATA_OUT) {
		/* sending user data from pext->ResponseAdr to device */
		ret = verify_area(VERIFY_READ, (void *)pext->ResponseAdr,
		    pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify read "
			    "ResponseAdr.\n",
			    __func__, ha->host_no, ha->instance);)
			atomic_set(&sp->ref_count, 0);
			add_to_free_queue (ha, sp);

			goto fc_scsi_passthru_done;
		}

		pfc_scsi_cmd->sc_data_direction = SCSI_DATA_WRITE;
		usr_temp   = (uint8_t *)pext->ResponseAdr;
		kernel_tmp = (uint8_t *)ha->ioctl_mem;
		ret = copy_from_user(kernel_tmp, usr_temp, pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy "
			    "failed(%d) on rsp buf.\n",
			    __func__, ha->host_no, ha->instance, ret);)
			atomic_set(&sp->ref_count, 0);
			add_to_free_queue (ha, sp);

			goto fc_scsi_passthru_done;
		}
	} else {
		pfc_scsi_cmd->sc_data_direction = SCSI_DATA_READ;
	}

	tq = ha->ioctl->ioctl_tq;
	lq = ha->ioctl->ioctl_lq;

	if (fclun && tq && lq ) {
		tq->olun[fclun->lun] = lq;
		tq->ha = ha;

		lq->fclun = fclun;
		fcport = fclun->fcport;

		sp->lun_queue = lq;
		sp->tgt_queue = tq;
		sp->fclun = fclun;
	}

	DEBUG9({
		b = SCSI_BUS_32(pfc_scsi_cmd);
		t = SCSI_TCN_32(pfc_scsi_cmd);
		l = SCSI_LUN_32(pfc_scsi_cmd);
	})
	DEBUG9(printk("%s(%ld): ha instance=%ld tq=%p lq=%p "
	    "fclun=%p.\n",
	    __func__, ha->host_no, ha->instance, tq, lq, fclun);)
	DEBUG9(printk("\tCDB=%02x %02x %02x %02x; b=%x t=%x l=%x.\n",
	    pfc_scsi_cmd->cmnd[0], pfc_scsi_cmd->cmnd[1], pfc_scsi_cmd->cmnd[2],
	    pfc_scsi_cmd->cmnd[3], b, t, l);)

	/*
	 * Check the status of the port
	 */
	if (qla2x00_check_port_status(ha, fcport) != QL_STATUS_SUCCESS) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto fc_scsi_passthru_done;
	}

	/* set flag to indicate IOCTL SCSI PassThru in progress */
	ha->ioctl->SCSIPT_InProgress = 1;
	ha->ioctl->ioctl_tov = (int)QLA_PT_CMD_DRV_TOV;

	/* prepare for receiving completion. */
	qla2x00_ioctl_sem_init(ha);
	CMD_COMPL_STATUS(pfc_scsi_cmd) = (int) IOCTL_INVALID_STATUS;

	/* send command to adapter */
	DEBUG9(printk("%s(%ld): inst=%ld sending command.\n",
	    __func__, ha->host_no, ha->instance);)

	add_to_pending_queue(ha, sp);

	qla2x00_next(ha);

	ha->ioctl->cmpl_timer.expires = jiffies + ha->ioctl->ioctl_tov * HZ;
	add_timer(&ha->ioctl->cmpl_timer);

	down(&ha->ioctl->cmpl_sem);

	del_timer(&ha->ioctl->cmpl_timer);

	if (ha->ioctl->SCSIPT_InProgress == 1) {

		printk(KERN_WARNING
		    "qla2x00: scsi%ld ERROR passthru command timeout.\n",
		    ha->host_no);

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto fc_scsi_passthru_done;
	}

	if (CMD_COMPL_STATUS(pfc_scsi_cmd) == (int)IOCTL_INVALID_STATUS) {

		DEBUG9(printk("%s(%ld): inst=%ld ERROR. cmd not completed.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_ERR;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto fc_scsi_passthru_done;
	}

	switch (CMD_COMPL_STATUS(pfc_scsi_cmd)) {
	case CS_INCOMPLETE:
	case CS_ABORTED:
	case CS_PORT_UNAVAILABLE:
	case CS_PORT_LOGGED_OUT:
	case CS_PORT_CONFIG_CHG:
	case CS_PORT_BUSY:
		DEBUG10(printk("%s(%ld): inst=%ld cs err = %x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pfc_scsi_cmd));)
		pext->Status = EXT_STATUS_BUSY;
		break;
	}

	if ((CMD_COMPL_STATUS(pfc_scsi_cmd) == CS_DATA_UNDERRUN) ||
	    (CMD_SCSI_STATUS(pfc_scsi_cmd) != 0))  {

		/* have done the post function */
		pext->Status       = EXT_STATUS_SCSI_STATUS;
		/* The SDMAPI is only concerned with the low-order byte */
		pext->DetailStatus = CMD_SCSI_STATUS(pfc_scsi_cmd) & 0xff;

		DEBUG9_10(printk("%s(%ld): inst=%ld data underrun or scsi err. "
		    "host status =0x%x, scsi status = 0x%x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pfc_scsi_cmd),
		    CMD_SCSI_STATUS(pfc_scsi_cmd));)

	} else if (CMD_COMPL_STATUS(pfc_scsi_cmd) != 0) {
		DEBUG9_10(printk("%s(%ld): inst=%ld cs err=%x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pfc_scsi_cmd));)
		pext->Status = EXT_STATUS_ERR;

		goto fc_scsi_passthru_done;
	}

	/* Process completed command */
	DEBUG9(printk("%s(%ld): inst=%ld done. host status=0x%x, "
	    "scsi status=0x%x.\n",
	    __func__, ha->host_no, ha->instance, CMD_COMPL_STATUS(pfc_scsi_cmd),
	    CMD_SCSI_STATUS(pfc_scsi_cmd));)

	/* copy up structure to make sense data available to user */
	pfc_scsi_pass->SenseLength = CMD_ACTUAL_SNSLEN(pfc_scsi_cmd);
	if (CMD_ACTUAL_SNSLEN(pfc_scsi_cmd)) {
		DEBUG9_10(printk("%s(%ld): inst=%ld sense[0]=%x sense[2]=%x.\n",
		    __func__, ha->host_no, ha->instance,
		    pfc_scsi_cmd->sense_buffer[0],
		    pfc_scsi_cmd->sense_buffer[2]);)

		for (i = 0; i < CMD_ACTUAL_SNSLEN(pfc_scsi_cmd); i++) {
			pfc_scsi_pass->SenseData[i] =
			pfc_scsi_cmd->sense_buffer[i];
		}

		ret = verify_area(VERIFY_WRITE, (void *)pext->RequestAdr,
		    sizeof(EXT_FC_SCSI_PASSTHRU));
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify WRITE "
			    "RequestAdr.\n",
			    __func__, ha->host_no, ha->instance);)
			goto fc_scsi_passthru_done;
		}

		usr_temp = (uint8_t *)pext->RequestAdr;
		kernel_tmp = (uint8_t *)pfc_scsi_pass;
		ret = copy_to_user(usr_temp, kernel_tmp,
		    sizeof(EXT_FC_SCSI_PASSTHRU));
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy sense "
			    "buffer.\n",
			    __func__, ha->host_no, ha->instance);)
			goto fc_scsi_passthru_done;
		}
	}

	scsi_direction = pfc_scsi_pass->Direction;

	if (scsi_direction == EXT_DEF_SCSI_PASSTHRU_DATA_IN) {

		DEBUG9(printk("%s(%ld): inst=%ld copying data.\n",
		    __func__, ha->host_no, ha->instance);)

		/* getting device data and putting in pext->ResponseAdr */
		ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
		    pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;

			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify write "
			    "ResponseAdr.\n",
			    __func__, ha->host_no, ha->instance);)

			goto fc_scsi_passthru_done;
		}

		/* now copy up the READ data to user */
		if ((CMD_COMPL_STATUS(pfc_scsi_cmd) == CS_DATA_UNDERRUN) &&
		    (CMD_RESID_LEN(pfc_scsi_cmd))) {

			transfer_len = pext->ResponseLen -
			    CMD_RESID_LEN(pfc_scsi_cmd);

			pext->ResponseLen = transfer_len;
		} else {
			transfer_len = pext->ResponseLen;
		}

		usr_temp = (uint8_t *)pext->ResponseAdr;
		kernel_tmp = (uint8_t *)ha->ioctl_mem;
		ret = copy_to_user(usr_temp, kernel_tmp, transfer_len);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk(
			    "%s(%ld): inst=%ld ERROR copy rsp buf\n",
			    __func__, ha->host_no, ha->instance);)
			goto fc_scsi_passthru_done;
		}
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

fc_scsi_passthru_done:

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}

/*
 * qla2x00_sc_scsi3_passthru
 *	Handles EXT_SC_SCSI3_PASSTHRU subcommand.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = EXT_IOCTL structure pointer.
 *	mode = not used.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_sc_scsi3_passthru(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
#define MAX_SCSI3_CDB_LEN	16

	int			ret = 0;
	fc_lun_t		temp_fclun;
	fc_lun_t		*fclun = NULL;
	fc_port_t		*fcport;
	os_lun_t		*lq;
	os_tgt_t		*tq;
	srb_t			*sp = NULL;
	uint8_t			*usr_temp, *kernel_tmp;
	uint32_t		transfer_len;
	uint32_t		i, b, t;
	uint32_t		scsi_direction;

	EXT_FC_SCSI_PASSTHRU	*pfc_scsi3_pass;

	Scsi_Cmnd		*pscsi3_cmd;
	Scsi_Device		*pscsi3_device;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if ((sp = qla2x00_get_new_sp(ha)) == NULL) {

		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc sp.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_NO_MEMORY;
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pfc_scsi3_pass,
	    sizeof(EXT_FC_SCSI_PASSTHRU))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_FC_SCSI_PASSTHRU));)
		return (ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pscsi3_cmd,
	    sizeof(Scsi_Cmnd))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Cmnd));)
		goto scsi3_passthru_done;
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&pscsi3_device,
	    sizeof(Scsi_Device))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(Scsi_Device));)
		goto scsi3_passthru_done;
	}

	/* clear ioctl_mem to be used */
	memset(ha->ioctl_mem, 0, ha->ioctl_mem_size);

	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    sizeof(EXT_FC_SCSI_PASSTHRU));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify READ "
		    "req buf.\n", __func__, ha->host_no, ha->instance);)
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto scsi3_passthru_done;
	}

	if (pext->ResponseLen > ha->ioctl_mem_size) {
		if (qla2x00_get_new_ioctl_dma_mem(ha, pext->ResponseLen) !=
		    QL_STATUS_SUCCESS) {

			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot "
			    "alloc requested DMA buffer size=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    pext->ResponseLen);)

			pext->Status = EXT_STATUS_NO_MEMORY;
			atomic_set(&sp->ref_count, 0);
			add_to_free_queue (ha, sp);
			goto scsi3_passthru_done;
		}
	}

	/* Copy request buffer */
	usr_temp   = (uint8_t *)pext->RequestAdr;
	kernel_tmp = (uint8_t *)pfc_scsi3_pass;
	ret = copy_from_user(kernel_tmp, usr_temp,
	    sizeof(EXT_FC_SCSI_PASSTHRU));
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR copy req buf ret=%d\n",
		    __func__, ha->host_no, ha->instance, ret);)
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto scsi3_passthru_done;
	}

	if (pfc_scsi3_pass->FCScsiAddr.DestType != EXT_DEF_DESTTYPE_WWPN) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR - wrong Dest type.\n",
		    __func__, ha->host_no, ha->instance);)
		ret = EXT_STATUS_ERR;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto scsi3_passthru_done;
	}

	/*
	 * For this ioctl command we always assume all 16 bytes are
	 * initialized.
	 */
	if (pfc_scsi3_pass->CdbLength != MAX_SCSI3_CDB_LEN) {
		pext->Status = EXT_STATUS_INVALID_PARAM;
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR -wrong Cdb Len %d.\n",
		    __func__, ha->host_no, ha->instance,
		    pfc_scsi3_pass->CdbLength);)
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto scsi3_passthru_done;
	}

	for (fcport = ha->fcport; (fcport); fcport = fcport->next) {
		if (memcmp(fcport->port_name,
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN, 8) == 0) {
			break;
		}
	}
	if (fcport == NULL) {
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;

		DEBUG9_10(printk("%s(%ld): inst=%ld DID NOT FIND Port for WWPN "
		    "%02x%02x%02x%02x%02x%02x%02x%02x.\n",
		    __func__, ha->host_no, ha->instance,
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[0],
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[1],
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[2],
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[3],
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[4],
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[5],
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[6],
		    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[7]);)

		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);

		goto scsi3_passthru_done;
	}

	/* Use a temporary fclun to send out the command. */
	fclun = &temp_fclun;
	fclun->fcport = fcport;
	fclun->lun = pfc_scsi3_pass->FCScsiAddr.Lun;
	fclun->flags = 0;
	fclun->next = NULL;

	/* set target coordinates */
	pscsi3_cmd->target = 0xff;  /* not used. just put something there. */
	pscsi3_cmd->lun = pfc_scsi3_pass->FCScsiAddr.Lun;

	DEBUG9(printk("%s(%ld): inst=%ld cmd for loopid=%04x L=%04x "
	    "WWPN=%02x%02x%02x%02x%02x%02x%02x%02x.\n",
	    __func__, ha->host_no, ha->instance,
	    fclun->fcport->loop_id, pscsi3_cmd->lun,
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[0],
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[1],
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[2],
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[3],
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[4],
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[5],
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[6],
	    pfc_scsi3_pass->FCScsiAddr.DestAddr.WWPN[7]);)

	sp->cmd_length = MAX_SCSI3_CDB_LEN;
	pscsi3_cmd->cmd_len = MAX_SCSI3_CDB_LEN;

	/* Does the kernel support 16byte CDBs? */
	if (sp->cmd_length > MAX_COMMAND_SIZE) {
		/* No, use workaround method */
		pscsi3_cmd->cmd_len = 0x0C;

		sp->more_cdb[0] = pfc_scsi3_pass->Cdb[12];
		sp->more_cdb[1] = pfc_scsi3_pass->Cdb[13];
		sp->more_cdb[2] = pfc_scsi3_pass->Cdb[14];
		sp->more_cdb[3] = pfc_scsi3_pass->Cdb[15];
	}

	memcpy(pscsi3_cmd->data_cmnd, pfc_scsi3_pass->Cdb, pscsi3_cmd->cmd_len);
	memcpy(pscsi3_cmd->cmnd, pfc_scsi3_pass->Cdb, pscsi3_cmd->cmd_len);

	DEBUG9(printk("%s(%ld): inst=%ld cdb buffer dump:\n",
	    __func__, ha->host_no, ha->instance);)
	DEBUG9(qla2x00_dump_buffer((uint8_t *)&pscsi3_cmd->data_cmnd[0], 16);)

	pscsi3_cmd->host      = ha->host;
	sp->ha                = ha;
	sp->cmd               = pscsi3_cmd;
	sp->flags             = SRB_IOCTL;

	/* set local scsi3_cmd's sp pointer to sp */
	CMD_SP(pscsi3_cmd)    = (void *) sp;

	/* mark this as a special delivery and collection command */
	pscsi3_cmd->flags     = 0;
	pscsi3_cmd->scsi_done = qla2x00_scsi_pt_done;

	pscsi3_cmd->device               = pscsi3_device;
	pscsi3_cmd->device->tagged_queue = 0;
	pscsi3_cmd->use_sg               = 0; /* no ScatterGather */
	pscsi3_cmd->request_bufflen      = pext->ResponseLen;
	pscsi3_cmd->request_buffer       = ha->ioctl_mem;
	pscsi3_cmd->timeout_per_command  = QLA_PT_CMD_TOV * HZ;

	if (pfc_scsi3_pass->Direction == EXT_DEF_SCSI_PASSTHRU_DATA_OUT) {
		/* sending user data from pext->ResponseAdr to device */
		ret = verify_area(VERIFY_READ, (void *)pext->ResponseAdr,
		    pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify read "
			    "ResponseAdr.\n",
			    __func__, ha->host_no, ha->instance);)
			atomic_set(&sp->ref_count, 0);
			add_to_free_queue (ha, sp);
			goto scsi3_passthru_done;
		}

		pscsi3_cmd->sc_data_direction = SCSI_DATA_WRITE;
		usr_temp   = (uint8_t *)pext->ResponseAdr;
		kernel_tmp = (uint8_t *)ha->ioctl_mem;
		ret = copy_from_user(kernel_tmp, usr_temp, pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buf "
			    "ret=%d.\n",
			    __func__, ha->host_no, ha->instance, ret);)
			atomic_set(&sp->ref_count, 0);
			add_to_free_queue (ha, sp);
			goto scsi3_passthru_done;
		}

	} else {
		pscsi3_cmd->sc_data_direction = SCSI_DATA_READ;
	}

	/* Use temporary LU and TGT queue */
	tq = ha->ioctl->ioctl_tq;
	lq = ha->ioctl->ioctl_lq;

	b = SCSI_BUS_32(pscsi3_cmd);
	t = SCSI_TCN_32(pscsi3_cmd);

	DEBUG9(printk("%s(%ld): ha instance=%ld tq=%p lq=%p fclun=%p.\n",
	    __func__, ha->host_no, ha->instance,tq,lq,fclun); )
	DEBUG9(printk("%s(%ld): CDB=%02x %02x %02x %02x; b=%x t=%x fclun=%x\n",
	    __func__, ha->host_no, pscsi3_cmd->cmnd[0], pscsi3_cmd->cmnd[1],
	    pscsi3_cmd->cmnd[2], pscsi3_cmd->cmnd[3], b, t,
	    SCSI_LUN_32(pscsi3_cmd));)

	if (tq && lq) {
		/*
		 * For now just save lq using the lower LUN byte value,
		 * even though this may not be the actual LUN number.
		 * Since we're only sending out passthru cmd one at a time,
		 * and only passthru is using FCP LUN format now, no need
		 * to change rest of driver just to decode the LUN.
		 */
		tq->olun[fclun->lun & 0xff] = lq;

		tq->ha = ha;
		lq->fclun = fclun;
		fcport = fclun->fcport;

		sp->lun_queue = lq;
		sp->tgt_queue = tq;
		sp->fclun = fclun;
	} else {
		lq = NULL;
		fcport = NULL;
	}

	/*
	 * Check the status of the port
	 */
	if (qla2x00_check_port_status(ha, fcport) != QL_STATUS_SUCCESS) {

		DEBUG9_10(printk("%s(%ld): inst=%ld port missing or loop down. "
		    "fcport=%p timer=%d state=%d dpc=%lx.\n",
		    __func__, ha->host_no, ha->instance, fcport,
		    atomic_read(&ha->loop_down_timer), ha->loop_state,
		    ha->dpc_flags);)

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto scsi3_passthru_done;
	}

	/* set flag to indicate IOCTL SCSI PassThru in progress */
	ha->ioctl->SCSIPT_InProgress = 1;
	ha->ioctl->ioctl_tov = (int)QLA_PT_CMD_DRV_TOV;

	/* prepare for receiving completion. */
	qla2x00_ioctl_sem_init(ha);
	CMD_COMPL_STATUS(pscsi3_cmd) = (int) IOCTL_INVALID_STATUS;

	/* send command to adapter */

	/*add_to_cmd_queue(ha, lq, sp);*/
	add_to_pending_queue(ha, sp);

	/*qla2x00_next(ha, tq, lq);*/
	qla2x00_next(ha);

	ha->ioctl->cmpl_timer.expires = jiffies + ha->ioctl->ioctl_tov * HZ;
	add_timer(&ha->ioctl->cmpl_timer);

	down(&ha->ioctl->cmpl_sem);

	del_timer(&ha->ioctl->cmpl_timer);

	if (ha->ioctl->SCSIPT_InProgress == 1) {

		printk(KERN_WARNING
		    "qla2x00: inst=%ld scsi%ld ERROR PT command timeout.\n",
		    ha->host_no, ha->instance);

		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto scsi3_passthru_done;

	}
	if (CMD_COMPL_STATUS(pscsi3_cmd) == (int)IOCTL_INVALID_STATUS) {

		DEBUG9(printk("%s(%ld): inst=%ld ERROR - cmd not completed.\n",
		    __func__, ha->host_no, ha->instance);)

		pext->Status = EXT_STATUS_ERR;
		atomic_set(&sp->ref_count, 0);
		add_to_free_queue (ha, sp);
		goto scsi3_passthru_done;
	}

	switch (CMD_COMPL_STATUS(pscsi3_cmd)) {
	case CS_INCOMPLETE:
	case CS_ABORTED:
	case CS_PORT_UNAVAILABLE:
	case CS_PORT_LOGGED_OUT:
	case CS_PORT_CONFIG_CHG:
	case CS_PORT_BUSY:
		DEBUG10(printk("%s(%ld): inst=%ld cs err = %x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pscsi3_cmd));)
		pext->Status = EXT_STATUS_BUSY;
		break;
	}

	if ((CMD_COMPL_STATUS(pscsi3_cmd) == CS_DATA_UNDERRUN) ||
	    (CMD_SCSI_STATUS(pscsi3_cmd) != 0)) {

		/* have done the post function */
		pext->Status       = EXT_STATUS_SCSI_STATUS;
		pext->DetailStatus = CMD_SCSI_STATUS(pscsi3_cmd) & 0xff;

		DEBUG9_10(printk("%s(%ld): inst=%ld data underrun or scsi err. "
		    "host status =0x%x, scsi status = 0x%x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pscsi3_cmd), CMD_SCSI_STATUS(pscsi3_cmd));)

	} else if (CMD_COMPL_STATUS(pscsi3_cmd) != 0) {
		DEBUG9_10(printk("%s(%ld): inst=%ld cs err = %x.\n",
		    __func__, ha->host_no, ha->instance,
		    CMD_COMPL_STATUS(pscsi3_cmd));)
		pext->Status = EXT_STATUS_ERR;
		goto scsi3_passthru_done;
	}

	/* Process completed command */
	DEBUG9(printk("%s(%ld): inst=%ld done. host status=0x%x, "
	    "scsi status=0x%x.\n",
	    __func__, ha->host_no, ha->instance, CMD_COMPL_STATUS(pscsi3_cmd),
	    CMD_SCSI_STATUS(pscsi3_cmd));)

	/* copy up structure to make sense data available to user */
	pfc_scsi3_pass->SenseLength = CMD_ACTUAL_SNSLEN(pscsi3_cmd);
	if (CMD_ACTUAL_SNSLEN(pscsi3_cmd)) {
		DEBUG9_10(printk("%s(%ld): inst=%ld sense[0]=%x sense[2]=%x.\n",
		    __func__, ha->host_no, ha->instance,
		    pscsi3_cmd->sense_buffer[0],
		    pscsi3_cmd->sense_buffer[2]);)

		for (i = 0; i < CMD_ACTUAL_SNSLEN(pscsi3_cmd); i++) {
			pfc_scsi3_pass->SenseData[i] =
			    pscsi3_cmd->sense_buffer[i];
		}

		ret = verify_area(VERIFY_WRITE, (void *)pext->RequestAdr,
		    sizeof(EXT_FC_SCSI_PASSTHRU));
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify WRITE "
			    "RequestAdr.\n",
			    __func__, ha->host_no, ha->instance);)
			goto scsi3_passthru_done;
		}

		usr_temp = (uint8_t *)pext->RequestAdr;
		kernel_tmp = (uint8_t *)pfc_scsi3_pass;
		ret = copy_to_user(usr_temp, kernel_tmp,
		    sizeof(EXT_FC_SCSI_PASSTHRU));
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy sense "
			    "buffer.\n",
			    __func__, ha->host_no, ha->instance);)
			goto scsi3_passthru_done;
		}
	}

	scsi_direction = pfc_scsi3_pass->Direction;

	if (scsi_direction == EXT_DEF_SCSI_PASSTHRU_DATA_IN) {

		DEBUG9(printk("%s(%ld): inst=%ld copying data.\n",
		    __func__, ha->host_no, ha->instance);)

		/* getting device data and putting in pext->ResponseAdr */
		ret = verify_area(VERIFY_WRITE, (void *)pext->ResponseAdr,
		    pext->ResponseLen);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;

			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR verify write "
			    "ResponseAdr.\n",
			    __func__, ha->host_no, ha->instance);)

			goto scsi3_passthru_done;
		}

		/* now copy up the READ data to user */
		if ((CMD_COMPL_STATUS(pscsi3_cmd) == CS_DATA_UNDERRUN) &&
		    (CMD_RESID_LEN(pscsi3_cmd))) {

			transfer_len = pext->ResponseLen -
			    CMD_RESID_LEN(pscsi3_cmd);

			pext->ResponseLen = transfer_len;
		} else {
			transfer_len = pext->ResponseLen;
		}

		usr_temp = (uint8_t *)pext->ResponseAdr;
		kernel_tmp = (uint8_t *)ha->ioctl_mem;
		ret = copy_to_user(usr_temp, kernel_tmp, transfer_len);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk(
			    "%s(%ld): inst=%ld ERROR copy rsp buf\n",
			    __func__, ha->host_no, ha->instance);)
			goto scsi3_passthru_done;
		}
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

scsi3_passthru_done:

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}

/*
 * qla2x00_send_els_rnid
 *	IOCTL to send extended link service RNID command to a target.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = User space CT arguments pointer.
 *	mode = flags.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_send_els_rnid(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
#define TGT_DEV 	 1
#define HOST_DEV 	 2

	EXT_RNID_REQ	*tmp_rnid;
	int		ret = 0;
	uint8_t 	dev_found = 0;
	uint16_t	dev_loop_id = 0;
	uint16_t	mb[MAILBOX_REGISTER_COUNT];
	uint32_t	copy_len;
	fc_port_t	*fcport;
	int		found;
	struct list_head *fcil;
	fc_initiator_t	*fcinitiator;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (ha->ioctl_mem_size < SEND_RNID_RSP_SIZE) {
		if (qla2x00_get_new_ioctl_dma_mem(ha,
		    SEND_RNID_RSP_SIZE) != QL_STATUS_SUCCESS) {

			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR cannot alloc "
			    "DMA buffer. size=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    SEND_RNID_RSP_SIZE);)

			pext->Status = EXT_STATUS_NO_MEMORY;
			return (ret);
		}
	}

	if (pext->RequestLen != sizeof(EXT_RNID_REQ)) {
		/* parameter error */
		DEBUG9_10(printk("%s(%ld): inst=%ld invalid req length %d.\n",
		    __func__, ha->host_no, ha->instance, pext->RequestLen);)
		pext->Status = EXT_STATUS_INVALID_PARAM;
		return (ret);
	}

	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    pext->RequestLen);

	if (ret != 0) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld req buf verify READ FAILED\n",
		    __func__, ha->host_no, ha->instance);)
		return (ret);
	}

	DEBUG9(printk("%s(%ld): inst=%ld req buf verified. Copying req data.\n",
	    __func__, ha->host_no, ha->instance);)

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&tmp_rnid,
	    sizeof(EXT_RNID_REQ))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_RNID_REQ));)
		return (ret);
	}

	ret = copy_from_user(tmp_rnid, pext->RequestAdr, pext->RequestLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR copy req buf ret=%d\n",
		    __func__, ha->host_no, ha->instance, ret);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* Find loop ID of the device */
	fcinitiator = NULL;
	switch (tmp_rnid->Addr.Type) {
	case EXT_DEF_TYPE_WWNN:

		DEBUG9(printk("%s(%ld): inst=%ld got node name.\n",
		    __func__, ha->host_no, ha->instance);)

		for (fcport = ha->fcport; (fcport); fcport = fcport->next) {
			/* if removed or missing */
			if (atomic_read(&fcport->state) == FC_ONLINE &&
			    memcmp((void *)tmp_rnid->Addr.FcAddr.WWNN,
			    (void *)fcport->node_name,
			    EXT_DEF_WWN_NAME_SIZE) == 0) {
				break;
			}
		}
		if (fcport != NULL) {
			DEBUG9(printk("%s(%ld): inst=%ld found tgt dev; "
			    "loop_id=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    fcport->loop_id);)

			dev_found = TGT_DEV;
			dev_loop_id = fcport->loop_id;
			break;
		}

		found = 0;
		fcinitiator = NULL;
		list_for_each(fcil, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if (memcmp(tmp_rnid->Addr.FcAddr.WWNN,
				 fcinitiator->node_name,
				 EXT_DEF_WWN_NAME_SIZE) == 0 &&
				fcinitiator->d_id.b24 != 0) {

				found++;
				break;
			}
		}
		if (found) {
			DEBUG9(printk("%s(%ld): inst=%ld found host device; "
			    "loop_id=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    fcinitiator->loop_id);)

			dev_found = HOST_DEV;
			dev_loop_id = fcinitiator->loop_id;
			break;
		}

		break;

	case EXT_DEF_TYPE_WWPN:
		DEBUG9(printk("%s(%ld): inst=%ld got port name.\n",
		    __func__, ha->host_no, ha->instance);)

		for (fcport = ha->fcport; (fcport); fcport = fcport->next) {
			/* if removed or missing */
			if (atomic_read(&fcport->state) == FC_ONLINE &&
			    memcmp((void *)tmp_rnid->Addr.FcAddr.WWPN,
			    (void *)fcport->port_name,
			    EXT_DEF_WWN_NAME_SIZE) == 0) {
				break;
			}
		}
		if (fcport != NULL) {
			DEBUG9(printk("%s(%ld): inst=%ld found tgt dev; "
			    "loop_id=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    fcport->loop_id);)

			dev_found = TGT_DEV; /* target device */
			dev_loop_id = fcport->loop_id;
			break;
		}

		found = 0;
		fcinitiator = NULL;
		list_for_each(fcil, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if (memcmp(tmp_rnid->Addr.FcAddr.WWPN,
				 fcinitiator->port_name,
				 EXT_DEF_WWN_NAME_SIZE) == 0 &&
				fcinitiator->d_id.b24 != 0) {

				found++;
				break;
			}
		}
		if (found) {
			DEBUG9(printk("%s(%ld): inst=%ld found host device; "
			    "loop_id=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    fcinitiator->loop_id);)

			dev_found = HOST_DEV;
			dev_loop_id = fcinitiator->loop_id;
			break;
		}

		break;

	case EXT_DEF_TYPE_PORTID:
		DEBUG9(printk("%s(%ld): inst=%ld got port ID.\n",
		    __func__, ha->host_no, ha->instance);)

		/* PORTID bytes entered must already be big endian */
		for (fcport = ha->fcport; (fcport); fcport = fcport->next) {
			/* if removed or missing */
			if (atomic_read(&fcport->state) == FC_ONLINE &&
			    memcmp((void *)&tmp_rnid->Addr.FcAddr.Id[1],
			    (void *)(fcport->d_id.r.d_id),
			    EXT_DEF_PORTID_SIZE_ACTUAL) == 0) {
				break;
			}
		}
		if (fcport != NULL) {
			DEBUG9(printk("%s(%ld): inst=%ld found tgt dev; "
			    "loop_id=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    fcport->loop_id);)

			dev_found = TGT_DEV; /* target device */
			dev_loop_id = fcport->loop_id;
			break;
		}

		found = 0;
		fcinitiator = NULL;
		list_for_each(fcil, &ha->fcinitiators) {
			fcinitiator = list_entry(fcil, fc_initiator_t, list);

			if (memcmp(&tmp_rnid->Addr.FcAddr.Id[1],
				&fcinitiator->d_id,
				EXT_DEF_PORTID_SIZE_ACTUAL) == 0) {

				found++;
				break;
			}
		}
		if (found) {
			DEBUG9(printk("%s(%ld): inst=%ld found host device; "
			    "loop_id=%x.\n",
			    __func__, ha->host_no, ha->instance,
			    fcinitiator->loop_id);)

			dev_found = HOST_DEV;
			dev_loop_id = fcinitiator->loop_id;
			break;
		}

		break;
	default:
		/* parameter error */
		pext->Status = EXT_STATUS_INVALID_PARAM;
		DEBUG9_10(printk("%s(%ld): inst=%ld invalid addressing type.\n",
		    __func__, ha->host_no, ha->instance);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	if (!dev_found ||
	    (dev_found == TGT_DEV && dev_loop_id > LAST_SNS_LOOP_ID)) {
		/* No matching device or the target device is not
		 * configured; just return error.
		 */
		pext->Status = EXT_STATUS_DEV_NOT_FOUND;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld device not found. dev_found=%d "
		    "dev_loop_id=%x.\n",
		    __func__, ha->host_no, ha->instance, dev_found,
		    dev_loop_id);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* check on loop down */
	if (ha->loop_state != LOOP_READY || 
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	    ABORTS_ACTIVE || ha->dpc_active) {

		pext->Status = EXT_STATUS_BUSY;
		DEBUG9_10(printk("%s(%ld): inst=%ld loop not ready.\n",
		    __func__, ha->host_no, ha->instance);)

		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	/* Check whether we need to login first. */
	if (dev_found == HOST_DEV && dev_loop_id > LAST_SNS_LOOP_ID) {
		/*
		 * Search for a usable loop ID before try to login to it.
		 */
		if ((dev_loop_id &= ~PORT_LOST_ID) > LAST_SNS_LOOP_ID) {
			/* Just start searching from first possible ID. */
			dev_loop_id = ha->min_external_loopid;
		}
		for (;;) {
			if (ha->fabricid[dev_loop_id].in_use == TRUE) {
				dev_loop_id++;
			} else {
				ha->fabricid[dev_loop_id].in_use = TRUE;
				break;
			}
		}

		DEBUG9(printk("%s(%ld): inst=%ld try relogin to host dev; "
		    "dev_loop_id=%x.\n",
		    __func__, ha->host_no, ha->instance, dev_loop_id);)

		for (;;) {
			if (dev_loop_id > LAST_SNS_LOOP_ID) {
				/* error */
				DEBUG10(printk("%s(%ld): inst=%ld "
				    "no valid loop_id for login.\n",
				    __func__, ha->host_no, ha->instance);)

				break;
			}

			qla2x00_login_fabric(ha, 
			    dev_loop_id,
			    fcinitiator->d_id.b.domain,
			    fcinitiator->d_id.b.area,
			    fcinitiator->d_id.b.al_pa,
			    &mb[0], 0);

			if (mb[0] != MBS_CMD_CMP &&
			    mb[0] != MBS_PORT_ID_IN_USE &&
			    mb[0] != MBS_LOOP_ID_IN_USE) {

	 			DEBUG10(printk("%s(%ld): inst=%ld "
				    "ERROR login mb[0]=%x mb[1]=%x.\n",
				    __func__, ha->host_no, ha->instance,
				    mb[0], mb[1]);)
				break;
			}

			if (mb[0] == MBS_CMD_CMP) {
				DEBUG9(printk("%s(%ld): inst=%ld host login "
				    "success; loop_id=%x.\n",
				    __func__, ha->host_no, ha->instance,
				    dev_loop_id);)

				fcinitiator->loop_id = dev_loop_id;
				break;
			} else if (mb[0] == MBS_PORT_ID_IN_USE) {
				ha->fabricid[dev_loop_id].in_use = FALSE;
				dev_loop_id = mb[1];

				DEBUG9(printk("%s(%ld): inst=%ld port %06x "
				    "using loop id=0x%04x.\n",
				    __func__, ha->host_no, ha->instance,
				    fcinitiator->d_id.b24,
				    dev_loop_id);)

				if (dev_loop_id <= LAST_SNS_LOOP_ID)
					ha->fabricid[dev_loop_id].in_use = TRUE;
				else
					/* Error */
					break;

			} else if (mb[0] == MBS_LOOP_ID_IN_USE) {
				/* Search for another usable loop_id */
				dev_loop_id++;
				while (ha->fabricid[dev_loop_id].in_use) {
					if (dev_loop_id++ > LAST_SNS_LOOP_ID) {
						/* Error */
						break;
					}
				}

				if (dev_loop_id <= LAST_SNS_LOOP_ID) {
					DEBUG9(printk(
					    "%s(%ld): inst=%ld previous loop "
					    "id in use. Retry with 0x%04x.\n",
					    __func__, ha->host_no, ha->instance,
					    dev_loop_id);)

					ha->fabricid[dev_loop_id].in_use = TRUE;
				} else {
					/* Error */
					break;
				}
			}
		}

		if (mb[0] != MBS_CMD_CMP) {
			pext->Status = EXT_STATUS_ERR;
			DEBUG9_10(printk( "%s(%ld): inst=%ld login failed.\n",
			    __func__, ha->host_no, ha->instance);)

			qla2x00_free_ioctl_scrap_mem(ha);
			return (ret);
		}
	}

	/* Send command */
	DEBUG9(printk("%s(%ld): inst=%ld sending rnid cmd.\n",
	    __func__, ha->host_no, ha->instance);)

	ret = qla2x00_send_rnid_mbx(ha, dev_loop_id,
	    (uint8_t)tmp_rnid->DataFormat, ha->ioctl_mem_phys,
	    SEND_RNID_RSP_SIZE, &mb[0]);

	if (ret != QLA2X00_SUCCESS) {
		/* error */
		pext->Status = EXT_STATUS_ERR;

                DEBUG9_10(printk("%s(%ld): inst=%ld FAILED. rval = %x.\n",
                    __func__, ha->host_no, ha->instance, mb[0]);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	DEBUG9(printk("%s(%ld): inst=%ld rnid cmd sent ok.\n",
	    __func__, ha->host_no, ha->instance);)

	/* Copy the response */
	copy_len = (pext->ResponseLen > SEND_RNID_RSP_SIZE) ?
	    SEND_RNID_RSP_SIZE : pext->ResponseLen;

	ret = verify_area(VERIFY_WRITE, (void  *)pext->ResponseAdr,
	    copy_len);

	if (ret != 0) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld rsp buf verify WRITE error\n",
		    __func__, ha->host_no, ha->instance);)
	} else {
		ret = copy_to_user((uint8_t *)pext->ResponseAdr,
		    (uint8_t *)ha->ioctl_mem, copy_len);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk(
			    "%s(%ld): inst=%ld ERROR copy rsp buf\n",
			    __func__, ha->host_no, ha->instance);)
			qla2x00_free_ioctl_scrap_mem(ha);
			return (ret);
		}

		if (SEND_RNID_RSP_SIZE > pext->ResponseLen) {
			pext->Status = EXT_STATUS_DATA_OVERRUN;
			DEBUG9(printk("%s(%ld): inst=%ld data overrun. "
			    "exiting normally.\n",
			    __func__, ha->host_no, ha->instance);)
		} else {
			pext->Status = EXT_STATUS_OK;
			DEBUG9(printk("%s(%ld): inst=%ld exiting normally.\n",
			    __func__, ha->host_no, ha->instance);)
		}
		pext->ResponseLen = copy_len;
	}

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}

/*
 * qla2x00_get_rnid_params
 *	IOCTL to get RNID parameters of the adapter.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = User space CT arguments pointer.
 *	mode = flags.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
static int
qla2x00_get_rnid_params(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int		ret = 0;
	int		tmp_rval = 0;
	uint32_t	copy_len;
	uint16_t	mb[MAILBOX_REGISTER_COUNT];

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	/* check on loop down */
	if (ha->loop_state != LOOP_READY || 
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	    ABORTS_ACTIVE || ha->dpc_active) {

		pext->Status = EXT_STATUS_BUSY;
		DEBUG9_10(printk("%s(%ld): inst=%ld loop not ready.\n",
		    __func__, ha->host_no, ha->instance);)

		return (ret);
	}

	/* Send command */
	tmp_rval = qla2x00_get_rnid_params_mbx(ha, ha->ioctl_mem_phys,
	    sizeof(EXT_RNID_DATA), &mb[0]);

	if (tmp_rval != QLA2X00_SUCCESS) {
		/* error */
		pext->Status = EXT_STATUS_ERR;

		DEBUG9_10(printk("%s(%ld): inst=%ld cmd FAILED=%x.\n",
		    __func__, ha->host_no, ha->instance, mb[0]);)
		return (ret);
	}

	/* Copy the response */
	copy_len = (pext->ResponseLen > sizeof(EXT_RNID_DATA)) ?
	    (uint32_t)sizeof(EXT_RNID_DATA) : pext->ResponseLen;
	ret = verify_area(VERIFY_WRITE, (void  *)pext->ResponseAdr,
	    copy_len);

	if (ret != 0) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld verify WRITE rsp buf error\n",
		    __func__, ha->host_no, ha->instance);)
	} else {
		ret = copy_to_user((void *)pext->ResponseAdr,
		    (void *)ha->ioctl_mem, copy_len);
		if (ret) {
			pext->Status = EXT_STATUS_COPY_ERR;
			DEBUG9_10(printk("%s(%ld): inst=%ld ERROR copy rsp buf\n",
			    __func__, ha->host_no, ha->instance);)
			return (ret);
		}

		pext->ResponseLen = copy_len;
		if (copy_len < sizeof(EXT_RNID_DATA)) {
			pext->Status = EXT_STATUS_DATA_OVERRUN;
			DEBUG9_10(printk("%s(%ld): inst=%ld data overrun. "
			    "exiting normally.\n",
			    __func__, ha->host_no, ha->instance);)
 		} else if (pext->ResponseLen > sizeof(EXT_RNID_DATA)) {
 			pext->Status = EXT_STATUS_DATA_UNDERRUN;
 			DEBUG9_10(printk("%s(%ld): inst=%ld data underrun. "
 			    "exiting normally.\n",
 			    __func__, ha->host_no, ha->instance);)
		} else {
			pext->Status = EXT_STATUS_OK;
			DEBUG9(printk("%s(%ld): inst=%ld exiting normally.\n",
			    __func__, ha->host_no, ha->instance);)
		}
	}

	return (ret);
}

/*
 * qla2x00_set_host_data
 *	IOCTL command to set host/adapter related data.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = User space CT arguments pointer.
 *	mode = flags.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_set_host_data(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	int	ret = 0;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	/* check on loop down */
	if (ha->loop_state != LOOP_READY || 
	    test_bit(CFG_ACTIVE, &ha->cfg_flags) ||
	    (test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags)) ||
	    ABORTS_ACTIVE || ha->dpc_active) {

		pext->Status = EXT_STATUS_BUSY;
		DEBUG9_10(printk("%s(%ld): inst=%ld loop not ready.\n",
		    __func__, ha->host_no, ha->instance);)

		return (ret);
	}

	/* switch on command subcode */
	switch (pext->SubCode) {
	case EXT_SC_SET_RNID:
		ret = qla2x00_set_rnid_params(ha, pext, mode);
		break;
	default:
		/* function not supported. */
		pext->Status = EXT_STATUS_UNSUPPORTED_SUBCODE;
		break;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return (ret);
}

/*
 * qla2x00_set_rnid_params
 *	IOCTL to set RNID parameters of the adapter.
 *
 * Input:
 *	ha = adapter state pointer.
 *	pext = User space CT arguments pointer.
 *	mode = flags.
 *
 * Returns:
 *	0 = success
 *	others = errno value
 *
 * Context:
 *	Kernel context.
 */
STATIC int
qla2x00_set_rnid_params(scsi_qla_host_t *ha, EXT_IOCTL *pext, int mode)
{
	EXT_SET_RNID_REQ	*tmp_set;
	EXT_RNID_DATA	*tmp_buf;
	int		ret = 0;
	int		tmp_rval = 0;
	uint16_t	mb[MAILBOX_REGISTER_COUNT];

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (pext->RequestLen != sizeof(EXT_SET_RNID_REQ)) {
		/* parameter error */
		pext->Status = EXT_STATUS_INVALID_PARAM;
		DEBUG9_10(printk("%s(%ld): inst=%ld invalid request length.\n",
		    __func__, ha->host_no, ha->instance);)
		return(ret);
	}

	ret = verify_area(VERIFY_READ, (void *)pext->RequestAdr,
	    pext->RequestLen);

	if (ret != 0) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk("%s(%ld): inst=%ld verify READ request buf.\n",
		    __func__, ha->host_no, ha->instance);)
		return(ret);
	}

	if (qla2x00_get_ioctl_scrap_mem(ha, (void **)&tmp_set,
	    sizeof(EXT_SET_RNID_REQ))) {
		/* not enough memory */
		pext->Status = EXT_STATUS_NO_MEMORY;
		DEBUG9_10(printk("%s(%ld): inst=%ld scrap not big enough. "
		    "size requested=%d.\n",
		    __func__, ha->host_no, ha->instance,
		    sizeof(EXT_SET_RNID_REQ));)
		return (ret);
	}

	ret = copy_from_user(tmp_set, pext->RequestAdr, pext->RequestLen);
	if (ret) {
		pext->Status = EXT_STATUS_COPY_ERR;
		DEBUG9_10(printk(
		    "%s(%ld): inst=%ld ERROR copy req buf ret=%d\n", 
		    __func__, ha->host_no, ha->instance, ret);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return(ret);
	}

	tmp_rval = qla2x00_get_rnid_params_mbx(ha, ha->ioctl_mem_phys,
	    sizeof(EXT_RNID_DATA), &mb[0]);
	if (tmp_rval != QLA2X00_SUCCESS) {
		/* error */
		pext->Status = EXT_STATUS_ERR;

                DEBUG9_10(printk("%s(%ld): inst=%ld read cmd FAILED=%x.\n",
                    __func__, ha->host_no, ha->instance, mb[0]);)
		qla2x00_free_ioctl_scrap_mem(ha);
		return (ret);
	}

	tmp_buf = (EXT_RNID_DATA *)ha->ioctl_mem;
	/* Now set the params. */
	memcpy(tmp_buf->IPVersion, tmp_set->IPVersion, 2);
	memcpy(tmp_buf->UDPPortNumber, tmp_set->UDPPortNumber, 2);
	memcpy(tmp_buf->IPAddress, tmp_set->IPAddress, 16);
	tmp_rval = qla2x00_set_rnid_params_mbx(ha, ha->ioctl_mem_phys,
	    sizeof(EXT_RNID_DATA), &mb[0]);

	if (tmp_rval != QLA2X00_SUCCESS) {
		/* error */
		pext->Status = EXT_STATUS_ERR;

		DEBUG9_10(printk("%s(%ld): inst=%ld set cmd FAILED=%x.\n",
		    __func__, ha->host_no, ha->instance, mb[0]);)
	} else {
		pext->Status = EXT_STATUS_OK;
		DEBUG9(printk("%s(%ld): inst=%ld exiting normally.\n",
		    __func__, ha->host_no, ha->instance);)
	}

	qla2x00_free_ioctl_scrap_mem(ha);
	return (ret);
}

/*
 * qla2x00_waitq_sem_timeout
 *	Timeout function to be called when a thread on the wait_q
 *	queue timed out.
 *
 * Input:
 *	data = data pointer for timeout function.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC void
qla2x00_waitq_sem_timeout(unsigned long data)
{
	wait_q_t *tmp_ptr = (wait_q_t *)data;

	DEBUG9(printk("%s: entered.\n", __func__);)

	if (tmp_ptr != NULL) {
		DEBUG9(printk("%s: wait_q thread=%p.\n", __func__, tmp_ptr);)
		up(&tmp_ptr->wait_q_sem);
	}

	DEBUG9(printk("%s: exiting.\n", __func__);)

}

/*
 * qla2x00_get_ioctl_access
 *	Serialization routine for the ioctl commands.
 *	When succeeded the exiting thread gains "access" and
 *	proceeds, otherwise it gives up and returns error.
 *	Each thread would wait tov seconds before giving up.
 *
 * Input:
 *	ha = adapter state pointer.
 *	tov = timeout value in seconds
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC uint8_t
qla2x00_get_ioctl_access(scsi_qla_host_t *ha, uint32_t tov)
{
	int		prev_val = 1;
	uint8_t		rval = QL_STATUS_SUCCESS;
	unsigned long	cpu_flags;
	struct timer_list	tmp_access_timer;
	wait_q_t	*ptmp_wq = NULL;


	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	while (1) {
		if (test_bit(IOCTL_WANT, (void *)&(ha->ioctl->access_bits)) ==
		    0) {

			DEBUG9(printk("%s(%ld): going to test access_bits.\n",
			    __func__, ha->host_no);)

			/* No one else is waiting. Go ahead and try to
			 * get access.
			 */
			if ((prev_val = test_and_set_bit(IOCTL_ACTIVE,
			    (void *)&ha->ioctl->access_bits)) == 0) {
				break;
			}
		}

		/* wait for previous command to finish */
		DEBUG9(printk("%s(%ld): inst=%ld access_bits=%x. busy. "
		    "Waiting for access. curr time=0x%lx.\n",
		    __func__, ha->host_no, ha->instance,
		    ha->ioctl->access_bits, jiffies);)

		/*
		 * Init timer and get semaphore from wait_q. if we got valid
		 * semaphore pointer the IOCTL_WANT flag would also had
		 * been set.
		 */
		qla2x00_wait_q_add(ha, &ptmp_wq);

		if (ptmp_wq == NULL) {
			/* queue full? problem? can't proceed. */
			DEBUG9_10(printk("%s(%ld): ERROR no more wait_q "
			    "allowed. exiting.\n", __func__, ha->host_no);)

			break;
		}

		init_timer(&tmp_access_timer);

		tmp_access_timer.data = (unsigned long)ptmp_wq;
		tmp_access_timer.function =
		    (void (*)(unsigned long))qla2x00_waitq_sem_timeout;
		tmp_access_timer.expires = jiffies + tov * HZ;

		DEBUG9(printk("%s(%ld): adding timer. "
		    "curr time=0x%lx timeoutval=0x%lx.\n",
		    __func__, ha->host_no, jiffies, tmp_access_timer.expires);)

		/* wait. */
		add_timer(&tmp_access_timer);

		DEBUG9(printk("%s(%ld): inst=%ld wait_q %p going to sleep. "
		    "current time=0x%lx.\n",
		    __func__, ha->host_no, ha->instance, ptmp_wq, jiffies);)

		down_interruptible(&ptmp_wq->wait_q_sem);

		DEBUG9(printk("%s(%ld): inst=%ld wait_q %p woke up. current "
		    "time=0x%lx.\n",
		    __func__, ha->host_no, ha->instance, ptmp_wq, jiffies);)

		del_timer(&tmp_access_timer);

		/* try to get lock again. we'll test later to see
		 * if we actually got the lock.
		 */
		prev_val = test_and_set_bit(IOCTL_ACTIVE,
		    (void *)&(ha->ioctl->access_bits));

		/*
		 * After we tried to get access then we check to see
		 * if we need to clear the IOCTL_WANT flag. Don't clear
		 * this flag before trying to get access or another
		 * new thread might grab it before we did.
		 */
		spin_lock_irqsave(&ha->ioctl->wait_q_lock, cpu_flags);
		if (prev_val != 0) {
			/* We'll return with error.
			 * Make sure we remove ourselves from wait_q.
			 */
			qla2x00_wait_q_remove(ha, ptmp_wq);
		}
		if (ha->ioctl->wait_q_head == NULL) {
			/* We're the last thread in wait_q queue. */
			clear_bit(IOCTL_WANT, (void *)&ha->ioctl->access_bits);
		}
		qla2x00_wait_q_memb_free(ha, ptmp_wq);
		spin_unlock_irqrestore(&ha->ioctl->wait_q_lock, cpu_flags);

		break;
	}

	if (prev_val == 0) {
		/* We got the lock */

		DEBUG9(printk("%s(%ld): inst=%ld got access.\n",
		    __func__, ha->host_no, ha->instance);)

	} else {
		/* Timeout or resource error. */
		DEBUG9_10(printk("%s(%ld): inst=%ld timed out "
		    "or wait_q error.\n", __func__, ha->host_no, ha->instance);)

		rval = QL_STATUS_TIMEOUT;
	}

	return (rval);
}

/*
 * qla2x00_release_ioctl_access
 *	Serialization routine for the ioctl commands.
 *	This releases "access" and checks on wai_q queue. If there's
 *	another thread waiting then wakes it up.
 *
 * Input:
 *	ha = adapter state pointer.
 *	tov = timeout value in seconds
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC uint8_t
qla2x00_release_ioctl_access(scsi_qla_host_t *ha)
{
	wait_q_t	*next_thread = NULL;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	clear_bit(IOCTL_ACTIVE, (void *)&(ha->ioctl->access_bits));

	/* Wake up one pending ioctl thread in wait_q */
	qla2x00_wait_q_get_next(ha, &next_thread);
	if (next_thread) {
		DEBUG9(printk(
		    "%s(%ld): inst=%ld found wait_q. Wake up waitq %p\n",
		    __func__, ha->host_no, ha->instance, &next_thread);)
		up(&next_thread->wait_q_sem);
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)

	return QL_STATUS_SUCCESS;
}

/*
 * qla2x00_wait_q_memb_alloc
 *	Finds a free wait_q member from the array. Must already got the
 *	wait_q_lock spinlock.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC void
qla2x00_wait_q_memb_alloc(scsi_qla_host_t *ha, wait_q_t **ret_wait_q_memb)
{
	uint8_t		i;
	wait_q_t	*ptmp = NULL;

	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	for (i = 0; i < MAX_IOCTL_WAIT_THREADS; i++) {
		if (!(ha->ioctl->wait_q_arr[i].flags & WQ_IN_USE)) {
			ha->ioctl->wait_q_arr[i].flags |= WQ_IN_USE;
			ptmp = &ha->ioctl->wait_q_arr[i];
			break;
		}
	}

	*ret_wait_q_memb = ptmp;

	DEBUG9(printk("%s(%ld): inst=%ld return waitq_memb=%p.\n",
	    __func__, ha->host_no, ha->instance, *ret_wait_q_memb);)
}

/*
 * qla2x00_wait_q_memb_free
 *	Frees the specified wait_q member. Must already got the wait_q_lock
 *	spinlock.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC void
qla2x00_wait_q_memb_free(scsi_qla_host_t *ha, wait_q_t *pfree_wait_q_memb)
{
	DEBUG9(printk("%s(%ld): inst=%ld entered.\n",
	    __func__, ha->host_no, ha->instance);)

	if (pfree_wait_q_memb != NULL) {
		DEBUG9(printk("%s(%ld): freeing %p.\n",
		    __func__, ha->host_no, pfree_wait_q_memb);)
		pfree_wait_q_memb->flags &= ~WQ_IN_USE;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)
}

/*
 * qla2x00_wait_q_add
 *	Allocates a wait_q_t struct and add to the wait_q list.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC uint8_t
qla2x00_wait_q_add(scsi_qla_host_t *ha, wait_q_t **ret_wq)
{
	uint8_t		rval = QL_STATUS_SUCCESS;
	unsigned long	cpu_flags;
	wait_q_t	*ptmp = NULL;

	spin_lock_irqsave(&ha->ioctl->wait_q_lock, cpu_flags);

	DEBUG9(printk("%s(%ld): inst=%ld got wait_q spinlock.\n",
	    __func__, ha->host_no, ha->instance);)

	qla2x00_wait_q_memb_alloc(ha, &ptmp);
	if (ptmp == NULL) {
		/* can't add any more threads */
		DEBUG9_10(printk("%s(%ld): inst=%ld ERROR no more ioctl "
		    "threads allowed.\n",
		    __func__, ha->host_no, ha->instance);)

		rval = QL_STATUS_RESOURCE_ERROR;
	} else {
		if (ha->ioctl->wait_q_tail == NULL) {
			/* First thread to queue. */
			set_bit(IOCTL_WANT, (void *)&ha->ioctl->access_bits);

			ha->ioctl->wait_q_head = ptmp;
		} else {
			ha->ioctl->wait_q_tail->pnext = ptmp;
		}
		ha->ioctl->wait_q_tail = ptmp;

		*ret_wq = ptmp;

		/* Now init the semaphore */

		init_MUTEX_LOCKED(&ptmp->wait_q_sem);

		rval = QL_STATUS_SUCCESS;
	}

	DEBUG9(printk("%s(%ld): inst=%ld going to release spinlock. "
	    "ret_wq=%p, rval=%d.\n",
	    __func__, ha->host_no, ha->instance, *ret_wq, rval);)

	spin_unlock_irqrestore(&ha->ioctl->wait_q_lock, cpu_flags);

	return rval;
}

/*
 * qla2x00_wait_q_get_next
 *	This just removes one member from head of wait_q.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC void
qla2x00_wait_q_get_next(scsi_qla_host_t *ha, wait_q_t **ret_wq)
{
	unsigned long	cpu_flags;

	if (test_bit(IOCTL_ACTIVE, (void *)&(ha->ioctl->access_bits)) != 0) {
		/* Another thread just became active. Exit. */
		*ret_wq = NULL;
		return;
	}

	/* Find the next thread to wake up */
	spin_lock_irqsave(&ha->ioctl->wait_q_lock, cpu_flags);

	DEBUG9(printk("%s(%ld): inst=%ld got wait_q spinlock.\n",
	    __func__, ha->host_no, ha->instance);)

	/* Remove from head */
	*ret_wq = ha->ioctl->wait_q_head;
	if (ha->ioctl->wait_q_head != NULL) {

		ha->ioctl->wait_q_head = ha->ioctl->wait_q_head->pnext;

		if (ha->ioctl->wait_q_head == NULL) {
			/* That's the last one in queue. */
			ha->ioctl->wait_q_tail = NULL;
		}

		(*ret_wq)->pnext = NULL;
	}

	DEBUG9(printk("%s(%ld): inst=%ld return ret_wq=%p. Going to release "
	    "spinlock.\n",
	    __func__, ha->host_no, ha->instance, *ret_wq);)
	spin_unlock_irqrestore(&ha->ioctl->wait_q_lock, cpu_flags);
}

/*
 * qla2x00_wait_q_remove
 *	Removes the specified member from wait_q.
 *	Must already got the wait_q_lock spin lock.
 *
 * Input:
 *	ha = adapter state pointer.
 *
 * Returns:
 *	qla2x00 local function return status code.
 *
 * Context:
 *	Kernel context.
 */
STATIC void
qla2x00_wait_q_remove(scsi_qla_host_t *ha, wait_q_t *rem_wq)
{
	wait_q_t	*ptmp_wq;
	wait_q_t	*ptmp_prev;

	DEBUG9(printk("%s(%ld): inst=%ld rem_wq=%p.\n",
	    __func__, ha->host_no, ha->instance, rem_wq);)

	/* Search then remove */
	ptmp_prev = NULL;
	for (ptmp_wq = ha->ioctl->wait_q_head; ptmp_wq != NULL;
	    ptmp_wq = ptmp_wq->pnext) {

		if (ptmp_wq == rem_wq) {
			/* Found it in wait_q. Remove. */

			DEBUG9(printk("%s(%ld): inst=%ld removing.\n",
			    __func__, ha->host_no, ha->instance);)

			if (ha->ioctl->wait_q_head == ptmp_wq) {
				ha->ioctl->wait_q_head = ptmp_wq->pnext;
			} else {
				ptmp_prev->pnext = ptmp_wq->pnext;
			}

			if (ha->ioctl->wait_q_tail == ptmp_wq) {
				ha->ioctl->wait_q_tail = ptmp_prev;
			}

			ptmp_wq->pnext = NULL;

			break;
		}
		ptmp_prev = ptmp_wq;
	}

	DEBUG9(printk("%s(%ld): inst=%ld exiting.\n",
	    __func__, ha->host_no, ha->instance);)
}


