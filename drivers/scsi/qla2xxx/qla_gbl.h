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
* Global include file.
******************************************************************************/


#if !defined(_QLA_GBL_H)
#define	_QLA_GBL_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include "exioct.h"
#include "qla_fo.h"

/*
 * Global Data in qla_fo.c source file.
 */
extern SysFoParams_t qla_fo_params;
/*
 * Global Function Prototypes in qla2x00.c source file.
 */
extern int qla2x00_get_prop_xstr(scsi_qla_host_t *, char *, uint8_t *, int);

extern void qla2x00_formatted_print(char *, uint64_t , uint8_t, uint8_t);
extern void qla2x00_formatted_dump_buffer(char *, uint8_t *, uint8_t ,
			uint32_t );
extern uint32_t qla2x00_fo_path_change(uint32_t ,
					       fc_lun_t *, fc_lun_t *);
extern scsi_qla_host_t *qla2x00_get_hba(int);

/*
 * Global Function Prototypes in qla_fo.c source file.
 */
extern uint32_t qla2x00_send_fo_notification(fc_lun_t *fclun_p, fc_lun_t *olun_p);
extern void qla2x00_fo_init_params(scsi_qla_host_t *ha);
extern BOOL qla2x00_fo_enabled(scsi_qla_host_t *ha, int instance);

/*
 * Global Data in qla_cfg.c source file.
 */
extern mp_host_t  *mp_hosts_base;
extern BOOL   mp_config_required;
/*
 * Global Function Prototypes in qla_cfg.c source file.
 */
extern int qla2x00_cfg_init (scsi_qla_host_t *ha);
extern int qla2x00_cfg_path_discovery(scsi_qla_host_t *ha);
extern int qla2x00_cfg_event_notify(scsi_qla_host_t *ha, uint32_t i_type);
extern fc_lun_t *qla2x00_cfg_failover(scsi_qla_host_t *ha, fc_lun_t *fp,
					      os_tgt_t *tgt, srb_t *sp);
extern int qla2x00_cfg_get_paths( EXT_IOCTL *, FO_GET_PATHS *, int);
extern int qla2x00_cfg_set_current_path( EXT_IOCTL *,
			FO_SET_CURRENT_PATH *, int);
extern void qla2x00_fo_properties(scsi_qla_host_t *ha);
extern mp_host_t * qla2x00_add_mp_host(uint8_t *);
extern void qla2x00_cfg_mem_free(scsi_qla_host_t *ha);
extern mp_host_t * qla2x00_alloc_host(scsi_qla_host_t *);
extern BOOL qla2x00_fo_check(scsi_qla_host_t *ha, srb_t *sp);
extern mp_path_t *qla2x00_find_path_by_name(mp_host_t *, mp_path_list_t *,
			uint8_t *name);

/*
 * Global Function Prototypes in qla_cfgln.c source file.
 */
extern void qla2x00_cfg_build_path_tree( scsi_qla_host_t *ha);
extern BOOL qla2x00_update_mp_device(mp_host_t *,
					     fc_port_t  *, uint16_t	);
extern void qla2x00_cfg_display_devices(void);

/*
 * Global Function Prototypes in qla_ioctl.c source file.
 */
extern int qla2x00_fo_ioctl(scsi_qla_host_t *, int, void *, int);

#if defined(__cplusplus)
}
#endif

#endif /* _QLA_GBL_H */
