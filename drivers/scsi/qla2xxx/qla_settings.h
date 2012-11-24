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
 * Compile time Options:
 *     0 - Disable and 1 - Enable
 */
#define  LOOP_ID_FROM_ONE              0   /* loop ID start from 1 when P2P */
#define  MEMORY_MAPPED_IO              0
#define  DEBUG_QLA2100_INTR            0
#define  USE_NVRAM_DEFAULTS	       0
#define  DEBUG_PRINT_NVRAM             0
#define  LOADING_RISC_ACTIVITY         0
#define  AUTO_ESCALATE_RESET           0   /* Automatically escalate resets */
#define  AUTO_ESCALATE_ABORT           0   /* Automatically escalate aborts */
#define  STOP_ON_ERROR                 0   /* Stop on aborts and resets  */
#define  STOP_ON_RESET                 0
#define  STOP_ON_ABORT                 0
#define  QLA2100_COMTRACE              0    /* One char tracing  */
#define  WATCH_THREADS_SIZ             0    /* watch size of pending queue */
#define  USE_PORTNAME                  1    /* option to use port names for targets */
#define  LUN_MASKING                   0
#define  USE_FLASH_DATABASE            0 /* Save persistent data to flash */
#define  QLA_SCSI_VENDOR_DIR           0 /* Decode vendor specific opcodes for direction */
#define QLA2100_LIPTEST    	       0
#define REQ_TRACE    		       1
#define USE_ABORT_TGT                  1 /* Use Abort Target mbx cmd */

#if defined(FC_IP_SUPPORT)
#define REG_FC4_ENABLED                1 /* Enable register_fc4 call */
#else
#define REG_FC4_ENABLED                0 /* Enable register_fc4 call */
#endif

#undef   TRACECODE                       /* include tracing code in watchdog routines */
#define  CHECK_BINDING
#define  DUMP_INQ_DATA                 0  /* DEBUG_QLA2100 */

#define  DEBUG_QLA2100                 0  /* For Debug of qla2x00 */
#define  DEBUG_GET_FW_DUMP             0  /* also set DEBUG_QLA2100 and
use COM1 and capture it */
#define  NO_LONG_DELAYS			0
#define  QL_TRACE_MEMORY		0

/*
 * This enables some performance code which is not enabled
 * normally:
 *
 * - a tasklet to process the done queue and send requests back to 
 *  the OS.
 */
#define	QLA2X_PERFORMANCE		1 

/* The following WORD_FW_LOAD is defined in Makefile for ia-64 builds
   and can also be decommented here for Word by Word confirmation of
   RISC code download operation */
/* #define  WORD_FW_LOAD               0  */

#define QLA2XXX_HOTSWAP_ENUMERATION	1
#define QLA2XXX_LOOP_RETRY_COUNT	10
#define QLA2XXX_LOOP_DOWN_TIMEOUT	20

#define MPIO_SUPPORT			0
#define VSA				0  /* Volume Set Addressing */

#define PERF_CODE			0  /* enable performance code */
#define EH_DEBUG                        0  /* enable new error handling debug */
/* 
 * When a lun is suspended for the "Not Ready" condition
 * then it will suspend the lun for increments of 6 sec delays.
 * SUSPEND_COUNT is that count.
 */
#define SUSPEND_COUNT 	 		10  /* 6 secs * 10 retries = 60 secs */
#define HSG80_SUSPEND_COUNT		300  /* 6 secs * 300 retries = 30 mins */
#define HSG80_PORT_RETRY_COUNT 	 	64   /* for COMPAQ-HSG80 */
					
/* Failover options */
#define MAX_RECOVERYTIME  		10  /* Max suspend time for a lun recovery time */
#define MAX_FAILBACKTIME  		5  /* (60) Max suspend time before failing back */

#define QLA_CMD_TIMER_DELTA 	 	3

#define MAX_RETRIES_OF_ISP_ABORT  	5  /*  */

/*
 * Under heavy I/O on SMP systems (8-way and IA64) with many command
 * timeouts, the scsi mid-layer will sometimes not wake-up the 
 * error-handling thread when an error-condition occurs.
 * 
 * This workaround if enabled will wakeup the error-handler if it is
 * stuck in this condition for sixty seconds.
 *
 */
#undef EH_WAKEUP_WORKAROUND
#undef EH_WAKEUP_WORKAROUND_REDHAT

/*
 * Defines the time in seconds that 
 * the driver extends the command timeout
 * to get around the problem where the
 * mid-layer only allows 5 retries for commands
 * that return BUS_BUSY
 */
#define EXTEND_CMD_TIMEOUT 	 	 60

/*
 * We need to hardcode this value since the firmware
 * does not allow us to retrieve the maximum number of 
 * IOCBs available during initializtion.
 *
 * Factors that affect this value include the amount of 
 * memory on-board (HBA) and firmware IP support.
 *
 */
#define MAX_IOCBS_AVAILBALE 	 	600

/*
 * Some vendor subsystems do not recover properly after a device reset.  Define
 * the following to force a logout after a successful device reset.
 */
#undef LOGOUT_AFTER_DEVICE_RESET

#include "qla_version.h"


