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
**
******************************************************************************/

#if !defined(_IO_HBA_QLA2100_H)		/* wrapper symbol for kernel use */
#define _IO_HBA_QLA2100_H		/* subject to change without notice */

#if !defined(LINUX_VERSION_CODE)
#include <linux/version.h>
#endif  /* LINUX_VERSION_CODE not defined */

#if !defined(HOSTS_C)

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * Driver debug definitions.
 */
/* #define QL_DEBUG_LEVEL_1  */ /* Output register accesses to COM1 */
/* #define QL_DEBUG_LEVEL_2  */ /* Output error msgs to COM1 */
/* #define QL_DEBUG_LEVEL_3  */ /* Output function trace msgs to COM1 */
/* #define QL_DEBUG_LEVEL_4  */ /* Output NVRAM trace msgs to COM1 */
/* #define QL_DEBUG_LEVEL_5  */ /* Output ring trace msgs to COM1 */
/* #define QL_DEBUG_LEVEL_6  */ /* Output WATCHDOG timer trace to COM1 */
/* #define QL_DEBUG_LEVEL_7  */ /* Output RISC load trace msgs to COM1 */
/* #define QL_DEBUG_LEVEL_8  */ /* Output ring saturation msgs to COM1 */
/* #define QL_DEBUG_LEVEL_9  */ /* Output IOCTL trace msgs */
/* #define QL_DEBUG_LEVEL_10 */ /* Output IOCTL error msgs */
/* #define QL_DEBUG_LEVEL_11 */ /* Output Mbx Cmd trace msgs */
/* #define QL_DEBUG_LEVEL_12 */ /* Output IP trace msgs */

#define QL_DEBUG_CONSOLE            /* Output to console */

#include <asm/bitops.h>
#include <asm/semaphore.h>

/*
 * Data bit definitions.
 */
#define BIT_0   0x1
#define BIT_1   0x2
#define BIT_2   0x4
#define BIT_3   0x8
#define BIT_4   0x10
#define BIT_5   0x20
#define BIT_6   0x40
#define BIT_7   0x80
#define BIT_8   0x100
#define BIT_9   0x200
#define BIT_10  0x400
#define BIT_11  0x800
#define BIT_12  0x1000
#define BIT_13  0x2000
#define BIT_14  0x4000
#define BIT_15  0x8000
#define BIT_16  0x10000
#define BIT_17  0x20000
#define BIT_18  0x40000
#define BIT_19  0x80000
#define BIT_20  0x100000
#define BIT_21  0x200000
#define BIT_22  0x400000
#define BIT_23  0x800000
#define BIT_24  0x1000000
#define BIT_25  0x2000000
#define BIT_26  0x4000000
#define BIT_27  0x8000000
#define BIT_28  0x10000000
#define BIT_29  0x20000000
#define BIT_30  0x40000000
#define BIT_31  0x80000000

#define LS_64BITS(x)	((uint32_t)(0xffffffff & ((u64)(x))))
#define MS_64BITS(x)	((uint32_t)(0xffffffff & (((u64)(x))>>16>>16)))

#define MSB(x)          (uint8_t)(((uint16_t)(x) >> 8) & 0xff)
#define LSB(x)          (uint8_t)(x & 0xff)
#define MSW(x)          (uint16_t)(((uint32_t)(x) >> 16) & 0xffff)
#define LSW(x)          (uint16_t)(x & 0xffff)
#define QL21_64BITS_3RDWD(x)   ((uint16_t) (( (x) >> 16) >> 16) & 0xffff)
#define QL21_64BITS_4THWD(x)   ((uint16_t) ((( (x) >>16)>>16)>>16) & 0xffff)

/*
 *  Local Macro Definitions.
 */
#if defined(QL_DEBUG_LEVEL_1) || defined(QL_DEBUG_LEVEL_2) || \
    defined(QL_DEBUG_LEVEL_3) || defined(QL_DEBUG_LEVEL_4) || \
    defined(QL_DEBUG_LEVEL_5) || defined(QL_DEBUG_LEVEL_6) || \
    defined(QL_DEBUG_LEVEL_7) || defined(QL_DEBUG_LEVEL_8) || \
    defined(QL_DEBUG_LEVEL_9) || defined(QL_DEBUG_LEVEL_10) || \
    defined(QL_DEBUG_LEVEL_11)
    #define QL_DEBUG_ROUTINES
#endif

#if !defined(TRUE)
    #define TRUE  1
#endif

#if !defined(FALSE)
    #define FALSE 0
#endif

typedef char BOOL;


/* 
 * Locking
 */
#include <linux/smp.h>
#define cpuid smp_processor_id()

/*
 * I/O register
*/
/* #define MEMORY_MAPPED_IO  */    /* Enable memory mapped I/O */
#undef MEMORY_MAPPED_IO            /* Disable memory mapped I/O */

#if defined(MEMORY_MAPPED_IO)
#define RD_REG_BYTE(addr)         readb(addr)
#define RD_REG_WORD(addr)         readw(addr)
#define RD_REG_DWORD(addr)        readl(addr)
#define WRT_REG_BYTE(addr, data)  writeb(data,addr)
#define WRT_REG_WORD(addr, data)  writew(data,addr)
#define WRT_REG_DWORD(addr, data) writel(data,addr)
#else   /* MEMORY_MAPPED_IO */
#define RD_REG_BYTE(addr)         (inb((unsigned long)addr))
#define RD_REG_WORD(addr)         (inw((unsigned long)addr))
#define RD_REG_DWORD(addr)        (inl((unsigned long)addr))
#define WRT_REG_BYTE(addr, data)  (outb(data,(unsigned long)addr))
#define WRT_REG_WORD(addr, data)  (outw(data,(unsigned long)addr))
#define WRT_REG_DWORD(addr, data) (outl(data,(unsigned long)addr))
#endif  /* MEMORY_MAPPED_IO */
/*
 * Fibre Channel device definitions.
 */
#define WWN_SIZE		8	/* Size of WWPN, WWN & WWNN */
#define MAX_FIBRE_DEVICES   	256
#define MAX_FIBRE_LUNS  	256
#define	MAX_RSCN_COUNT		10
#define	MAX_HOST_COUNT		16

/*
 * Host adapter default definitions.
 */
#define MAX_BUSES            1  /* We only have one bus today */
#define MAX_TARGETS_2100     MAX_FIBRE_DEVICES
#define MAX_TARGETS_2200     MAX_FIBRE_DEVICES
#define MAX_TARGETS          MAX_FIBRE_DEVICES
#define MIN_LUNS             8
#define MAX_LUNS             MAX_FIBRE_LUNS
#define MAX_CMDS_PER_LUN     255 
#define MAX_SRBS             4096


                                    
/*
 * Fibre Channel device definitions.
 */
#define LAST_LOCAL_LOOP_ID  0x7d
#define SNS_FL_PORT         0x7e
#define FABRIC_CONTROLLER   0x7f
#define SIMPLE_NAME_SERVER  0x80
#define SNS_FIRST_LOOP_ID   0x81
#define LAST_SNS_LOOP_ID    0xfe
#define MANAGEMENT_SERVER   0xfe
#define BROADCAST           0xff
#define SNS_ACCEPT          0x0280      /* 8002 swapped */
#define SNS_REJECT          0x0180      /* 8001 swapped */

/* Loop ID's used as database flags, must be higher than any valid Loop ID */
#define PORT_UNUSED         0x100       /* Port never been used. */
#define PORT_AVAILABLE      0x101       /* Device does not exist on port. */
#define PORT_NEED_MAP       0x102       
#define PORT_LOST_ID        0x200       
#define PORT_LOGIN_NEEDED   0x400       

/*
 * Timeout timer counts in seconds
 */
#define QLA2100_WDG_TIME_QUANTUM   5    /* In seconds */
#define PORT_RETRY_TIME            2
#define LOOP_DOWN_TIMEOUT          60
#define LOOP_DOWN_TIME             120 		/* 240 */
#define	LOOP_DOWN_RESET		(LOOP_DOWN_TIME - 30)

/* Maximum outstanding commands in ISP queues (1-65535) */
#define MAX_OUTSTANDING_COMMANDS   1024

/* ISP request and response entry counts (37-65535) */
#define REQUEST_ENTRY_CNT       128     /* Number of request entries. */
#if defined(ISP2100) || defined(ISP2200)
#define RESPONSE_ENTRY_CNT      64      /* Number of response entries.*/
#else
#define RESPONSE_ENTRY_CNT      512     /* Number of response entries.*/
#endif

#define  SCSI_BUS_32(scp)   ((scp)->channel)
#define  SCSI_TCN_32(scp)    ((scp)->target)
#define  SCSI_LUN_32(scp)    ((scp)->lun)



/*
 * UnixWare required definitions.
 */
#define HBA_PREFIX qla2100

/* Physical DMA memory requirements */
#define QLA2100_MEMALIGN    4
#define QLA2100_BOUNDARY    0x80000000  /* 2GB */

/* Number of segments 1 - 65535 */
#define SG_SEGMENTS     SG_ALL             /* Cmd entry + 6 continuations */

/*
 * SCSI Request Block 
 */
typedef struct srb
{
    struct list_head   list;
    struct srb  *s_next;             /* (4) Next block on LU queue */
    struct srb  *s_prev;             /* (4) Previous block on LU queue */
    Scsi_Cmnd  *cmd;                 /* Linux SCSI command pkt */
    struct scsi_qla_host *ha;		/* ha this SP is queued on */
    uint8_t     more_cdb[4];         /* For 16 bytes CDB pass thru cmd since
                                        linux SCSI cdb is 12 bytes. */ 
    uint8_t     dir;                 /* direction of transfer */
    uint8_t     unused1;
    uint8_t     ccode;               /* risc completion code */
    uint8_t     scode;               /* scsi status code */
    
    uint16_t    flags;               /* Status flags - defined below */
    uint16_t     state;
#define SRB_FREE_STATE          0    /* Request returned back */
#define SRB_PENDING_STATE       1    /* Request being queued in LUN Q */
#define SRB_ACTIVE_STATE        2    /* Request in Active Array */
#define SRB_DONE_STATE          3    /* Request Queued in Done Queue */
#define SRB_RETRY_STATE         4    /* Request in Retry Queue */
#define SRB_SUSPENDED_STATE     5    /* Request in suspended state */
#define SRB_NO_QUEUE_STATE      6    /* Request is in between states */
#define SRB_ACTIVE_TIMEOUT_STATE 7   /* Request in Active Array but timed out */
#define SRB_FAILOVER_STATE 	8    /* Request in Failover Queue */
#define SRB_SCSI_RETRY_STATE    9    /* Request in Scsi Retry Queue */

    uint8_t     used;		     /* used by allocation code */
    uint8_t     ref_num;             /* reference SRB number */	
    uint16_t    magic;               /* qlogic magic number */
#define SRB_MAGIC       0x10CB

    u_long      host_no;             /* Host number of allocating host */
    struct      timer_list   timer;  /* used to timeout command */
    dma_addr_t	 saved_dma_handle;    /* for unmap of single transfers */

    atomic_t	 ref_count;	      /* reference count for this structure */			
	/* Target/LUN queue pointers. */
    struct os_tgt		*tgt_queue;	/* ptr to visible ha's target */
    struct os_lun		*lun_queue;	/* ptr to visible ha's lun */
	struct fc_lun		*fclun;		/* FC LUN context pointer. */
	/* Raw completion info for use by failover ? */
    uint8_t	fo_retry_cnt;	/* Retry count this request */
    uint8_t	err_id;		/* error id */
    uint8_t	cmd_length;		/* command length */
    uint8_t	unused3;

    int      delay;             /* delay in seconds */
    int      ext_history;             /*  */
	
    u_long      e_start;             /* jiffies at start of extend timeout */
    u_long      r_start;             /* jiffies at start of request */
    u_long      u_start;             /* jiffies when sent to F/W    */
    u_long      f_start;            /*ra 10/29/01*/ /*jiffies when put in failov					er queue*/
    uint32_t    resid;              /* Residual transfer length */
    uint16_t    sense_len;          /* Sense data length */
    uint32_t    request_sense_length;
    void        *request_sense_ptr;

    uint32_t 	 iocb_cnt;
}srb_t;

/*
 * SRB flag definitions
 */
#define SRB_TIMEOUT          BIT_0	/* Command timed out */
#define SRB_DMA_VALID        BIT_1	/* Command sent to ISP */
#define SRB_WATCHDOG         BIT_2	/* Command on watchdog list */
#define SRB_ABORT_PENDING    BIT_3	/* Command abort sent to device */

#define SRB_ABORTED          BIT_4	/* Command aborted command already */
#define SRB_RETRY            BIT_5	/* Command needs retrying */
#define SRB_GOT_SENSE	     BIT_6	/* Command has sense data */
#define SRB_FAILOVER         BIT_7	/* Command in failover state */

#define SRB_BUSY             BIT_8	/* Command is in busy retry state */
#define SRB_FO_CANCEL        BIT_9	/* Command don't need to do failover */
#define	SRB_IOCTL	     BIT_10	/* IOCTL command. */
#define	SRB_ISP_STARTED	     BIT_11	/* Command sent to ISP. */

#define	SRB_ISP_COMPLETED    BIT_12	/* ISP finished with command */


/*
 *  ISP PCI Configuration Register Set
 */
typedef volatile struct
{
    uint16_t vendor_id;                 /* 0x0 */
    uint16_t device_id;                 /* 0x2 */
    uint16_t command;                   /* 0x4 */
    uint16_t status;                    /* 0x6 */
    uint8_t revision_id;                /* 0x8 */
    uint8_t programming_interface;      /* 0x9 */
    uint8_t sub_class;                  /* 0xa */
    uint8_t base_class;                 /* 0xb */
    uint8_t cache_line;                 /* 0xc */
    uint8_t latency_timer;              /* 0xd */
    uint8_t header_type;                /* 0xe */
    uint8_t bist;                       /* 0xf */
    uint32_t base_port;                 /* 0x10 */
    uint32_t mem_base_addr;             /* 0x14 */
    uint32_t base_addr[4];              /* 0x18-0x24 */
    uint32_t reserved_1[2];             /* 0x28-0x2c */
    uint16_t expansion_rom;             /* 0x30 */
    uint32_t reserved_2[2];             /* 0x34-0x38 */
    uint8_t interrupt_line;             /* 0x3c */
    uint8_t interrupt_pin;              /* 0x3d */
    uint8_t min_grant;                  /* 0x3e */
    uint8_t max_latency;                /* 0x3f */
}config_reg_t __attribute__((packed));


#if defined(ISP2100) || defined(ISP2200)
/*
 *  ISP I/O Register Set structure definitions for ISP2200 and ISP2100.
 */
typedef volatile struct
{
    uint16_t flash_address;             /* Flash BIOS address */
    uint16_t flash_data;                /* Flash BIOS data */
    uint16_t unused_1[1];               /* Gap */
    uint16_t ctrl_status;               /* Control/Status */
					/* Flash upper 64K bank select */
        #define CSR_FLASH_64K_BANK	BIT_3  
					/* Flash BIOS Read/Write enable */
        #define CSR_FLASH_ENABLE	BIT_1  
					/* ISP soft reset */
        #define CSR_ISP_SOFT_RESET	BIT_0   
    uint16_t ictrl;                     /* Interrupt control */
        #define ISP_EN_INT      BIT_15  /* ISP enable interrupts. */
        #define ISP_EN_RISC     BIT_3   /* ISP enable RISC interrupts. */
    uint16_t istatus;                   /* Interrupt status */
        #define RISC_INT        BIT_3   /* RISC interrupt */
    uint16_t semaphore;                 /* Semaphore */
    uint16_t nvram;                     /* NVRAM register. */
        #define NV_DESELECT     0
        #define NV_CLOCK        BIT_0
        #define NV_SELECT       BIT_1
        #define NV_DATA_OUT     BIT_2
        #define NV_DATA_IN      BIT_3

    uint16_t mailbox0;                  /* Mailbox 0 */
    uint16_t mailbox1;                  /* Mailbox 1 */
    uint16_t mailbox2;                  /* Mailbox 2 */
    uint16_t mailbox3;                  /* Mailbox 3 */
    uint16_t mailbox4;                  /* Mailbox 4 */
    uint16_t mailbox5;                  /* Mailbox 5 */
    uint16_t mailbox6;                  /* Mailbox 6 */
    uint16_t mailbox7;                  /* Mailbox 7 */
    uint16_t unused_2[0x3b];	        /* Gap */

    uint16_t fpm_diag_config;
    uint16_t unused_3[0x6];		/* Gap */
    uint16_t pcr;	        	/* Processor Control Register.*/
    uint16_t unused_4[0x5];		/* Gap */
    uint16_t mctr;		        /* Memory Configuration and Timing. */
    uint16_t unused_5[0x3];		/* Gap */
    uint16_t fb_cmd;
    uint16_t unused_6[0x3];		/* Gap */

    uint16_t host_cmd;                  /* Host command and control */
        #define HOST_INT      BIT_7     /* host interrupt bit */

    uint16_t unused_7[5];		/* Gap */
    uint16_t gpiod;			/* GPIO data register */
    uint16_t gpioe;			/* GPIO enable register */

#if defined(ISP2200)
    uint16_t unused_8[8];		/* Gap */
    uint16_t mailbox8;                  /* Mailbox 8 */
    uint16_t mailbox9;                  /* Mailbox 9 */
    uint16_t mailbox10;                 /* Mailbox 10 */
    uint16_t mailbox11;                 /* Mailbox 11 */
    uint16_t mailbox12;                 /* Mailbox 12 */
    uint16_t mailbox13;                 /* Mailbox 13 */
    uint16_t mailbox14;                 /* Mailbox 14 */
    uint16_t mailbox15;                 /* Mailbox 15 */
    uint16_t mailbox16;                 /* Mailbox 16 */
    uint16_t mailbox17;                 /* Mailbox 17 */
    uint16_t mailbox18;                 /* Mailbox 18 */
    uint16_t mailbox19;                 /* Mailbox 19 */
    uint16_t mailbox20;                 /* Mailbox 20 */
    uint16_t mailbox21;                 /* Mailbox 21 */
    uint16_t mailbox22;                 /* Mailbox 22 */
    uint16_t mailbox23;                 /* Mailbox 23 */
#endif
} device_reg_t;

#else
/*
 *  I/O Register Set structure definitions for ISP2300.
 */
typedef volatile struct
{
    uint16_t flash_address;             /* Flash BIOS address */
    uint16_t flash_data;                /* Flash BIOS data */
    uint16_t unused_1[1];               /* Gap */
    uint16_t ctrl_status;               /* Control/Status */
					/* Flash upper 64K bank select */
        #define CSR_FLASH_64K_BANK	BIT_3  
					/* Flash BIOS Read/Write enable */
        #define CSR_FLASH_ENABLE	BIT_1  
					/* ISP soft reset */
        #define CSR_ISP_SOFT_RESET	BIT_0   
    uint16_t ictrl;                     /* Interrupt control */
        #define ISP_EN_INT      BIT_15  /* ISP enable interrupts. */
    	#define ISP_EN_RISC     BIT_3   /* ISP enable RISC interrupts. */
    uint16_t istatus;                   /* Interrupt status @0xa*/
        #define RISC_INT        BIT_3   /* RISC interrupt */
    uint16_t semaphore;                 /* Semaphore */
    uint16_t nvram;                     /* NVRAM register. @0xf */
        #define NV_DESELECT     0
        #define NV_CLOCK        BIT_0
        #define NV_SELECT       BIT_1
        #define NV_DATA_OUT     BIT_2
        #define NV_DATA_IN      BIT_3
        #define NV_BUSY         BIT_15
    uint16_t req_q_in;                  /* @0x10 */
    uint16_t req_q_out;                 /* @0x12 */
    uint16_t rsp_q_in;                  /* @0x14 */
    uint16_t rsp_q_out;                 /* @0x16 */ 
    uint16_t host_status_lo;            /* RISC to Host Status Low */
        #define HOST_STATUS_INT   BIT_15  /* RISC int */
        #define ROM_MB_CMD_COMP   0x01  /* ROM mailbox cmd complete */
        #define ROM_MB_CMD_ERROR  0x02  /*ROM mailbox cmd unsuccessful*/
        #define MB_CMD_COMP       0x10  /* Mailbox cmd complete */
        #define MB_CMD_ERROR      0x11  /* Mailbox cmd unsuccessful */
        #define ASYNC_EVENT       0x12  /* Asynchronous event */
        #define RESPONSE_QUEUE_INT 0x13 /* Response Queue update */
        #define RIO_ONE           0x15  /* RIO one 16 bit handle */
        #define FAST_SCSI_COMP    0x16  /* Fast Post SCSI complete */
    uint16_t host_status_hi;            /* RISC to Host Status High */
    uint16_t host_semaphore;            /* Host to Host Semaphore */
    uint16_t unused_2[0x11];            /* Gap */
    uint16_t mailbox0;                  /* Mailbox 0 @0x40 */
    uint16_t mailbox1;                  /* Mailbox 1 */
    uint16_t mailbox2;                  /* Mailbox 2 */
    uint16_t mailbox3;                  /* Mailbox 3 */
    uint16_t mailbox4;                  /* Mailbox 4 */
    uint16_t mailbox5;                  /* Mailbox 5 */
    uint16_t mailbox6;                  /* Mailbox 6 */
    uint16_t mailbox7;                  /* Mailbox 7 @0x4E */
    uint16_t mailbox8;                  /* Mailbox 8 */
    uint16_t mailbox9;                  /* Mailbox 9 */
    uint16_t mailbox10;                 /* Mailbox 10 */
    uint16_t mailbox11;                 /* Mailbox 11 */
    uint16_t mailbox12;                 /* Mailbox 12 */
    uint16_t mailbox13;                 /* Mailbox 13 */
    uint16_t mailbox14;                 /* Mailbox 14 */
    uint16_t mailbox15;                 /* Mailbox 15 */
    uint16_t mailbox16;                 /* Mailbox 16 */
    uint16_t mailbox17;                 /* Mailbox 17 */
    uint16_t mailbox18;                 /* Mailbox 18 */
    uint16_t mailbox19;                 /* Mailbox 19 */
    uint16_t mailbox20;                 /* Mailbox 20 */
    uint16_t mailbox21;                 /* Mailbox 21 */
    uint16_t mailbox22;                 /* Mailbox 22 */
    uint16_t mailbox23;                 /* Mailbox 23 */
    uint16_t mailbox24;                  /* Mailbox 24 */
    uint16_t mailbox25;                  /* Mailbox 25 */
    uint16_t mailbox26;                 /* Mailbox 26 */
    uint16_t mailbox27;                 /* Mailbox 27 */
    uint16_t mailbox28;                 /* Mailbox 28 */
    uint16_t mailbox29;                 /* Mailbox 29 */
    uint16_t mailbox30;                 /* Mailbox 30 */
    uint16_t mailbox31;                 /* Mailbox 31 @0x7E */
    uint16_t unused4[0xb];              /* gap */

    uint16_t fpm_diag_config;
    uint16_t unused_3[0x6];		/* Gap */
    uint16_t pcr;	   	        /* Processor Control Register.*/
    uint16_t unused_4[0x5];		/* Gap */
    uint16_t mctr;		        /* Memory Configuration and Timing. */
    uint16_t unused_5[0x3];		/* Gap */
    uint16_t fb_cmd;
    uint16_t unused_6[0x3];		/* Gap */
    uint16_t host_cmd;                  /* Host command and control */
        #define HOST_INT      BIT_7     /* host interrupt bit */

    uint16_t unused_7[5];		/* Gap */
    uint16_t gpiod;			/* GPIO data register */
    uint16_t gpioe;			/* GPIO enable register */
}device_reg_t;
#endif

#if defined(ISP2100)
#define	MAILBOX_REGISTER_COUNT	8
#elif defined(ISP2200)
#define	MAILBOX_REGISTER_COUNT	24
#elif defined(ISP2300)
#define	MAILBOX_REGISTER_COUNT	32
#endif

typedef struct {
	uint32_t out_mb;	/* outbound from driver */
	uint32_t in_mb;		/* Incoming from RISC */
	uint16_t mb[MAILBOX_REGISTER_COUNT];
	long	buf_size;
	void	*bufp;
	uint32_t tov;
	uint8_t	flags;
#define MBX_DMA_IN	BIT_0
#define	MBX_DMA_OUT	BIT_1
#define IOCTL_CMD	BIT_2
} mbx_cmd_t;

#define	MBX_TOV_SECONDS	30

/*
 *  ISP product identification definitions in mailboxes after reset.
 */
#define PROD_ID_1           0x4953
#define PROD_ID_2           0x0000
#define PROD_ID_2a          0x5020
#define PROD_ID_3           0x2020
#define PROD_ID_4           0x1
#define PROD_ID_4a          0x2

/*
 * ISP host command and control register command definitions
 */
#define HC_RESET_RISC       0x1000      /* Reset RISC */
#define HC_PAUSE_RISC       0x2000      /* Pause RISC */
#define HC_RELEASE_RISC     0x3000      /* Release RISC from reset. */
#define HC_SET_HOST_INT     0x5000      /* Set host interrupt */
#define HC_CLR_HOST_INT     0x6000      /* Clear HOST interrupt */
#define HC_CLR_RISC_INT     0x7000      /* Clear RISC interrupt */
#define HC_RISC_PAUSE       BIT_5
#define	HC_DISABLE_PARITY_PAUSE	0x4001	/* Disable parity error RISC pause. */
#define HC_ENABLE_PARITY    0xA000      /* Enable PARITY interrupt */

/*
 * ISP mailbox Self-Test status codes
 */
#define MBS_FRM_ALIVE       0           /* Firmware Alive. */
#define MBS_CHKSUM_ERR      1           /* Checksum Error. */
#define MBS_BUSY            4           /* Busy. */

/*
 * ISP mailbox command complete status codes
 */
#define MBS_CMD_CMP         0x4000      /* Command Complete. */
#define MBS_INV_CMD         0x4001      /* Invalid Command. */
#define MBS_HOST_INF_ERR    0x4002      /* Host Interface Error. */
#define MBS_TEST_FAILED     0x4003      /* Test Failed. */
#define MBS_CMD_ERR         0x4005      /* Command Error. */
#define MBS_CMD_PARAM_ERR   0x4006      /* Command Parameter Error. */
#define MBS_FATAL_ERROR     0xF000      /* Command Fatal Error. */

#define MBS_FIRMWARE_ALIVE          0x0000 
#define MBS_COMMAND_COMPLETE        0x4000 
#define MBS_INVALID_COMMAND         0x4001 

/* QLogic subroutine status definitions */
#define QL_STATUS_SUCCESS           0
#define QL_STATUS_ERROR             1
#define QL_STATUS_FATAL_ERROR       2
#define QL_STATUS_RESOURCE_ERROR    3
#define QL_STATUS_LOOP_ID_IN_USE    4
#define QL_STATUS_NO_DATA           5
#define QL_STATUS_TIMEOUT           6
/*
 * ISP mailbox asynchronous event status codes
 */
#define MBA_ASYNC_EVENT         0x8000  /* Asynchronous event. */
#define MBA_RESET               0x8001  /* Reset Detected. */
#define MBA_SYSTEM_ERR          0x8002  /* System Error. */
#define MBA_REQ_TRANSFER_ERR    0x8003  /* Request Transfer Error. */
#define MBA_RSP_TRANSFER_ERR    0x8004  /* Response Transfer Error. */
#define MBA_WAKEUP_THRES        0x8005  /* Request Queue Wake-up. */
#define MBA_LIP_OCCURRED        0x8010  /* Loop Initialization Procedure */
                                        /* occurred. */
#define MBA_LOOP_UP             0x8011  /* FC Loop UP. */
#define MBA_LOOP_DOWN           0x8012  /* FC Loop Down. */
#define MBA_LIP_RESET           0x8013  /* LIP reset occurred. */
#define MBA_PORT_UPDATE         0x8014  /* Port Database update. */
#define MBA_SCR_UPDATE          0x8015  /* State Change Registration. */
#define MBA_RSCN_UPDATE         MBA_SCR_UPDATE
#define MBA_SCSI_COMPLETION     0x8020  /* SCSI Command Complete. */
#define MBA_CTIO_COMPLETION     0x8021  /* CTIO Complete. */
#if !defined(ISP2100)
#define MBA_LINK_MODE_UP        0x8030  /* FC Link Mode UP. */
#define MBA_UPDATE_CONFIG       0x8036  /* FC Update Configuration. */
#endif

/*
 * ISP mailbox commands
 */
#define MBC_LOAD_RAM              1     /* Load RAM. */
#define MBC_EXECUTE_FIRMWARE      2     /* Execute firmware. */
#define MBC_WRITE_RAM_WORD        4     /* Write RAM word. */
#define MBC_READ_RAM_WORD         5     /* Read RAM word. */
#define MBC_MAILBOX_REGISTER_TEST 6     /* Wrap incoming mailboxes */
#define MBC_VERIFY_CHECKSUM       7     /* Verify checksum. */
#define MBC_ABOUT_FIRMWARE        8     /* Get firmware revision. */
#define MBC_LOAD_RAM_A64          9     /* Load RAM by 64-bit address. */
#define MBC_DUMP_RAM              0xA   /* READ BACK FW */
#define MBC_LOAD_RAM_EXTENDED     0xB   /* Load Extended RAM */
#define MBC_DUMP_SRAM             0xC   /* Dump SRAM    */
#define MBC_IOCB_EXECUTE          0x12  /* Execute an IOCB command */
#define MBC_ABORT_COMMAND         0x15  /* Abort IOCB command. */
#define MBC_ABORT_DEVICE          0x16  /* Abort device (ID/LUN). */
#define MBC_ABORT_TARGET          0x17  /* Abort target (ID). */
#define MBC_TARGET_RESET_ALL      0x18  /* Reset all local targets. */
#define MBC_GET_ADAPTER_LOOP_ID   0x20  /* Get loop id of ISP2100. */
#define MBC_GET_RETRY_COUNT       0x22  /* GET RATOV & retry count */
#define MBC_GET_FIRMWARE_OPTIONS  0x28  /* Get firmware options. */
#define MBC_SET_RETRY_COUNT       0x32  /* SET RATOV & retry count */
#define MBC_SET_FIRMWARE_OPTIONS  0x38  /* Set firmware options. */
#define MBC_GET_RESOURCE_COUNTS   0x42  /* GET Resource counts */
#define MBC_DIAGNOSTIC_LOOP_BACK  0x45  /* Perform LoopBack diagnostic */
#define MBC_ENHANCED_GET_PORT_DATABASE     0x47  /* Get port database. */
#define MBC_IOCB_EXECUTE_A64	  0x54  /* Execute an IOCB command (64bit) */
#define	MBC_SEND_RNID_ELS         0x57	/* Send RNID ELS request */
#define	MBC_SET_RNID_PARAMS       0x59	/* Set RNID parameters */
#define	MBC_GET_RNID_PARAMS       0x5a	/* Get RNID parameters */
#define MBC_INITIALIZE_FIRMWARE   0x60  /* Initialize firmware */
#define MBC_INITIATE_LIP          0x62  /* Initiate Loop Initialization */
                                        /* Procedure */
#define MBC_GET_FCAL_MAP	0x63  /* Get FC/AL position map */
#define MBC_GET_PORT_DATABASE     0x64  /* Get port database. */
#define MBC_TARGET_RESET	  0x66  /* Target reset. */
#define MBC_GET_FIRMWARE_STATE    0x69  /* Get firmware state. */
#define MBC_GET_PORT_NAME         0x6a  /* Get port name. */
#define MBC_GET_LINK_STATUS       0x6b  /* Get link status. */
#define MBC_LIP_RESET             0x6c  /* LIP reset. */
#define MBC_SEND_SNS_COMMAND      0x6e  /* Send Simple Name Server command. */
#define MBC_LOGIN_FABRIC_PORT     0x6f  /* Login fabric port. */
#define MBC_LOGOUT_FABRIC_PORT    0x71  /* Logout fabric port. */
#define MBC_LIP_FULL_LOGIN        0x72  /* Full login LIP. */
#define	MBC_LOGIN_LOOP_PORT       0x74	/* Login Loop Port. */
#define MBC_GET_PORT_LIST         0x75  /* Get port list. */
#define	MBC_INITIALIZE_RECEIVE_QUEUE	0x77	/* Initialize receive queue */
#define	MBC_SEND_FARP_REQ_COMMAND	0x78	/* FARP request. */
#define	MBC_SEND_FARP_REPLY_COMMAND	0x79	/* FARP reply. */
#define	MBC_PORT_LOOP_NAME_LIST		0x7C	/* Get port/node name list. */
#define	MBC_SEND_LFA_COMMAND		0x7D	/* Send Loop Fabric Address */
#define	MBC_LUN_RESET			0x7E	/* Send LUN reset */

/* Firmware return data sizes */
#define FCAL_MAP_SIZE	128

/* Mailbox bit definitions for out_mb and in_mb */
#define	MBX_31		BIT_31
#define	MBX_30		BIT_30
#define	MBX_29		BIT_29
#define	MBX_28		BIT_28
#define	MBX_27		BIT_27
#define	MBX_26		BIT_26
#define	MBX_25		BIT_25
#define	MBX_24		BIT_24
#define	MBX_23		BIT_23
#define	MBX_22		BIT_22
#define	MBX_21		BIT_21
#define	MBX_20		BIT_20
#define	MBX_19		BIT_19
#define	MBX_18		BIT_18
#define	MBX_17		BIT_17
#define	MBX_16		BIT_16
#define	MBX_15		BIT_15
#define	MBX_14		BIT_14
#define	MBX_13		BIT_13
#define	MBX_12		BIT_12
#define	MBX_11		BIT_11
#define	MBX_10		BIT_10
#define	MBX_9		BIT_9
#define	MBX_8		BIT_8
#define	MBX_7		BIT_7
#define	MBX_6		BIT_6
#define	MBX_5		BIT_5
#define	MBX_4		BIT_4
#define	MBX_3		BIT_3
#define	MBX_2		BIT_2
#define	MBX_1		BIT_1
#define	MBX_0		BIT_0

/*
 * Firmware state codes from get firmware state mailbox command
 */
#define FSTATE_CONFIG_WAIT      0
#define FSTATE_WAIT_AL_PA       1
#define FSTATE_WAIT_LOGIN       2
#define FSTATE_READY            3
#define FSTATE_LOSS_OF_SYNC     4
#define FSTATE_ERROR            5
#define FSTATE_REINIT           6
#define FSTATE_NON_PART         7

#define FSTATE_CONFIG_CORRECT      0
#define FSTATE_P2P_RCV_LIP         1
#define FSTATE_P2P_CHOOSE_LOOP     2
#define FSTATE_P2P_RCV_UNIDEN_LIP  3
#define FSTATE_FATAL_ERROR         4
#define FSTATE_LOOP_BACK_CONN      5

/*
 * Port Database structure definition
 * Little endian except where noted.
 */
#define	PORT_DATABASE_SIZE	128	/* bytes */
typedef struct {
	uint8_t options;
	uint8_t control;
	uint8_t master_state;
	uint8_t slave_state;
#define	PD_STATE_DISCOVERY			0
#define	PD_STATE_WAIT_DISCOVERY_ACK		1
#define	PD_STATE_PORT_LOGIN			2
#define	PD_STATE_WAIT_PORT_LOGIN_ACK		3
#define	PD_STATE_PROCESS_LOGIN			4
#define	PD_STATE_WAIT_PROCESS_LOGIN_ACK		5
#define	PD_STATE_PORT_LOGGED_IN			6
#define	PD_STATE_PORT_UNAVAILABLE		7
#define	PD_STATE_PROCESS_LOGOUT			8
#define	PD_STATE_WAIT_PROCESS_LOGOUT_ACK	9
#define	PD_STATE_PORT_LOGOUT			10
#define	PD_STATE_WAIT_PORT_LOGOUT_ACK		11
	uint8_t reserved[2];
	uint8_t hard_address;
	uint8_t reserved_1;
	uint8_t port_id[4];
	uint8_t node_name[8];			/* Big endian. */
	uint8_t port_name[8];			/* Big endian. */
	uint16_t execution_throttle;
	uint16_t execution_count;
	uint8_t reset_count;
	uint8_t reserved_2;
	uint16_t resource_allocation;
	uint16_t current_allocation;
	uint16_t queue_head;
	uint16_t queue_tail;
	uint16_t transmit_execution_list_next;
	uint16_t transmit_execution_list_previous;
	uint16_t common_features;
	uint16_t total_concurrent_sequences;
	uint16_t RO_by_information_category;
	uint8_t recipient;
	uint8_t initiator;
	uint16_t receive_data_size;
	uint16_t concurrent_sequences;
	uint16_t open_sequences_per_exchange;
	uint16_t lun_abort_flags;
	uint16_t lun_stop_flags;
	uint16_t stop_queue_head;
	uint16_t stop_queue_tail;
	uint16_t port_retry_timer;
	uint16_t next_sequence_id;
	uint16_t frame_count;
	uint16_t PRLI_payload_length;
	uint8_t prli_svc_param_word_0[2];	/* Big endian */
						/* Bits 15-0 of word 0 */
	uint8_t prli_svc_param_word_3[2];	/* Big endian */
						/* Bits 15-0 of word 3 */
	uint16_t loop_id;
	uint16_t extended_lun_info_list_pointer;
	uint16_t extended_lun_stop_list_pointer;
} port_database_t;

/*
 * ISP Initialization Control Block.
 */

struct qla2100_firmware_options
{
#if defined(__BIG_ENDIAN)
        uint8_t unused_15                    :1;
        uint8_t enable_name_change           :1;
        uint8_t enable_full_login_on_lip     :1;
        uint8_t enable_stop_q_on_full        :1;

        uint8_t previous_assigned_addressing :1;
        uint8_t enable_decending_soft_assign :1;
        uint8_t disable_initial_lip          :1;
        uint8_t enable_port_update_event     :1;

        uint8_t enable_lun_response          :1;
        uint8_t enable_adisc                 :1;
        uint8_t disable_initiator_mode       :1;
        uint8_t enable_target_mode           :1;

        uint8_t enable_fast_posting          :1;
        uint8_t enable_full_duplex           :1;
        uint8_t enable_fairness              :1;
        uint8_t enable_hard_loop_id          :1;
#else
        uint8_t enable_hard_loop_id          :1;
        uint8_t enable_fairness              :1;
        uint8_t enable_full_duplex           :1;
        uint8_t enable_fast_posting          :1;

        uint8_t enable_target_mode           :1;
        uint8_t disable_initiator_mode       :1;
        uint8_t enable_adisc                 :1;
        uint8_t enable_lun_response          :1;

        uint8_t enable_port_update_event     :1;
        uint8_t disable_initial_lip          :1;
        uint8_t enable_decending_soft_assign :1;
        uint8_t previous_assigned_addressing :1;

        uint8_t enable_stop_q_on_full        :1;
        uint8_t enable_full_login_on_lip     :1;
        uint8_t enable_name_change           :1;
        uint8_t unused_15                    :1;
#endif
};

struct qla2x00_firmware_options
{
#if defined(__BIG_ENDIAN)
        uint8_t expanded_ifwcb               :1;
        uint8_t node_name_option             :1;
        uint8_t enable_full_login_on_lip     :1;
        uint8_t enable_stop_q_on_full        :1;

        uint8_t previous_assigned_addressing :1;
        uint8_t enable_decending_soft_assign :1;
        uint8_t disable_initial_lip          :1;
        uint8_t enable_port_update_event     :1;

        uint8_t enable_lun_response          :1;
        uint8_t enable_adisc                 :1;
        uint8_t disable_initiator_mode       :1;
        uint8_t enable_target_mode           :1;

        uint8_t enable_fast_posting          :1;
        uint8_t enable_full_duplex           :1;
        uint8_t enable_fairness              :1;
        uint8_t enable_hard_loop_id          :1;
#else
        uint8_t enable_hard_loop_id          :1;
        uint8_t enable_fairness              :1;
        uint8_t enable_full_duplex           :1;
        uint8_t enable_fast_posting          :1;

        uint8_t enable_target_mode           :1;
        uint8_t disable_initiator_mode       :1;
        uint8_t enable_adisc                 :1;
        uint8_t enable_lun_response          :1;

        uint8_t enable_port_update_event     :1;
        uint8_t disable_initial_lip          :1;
        uint8_t enable_decending_soft_assign :1;
        uint8_t previous_assigned_addressing :1;

        uint8_t enable_stop_q_on_full        :1;
        uint8_t enable_full_login_on_lip     :1;
        uint8_t node_name_option             :1;
        uint8_t expanded_ifwcb               :1;
#endif
};

struct qla2x00_additional_firmware_options
{
#if defined(__BIG_ENDIAN)
        uint8_t unused_15                    :1; /* bit 0 */
        uint8_t enable_cmd_q_target_mode     :1; /* bit 1 */
        uint8_t enable_fc_confirm            :1; /* bit 2 */
        uint8_t enable_fc_tape               :1; /* bit 3 */

        uint8_t unused_11                    :1; /* bit 4 */
        uint8_t unused_10                    :1; /* bit 5 */
        uint8_t enable_ack0                  :1; /* bit 6 */
        uint8_t enable_class2                :1; /* bit 7 */

        uint8_t nonpart_if_hard_addr_failed  :1; /* bit 8 */
        uint8_t connection_options           :3; /* bits 9-11 */
                #define LOOP      0
                #define P2P       1
                #define LOOP_P2P  2
                #define P2P_LOOP  3

        uint8_t operation_mode               :4; /* bits 12-15 */
#else
        uint8_t operation_mode               :4; /* bits 0-3 */

        uint8_t connection_options           :3; /* bits 4-6 */
                #define LOOP      0
                #define P2P       1
                #define LOOP_P2P  2
                #define P2P_LOOP  3
        uint8_t nonpart_if_hard_addr_failed  :1; /* bit 7 */

        uint8_t enable_class2                :1; /* bit 8 */
        uint8_t enable_ack0                  :1; /* bit 9 */
        uint8_t unused_10                    :1; /* bit 10 */
        uint8_t unused_11                    :1; /* bit 11 */

        uint8_t enable_fc_tape               :1; /* bit 12 */
        uint8_t enable_fc_confirm            :1; /* bit 13 */
        uint8_t enable_cmd_q_target_mode     :1; /* bit 14 */
        uint8_t unused_15                    :1; /* bit 15 */
#endif
};

struct qla2x00_special_options
{
#if defined(__BIG_ENDIAN)
	uint8_t	data_rate			:2;
	uint8_t	enable_50_ohm_termination	:1;
	uint8_t	unused_12			:1;

	uint8_t	unused_11			:1;
	uint8_t	unused_10			:1;
	uint8_t	unused_9			:1;
	uint8_t	unused_8			:1;

	uint8_t	disable_auto_plogi_local_loop	:1;
	uint8_t	enable_ooo_frame_handling	:1;
	uint8_t	fcp_rsp_payload			:2;

	uint8_t	unused_3			:1;
	uint8_t	unused_2			:1;
	uint8_t	soft_id_only			:1;
	uint8_t	enable_read_xfr_rdy		:1;
#else
	uint8_t	enable_read_xfr_rdy		:1;
	uint8_t	soft_id_only			:1;
	uint8_t	unused_2			:1;
	uint8_t	unused_3			:1;

	uint8_t	fcp_rsp_payload			:2;
	uint8_t	enable_ooo_frame_handling	:1;
	uint8_t	disable_auto_plogi_local_loop	:1;

	uint8_t	unused_8			:1;
	uint8_t	unused_9			:1;
	uint8_t	unused_10			:1;
	uint8_t	unused_11			:1;

	uint8_t	unused_12			:1;
	uint8_t	enable_50_ohm_termination	:1;
	uint8_t	data_rate			:2;
#endif
};
#define SO_DATA_RATE_1GB	0
#define SO_DATA_RATE_2GB	1
#define SO_DATA_RATE_AUTO	2

typedef struct
{
    uint8_t  version;
        #define ICB_VERSION 1
    uint8_t  reserved_1;
    struct qla2x00_firmware_options firmware_options;
    uint16_t frame_length;
    uint16_t iocb_allocation;
    uint16_t execution_throttle;
    uint8_t  retry_count;
    uint8_t  retry_delay;
/* TODO: Fix ISP2100 portname/nodename */
//#if defined(ISP2100)
//    uint8_t  node_name[WWN_SIZE];
//#else
    uint8_t  port_name[WWN_SIZE];
////#endif
    uint16_t adapter_hard_loop_id;
    uint8_t  inquiry_data;
    uint8_t  login_timeout;
/* TODO: Fix ISP2100 portname/nodename */
//#if defined(ISP2100)
//    uint8_t  reserved_2[8];
//#else
    uint8_t  node_name[WWN_SIZE];
//#endif
    uint16_t request_q_outpointer;
    uint16_t response_q_inpointer;
    uint16_t request_q_length;
    uint16_t response_q_length;
    uint32_t request_q_address[2];
    uint32_t response_q_address[2];
    uint16_t lun_enables;
    uint8_t  command_resource_count;
    uint8_t  immediate_notify_resource_count;
    uint16_t timeout;
    uint16_t reserved_3;
    struct qla2x00_additional_firmware_options additional_firmware_options;
    uint8_t     response_accum_timer;
    uint8_t     interrupt_delay_timer;
    struct qla2x00_special_options special_options;
    uint16_t    reserved_4[13];
}init_cb_t;

/*
 * ISP Get/Set Target Parameters mailbox command control flags.
 */

/*
 * Get Link Status mailbox command return buffer.
 */
typedef struct
{
	uint32_t	link_fail_cnt;
	uint32_t	loss_sync_cnt;
	uint32_t	loss_sig_cnt;
	uint32_t	prim_seq_err_cnt;
	uint32_t	inval_xmit_word_cnt;
	uint32_t	inval_crc_cnt;
} link_stat_t;

/*
 * NVRAM Command values.
 */
#define NV_START_BIT            BIT_2
#define NV_WRITE_OP             (BIT_26+BIT_24)
#define NV_READ_OP              (BIT_26+BIT_25)
#define NV_ERASE_OP             (BIT_26+BIT_25+BIT_24)
#define NV_MASK_OP              (BIT_26+BIT_25+BIT_24)
#define NV_DELAY_COUNT          10

/*
 * NVRAM host parameter.
 */

struct qla2xxx_host_p
{
#if defined(__BIG_ENDIAN)
        uint8_t unused_15               :1;
        uint8_t unused_14               :1;
        uint8_t unused_13               :1;
        uint8_t enable_database_storage :1;

        uint8_t enable_target_reset     :1;
        uint8_t enable_lip_full_login   :1;
        uint8_t enable_lip_reset        :1;
        uint8_t enable_64bit_addressing :1;

        uint8_t enable_extended_logging :1;
        uint8_t pci_parity_disable      :1;
        uint8_t set_cache_line_size_1   :1;
        uint8_t disable_risc_code_load  :1;

        uint8_t enable_selectable_boot  :1;
        uint8_t disable_luns            :1;
        uint8_t disable_bios            :1;
        uint8_t unused_0                :1;
#else
        uint8_t unused_0                :1;
        uint8_t disable_bios            :1;
        uint8_t disable_luns            :1;
        uint8_t enable_selectable_boot  :1;

        uint8_t disable_risc_code_load  :1;
        uint8_t set_cache_line_size_1   :1;
        uint8_t pci_parity_disable      :1;
        uint8_t enable_extended_logging :1;

        uint8_t enable_64bit_addressing :1;
        uint8_t enable_lip_reset        :1;
        uint8_t enable_lip_full_login   :1;
        uint8_t enable_target_reset     :1;

        uint8_t enable_database_storage :1;
        uint8_t unused_13               :1;
        uint8_t unused_14               :1;
        uint8_t unused_15               :1;
#endif
};

struct qla2x00_seriallink_firmware_options
{
#if defined(__BIG_ENDIAN)
        uint16_t unused_15              :1;
        uint16_t unused_14              :1;
        uint16_t unused_13              :1;
        uint16_t unused_12              :1;
        uint16_t unused_11              :1;
		uint16_t output_enable			:1;
		uint16_t output_emphasis_2g		:2;
		uint16_t output_swing_2g		:3;
		uint16_t output_emphasis_1g		:2;
		uint16_t output_swing_1g		:3;
#else
		uint16_t output_swing_1g		:3;
		uint16_t output_emphasis_1g		:2;
		uint16_t output_swing_2g		:3;
		uint16_t output_emphasis_2g		:2;
		uint16_t output_enable			:1;
        uint16_t unused_11              :1;
        uint16_t unused_12              :1;
        uint16_t unused_13              :1;
        uint16_t unused_14              :1;
        uint16_t unused_15              :1;
#endif
};

struct qla2x00_hba_features
{
#if defined(__BIG_ENDIAN)
        uint8_t unused_12               :1;
        uint8_t unused_11               :1;
        uint8_t unused_10               :1;
        uint8_t unused_9                :1;

        uint8_t unused_8                :1;
        uint8_t unused_7                :1;
        uint8_t unused_6                :1;
        uint8_t unused_5                :1;

        uint8_t unused_4                :1;
        uint8_t unused_3                :1;
        uint8_t unused_2                :1;
        uint8_t unused_1                :1;

        uint8_t multi_chip_hba          :1;
        uint8_t buffer_plus_module      :1;
        uint8_t risc_ram_parity         :1;
        uint8_t external_gbic           :1;
#else
        uint8_t external_gbic           :1;
        uint8_t risc_ram_parity         :1;
        uint8_t buffer_plus_module      :1;
        uint8_t multi_chip_hba          :1;

        uint8_t unused_1                :1;
        uint8_t unused_2                :1;
        uint8_t unused_3                :1;
        uint8_t unused_4                :1;

        uint8_t unused_5                :1;
        uint8_t unused_6                :1;
        uint8_t unused_7                :1;
        uint8_t unused_8                :1;

        uint8_t unused_9                :1;
        uint8_t unused_10               :1;
        uint8_t unused_11               :1;
        uint8_t unused_12               :1;
#endif
};

/*
 *  ISP2[23]XX NVRAM structure definitions.
 */

typedef struct
{
    uint8_t     id[4];
    uint8_t     nvram_version;
    uint8_t     reserved_0;

	/*========Start of ICB block ====================*/
    /*
     * NVRAM RISC parameter block
     */

    uint8_t     parameter_block_version;
    uint8_t     reserved_1;

	/* Firmware options */
    struct qla2x00_firmware_options firmware_options;

    uint16_t    frame_payload_size;
    uint16_t    max_iocb_allocation;
    uint16_t    execution_throttle;
    uint8_t     retry_count;
    uint8_t     retry_delay;
    uint8_t     port_name[WWN_SIZE];			/* 0 .. 7 */
    uint16_t    adapter_hard_loop_id;
    uint8_t     inquiry_data;
    uint8_t     login_timeout;

    uint8_t     node_name[WWN_SIZE];			/* 0 .. 7 */

    /* Expanded RISC parameter block */

	/* extended_parameter_options 16bits */
    struct qla2x00_additional_firmware_options additional_firmware_options;

    uint8_t     response_accum_timer;
    uint8_t     interrupt_delay_timer;

	/* special options */
    struct qla2x00_special_options special_options;

    uint16_t    reserved_2[12];
	/*========End of ICB block ====================*/

	/*
	 * Serial Link Control for output Swing and Emphasis
	 */
    struct qla2x00_seriallink_firmware_options	serial_options;

    /*
     * NVRAM host parameter block
     */

    struct qla2xxx_host_p host_p;

    uint8_t     boot_node_name[WWN_SIZE];
    uint8_t     boot_lun_number;
    uint8_t     reset_delay;
    uint8_t     port_down_retry_count;
    uint8_t     reserved_3;

    uint16_t    maximum_luns_per_target;

    uint16_t    reserved_6[7];

    /* Offset 100 */
    uint16_t    reserved_7[25];

    /* Offset 150 */
    uint16_t    reserved_8[25];

    /* Offset 200 */
    uint8_t oem_id;

    uint8_t oem_spare0;

    uint8_t oem_string[6];

    uint8_t oem_part[8];

    uint8_t oem_fru[8];

    uint8_t oem_ec[8];

    /* Offset 232 */
    struct qla2x00_hba_features hba_features;

    uint16_t   reserved_9;
    uint16_t   reserved_10;
    uint16_t   reserved_11;

    uint16_t   reserved_12;
    uint16_t   reserved_13;

    /* Subsystem ID must be at offset 244 */
    uint16_t    subsystem_vendor_id;

    uint16_t    reserved_14;

    /* Subsystem device ID must be at offset 248 */
    uint16_t    subsystem_device_id;

    uint16_t    reserved_15[2];
    uint8_t     reserved_16;
    uint8_t     checksum;
}nvram22_t;

/*
 *  ISP2100 NVRAM structure definitions.
 */

typedef struct
{
    uint8_t     id[4];
    uint8_t     nvram_version;
    uint8_t     reserved_0;

    /*
     * NVRAM RISC parameter block
     */

    uint8_t     parameter_block_version;
    uint8_t     reserved_1;

    struct qla2100_firmware_options firmware_options;

    uint16_t    frame_payload_size;
    uint16_t    max_iocb_allocation;
    uint16_t    execution_throttle;
    uint8_t     retry_count;
    uint8_t     retry_delay;
    uint8_t     node_name[WWN_SIZE];
    uint16_t    adapter_hard_loop_id;
    uint8_t     reserved_2;
    uint8_t     login_timeout;
    uint16_t    reserved_3[4];

    /* Reserved for expanded RISC parameter block */
    uint16_t    reserved_4[16];

    /*
     * NVRAM host parameter block
     */

    struct qla2xxx_host_p host_p;

    uint8_t     boot_node_name[WWN_SIZE];
    uint8_t     boot_lun_number;
    uint8_t     reset_delay;
    uint8_t     port_down_retry_count;
    uint8_t     reserved_5;

    uint16_t    maximum_luns_per_target;

    uint16_t    reserved_6[7];

    /* Offset 100 */
    uint16_t    reserved_7[25];

    /* Offset 150 */
    uint16_t    reserved_8[25];

    /* Offset 200 */
    uint16_t    reserved_9[22];

    /* Subsystem ID must be at offset 244 */
    uint16_t    subsystem_vendor_id;

    uint16_t    reserved_10;

    /* Subsystem device ID must be at offset 248 */
    uint16_t    subsystem_device_id;

    uint16_t    reserved_11[2];
    uint8_t     reserved_12;
    uint8_t     checksum;
}nvram21_t;

/*
 * ISP queue - command entry structure definition.
 */
#define MAX_CMDSZ   16                  /* SCSI maximum CDB size. */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define COMMAND_TYPE    0x11    /* Command entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t handle;                    /* System handle. */
    uint8_t  reserved;
    uint8_t  target;                    /* SCSI ID */
    uint16_t lun;                       /* SCSI LUN */
    uint16_t control_flags;             /* Control flags. */
#define CF_HEAD_TAG		BIT_1
#define CF_ORDERED_TAG		BIT_2
#define CF_SIMPLE_TAG		BIT_3
#define CF_READ			BIT_5
#define CF_WRITE		BIT_6
    uint16_t reserved_1;
    uint16_t timeout;                   /* Command timeout. */
    uint16_t dseg_count;                /* Data segment count. */
    uint8_t  scsi_cdb[MAX_CMDSZ];       /* SCSI command words. */
    uint32_t byte_count;                /* Total byte count. */
    uint32_t dseg_0_address;            /* Data segment 0 address. */
    uint32_t dseg_0_length;             /* Data segment 0 length. */
    uint32_t dseg_1_address;            /* Data segment 1 address. */
    uint32_t dseg_1_length;             /* Data segment 1 length. */
    uint32_t dseg_2_address;            /* Data segment 2 address. */
    uint32_t dseg_2_length;             /* Data segment 2 length. */
}cmd_entry_t;

/*
 * ISP queue - 64-Bit addressing, command entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define COMMAND_A64_TYPE 0x19   /* Command A64 entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t handle;                    /* System handle. */
    uint8_t  reserved;
    uint8_t  target;                    /* SCSI ID */
    uint16_t lun;                       /* SCSI LUN */
    uint16_t control_flags;             /* Control flags. */
    uint16_t reserved_1;
    uint16_t timeout;                   /* Command timeout. */
    uint16_t dseg_count;                /* Data segment count. */
    uint8_t  scsi_cdb[MAX_CMDSZ];       /* SCSI command words. */
    uint32_t byte_count;                /* Total byte count. */
    uint32_t dseg_0_address[2];         /* Data segment 0 address. */
    uint32_t dseg_0_length;             /* Data segment 0 length. */
    uint32_t dseg_1_address[2];         /* Data segment 1 address. */
    uint32_t dseg_1_length;             /* Data segment 1 length. */
}cmd_a64_entry_t, request_t;

/*
 * ISP queue - continuation entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define CONTINUE_TYPE   0x02    /* Continuation entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t reserved;
    uint32_t dseg_0_address;            /* Data segment 0 address. */
    uint32_t dseg_0_length;             /* Data segment 0 length. */
    uint32_t dseg_1_address;            /* Data segment 1 address. */
    uint32_t dseg_1_length;             /* Data segment 1 length. */
    uint32_t dseg_2_address;            /* Data segment 2 address. */
    uint32_t dseg_2_length;             /* Data segment 2 length. */
    uint32_t dseg_3_address;            /* Data segment 3 address. */
    uint32_t dseg_3_length;             /* Data segment 3 length. */
    uint32_t dseg_4_address;            /* Data segment 4 address. */
    uint32_t dseg_4_length;             /* Data segment 4 length. */
    uint32_t dseg_5_address;            /* Data segment 5 address. */
    uint32_t dseg_5_length;             /* Data segment 5 length. */
    uint32_t dseg_6_address;            /* Data segment 6 address. */
    uint32_t dseg_6_length;             /* Data segment 6 length. */
}cont_entry_t;

/*
 * ISP queue - 64-Bit addressing, continuation entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define CONTINUE_A64_TYPE 0x0A  /* Continuation A64 entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t dseg_0_address[2];         /* Data segment 0 address. */
    uint32_t dseg_0_length;             /* Data segment 0 length. */
    uint32_t dseg_1_address[2];         /* Data segment 1 address. */
    uint32_t dseg_1_length;             /* Data segment 1 length. */
    uint32_t dseg_2_address[2];         /* Data segment 2 address. */
    uint32_t dseg_2_length;             /* Data segment 2 length. */
    uint32_t dseg_3_address[2];         /* Data segment 3 address. */
    uint32_t dseg_3_length;             /* Data segment 3 length. */
    uint32_t dseg_4_address[2];         /* Data segment 4 address. */
    uint32_t dseg_4_length;             /* Data segment 4 length. */
}cont_a64_entry_t;

#define STS_SENSE_BUF_LEN      32
/*
 * ISP queue - status entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define STATUS_TYPE     0x03    /* Status entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
        #define RF_INV_E_ORDER  BIT_5   /* Invalid entry order. */
        #define RF_INV_E_COUNT  BIT_4   /* Invalid entry count. */
        #define RF_INV_E_PARAM  BIT_3   /* Invalid entry parameter. */
        #define RF_INV_E_TYPE   BIT_2   /* Invalid entry type. */
        #define RF_BUSY         BIT_1   /* Busy */
    uint32_t handle;                    /* System handle. */
    uint16_t scsi_status;               /* SCSI status. */
    uint16_t comp_status;               /* Completion status. */
    uint16_t state_flags;               /* State flags. */
    uint16_t status_flags;              /* Status flags. */
    #define IOCBSTAT_SF_LOGO	BIT_13	/* logo after 2 abts w/no */
    					/*   response (2 sec) */
    uint16_t rsp_info_len;              /* Response Info Length. */
    uint16_t req_sense_length;          /* Request sense data length. */
    uint32_t residual_length;           /* Residual transfer length. */
    uint8_t  rsp_info[8];               /* FCP response information. */
    uint8_t  req_sense_data[STS_SENSE_BUF_LEN];/* Request sense data. */
}sts_entry_t, response_t;

/*
 * ISP queue - marker entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define MARKER_TYPE     0x04    /* Marker entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved;
    uint8_t  target;                    /* SCSI ID */
    uint8_t  modifier;                  /* Modifier (7-0). */
        #define MK_SYNC_ID_LUN      0   /* Synchronize ID/LUN */
        #define MK_SYNC_ID          1   /* Synchronize ID */
        #define MK_SYNC_ALL         2   /* Synchronize all ID/LUN */
        #define MK_SYNC_LIP         3   /* Synchronize all ID/LUN, */
                                        /* clear port changed, */
                                        /* use sequence number. */
    uint8_t  reserved_1;
    uint16_t sequence_number;           /* Sequence number of event */
    uint16_t lun;                       /* SCSI LUN */
    uint8_t  reserved_2[48];
}mrk_entry_t;

/*
 * ISP queue - enable LUN entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define ENABLE_LUN_TYPE 0x0B    /* Enable LUN entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  reserved_1;
    uint16_t reserved_2;
    uint32_t reserved_3;
    uint8_t  status;
    uint8_t  reserved_4;
    uint8_t  command_count;             /* Number of ATIOs allocated. */
    uint8_t  immed_notify_count;        /* Number of Immediate Notify */
                                        /* entries allocated. */
    uint16_t reserved_5;
    uint16_t timeout;                   /* 0 = 30 seconds, 0xFFFF = disable */
    uint16_t reserved_6[20];
}elun_entry_t;

/*
 * ISP queue - modify LUN entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define MODIFY_LUN_TYPE 0x0C    /* Modify LUN entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  reserved_1;
    uint8_t  operators;
    uint8_t  reserved_2;
    uint32_t reserved_3;
    uint8_t  status;
    uint8_t  reserved_4;
    uint8_t  command_count;             /* Number of ATIOs allocated. */
    uint8_t  immed_notify_count;        /* Number of Immediate Notify */
                                        /* entries allocated. */
    uint16_t reserved_5;
    uint16_t timeout;                   /* 0 = 30 seconds, 0xFFFF = disable */
    uint16_t reserved_7[20];
}modify_lun_entry_t;

/*
 * ISP queue - immediate notify entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define IMMED_NOTIFY_TYPE 0x0D  /* Immediate notify entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  initiator_id;
    uint8_t  reserved_1;
    uint8_t  target_id;
    uint32_t reserved_2;
    uint16_t status;
    uint16_t task_flags;
    uint16_t seq_id;
    uint16_t reserved_5[11];
    uint16_t scsi_status;
    uint8_t  sense_data[18];
}notify_entry_t;

/*
 * ISP queue - notify acknowledge entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define NOTIFY_ACK_TYPE 0x0E    /* Notify acknowledge entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  initiator_id;
    uint8_t  reserved_1;
    uint8_t  target_id;
    uint16_t flags;
    uint16_t reserved_2;
    uint16_t status;
    uint16_t task_flags;
    uint16_t seq_id;
    uint16_t reserved_3[21];
}nack_entry_t;

/*
 * ISP queue - Accept Target I/O (ATIO) entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define ACCEPT_TGT_IO_TYPE 0x16 /* Accept target I/O entry. */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  initiator_id;
    uint16_t exchange_id;
    uint16_t flags;
    uint16_t status;
    uint8_t  reserved_1;
    uint8_t  task_codes;
    uint8_t  task_flags;
    uint8_t  execution_codes;
    uint8_t  cdb[MAX_CMDSZ];
    uint32_t data_length;
    uint16_t lun;
    uint16_t reserved_2A;
    uint16_t scsi_status;
    uint8_t  sense_data[18];
}atio_entry_t;

/*
 * ISP queue - Continue Target I/O (CTIO) entry for status mode 0
 *             structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                  /* Entry type. */
        #define CONTINUE_TGT_IO_TYPE 0x17 /* CTIO entry */
    uint8_t  entry_count;                 /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  initiator_id;
    uint16_t exchange_id;
    uint16_t flags;
    uint16_t status;
    uint16_t timeout;                   /* 0 = 30 seconds, 0xFFFF = disable */
    uint16_t dseg_count;                /* Data segment count. */
    uint32_t relative_offset;
    uint32_t residual;
    uint16_t reserved_1[3];
    uint16_t scsi_status;
    uint32_t transfer_length;
    uint32_t dseg_0_address;            /* Data segment 0 address. */
    uint32_t dseg_0_length;             /* Data segment 0 length. */
    uint32_t dseg_1_address;            /* Data segment 1 address. */
    uint32_t dseg_1_length;             /* Data segment 1 length. */
    uint32_t dseg_2_address;            /* Data segment 2 address. */
    uint32_t dseg_2_length;             /* Data segment 2 length. */
}ctio_entry_t;

/*
 * ISP queue - CTIO returned entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define CTIO_RET_TYPE   0x17    /* CTIO return entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  initiator_id;
    uint16_t exchange_id;
    uint16_t flags;
    uint16_t status;
    uint16_t timeout;                   /* 0 = 30 seconds, 0xFFFF = disable */
    uint16_t dseg_count;                /* Data segment count. */
    uint32_t relative_offset;
    uint32_t residual;
    uint16_t reserved_1[8];
    uint16_t scsi_status;
    uint8_t  sense_data[18];
}ctio_ret_entry_t;

/*
 * ISP queue - CTIO A64 entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define CTIO_A64_TYPE 0x1F      /* CTIO A64 entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  initiator_id;
    uint16_t exchange_id;
    uint16_t flags;
    uint16_t status;
    uint16_t timeout;                   /* 0 = 30 seconds, 0xFFFF = disable */
    uint16_t dseg_count;                /* Data segment count. */
    uint32_t relative_offset;
    uint32_t residual;
    uint16_t reserved_1[3];
    uint16_t scsi_status;
    uint32_t transfer_length;
    uint32_t dseg_0_address[2];         /* Data segment 0 address. */
    uint32_t dseg_0_length;             /* Data segment 0 length. */
    uint32_t dseg_1_address[2];         /* Data segment 1 address. */
    uint32_t dseg_1_length;             /* Data segment 1 length. */
}ctio_a64_entry_t;

/*
 * ISP queue - CTIO returned entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define CTIO_A64_RET_TYPE 0x1F  /* CTIO A64 returned entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint8_t  reserved_8;
    uint8_t  initiator_id;
    uint16_t exchange_id;
    uint16_t flags;
    uint16_t status;
    uint16_t timeout;                   /* 0 = 30 seconds, 0xFFFF = disable */
    uint16_t dseg_count;                /* Data segment count. */
    uint32_t relative_offset;
    uint32_t residual;
    uint16_t reserved_1[8];
    uint16_t scsi_status;
    uint8_t  sense_data[18];
}ctio_a64_ret_entry_t;

/*
 * ISP queue - Status Contination entry structure definition.
 */
#define EXT_STS_SENSE_BUF_LEN      60
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define STATUS_CONT_TYPE 0x10   /* Status contination entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  reserved;
    uint8_t  entry_status;              /* Entry Status. */
    uint8_t  req_sense_data[EXT_STS_SENSE_BUF_LEN];   /* Extended sense data. */
}sts_cont_entry_t;

/*
 * ISP queue - Command Set entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define CMD_SET_TYPE 0x18       /* Command set entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint16_t reserved;
    uint16_t status;
    uint16_t control_flags;             /* Control flags. */
    uint16_t count;
    uint32_t iocb_0_address;
    uint32_t iocb_1_address;
    uint32_t iocb_2_address;
    uint32_t iocb_3_address;
    uint32_t iocb_4_address;
    uint32_t iocb_5_address;
    uint32_t iocb_6_address;
    uint32_t iocb_7_address;
    uint32_t iocb_8_address;
    uint32_t iocb_9_address;
    uint32_t iocb_10_address;
    uint32_t iocb_11_address;
}cmd_set_entry_t;

/*
 * ISP queue - Command Set A64 entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define CMD_SET_TYPE 0x18       /* Command set entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t sys_define_2;              /* System defined. */
    uint16_t reserved;
    uint16_t status;
    uint16_t control_flags;             /* Control flags. */
    uint16_t count;
    uint32_t iocb_0_address[2];
    uint32_t iocb_1_address[2];
    uint32_t iocb_2_address[2];
    uint32_t iocb_3_address[2];
    uint32_t iocb_4_address[2];
    uint32_t iocb_5_address[2];
}cmd_set_a64_entry_t;

/* 4.11
 * ISP queue - Command Set entry structure definition.
 */
typedef struct
{
    uint8_t  entry_type;                /* Entry type. */
        #define MS_IOCB_TYPE 0x29       /*  Management Server IOCB entry */
    uint8_t  entry_count;               /* Entry count. */
    uint8_t  sys_define;                /* System defined. */
    uint8_t  entry_status;              /* Entry Status. */
    uint32_t handle1;                   /* System handle. */
    uint8_t  reserved;
    uint8_t  loop_id;
    uint16_t status;
    uint16_t control_flags;             /* Control flags. */
    uint16_t reserved2;
    uint16_t timeout;
    uint16_t cmd_dsd_count;
    uint16_t total_dsd_count;
    uint8_t  type;
    uint8_t  r_ctl;
    uint16_t rx_id;
    uint16_t reserved3;
    uint32_t handle2;
    uint32_t rsp_bytecount;
    uint32_t req_bytecount;
    uint32_t dseg_req_address[2];         /* Data segment 0 address. */
    uint32_t dseg_req_length;             /* Data segment 0 length. */
    uint32_t dseg_rsp_address[2];         /* Data segment 1 address. */
    uint32_t dseg_rsp_length;             /* Data segment 1 length. */
} ms_iocb_entry_t;


/*
 * ISP request and response queue entry sizes
 */
#define RESPONSE_ENTRY_SIZE     (sizeof(response_t))
#define REQUEST_ENTRY_SIZE      (sizeof(request_t))

/*
 * ISP status entry - completion status definitions.
 */
#define CS_COMPLETE         0x0         /* No errors */
#define CS_INCOMPLETE       0x1         /* Incomplete transfer of cmd. */
#define CS_DMA              0x2         /* A DMA direction error. */
#define CS_TRANSPORT        0x3         /* Transport error. */
#define CS_RESET            0x4         /* SCSI bus reset occurred */
#define CS_ABORTED          0x5         /* System aborted command. */
#define CS_TIMEOUT          0x6         /* Timeout error. */
#define CS_DATA_OVERRUN     0x7         /* Data overrun. */
#define CS_DATA_UNDERRUN    0x15        /* Data Underrun. */
#define CS_ABORT_MSG        0xE         /* Target rejected abort msg. */
#define CS_DEV_RESET_MSG    0x12        /* Target rejected dev rst msg. */
#define CS_PORT_UNAVAILABLE 0x28        /* Port unavailable (selection timeout) */
#define CS_PORT_LOGGED_OUT  0x29        /* Port Logged Out */
#define CS_PORT_CONFIG_CHG  0x2A        /* Port Configuration Changed */
#define CS_PORT_BUSY        0x2B        /* Port Busy */
#define CS_BAD_PAYLOAD      0x80        /* Driver defined */
#define CS_UNKNOWN          0x81        /* Driver defined */
#define CS_RETRY            0x82        /* Driver defined */
#define CS_QUEUE_FULL	    0x1c        /* Target queue full*/

/*
 * ISP status entry - SCSI status byte bit definitions.
 */
#define SS_MASK			0xfff /* Mask off reserved bits BIT_12-BIT_15*/
#define SS_RESIDUAL_UNDER       BIT_11
#define SS_RESIDUAL_OVER        BIT_10
#define SS_SENSE_LEN_VALID      BIT_9
#if defined(ISP2100)
#define SS_RESIDUAL_LEN_VALID   BIT_8
#else
#define SS_RESPONSE_INFO_LEN_VALID BIT_8
#endif

#define SS_RESERVE_CONFLICT     (BIT_4 | BIT_3)
#define SS_BUSY_CONDITION       BIT_3
#define SS_CONDITION_MET        BIT_2
#define SS_CHECK_CONDITION      BIT_1

/*
 * ISP target entries - Flags bit definitions.
 */
#define OF_RESET            BIT_5       /* Reset LIP flag */
#define OF_DATA_IN          BIT_6       /* Data in to initiator */
                                        /*  (data from target to initiator) */
#define OF_DATA_OUT         BIT_7       /* Data out from initiator */
                                        /*  (data from initiator to target) */
#define OF_NO_DATA          (BIT_7 | BIT_6)
#define OF_INC_RC           BIT_8       /* Increment command resource count */
#define OF_FAST_POST        BIT_9       /* Enable mailbox fast posting. */
#define OF_SSTS             BIT_15      /* Send SCSI status */

/*
 * Target Read/Write buffer structure.
 */
#define TARGET_DATA_OFFSET  4
#define TARGET_DATA_SIZE    0x2000      /* 8K */
#define TARGET_INQ_OFFSET   (TARGET_DATA_OFFSET + TARGET_DATA_SIZE)
#define TARGET_SENSE_SIZE   18
#define TARGET_BUF_SIZE     36

#define TARGET_OFFLINE  BIT_0
/*
 * 24 bit port ID type definition.
 */
typedef union {
	uint32_t	b24  : 24;

	struct {
		uint8_t d_id[3];
		uint8_t rsvd_1;
	}r;

	struct {
		uint8_t al_pa;
		uint8_t area;
		uint8_t domain;
		uint8_t rsvd_1;
	}b;
} port_id_t;

typedef struct
{
    port_id_t d_id;
    uint8_t   name[WWN_SIZE];
    uint8_t   wwn[WWN_SIZE];          /* port name */
    uint16_t  loop_id;
    uint16_t   flag;
  /* flags bits defined as follows */
#define DEV_PUBLIC          BIT_0
#define DEV_LUNMASK_SET     BIT_1  /* some LUNs masked for this device */
#define	DEV_TAPE_DEVICE		BIT_2
#define	DEV_RELOGIN	        BIT_3
#define	DEV_PORT_DOWN	    BIT_4
#define	DEV_CONFIGURED    	BIT_5
#define	DEV_ABSENCE    		BIT_6
#define	DEV_RETURN    		BIT_7
#define	DEV_INITIATOR  		BIT_8
#define	DEV_FLAG_VSA  		BIT_9
	int			port_login_retry_count;
    uint8_t  port_timer;
}fcdev_t;

/* New device name list struct; used in configure_fabric. */
struct new_dev {
    port_id_t  d_id;
    uint8_t    name[WWN_SIZE];
    uint8_t    wwn[WWN_SIZE];          /* port name */
};
#define LOGOUT_PERFORMED  0x01
/*
 * Inquiry command structure.
 */
#define INQ_SCSI_OPCODE	0x12
#define	INQ_DATA_SIZE	36

typedef struct {
	union {
		cmd_entry_t cmd;
		sts_entry_t rsp;
	} p;
	uint8_t inq[INQ_DATA_SIZE];
} inq_cmd_rsp_t;

/*
 * Report LUN command structure.
 */
#define RPT_LUN_SCSI_OPCODE	0xA0
#define CHAR_TO_SHORT(a, b)	(uint16_t)((uint8_t)b << 8 | (uint8_t)a)

typedef struct {
	uint32_t	len;
	uint32_t	rsrv;
} rpt_hdr_t;

typedef struct {
	struct {
		uint8_t		b : 6;
		uint8_t		address_method : 2;
	} msb;
	uint8_t		lsb;
	uint8_t		unused[6];
} rpt_lun_t;

typedef struct {
	rpt_hdr_t	hdr;
	rpt_lun_t	lst[MAX_LUNS];
} rpt_lun_lst_t;

typedef struct {
	union {
		cmd_entry_t cmd;
		sts_entry_t rsp;
	} p;
	rpt_lun_lst_t list;
} rpt_lun_cmd_rsp_t;

/*
 * SCSI Target Queue structure
 */
typedef struct os_tgt {
	struct os_lun		*olun[MAX_LUNS]; /* LUN context pointer. */
	uint8_t			port_down_retry_count;
	struct scsi_qla_host	*ha;
    	uint32_t		down_timer;

	/* Persistent binding information */
	port_id_t		d_id;
	uint8_t			node_name[WWN_SIZE];
	uint8_t			port_name[WWN_SIZE];
	struct fc_port		*vis_port;

	uint8_t			flags;
#define	TGT_BUSY		BIT_0		/* Reached hi-water mark */
#define	TGT_TAGGED_QUEUE	BIT_1		/* Tagged queuing. */
} os_tgt_t;

/*
 * SCSI LUN Queue structure
 */
typedef struct os_lun {
	struct fc_lun	*fclun;		/* FC LUN context pointer. */
    	spinlock_t      q_lock;       /* Lun Lock */

	u_long		io_cnt;     /* total xfer count since boot */
	u_long		out_cnt;    /* total outstanding IO count */
	u_long		w_cnt;      /* total writes */
	u_long		r_cnt;      /* total reads */
	u_long		act_time;   /* total active time  */
	u_long		resp_time;  /* total response time (target + f/w) */

	unsigned long	q_flag;
#define	LUN_MPIO_BUSY		2	/* Lun is changing paths  */
#define	LUN_SCSI_SCAN_DONE	BIT_3	/* indicates the scsi scan is done */
#define	LUN_EXEC_DELAYED	7	/* Lun execution is delayed */

	u_long		q_timeout;           /* total command timeouts */
	atomic_t	q_timer;  /* suspend timer */
	uint32_t	q_count;	/* current count */
	uint32_t	q_max;		/* maxmum count lun can be suspended */
	uint8_t		q_state;	/* lun State */
#define	LUN_STATE_READY	1	/* indicates the lun is ready for i/o */
#define	LUN_STATE_RUN	2	/* indicates the lun has a timer running */
#define	LUN_STATE_WAIT	3	/* indicates the lun is suspended */
#define	LUN_STATE_TIMEOUT  4	/* indicates the lun has timed out */

} os_lun_t;


/* LUN BitMask structure definition, array of 32bit words,
 * 1 bit per lun.  When bit == 1, the lun is masked.
 * Most significant bit of mask[0] is lun 0, bit 24 is lun 7.
 */
typedef struct lun_bit_mask {
	/* Must allocate at least enough bits to accomodate all LUNs */
#if ((MAX_FIBRE_LUNS & 0x7) == 0)
	UINT8	mask[MAX_FIBRE_LUNS >> 3];
#else
	uint8_t	mask[(MAX_FIBRE_LUNS + 8) >> 3];
#endif
} lun_bit_mask_t;

/*
 * Fibre channel port structure.
 */
typedef struct fc_port {
	struct fc_port		*next;
	struct fc_lun		*fclun;
	struct scsi_qla_host	*ha;
	struct scsi_qla_host	*vis_ha; /* only used when suspending lun */
	port_id_t		d_id;
	uint16_t		loop_id;
	uint16_t		old_loop_id;
	int16_t			lun_cnt;
	int16_t			dev_id;	/* index in fc_dev table */
#define FC_NO_LOOP_ID		0x100
	uint8_t			node_name[WWN_SIZE];	/* Big Endian. */
	uint8_t			port_name[WWN_SIZE];	/* Big Endian. */
	uint8_t			mp_byte;	/* multi-path byte (not used) */
    	uint8_t		cur_path;		/* current path id */
	int			port_login_retry_count;
	int		login_retry;
	atomic_t		state;		/* port state */
#define FC_DEVICE_DEAD		1
#define FC_DEVICE_LOST		2
#define FC_ONLINE		3
#define FC_LOGIN_NEEDED		4

	uint8_t			flags;
#define	FC_FABRIC_DEVICE	BIT_0
#define	FC_TAPE_DEVICE		BIT_1
#define	FC_INITIATOR_DEVICE	BIT_2
#define	FC_CONFIG		BIT_3
#define	FC_VSA			BIT_4
#define	FC_HD_DEVICE		BIT_5
#define	FC_SUPPORT_RPT_LUNS	BIT_6
	atomic_t	port_down_timer;
	lun_bit_mask_t	lun_mask;
} fc_port_t;

/*
 * Fibre channel LUN structure.
 */
typedef struct fc_lun {
	struct fc_lun		*next;
	fc_port_t		*fcport;
	uint16_t		lun;
	uint8_t			max_path_retries;
	uint8_t			flags;
#define	FC_DISCON_LUN		BIT_0
	u_long			kbytes;
} fc_lun_t;

typedef struct
{
    uint8_t   in_use;
}fabricid_t;

typedef struct {
	struct list_head	list;

	uint8_t		node_name[WWN_SIZE];
	uint8_t		port_name[WWN_SIZE];
	port_id_t	d_id;
	uint16_t	loop_id;
} fc_initiator_t;

/*
 * Registered State Change Notification structures.
 */
typedef struct {
    port_id_t d_id;
    uint8_t format;
} rscn_t;

/*
 * Flash Database structures.
 */
#define FLASH_DATABASE_0        0x1c000
#define FLASH_DATABASE_1        0x18000
#define FLASH_DATABASE_VERSION  1

typedef struct
{
    uint32_t seq;
    uint8_t  version;
    uint8_t  checksum;
    uint16_t size;
    uint8_t  spares[8];
}flash_hdr_t;

typedef struct
{
    uint8_t name[WWN_SIZE];
    uint8_t  spares[8];
}flash_node_t;

typedef struct
{
    flash_hdr_t  hdr;
    flash_node_t node[MAX_FIBRE_DEVICES];
}flash_database_t;

/*
 * SNS structures.
 */
#define	RFT_CMD_SIZE	60
#define	RFT_DATA_SIZE	16

#define	RFF_CMD_SIZE	32
#define	RFF_DATA_SIZE	16

#define	GAN_CMD_SIZE	28
#define	GAN_DATA_SIZE	(620 + 16)

#define	GID_CMD_SIZE	28
#define	GID_DATA_SIZE	(MAX_FIBRE_DEVICES * 4 + 16)

#define	GPN_CMD_SIZE	28
#define	GPN_DATA_SIZE	(8 + 16)

#define	GNN_CMD_SIZE	28
#define	GNN_DATA_SIZE	(8 + 16)

typedef struct {
	union {
		struct {
			uint16_t buffer_length;
			uint16_t reserved_1;
			uint32_t buffer_address[2];
			uint16_t subcommand_length;
			uint16_t reserved_2;
			uint16_t subcommand;
			uint16_t size;
			uint32_t reserved_3;
			uint8_t param[36];
		} cmd;

		uint8_t gan_rsp[GAN_DATA_SIZE];
		uint8_t gid_rsp[GID_DATA_SIZE];
		uint8_t gpn_rsp[GPN_DATA_SIZE];
		uint8_t gnn_rsp[GNN_DATA_SIZE];
		uint8_t rft_rsp[RFT_DATA_SIZE];
		uint8_t rff_rsp[RFF_DATA_SIZE];
	} p;
} sns_cmd_rsp_t;

/*
 * SNS request/response structures for GP_IDNN.
 */
typedef struct
{
    uint8_t    controlbyte;
    uint8_t    port_id[3];
    uint32_t   reserved;
    uint8_t    nodename[WWN_SIZE];
}port_data_t;

#if defined(ISP2100)
#define GP_IDNN_LENGTH  (126 * sizeof(port_data_t)) + 16
#else
#define GP_IDNN_LENGTH  (256 * sizeof(port_data_t)) + 16
#endif

typedef union
{
    struct
    {
        uint16_t buffer_length;
        uint16_t reserved;
        uint32_t buffer_address[2];
        uint16_t subcommand_length;
        uint16_t reserved_1;
        uint16_t subcommand;
        uint16_t length;
        uint32_t reserved2;
        uint32_t protocol;
        uint8_t  param[GP_IDNN_LENGTH - 28];
    }req;

    struct
    {
        uint8_t revision;
        uint8_t inid[3];
        uint8_t fcstype;
        uint8_t subtype;
        uint8_t options;
        uint8_t reserved;
        uint16_t response;
        uint16_t residual;
        uint8_t reserved1;
        uint8_t reason_code;
        uint8_t explanation_code;
        uint8_t vendor_unique;
#if defined(ISP2100)
        port_data_t  port_data[126];
#else
        port_data_t  port_data[256];
#endif
    }rsp;

}gp_idnn_t;

#if defined(ISP2100)
#define GN_LIST_LENGTH  126 * sizeof(port_list_entry_t)
#else
#define GN_LIST_LENGTH  256 * sizeof(port_list_entry_t)
#endif
/*
 * Structure used in Get Port List mailbox command (0x75).
 */
typedef struct
{
    uint8_t    name[WWN_SIZE];
    uint16_t   loop_id;
}port_list_entry_t;

/*
 * Structure used for device info.
 */
typedef struct
{
    uint8_t    name[WWN_SIZE];
    uint8_t    wwn[WWN_SIZE];
    uint16_t   loop_id;
    uint8_t    port_id[3];
}device_data_t;

/* Mailbox command completion status */
#define MBS_PORT_ID_IN_USE              0x4007
#define MBS_LOOP_ID_IN_USE              0x4008
#define MBS_ALL_LOOP_IDS_IN_USE         0x4009
#define MBS_NAME_SERVER_NOT_LOGGED_IN   0x400A


#define MAX_IOCTL_WAIT_THREADS	32
typedef struct _wait_q_t {
	uint8_t			flags;
#define WQ_IN_USE	0x1

	struct semaphore	wait_q_sem;
	struct _wait_q_t	*pnext;
} wait_q_t;

typedef struct hba_ioctl{

	/* Ioctl cmd serialization */
	uint16_t	access_bits; /* bits should be used atomically */
#define IOCTL_ACTIVE	1 /* first bit */
#define IOCTL_WANT	2 /* 2nd bit */

	spinlock_t	wait_q_lock; /* IOCTL wait_q Queue Lock */
	wait_q_t	wait_q_arr[MAX_IOCTL_WAIT_THREADS];
	wait_q_t	*wait_q_head;
	wait_q_t	*wait_q_tail;

	/* Passthru cmd/completion */
	struct semaphore	cmpl_sem;
	struct timer_list	cmpl_timer;
	uint8_t		ioctl_tov;
	uint8_t		SCSIPT_InProgress;
	uint8_t		MSIOCB_InProgress;

	os_tgt_t	*ioctl_tq;
	os_lun_t	*ioctl_lq;

	/* AEN queue */
	void		*aen_tracking_queue;/* points to async events buffer */
	uint8_t		aen_q_head;	/* index to the current head of q */
	uint8_t		aen_q_tail;	/* index to the current tail of q */

	/* Misc. */
	uint32_t	flags;
#define	IOCTL_OPEN			BIT_0
#define	IOCTL_AEN_TRACKING_ENABLE	BIT_1
	uint8_t		*scrap_mem;	/* per ha scrap buf for ioctl usage */
	uint32_t	scrap_mem_size; /* total size */
	uint32_t	scrap_mem_used; /* portion used */

} hba_ioctl_context;

/* Mailbox command semaphore queue for command serialization */
typedef struct _mbx_cmdq_t {
	struct semaphore	cmd_sem;
	struct _mbx_cmdq_t	*pnext;
} mbx_cmdq_t;

#if defined(QLA2XXX_CONFIG_BOOTIMG)
#include	<linux/notifier.h>	/* For the notifier block. */
#endif

/*
 * Linux Host Adapter structure
 */
typedef struct scsi_qla_host
{
#if defined(QLA2XXX_CONFIG_BOOTIMG)
	/*
	 * This is used for the bootimg notifier block, it MUST be
	 * first in the structre so this structure can be easily
	 * obtained from the notifier block.
	 */
	struct notifier_block bootimg_notifier;
#endif
	/* Linux adapter configuration data */
	struct Scsi_Host *host;             /* pointer to host data */
	struct scsi_qla_host   *next;
	device_reg_t     *iobase;           /* Base Memory-mapped I/O address */
	struct pci_dev   *pdev;
	uint8_t          devnum;
	volatile unsigned char  *mmpbase;      /* memory mapped address */
	u_long            host_no;
	u_long            instance;
	uint8_t           revision;
	uint8_t           ports;
	u_long            actthreads;
	u_long            ipreq_cnt;
	u_long            qthreads;
	u_long            spurious_int;
	uint32_t        total_isr_cnt;		/* Interrupt count */
	uint32_t        total_isp_aborts;	/* controller err cnt */
	uint32_t        total_lip_cnt;		/* LIP cnt */
	uint32_t	total_dev_errs;		/* device error cnt */
	uint32_t	total_ios;		/* IO cnt */
	uint64_t	total_bytes;		/* xfr byte cnt */

	/* Adapter I/O statistics for failover */
	uint64_t	IosRequested;
	uint64_t	BytesRequested;
	uint64_t	IosExecuted;
	uint64_t	BytesExecuted;

	uint32_t         device_id;
	uint16_t         subsystem_vendor;
	uint16_t         subsystem_device;
 
	/* ISP connection configuration data */
	uint16_t         max_public_loop_ids;
	uint16_t         min_external_loopid; /* First external loop Id */
	uint8_t          current_topology; /* Current ISP configuration */
	uint8_t          prev_topology;    /* Previous ISP configuration */
                     #define ISP_CFG_NL     1
                     #define ISP_CFG_N      2
                     #define ISP_CFG_FL     4
                     #define ISP_CFG_F      8
	uint8_t         id;                 /* Host adapter SCSI id */
	uint16_t        loop_id;       /* Host adapter loop id */
	port_id_t       d_id;           /* Host adapter port id */

	uint8_t         operating_mode;  /* current F/W operating mode */
	                                 /* 0 - LOOP, 1 - P2P, 2 - LOOP_P2P,
	                                  * 3 - P2P_LOOP
	                                  */
	uint8_t         active_fc4_types;/* active fc4 types */
	uint8_t         current_speed;   /* current F/W operating speed */

	/* NVRAM configuration data */
	uint16_t        loop_reset_delay;   /* Loop reset delay. */
	uint16_t        hiwat;              /* High water mark per device. */
	uint16_t        execution_throttle; /* queue depth */ 
	uint16_t        minimum_timeout;    /* Minimum timeout. */
	uint8_t         retry_count;
	uint8_t         login_timeout;
	int             port_down_retry_count;
	uint8_t         loop_down_timeout;
	uint16_t        max_probe_luns;
	uint16_t        max_luns;
	uint16_t        max_targets;
	
	/* Fibre Channel Device List. */
	fc_port_t		*fcport;

	/* OS target queue pointers. */
	os_tgt_t		*otgt[MAX_FIBRE_DEVICES];

	/* Fibre Channel Device Database and LIP sequence. */
	fcdev_t           fc_db[MAX_FIBRE_DEVICES]; /* Driver database. */
	uint32_t          flash_db;         /* Flash database address in use. */
	fabricid_t        fabricid[MAX_FIBRE_DEVICES]; /* Fabric ids table . */
	uint32_t          flash_seq;        /* Flash database seq # in use. */
	volatile uint16_t lip_seq;          /* LIP sequence number. */
	
	/* Tracks host adapters we find */	
	struct list_head	fcinitiators;	/* Initiator database */
    
	  /* RSCN queue. */
	rscn_t rscn_queue[MAX_RSCN_COUNT];
	uint8_t rscn_in_ptr;
	uint8_t rscn_out_ptr;

 
	/* Doneq bottom half handler */
	struct tasklet_struct run_qla_task;
	/*
	 * Need to hold the list_lock with irq's disabled in order to
	 * access the following list.
	 * This list_lock is of lower priority than the io_request_lock.
	 */
	/*********************************************************/
        spinlock_t              list_lock;      /* lock to guard lists which 
						   hold srb_t's*/
        struct list_head        retry_queue;    /* watchdog queue */
        struct list_head        done_queue;     /* job on done queue */
        struct list_head        failover_queue; /* failover list link. */
	struct list_head        free_queue;     /* SRB free queue */
	struct list_head        scsi_retry_queue;     /* SCSI retry queue */
	
	struct list_head        pending_queue;	/* SCSI command pending queue */

        /*********************************************************/

	/* This spinlock is used to protect "io transactions", you must	
	 * aquire it before doing any IO to the card, eg with RD_REG*() and
	 * WRT_REG*() for the duration of your entire commandtransaction.
	 *
	 * This spinlock is of lower priority than the io request lock.
	 */

	spinlock_t		hardware_lock;

	/* Linux kernel thread */
	struct task_struct  *dpc_handler;     /* kernel thread */
	struct semaphore    *dpc_wait;       /* DPC waits on this semaphore */
	struct semaphore    *dpc_notify;     /* requester waits for DPC on this semaphore */
	struct semaphore    dpc_sem;       /* DPC's semaphore */
	uint8_t dpc_active;                  /* DPC routine is active */

	/* Received ISP mailbox data. */
	volatile uint16_t mailbox_out[MAILBOX_REGISTER_COUNT];

	/* Outstandings ISP commands. */
	srb_t           *outstanding_cmds[MAX_OUTSTANDING_COMMANDS];
	uint32_t current_outstanding_cmd; 

	/* ISP ring lock, rings, and indexes */
	dma_addr_t	request_dma;        /* Physical address. */
	request_t       *request_ring;      /* Base virtual address */
	request_t       *request_ring_ptr;  /* Current address. */
	uint16_t        req_ring_index;     /* Current index. */
	uint16_t        req_q_cnt;          /* Number of available entries. */

	dma_addr_t	response_dma;       /* Physical address. */
	response_t      *response_ring;     /* Base virtual address */
	response_t      *response_ring_ptr; /* Current address. */
	uint16_t        rsp_ring_index;     /* Current index. */
    
#if defined(FC_IP_SUPPORT)
	/* Data for IP support */
	uint8_t		ip_port_name[WWN_SIZE];

	struct risc_rec_entry *risc_rec_q;	/* RISC receive queue */
	dma_addr_t	risc_rec_q_dma;		/*  physical address */
	uint16_t	rec_entries_in;
	uint16_t	rec_entries_out;

	struct send_cb	*active_scb_q[MAX_SEND_PACKETS];
	uint32_t	current_scb_q_idx;

	uint32_t	mtu;
	uint16_t	header_size;
	uint16_t        max_receive_buffers;
	struct buffer_cb *receive_buffers;
	uint32_t	receive_buff_data_size;

	void		(*send_completion_routine)
				(struct send_cb *scb);
	void		*receive_packets_context;
	void		(*receive_packets_routine)
				(void *context, struct buffer_cb *bcb);
	void		*notify_context;
	void		(*notify_routine)
				(void *context, uint32_t type);

	struct ip_device ipdev_db[QLLAN_MAX_IP_DEVICES];
	struct ip_device *ipdev_db_next_free;
	struct ip_device *ipdev_db_top;
	struct ip_device *ipdev_db_bottom;
#endif

	/* Firmware Initialization Control Block data */
	dma_addr_t	init_cb_dma;         /* Physical address. */
	init_cb_t       *init_cb;
  
	/* Timeout timers. */
	uint8_t         queue_restart_timer;   
	atomic_t         loop_down_timer;         /* loop down timer */
	uint8_t         loop_down_abort_time;    /* port down timer */
	uint32_t        timer_active;
	uint32_t        forceLip;
	struct timer_list        timer;

	/* These are used by mailbox operations. */
	mbx_cmd_t	*mcp;
	unsigned long	mbx_cmd_flags;
#define MBX_CMD_ACTIVE	1 /* first bit */
#define MBX_CMD_WANT	2 /* 2nd bit */
#define MBX_INTERRUPT	3 /* 3rd bit */
#define MBX_INTR_WAIT   4 /* 4rd bit */

	spinlock_t	mbx_reg_lock;   /* Mbx Cmd Register Lock */
	spinlock_t	mbx_q_lock;     /* Mbx Active Cmd Queue Lock */
	spinlock_t	mbx_bits_lock;  /* Mailbox access bits Lock */

	uint32_t	mbx_lock_bits;  /* controlled by mbx_bits_lock */
#define MBX_CMD_LOCK	1 /* first bit */
#define MBX_CMD_WANT	2 /* 2nd bit */

	struct semaphore  mbx_intr_sem;  /* Used for completion notification */

	mbx_cmdq_t	*mbx_sem_pool_head;  /* Head Pointer to a list of
			                      * recyclable mbx semaphore pool
			                      * to be used during run time.
			                      */
	mbx_cmdq_t	*mbx_sem_pool_tail;  /* Tail Pointer to semaphore pool*/
#define MBQ_INIT_LEN	16 /* initial mbx sem pool q len. actual len may vary */

	mbx_cmdq_t	*mbx_q_head; /* Head Pointer to sem q for active cmds */
	mbx_cmdq_t	*mbx_q_tail; /* Tail Pointer to sem q for active cmds */


        uint32_t	retry_q_cnt; 
        uint32_t	scsi_retry_q_cnt; 
        uint32_t	failover_cnt; 

	uint8_t	*cmdline;

        uint32_t	login_retry_count; 
    
	volatile struct
	{
		uint32_t     online                  :1;   /* 0 */
		uint32_t     enable_64bit_addressing :1;   /* 1 */
		uint32_t     mbox_int                :1;   /* 2 */
		uint32_t     mbox_busy               :1;   /* 3 */

		uint32_t     port_name_used          :1;   /* 4 */
		uint32_t     failover_enabled        :1;   /* 5 */
		uint32_t     watchdog_enabled        :1;   /* 6 */
		uint32_t     cfg_suspended   	     :1;   /* 7 */

		uint32_t     disable_host_adapter    :1;   /* 8 */
		uint32_t     rscn_queue_overflow     :1;   /* 9 */
		uint32_t     reset_active            :1;   /* 10 */
		uint32_t     link_down_error_enable  :1;   /* 11 */

		uint32_t     disable_risc_code_load  :1;   /* 12 */
		uint32_t     set_cache_line_size_1   :1;   /* 13 */
		uint32_t     enable_target_mode      :1;   /* 14 */
		uint32_t     disable_luns            :1;   /* 15 */

		uint32_t     enable_lip_reset        :1;   /* 16 */
		uint32_t     enable_lip_full_login   :1;   /* 17 */
		uint32_t     enable_target_reset     :1;   /* 18 */
		uint32_t     updated_fc_db           :1;   /* 19 */

		uint32_t     enable_flash_db_update  :1;   /* 20 */
		uint32_t     in_isr                  :1;   /* 21 */
		uint32_t     dpc_sched               :1;   /* 23 */

		uint32_t     nvram_config_done       :1;   /* 24 */
		uint32_t     update_config_needed    :1;   /* 25 */
		uint32_t     management_server_logged_in    :1; /* 26 */
#if defined(FC_IP_SUPPORT)
                uint32_t     enable_ip               :1;   /* 27 */
#endif

	} flags;

	uint32_t     device_flags;
#define DFLG_LOCAL_DEVICES		BIT_0
#define DFLG_RETRY_LOCAL_DEVICES	BIT_1
#define DFLG_FABRIC_DEVICES		BIT_2
#define	SWITCH_FOUND			BIT_3
#define	DFLG_NO_CABLE			BIT_4

	unsigned long	cpu_flags;

        uint8_t		marker_needed; 
	uint8_t		missing_targets;
	uint8_t		sns_retry_cnt;
	uint8_t		cmd_wait_cnt;
	uint8_t		mem_err;

	unsigned long   dpc_flags;
#define	RESET_MARKER_NEEDED	0	/* initiate sending a marker to ISP */
#define	RESET_ACTIVE		1
#define	ISP_ABORT_NEEDED	2	/* initiate ISP Abort */
#define	ABORT_ISP_ACTIVE	3	/* isp abort in progress */

#define	LOOP_RESYNC_NEEDED	4	/* initiate a configure fabric sequence */
#define	LOOP_RESYNC_ACTIVE	5
#define	COMMAND_WAIT_NEEDED	6
#define	COMMAND_WAIT_ACTIVE	7

#define LOCAL_LOOP_UPDATE       8	/* Perform a local loop update */
#define RSCN_UPDATE             9	/* Perform a RSCN update */
#define MAILBOX_RETRY           10
#define ISP_RESET_NEEDED        11	/* Initiate a ISP reset ??? */

#define FAILOVER_EVENT_NEEDED   12
#define FAILOVER_EVENT		13
#define FAILOVER_NEEDED   	14
#define LOOP_RESET_NEEDED	15

#define DEVICE_RESET_NEEDED	16
#define DEVICE_ABORT_NEEDED	17
#define SCSI_RESTART_NEEDED	18	/* Processes any requests in scsi retry queue */
#define PORT_RESTART_NEEDED	19	/* Processes any requests in retry queue */

#define RESTART_QUEUES_NEEDED	20	/* Restarts requeusts in the lun queue */
#define ABORT_QUEUES_NEEDED	21
#define RELOGIN_NEEDED	        22
#define LOGIN_RETRY_NEEDED	23	/* initiates any fabric logins that are required */ 
#define REGISTER_FC4_NEEDED	24	/* set when need to register again.*/

#define TASKLET_SCHED		25	/* Tasklet is scheduled.  */ 
#define DONE_RUNNING		26	/* Done task is running. */
#define ISP_ABORT_RETRY         27      /* ISP aborted. */


/* macro for timer to start dpc for handling mailbox commands */
#define MAILBOX_CMD_NEEDED	(LOOP_RESET_NEEDED|DEVICE_RESET_NEEDED|   \
    DEVICE_ABORT_NEEDED|ISP_ABORT_NEEDED)

	/* These 3 fields are used by the reset done in dpc thread */
	uint16_t	reset_bus_id;
	uint16_t	reset_tgt_id;
	uint16_t	reset_lun;

	uint8_t		interrupts_on;
	uint8_t		init_done;

	volatile uint16_t loop_state;
#define LOOP_TIMEOUT 0x01
#define LOOP_DOWN    0x02
#define LOOP_UP      0x04
#define LOOP_UPDATE  0x08
#define LOOP_READY   0x10

	mbx_cmd_t 	mc;
	uint32_t	mbx_flags;
#define  MBX_IN_PROGRESS  BIT_0
#define  MBX_BUSY       BIT_1 /* Got the Access */
#define  MBX_SLEEPING_ON_SEM  BIT_2 
#define  MBX_POLLING_FOR_COMP  BIT_3
#define  MBX_COMPLETED      BIT_4
#define  MBX_TIMEDOUT       BIT_5 
#define  MBX_ACCESS_TIMEDOUT BIT_6

/* following are new and needed for IOCTL support */
	hba_ioctl_context *ioctl;
	uint8_t     node_name[WWN_SIZE];

	uint8_t     optrom_major; 
	uint8_t     optrom_minor; 

	uint8_t     nvram_version; 

	void        *ioctl_mem;
	dma_addr_t  ioctl_mem_phys;
	uint32_t    ioctl_mem_size;
	uint32_t    isp_abort_cnt;

	/* HBA serial number */
	uint8_t     serial0;
	uint8_t     serial1;
	uint8_t     serial2;

	/* oem related items */
	uint8_t	oem_id;
	uint8_t oem_spare0;
	uint8_t oem_part[6];
	uint8_t oem_fru[8];
	uint8_t oem_ec[8];
	uint8_t oem_string[8];

	uint32_t    dump_done;
	unsigned long    done_q_cnt;
	unsigned long    pending_in_q;

	uint32_t failover_type;
	uint32_t failback_delay;
	unsigned long   cfg_flags;
#define	CFG_ACTIVE	0	/* CFG during a failover, event update, or ioctl */
	/* uint8_t	cfg_active; */
	int	eh_start;

	uint32_t 	 iocb_hiwat;
	uint32_t 	 iocb_cnt;
	uint32_t 	 iocb_overflow_cnt;
	
	int	srb_cnt;
	int	srb_alloc_cnt;	/*Number of allocated SRBs  */

	uint32_t mbox_trace;

	uint32_t	binding_type;
#define BIND_BY_PORT_NAME	0
#define BIND_BY_PORT_ID		1
#define BIND_BY_NODE_NAME	2

	srb_t	*status_srb;    /* Keep track of Status Continuation Entries */

	uint32_t	dropped_frame_error_cnt;

	uint16_t	fw_options1;
	uint16_t	fw_options2;
	uint16_t	fw_options3;
	struct qla2x00_seriallink_firmware_options fw_seriallink_options;
} scsi_qla_host_t;

#if defined(__BIG_ENDIAN)
/* Big endian machine correction defines. */
#define	LITTLE_ENDIAN_16(x)	qla2x00_chg_endian((uint8_t *)&(x), 2)
#define	LITTLE_ENDIAN_24(x)	qla2x00_chg_endian((uint8_t *)&(x), 3)
#define	LITTLE_ENDIAN_32(x)	qla2x00_chg_endian((uint8_t *)&(x), 4)
#define	LITTLE_ENDIAN_64(x)	qla2x00_chg_endian((uint8_t *)&(x), 8)
#define	BIG_ENDIAN_16(x)
#define	BIG_ENDIAN_24(x)
#define	BIG_ENDIAN_32(x)
#define	BIG_ENDIAN_64(x)

#else
/* Little endian machine correction defines. */
#define	LITTLE_ENDIAN_16(x)
#define	LITTLE_ENDIAN_24(x)
#define	LITTLE_ENDIAN_32(x)
#define	LITTLE_ENDIAN_64(x)
#define	BIG_ENDIAN_16(x)	qla2x00_chg_endian((uint8_t *)&(x), 2)
#define	BIG_ENDIAN_24(x)	qla2x00_chg_endian((uint8_t *)&(x), 3)
#define	BIG_ENDIAN_32(x)	qla2x00_chg_endian((uint8_t *)&(x), 4)
#define	BIG_ENDIAN_64(x)	qla2x00_chg_endian((uint8_t *)&(x), 8)

#endif

/*
 * Macros to help code, maintain, etc.
 */
#define	LOOP_TRANSITION(ha)	( test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags) || \
				  test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags) )

#define	LOOP_NOT_READY(ha)	 ( (test_bit(ISP_ABORT_NEEDED, &ha->dpc_flags) || \
				    test_bit(ABORT_ISP_ACTIVE, &ha->dpc_flags) || \
                                    test_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags) || \
				    test_bit(LOOP_RESYNC_ACTIVE, &ha->dpc_flags) || \
				    test_bit(COMMAND_WAIT_NEEDED, &ha->dpc_flags) || \
                                    test_bit(COMMAND_WAIT_ACTIVE, &ha->dpc_flags)) ||  \
				 ha->loop_state == LOOP_DOWN)
				 
#define	LOOP_RDY(ha)	 ( !LOOP_NOT_READY(ha) )

#define	TGT_Q(ha, t)		(ha->otgt[t])
#define	LUN_Q(ha, t, l)		(TGT_Q(ha, t)->olun[l])
#define GET_LU_Q(ha, t, l)  ( (TGT_Q(ha,t) != NULL)? TGT_Q(ha, t)->olun[l] : NULL)
#define PORT_DOWN_TIMER(ha, t)    ((ha)->fc_db[(t)].port_timer)
#define PORT(ha, t)    		((ha)->fc_db[(t)])
#define PORT_LOGIN_RETRY(fcport)    ((fcport)->port_login_retry_count)

#define MBOX_TRACE(ha,b)		{(ha)->mbox_trace |= (b);}

#define	MBS_MASK			0x3fff
#define	MBS_END				0x100
#define	QLA2X00_SUCCESS		(MBS_COMMAND_COMPLETE & MBS_MASK)
#define	QLA2X00_FAILED		(MBS_END + 2)
#define	QLA2X00_FUNCTION_FAILED		(MBS_END + 2)

inline void *qla2x00_kmem_zalloc(int siz, int code, int id);
inline void qla2x00_kmem_free(void *ptr);

#define  KMEM_ZALLOC(siz,id)	qla2x00_kmem_zalloc((siz), GFP_ATOMIC, (id) )
#define  KMEM_FREE(ip,siz)	kfree((ip))

#if defined(__cplusplus)
}
#endif

void qla2x00_device_queue_depth(scsi_qla_host_t *, Scsi_Device *);
#endif

#if defined(__386__)
#  define QLA2100_BIOSPARAM  qla2x00_biosparam
#else
#  define QLA2100_BIOSPARAM  NULL
#endif

/*
 *  Linux - SCSI Driver Interface Function Prototypes.
 */
int qla2x00_ioctl(Scsi_Device *, int , void *);
int qla2x00_proc_info ( char *, char **, off_t, int, int, int);
const char * qla2x00_info(struct Scsi_Host *host);
int qla2x00_detect(Scsi_Host_Template *);
int qla2x00_release(struct Scsi_Host *);
const char * qla2x00_info(struct Scsi_Host *);
int qla2x00_queuecommand(Scsi_Cmnd *, void (* done)(Scsi_Cmnd *));
int qla2x00_abort(Scsi_Cmnd *);
int qla2x00_reset(Scsi_Cmnd *, unsigned int);
int qla2x00_biosparam(Disk *, kdev_t, int[]);
void qla2x00_intr_handler(int, void *, struct pt_regs *);
#if !defined(MODULE)
static int __init qla2100_setup (char *s);
#else
void qla2x00_setup(char *s);
#endif

/*
 * Scsi_Host_template (see hosts.h) 
 * Device driver Interfaces to mid-level SCSI driver.
 */

/* Kernel version specific template additions */

/*
 * max_sectors
 *
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,8)
#define TEMPLATE_MAX_SECTORS	max_sectors: 8192,
#else
#define TEMPLATE_MAX_SECTORS 
#endif
/*
 * highmem_io
 *
 */
#if defined CONFIG_HIGHIO || LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,18)
/* Assume RedHat Distribution with its different HIGHIO conventions. */
#define TEMPLATE_HIGHMEM_IO        can_dma_32: 1,
#else
#define TEMPLATE_HIGHMEM_IO	   highmem_io : 1,
#endif
#else
#define TEMPLATE_HIGHMEM_IO 
#endif
/*
 * use_new_eh_code
 *
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#define TEMPLATE_USE_NEW_EH_CODE
#else
#define TEMPLATE_USE_NEW_EH_CODE use_new_eh_code: 1,
#endif
/*
 * emulated
 *
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#define TEMPLATE_EMULATED
#else
#define TEMPLATE_EMULATED emulated: 0,
#endif
/*
 * next
 *
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#define TEMPLATE_NEXT
#else
#define TEMPLATE_NEXT next: NULL,
#endif
/*
 * module
 *
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#define TEMPLATE_MODULE
#else
#define TEMPLATE_MODULE module: NULL,
#endif
/*
 * proc_dir
 *
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#define TEMPLATE_PROC_DIR
#else
#define TEMPLATE_PROC_DIR proc_dir: NULL,
#endif



#define QLA2100_LINUX_TEMPLATE {				\
TEMPLATE_NEXT 	 	 	 	 	 	 	\
TEMPLATE_MODULE 	  	 	 	 	 	\
TEMPLATE_PROC_DIR 	  	 	 	 	 	\
	proc_info: qla2x00_proc_info,	                        \
	name:		"Qlogic Fibre Channel 2x00",		\
	detect:		qla2x00_detect,				\
	release:	qla2x00_release,			\
	info:		qla2x00_info,				\
	ioctl: qla2x00_ioctl,					\
	command: NULL,						\
	queuecommand: qla2x00_queuecommand,			\
	eh_strategy_handler: NULL,				\
	eh_abort_handler: qla2xxx_eh_abort,			\
	eh_device_reset_handler: qla2xxx_eh_device_reset,	\
	eh_bus_reset_handler: qla2xxx_eh_bus_reset,		\
	eh_host_reset_handler: qla2xxx_eh_host_reset,		\
	abort: NULL,						\
	reset: NULL,						\
	slave_attach: NULL,					\
	bios_param: QLA2100_BIOSPARAM,				\
	can_queue: 255,		/* max simultaneous cmds      */\
	this_id: -1,		/* scsi id of host adapter    */\
	sg_tablesize: SG_SEGMENTS,	/* max scatter-gather cmds */\
	cmd_per_lun: 3,		/* cmds per lun (linked cmds) */\
	present: 0,		/* number of 7xxx's present   */\
	unchecked_isa_dma: 0,	/* no memory DMA restrictions */\
	use_clustering: ENABLE_CLUSTERING,			\
TEMPLATE_USE_NEW_EH_CODE 	 	 	 	 	\
TEMPLATE_MAX_SECTORS						\
TEMPLATE_HIGHMEM_IO						\
TEMPLATE_EMULATED						\
}

#endif /* _IO_HBA_QLA2100_H */

