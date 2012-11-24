/* tc35815.c: A TOSHIBA TC35815CF PCI 10/100Mbps ethernet driver for linux.
 *
 * Copyright 2001 MontaVista Software Inc.
 * Author: MontaVista Software, Inc. 
 *                ahennessy@mvista.com
 *
 * Based on skelton.c by Donald Becker.
 * Copyright (C) 2000-2001 Toshiba Corporation
 *
 * Cleaned up various non portable stuff (save_and_cli etc) and made it
 * build on x86 platforms -- Alan Cox <alan@redhat.com> 20020302
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 * WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 * USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * TODO:
 *	Switch to spin_lock not lock_kernel for scalability.
 */

static const char *version = "tc35815_1.c:v1.13\n";
#define MODNAME			"tc35815"
#define TC35815_PROC_ENTRY "tc35815"

#include <linux/config.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <linux/errno.h>
#include <linux/init.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/mii.h>
#include <asm/byteorder.h>

/* First, a few definitions that the brave might change. */

/* use 0 for production, 1 for verification, >2 for debug */
#ifndef TC35815_DEBUG
#define TC35815_DEBUG 2
#endif
static unsigned int tc35815_debug = TC35815_DEBUG;

#ifdef TC35815_DEBUG
#  define assert(expr) do {} while (0)
#else
#  define assert(expr) \
        if(!(expr)) {					\
        printk( "Assertion failed! %s,%s,%s,line=%d\n",	\
        #expr,__FILE__,__FUNCTION__,__LINE__);		\
        }
#endif

#define GATHER_TXINT	/* On-Demand Tx Interrupt */
#define WORKAROUND_LOSTCAR

typedef enum {
	TC35815CF = 0,
	TC35815_NWU,
} board_t;

/* indexed by board_t, above */
static struct {
	const char *name;
} board_info[] __devinitdata = {
	{ "TOSHIBA TC35815CF 10/100BaseTX" },
	{ "TOSHIBA TC35815 with Wake on LAN" },
};

static struct pci_device_id tc35815_pci_tbl[] __devinitdata = {
	{PCI_VENDOR_ID_TOSHIBA_2, PCI_DEVICE_ID_TOSHIBA_TC35815CF, PCI_ANY_ID, PCI_ANY_ID, 0, 0, TC35815CF },
	{PCI_VENDOR_ID_TOSHIBA_2, PCI_DEVICE_ID_TOSHIBA_TC35815_NWU, PCI_ANY_ID, PCI_ANY_ID, 0, 0, TC35815_NWU },
	{0,}
};
MODULE_DEVICE_TABLE (pci, tc35815_pci_tbl);

/*
 * Registers
 */
struct tc35815_regs {
	volatile __u32 DMA_Ctl;		/* 0x00 */
	volatile __u32 TxFrmPtr;
	volatile __u32 TxThrsh;
	volatile __u32 TxPollCtr;
	volatile __u32 BLFrmPtr;
	volatile __u32 RxFragSize;
	volatile __u32 Int_En;
	volatile __u32 FDA_Bas;
	volatile __u32 FDA_Lim;		/* 0x20 */
	volatile __u32 Int_Src;
	volatile __u32 unused0[2];
	volatile __u32 PauseCnt;
	volatile __u32 RemPauCnt;
	volatile __u32 TxCtlFrmStat;
	volatile __u32 unused1;
	volatile __u32 MAC_Ctl;		/* 0x40 */
	volatile __u32 CAM_Ctl;
	volatile __u32 Tx_Ctl;
	volatile __u32 Tx_Stat;
	volatile __u32 Rx_Ctl;
	volatile __u32 Rx_Stat;
	volatile __u32 MD_Data;
	volatile __u32 MD_CA;
	volatile __u32 CAM_Adr;		/* 0x60 */
	volatile __u32 CAM_Data;
	volatile __u32 CAM_Ena;
	volatile __u32 PROM_Ctl;
	volatile __u32 PROM_Data;
	volatile __u32 Algn_Cnt;
	volatile __u32 CRC_Cnt;
	volatile __u32 Miss_Cnt;
};

/*
 * Bit assignments
 */
/* DMA_Ctl bit asign ------------------------------------------------------- */
#define DMA_IntMask            0x00040000 /* 1:Interupt mask                 */
#define DMA_SWIntReq           0x00020000 /* 1:Software Interrupt request    */
#define DMA_TxWakeUp           0x00010000 /* 1:Transmit Wake Up              */
#define DMA_RxBigE             0x00008000 /* 1:Receive Big Endian            */
#define DMA_TxBigE             0x00004000 /* 1:Transmit Big Endian           */
#define DMA_TestMode           0x00002000 /* 1:Test Mode                     */
#define DMA_PowrMgmnt          0x00001000 /* 1:Power Management              */
#define DMA_DmBurst_Mask       0x000001fc /* DMA Burst size                  */

/* RxFragSize bit asign ---------------------------------------------------- */
#define RxFrag_EnPack          0x00008000 /* 1:Enable Packing                */
#define RxFrag_MinFragMask     0x00000ffc /* Minimum Fragment                */

/* MAC_Ctl bit asign ------------------------------------------------------- */
#define MAC_Link10             0x00008000 /* 1:Link Status 10Mbits           */
#define MAC_EnMissRoll         0x00002000 /* 1:Enable Missed Roll            */
#define MAC_MissRoll           0x00000400 /* 1:Missed Roll                   */
#define MAC_Loop10             0x00000080 /* 1:Loop 10 Mbps                  */
#define MAC_Conn_Auto          0x00000000 /*00:Connection mode (Automatic)   */
#define MAC_Conn_10M           0x00000020 /*01:                (10Mbps endec)*/
#define MAC_Conn_Mll           0x00000040 /*10:                (Mll clock)   */
#define MAC_MacLoop            0x00000010 /* 1:MAC Loopback                  */
#define MAC_FullDup            0x00000008 /* 1:Full Duplex 0:Half Duplex     */
#define MAC_Reset              0x00000004 /* 1:Software Reset                */
#define MAC_HaltImm            0x00000002 /* 1:Halt Immediate                */
#define MAC_HaltReq            0x00000001 /* 1:Halt request                  */

/* PROM_Ctl bit asign ------------------------------------------------------ */
#define PROM_Busy              0x00008000 /* 1:Busy (Start Operation)        */
#define PROM_Read              0x00004000 /*10:Read operation                */
#define PROM_Write             0x00002000 /*01:Write operation               */
#define PROM_Erase             0x00006000 /*11:Erase operation               */
                                          /*00:Enable or Disable Writting,   */
                                          /*      as specified in PROM_Addr. */
#define PROM_Addr_Ena          0x00000030 /*11xxxx:PROM Write enable         */
                                          /*00xxxx:           disable        */

/* CAM_Ctl bit asign ------------------------------------------------------- */
#define CAM_CompEn             0x00000010 /* 1:CAM Compare Enable            */
#define CAM_NegCAM             0x00000008 /* 1:Reject packets CAM recognizes,*/
                                          /*                    accept other */
#define CAM_BroadAcc           0x00000004 /* 1:Broadcast assept              */
#define CAM_GroupAcc           0x00000002 /* 1:Multicast assept              */
#define CAM_StationAcc         0x00000001 /* 1:unicast accept                */

/* CAM_Ena bit asign ------------------------------------------------------- */
#define CAM_ENTRY_MAX                  21   /* CAM Data entry max count      */
#define CAM_Ena_Mask ((1<<CAM_ENTRY_MAX)-1) /* CAM Enable bits (Max 21bits)  */
#define CAM_Ena_Bit(index)         (1<<(index))
#define CAM_ENTRY_DESTINATION	0
#define CAM_ENTRY_SOURCE	1
#define CAM_ENTRY_MACCTL	20

/* Tx_Ctl bit asign -------------------------------------------------------- */
#define Tx_En                  0x00000001 /* 1:Transmit enable               */
#define Tx_TxHalt              0x00000002 /* 1:Transmit Halt Request         */
#define Tx_NoPad               0x00000004 /* 1:Suppress Padding              */
#define Tx_NoCRC               0x00000008 /* 1:Suppress Padding              */
#define Tx_FBack               0x00000010 /* 1:Fast Back-off                 */
#define Tx_EnUnder             0x00000100 /* 1:Enable Underrun               */
#define Tx_EnExDefer           0x00000200 /* 1:Enable Excessive Deferral     */
#define Tx_EnLCarr             0x00000400 /* 1:Enable Lost Carrier           */
#define Tx_EnExColl            0x00000800 /* 1:Enable Excessive Collision    */
#define Tx_EnLateColl          0x00001000 /* 1:Enable Late Collision         */
#define Tx_EnTxPar             0x00002000 /* 1:Enable Transmit Parity        */
#define Tx_EnComp              0x00004000 /* 1:Enable Completion             */

/* Tx_Stat bit asign ------------------------------------------------------- */
#define Tx_TxColl_MASK         0x0000000F /* Tx Collision Count              */
#define Tx_ExColl              0x00000010 /* Excessive Collision             */
#define Tx_TXDefer             0x00000020 /* Transmit Defered                */
#define Tx_Paused              0x00000040 /* Transmit Paused                 */
#define Tx_IntTx               0x00000080 /* Interrupt on Tx                 */
#define Tx_Under               0x00000100 /* Underrun                        */
#define Tx_Defer               0x00000200 /* Deferral                        */
#define Tx_NCarr               0x00000400 /* No Carrier                      */
#define Tx_10Stat              0x00000800 /* 10Mbps Status                   */
#define Tx_LateColl            0x00001000 /* Late Collision                  */
#define Tx_TxPar               0x00002000 /* Tx Parity Error                 */
#define Tx_Comp                0x00004000 /* Completion                      */
#define Tx_Halted              0x00008000 /* Tx Halted                       */
#define Tx_SQErr               0x00010000 /* Signal Quality Error(SQE)       */

/* Rx_Ctl bit asign -------------------------------------------------------- */
#define Rx_EnGood              0x00004000 /* 1:Enable Good                   */
#define Rx_EnRxPar             0x00002000 /* 1:Enable Receive Parity         */
#define Rx_EnLongErr           0x00000800 /* 1:Enable Long Error             */
#define Rx_EnOver              0x00000400 /* 1:Enable OverFlow               */
#define Rx_EnCRCErr            0x00000200 /* 1:Enable CRC Error              */
#define Rx_EnAlign             0x00000100 /* 1:Enable Alignment              */
#define Rx_IgnoreCRC           0x00000040 /* 1:Ignore CRC Value              */
#define Rx_StripCRC            0x00000010 /* 1:Strip CRC Value               */
#define Rx_ShortEn             0x00000008 /* 1:Short Enable                  */
#define Rx_LongEn              0x00000004 /* 1:Long Enable                   */
#define Rx_RxHalt              0x00000002 /* 1:Receive Halt Request          */
#define Rx_RxEn                0x00000001 /* 1:Receive Intrrupt Enable       */

/* Rx_Stat bit asign ------------------------------------------------------- */
#define Rx_Halted              0x00008000 /* Rx Halted                       */
#define Rx_Good                0x00004000 /* Rx Good                         */
#define Rx_RxPar               0x00002000 /* Rx Parity Error                 */
                            /* 0x00001000    not use                         */
#define Rx_LongErr             0x00000800 /* Rx Long Error                   */
#define Rx_Over                0x00000400 /* Rx Overflow                     */
#define Rx_CRCErr              0x00000200 /* Rx CRC Error                    */
#define Rx_Align               0x00000100 /* Rx Alignment Error              */
#define Rx_10Stat              0x00000080 /* Rx 10Mbps Status                */
#define Rx_IntRx               0x00000040 /* Rx Interrupt                    */
#define Rx_CtlRecd             0x00000020 /* Rx Control Receive              */

#define Rx_Stat_Mask           0x0000EFC0 /* Rx All Status Mask              */

/* Int_En bit asign -------------------------------------------------------- */
#define Int_NRAbtEn            0x00000800 /* 1:Non-recoverable Abort Enable  */
#define Int_TxCtlCmpEn         0x00000400 /* 1:Transmit Control Complete Enable */
#define Int_DmParErrEn         0x00000200 /* 1:DMA Parity Error Enable       */
#define Int_DParDEn            0x00000100 /* 1:Data Parity Error Enable      */
#define Int_EarNotEn           0x00000080 /* 1:Early Notify Enable           */
#define Int_DParErrEn          0x00000040 /* 1:Detected Parity Error Enable  */
#define Int_SSysErrEn          0x00000020 /* 1:Signalled System Error Enable */
#define Int_RMasAbtEn          0x00000010 /* 1:Received Master Abort Enable  */
#define Int_RTargAbtEn         0x00000008 /* 1:Received Target Abort Enable  */
#define Int_STargAbtEn         0x00000004 /* 1:Signalled Target Abort Enable */
#define Int_BLExEn             0x00000002 /* 1:Buffer List Exhausted Enable  */
#define Int_FDAExEn            0x00000001 /* 1:Free Descriptor Area          */
                                          /*               Exhausted Enable  */

/* Int_Src bit asign ------------------------------------------------------- */
#define Int_NRabt              0x00004000 /* 1:Non Recoverable error         */
#define Int_DmParErrStat       0x00002000 /* 1:DMA Parity Error & Clear      */
#define Int_BLEx               0x00001000 /* 1:Buffer List Empty & Clear     */
#define Int_FDAEx              0x00000800 /* 1:FDA Empty & Clear             */
#define Int_IntNRAbt           0x00000400 /* 1:Non Recoverable Abort         */
#define	Int_IntCmp             0x00000200 /* 1:MAC control packet complete   */
#define Int_IntExBD            0x00000100 /* 1:Interrupt Extra BD & Clear    */
#define Int_DmParErr           0x00000080 /* 1:DMA Parity Error & Clear      */
#define Int_IntEarNot          0x00000040 /* 1:Receive Data write & Clear    */
#define Int_SWInt              0x00000020 /* 1:Software request & Clear      */
#define Int_IntBLEx            0x00000010 /* 1:Buffer List Empty & Clear     */
#define Int_IntFDAEx           0x00000008 /* 1:FDA Empty & Clear             */
#define Int_IntPCI             0x00000004 /* 1:PCI controller & Clear        */
#define Int_IntMacRx           0x00000002 /* 1:Rx controller & Clear         */
#define Int_IntMacTx           0x00000001 /* 1:Tx controller & Clear         */

/* MD_CA bit asign --------------------------------------------------------- */
#define MD_CA_PreSup           0x00001000 /* 1:Preamble Supress              */
#define MD_CA_Busy             0x00000800 /* 1:Busy (Start Operation)        */
#define MD_CA_Wr               0x00000400 /* 1:Write 0:Read                  */


/*
 * Descriptors
 */

/* Frame descripter */
struct FDesc {
	volatile __u32 FDNext;
	volatile __u32 FDSystem;
	volatile __u32 FDStat;
	volatile __u32 FDCtl;
};

/* Buffer descripter */
struct BDesc {
	volatile __u32 BuffData;
	volatile __u32 BDCtl;
};

#define FD_ALIGN	16

/* Frame Descripter bit asign ---------------------------------------------- */
#define FD_FDLength_MASK       0x0000FFFF /* Length MASK                     */
#define FD_BDCnt_MASK          0x001F0000 /* BD count MASK in FD             */
#define FD_FrmOpt_MASK         0x7C000000 /* Frame option MASK               */
#define FD_FrmOpt_BigEndian    0x40000000 /* Tx/Rx */
#define FD_FrmOpt_IntTx        0x20000000 /* Tx only */
#define FD_FrmOpt_NoCRC        0x10000000 /* Tx only */
#define FD_FrmOpt_NoPadding    0x08000000 /* Tx only */
#define FD_FrmOpt_Packing      0x04000000 /* Rx only */
#define FD_CownsFD             0x80000000 /* FD Controller owner bit         */
#define FD_Next_EOL            0x00000001 /* FD EOL indicator                */
#define FD_BDCnt_SHIFT         16

/* Buffer Descripter bit asign --------------------------------------------- */
#define BD_BuffLength_MASK     0x0000FFFF /* Recieve Data Size               */
#define BD_RxBDID_MASK         0x00FF0000 /* BD ID Number MASK               */
#define BD_RxBDSeqN_MASK       0x7F000000 /* Rx BD Sequence Number           */
#define BD_CownsBD             0x80000000 /* BD Controller owner bit         */
#define BD_RxBDID_SHIFT        16
#define BD_RxBDSeqN_SHIFT      24


/* Some useful constants. */
#undef NO_CHECK_CARRIER	/* Does not check No-Carrier with TP */

#ifdef NO_CHECK_CARRIER
#define TX_CTL_CMD	(Tx_EnComp | Tx_EnTxPar | Tx_EnLateColl | \
	Tx_EnExColl | Tx_EnExDefer | Tx_EnUnder | \
	Tx_En)	/* maybe  0x7b01 */
#else
#define TX_CTL_CMD	(Tx_EnComp | Tx_EnTxPar | Tx_EnLateColl | \
	Tx_EnExColl | Tx_EnLCarr | Tx_EnExDefer | Tx_EnUnder | \
	Tx_En)	/* maybe  0x7b01 */
#endif
#define RX_CTL_CMD	(Rx_EnGood | Rx_EnRxPar | Rx_EnLongErr | Rx_EnOver \
	| Rx_EnCRCErr | Rx_EnAlign | Rx_RxEn)	/* maybe 0x6f01 */
#define INT_EN_CMD  (Int_NRAbtEn | \
	Int_DmParErrEn | Int_DParDEn | Int_DParErrEn | \
	Int_SSysErrEn  | Int_RMasAbtEn | Int_RTargAbtEn | \
	Int_STargAbtEn | \
	Int_BLExEn  | Int_FDAExEn) /* maybe 0xb7f*/

/* Tuning parameters */
#define DMA_BURST_SIZE	32
#define TX_THRESHOLD	1024
#define TX_THRESHOLD_MAX 1536       /* used threshold with packet max byte for low pci transfer ability.*/
#define TX_THRESHOLD_KEEP_LIMIT 100  /* setting threshold max value when overrun error occured this count. */

#define FD_PAGE_NUM 2
#define FD_PAGE_ORDER 1
/* 16 + RX_BUF_PAGES * 8 + RX_FD_NUM * 16 + TX_FD_NUM * 32 <= PAGE_SIZE*2 */
#define RX_BUF_PAGES	8	/* >= 2 */
#define RX_FD_NUM	250	/* >= 32 */
#define TX_FD_NUM	128

struct TxFD {
	struct FDesc fd;
	struct BDesc bd;
	struct BDesc unused;
};

struct RxFD {
	struct FDesc fd;
	struct BDesc bd[0];	/* variable length */
};

struct FrFD {
	struct FDesc fd;
	struct BDesc bd[RX_BUF_PAGES];
};

#define TC35815_TX_TIMEOUT  ((400*HZ)/1000)

/* Timer state engine. */
enum tc35815_timer_state {
	arbwait  = 0,	/* Waiting for auto negotiation to complete.          */
	lupwait  = 1,	/* Auto-neg complete, awaiting link-up status.        */
	ltrywait = 2,	/* Forcing try of all modes, from fastest to slowest. */
	asleep   = 3,	/* Time inactive.                                     */
	lcheck   = 4,	/* Check link status.                                 */
};

/* Information that need to be kept for each board. */
struct tc35815_local {
	int bindex;
	struct pci_dev *pci_dev;

	/* statistics */
	struct net_device_stats stats;
	struct {
		int max_tx_qlen;
		int tx_ints;
		int rx_ints;
	        int tx_underrun;
	} lstats;

	/* Tx control lock.  This protects the transmit buffer ring
	 * state along with the "tx full" state of the driver.  This
	 * means all netif_queue flow control actions are protected
	 * by this lock as well.
	 */
	spinlock_t lock;

	int phy_addr;
	int option;
#define TC35815_OPT_AUTO	0x00
#define TC35815_OPT_10M	0x01
#define TC35815_OPT_100M	0x02
#define TC35815_OPT_FULLDUP	0x04
	int linkspeed;	/* 10 or 100 */
	int fullduplex;
	unsigned short saved_lpa;
	struct timer_list timer;
	enum tc35815_timer_state timer_state; /* State of auto-neg timer. */
	unsigned int timer_ticks;	/* Number of clicks at each state  */

	/*
	 * Transmitting: Batch Mode.
	 *	1 BD in 1 TxFD.
	 * Receiving: Packing Mode.
	 *	1 circular FD for Free Buffer List.
	 *	RX_BUG_PAGES BD in Free Buffer FD.
	 *	One Free Buffer BD has PAGE_SIZE data buffer.
	 */
	void * fd_buf;	/* for TxFD, TxFD, FrFD */
	dma_addr_t fd_buf_dma;
	struct TxFD *tfd_base;
	unsigned int tfd_start;
	unsigned int tfd_end;
	struct RxFD *rfd_base;
	struct RxFD *rfd_limit;
	struct RxFD *rfd_cur;
	struct FrFD *fbl_ptr;
	unsigned char fbl_curid;
	void * data_buf[RX_BUF_PAGES];		/* packing */
	dma_addr_t data_buf_dma[RX_BUF_PAGES];		/* packing */

	struct {
		struct sk_buff *skb;
		dma_addr_t skb_dma;
	} tx_skbs[TX_FD_NUM];
};

inline dma_addr_t fd_virt_to_bus(struct tc35815_local *lp, void *virt)
{
#if 0
	if ((u8 *)virt < (u8 *)lp->fd_buf ||
	    (u8 *)virt >= (u8 *)lp->fd_buf + PAGE_SIZE * FD_PAGE_NUM)
		panic("tc35815: fd_virt_to_bus: %x %x\n", lp, virt);
#endif
	return lp->fd_buf_dma + ((u8 *)virt - (u8 *)lp->fd_buf);
}
inline void *fd_bus_to_virt(struct tc35815_local *lp, dma_addr_t bus)
{
#if 0
	if (bus < lp->fd_buf_dma ||
	    bus >= lp->fd_buf_dma + PAGE_SIZE * FD_PAGE_NUM)
		panic("tc35815: fd_bus_to_virt: %x %x\n", lp, bus);
#endif
	return (void *)((u8 *)lp->fd_buf + (bus - lp->fd_buf_dma));
}
inline dma_addr_t rxbuf_virt_to_bus(struct tc35815_local *lp, void *virt)
{
	int i;
	for (i = 0; i < RX_BUF_PAGES; i++) {
		if ((u8 *)virt >= (u8 *)lp->data_buf[i] &&
		    (u8 *)virt < (u8 *)lp->data_buf[i] + PAGE_SIZE)
			return lp->data_buf_dma[i] + ((u8 *)virt - (u8 *)lp->data_buf[i]);
	}
#if 0
	panic("tc35815: rxbuf_virt_to_bus: %x %x\n", lp, virt);
#endif
	return 0;
}
inline void *rxbuf_bus_to_virt(struct tc35815_local *lp, dma_addr_t bus)
{
	int i;
	for (i = 0; i < RX_BUF_PAGES; i++) {
		if (bus >= lp->data_buf_dma[i] &&
		    bus < lp->data_buf_dma[i] + PAGE_SIZE)
			return (void *)((u8 *)lp->data_buf[i] +
					(bus - lp->data_buf_dma[i]));
	}
#if 0
	panic("tc35815: rxbuf_bus_to_virt: %x %x\n", lp, bus);
#endif
	return NULL;
}

/* Index to functions, as function prototypes. */

static int	tc35815_open(struct net_device *dev);
static int	tc35815_send_packet(struct sk_buff *skb, struct net_device *dev);
static void	tc35815_interrupt(int irq, void *dev_id, struct pt_regs *regs);
static void	tc35815_rx(struct net_device *dev);
static void	tc35815_txdone(struct net_device *dev);
static void	tc35815_txdone_nolock(struct net_device *dev);
static int	tc35815_close(struct net_device *dev);
static struct	net_device_stats *tc35815_get_stats(struct net_device *dev);
static void	tc35815_set_multicast_list(struct net_device *dev);
static void     tc35815_tx_timeout(struct net_device *dev);

/* Example routines you must write ;->. */
static void 	tc35815_chip_reset(struct tc35815_regs *tr);
static void 	tc35815_chip_init(struct net_device *dev);
static void	tc35815_find_phy(struct net_device *dev);
static void 	tc35815_phy_chip_init(struct net_device *dev);
static int 	tc35815_proc_info(char *buffer, char **start, off_t offset, int length, int *eof, void *data);

static void	panic_queues(struct net_device *dev);

static int __devinit tc35815_init_board (struct pci_dev *pdev,
					 struct net_device **dev_out,
					 void **ioaddr_out)
{
	void *ioaddr = NULL;
	struct net_device *dev;
	struct tc35815_local *lp;
	int rc, i;
	unsigned long mmio_start, mmio_end, mmio_flags, mmio_len;

	assert (pdev != NULL);
	assert (ioaddr_out != NULL);

	*ioaddr_out = NULL;
	*dev_out = NULL;

	if (!pdev->irq) {
		printk (KERN_WARNING MODNAME ": no IRQ assigned (%s).\n",
			pdev->slot_name);
		return -ENODEV;
	}

	/* dev zeroed in init_etherdev */
	dev = alloc_etherdev (sizeof (*lp));
	if (dev == NULL) {
		printk (KERN_ERR MODNAME ": unable to alloc new ethernet\n");
		return -ENOMEM;
	}
	SET_MODULE_OWNER(dev);
	lp = dev->priv;

	/* enable device (incl. PCI PM wakeup), and bus-mastering */
	rc = pci_enable_device (pdev);
	if (rc)
		goto err_out;

	mmio_start = pci_resource_start (pdev, 1);
	mmio_end = pci_resource_end (pdev, 1);
	mmio_flags = pci_resource_flags (pdev, 1);
	mmio_len = pci_resource_len (pdev, 1);

	/* set this immediately, we need to know before
	 * we talk to the chip directly */

	/* make sure PCI base addr 1 is MMIO */
	if (!(mmio_flags & IORESOURCE_MEM)) {
		printk (KERN_ERR MODNAME ": region #1 not an MMIO resource, aborting\n");
		rc = -ENODEV;
		goto err_out;
	}

	/* check for weird/broken PCI region reporting */
	if ((mmio_len < sizeof(struct tc35815_regs))) {
		printk (KERN_ERR MODNAME ": Invalid PCI region size(s), aborting\n");
		rc = -ENODEV;
		goto err_out;
	}

	rc = pci_request_regions (pdev, MODNAME);
	if (rc)
		goto err_out;

	pci_set_master (pdev);

	/* ioremap MMIO region */
	ioaddr = ioremap (mmio_start, mmio_len);
	if (ioaddr == NULL) {
		printk (KERN_ERR MODNAME ": cannot remap MMIO, aborting\n");
		rc = -EIO;
		goto err_out_free_res;
	}

	/* Soft reset the chip. */
	tc35815_chip_reset((struct tc35815_regs*)ioaddr);

	i = register_netdev (dev);
	if (i)
		goto err_out_unmap;

	*ioaddr_out = ioaddr;
	*dev_out = dev;
	return 0;

err_out_unmap:
	iounmap(ioaddr);
err_out_free_res:
	pci_release_regions (pdev);
err_out:
	kfree (dev);
	return rc;
}


static void __devinit tc35815_init_dev_addr (struct net_device *dev)
{
	struct tc35815_regs *tr = (struct tc35815_regs *)dev->base_addr;
	int i;

#ifdef CONFIG_TOSHIBA_RBTX4938	/* TX4938-builtin LANC only */
	struct tc35815_local *np = (struct tc35815_local *)dev->priv;
        extern int rbtx4938_get_tx4938_ethaddr(struct pci_dev *dev, unsigned char *addr);
	if (rbtx4938_get_tx4938_ethaddr &&
	    rbtx4938_get_tx4938_ethaddr(np->pci_dev, dev->dev_addr) == 0)
		return;
#endif
	while (readl(&tr->PROM_Ctl) & PROM_Busy)
		;
	for (i = 0; i < 6; i += 2) {
		unsigned short data;
		writel(PROM_Busy | PROM_Read | (i / 2 + 2), &tr->PROM_Ctl);
		while (readl(&tr->PROM_Ctl) & PROM_Busy)
			;
		data = readl(&tr->PROM_Data);
		dev->dev_addr[i] = data & 0xff;
		dev->dev_addr[i+1] = data >> 8;
	}
}

static int __devinit tc35815_init_one (struct pci_dev *pdev,
				       const struct pci_device_id *ent)
{
	struct net_device *dev = NULL;
	struct tc35815_local *lp;
	int i;
	void *ioaddr = NULL;
	static int board_idx = -1;

/* when built into the kernel, we only print version if device is found */
#ifndef MODULE
	static int printed_version;
	if (!printed_version++)
		printk(version);
#endif

	assert (pdev != NULL);
	assert (ent != NULL);

	board_idx++;

	i = tc35815_init_board (pdev, &dev, &ioaddr);
	if (i < 0)
		return i;

	assert (ioaddr != NULL);
	assert (dev != NULL);

	/* Initialize the device structure. */
	dev->open = tc35815_open;
	dev->hard_start_xmit = tc35815_send_packet;
	dev->stop = tc35815_close;
	dev->get_stats = tc35815_get_stats;
	dev->set_multicast_list = tc35815_set_multicast_list;
	dev->tx_timeout = tc35815_tx_timeout;
	dev->watchdog_timeo = TC35815_TX_TIMEOUT;

	dev->irq = pdev->irq;
	dev->base_addr = (unsigned long) ioaddr;

	/* dev->priv/lp zeroed and aligned in init_etherdev */
	lp = dev->priv;
	spin_lock_init(&lp->lock);
	lp->bindex = board_idx;
	lp->pci_dev = pdev;

	pci_set_drvdata(pdev, dev);

	/* Retrieve the ethernet address. */
	tc35815_init_dev_addr(dev);

	printk (KERN_INFO "%s: %s at 0x%lx, "
		"%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
		"IRQ %d\n",
		dev->name,
		board_info[ent->driver_data].name,
		dev->base_addr,
		dev->dev_addr[0], dev->dev_addr[1],
		dev->dev_addr[2], dev->dev_addr[3],
		dev->dev_addr[4], dev->dev_addr[5],
		dev->irq);

	if (dev->mem_start > 0) {
		lp->option = dev->mem_start;
		if ((lp->option & TC35815_OPT_10M) &&
		    (lp->option & TC35815_OPT_100M)) {
			/* if both speed speficied, auto select. */
			lp->option &= ~(TC35815_OPT_10M | TC35815_OPT_100M);
		}
	}

#ifdef CONFIG_TOSHIBA_JMR3927
		//XXX fixme
		lp->option |= TC35815_OPT_10M;
#endif                             

	init_timer(&lp->timer);
	tc35815_find_phy(dev);

#ifdef CONFIG_PROC_FS
	{
		char pentry[64];
		sprintf(pentry, "%s-%d", TC35815_PROC_ENTRY, lp->bindex);
		create_proc_read_entry(pentry, 0, proc_net,
				       tc35815_proc_info, dev);
	}
#endif
	return 0;
}


static void __devexit tc35815_remove_one (struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata (pdev);
	struct tc35815_local *np;
	unsigned long mmio_addr;

	assert (dev != NULL);

	np = dev->priv;
	assert (np != NULL);
	mmio_addr = dev->base_addr;

#ifdef CONFIG_PROC_FS
	{
		char pentry[64];
		sprintf(pentry, "%s-%d", TC35815_PROC_ENTRY, np->bindex);
		remove_proc_entry(pentry, NULL) ; 
	}
#endif

	unregister_netdev (dev);

	if (mmio_addr) {
		iounmap ((void *)mmio_addr);
		pci_release_regions (pdev);
	}

	kfree (dev);

	pci_set_drvdata (pdev, NULL);
}

static int
tc35815_init_queues(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	int i;
	unsigned long fd_addr;

	if (!lp->fd_buf) {
		if (sizeof(struct FDesc) +
		    sizeof(struct BDesc) * RX_BUF_PAGES +
		    sizeof(struct FDesc) * RX_FD_NUM +
		    sizeof(struct TxFD) * TX_FD_NUM > PAGE_SIZE * FD_PAGE_NUM) {
			printk("%s: Invalid Queue Size.\n", dev->name);
			return -ENOMEM;
		}

		if ((lp->fd_buf = pci_alloc_consistent(lp->pci_dev, PAGE_SIZE * FD_PAGE_NUM, &lp->fd_buf_dma)) == 0)
			return -ENOMEM;
		for (i = 0; i < RX_BUF_PAGES; i++) {
			if ((lp->data_buf[i] = pci_alloc_consistent(lp->pci_dev, PAGE_SIZE, &lp->data_buf_dma[i])) == 0) {
				while (--i >= 0) {
					pci_free_consistent(lp->pci_dev, PAGE_SIZE, lp->data_buf[i], lp->data_buf_dma[i]);
					lp->data_buf[i] = 0;
				}
				pci_free_consistent(lp->pci_dev, PAGE_SIZE * FD_PAGE_NUM, lp->fd_buf, lp->fd_buf_dma);
				lp->fd_buf = 0;
				return -ENOMEM;
			}
		}
		printk(KERN_DEBUG "%s: FD buf %p DataBuf", dev->name, lp->fd_buf);
		for (i = 0; i < RX_BUF_PAGES; i++) {
			printk(" %p", lp->data_buf[i]);
		}
		printk("\n");
	} else {
		for (i = 0; i < FD_PAGE_NUM; i++) {
			clear_page((void *)((unsigned long)lp->fd_buf + i * PAGE_SIZE));
		}
	}
	fd_addr = (unsigned long)lp->fd_buf;

	/* Free Descriptors (for Receive) */
	lp->rfd_base = (struct RxFD *)fd_addr;
	fd_addr += sizeof(struct RxFD) * RX_FD_NUM;
	for (i = 0; i < RX_FD_NUM; i++) {
		lp->rfd_base[i].fd.FDCtl = cpu_to_le32(FD_CownsFD);
	}
	lp->rfd_cur = lp->rfd_base;
	lp->rfd_limit = (struct RxFD *)(fd_addr -
					sizeof(struct FDesc) -
					sizeof(struct BDesc) * 30);

	/* Transmit Descriptors */
	lp->tfd_base = (struct TxFD *)fd_addr;
	fd_addr += sizeof(struct TxFD) * TX_FD_NUM;
	for (i = 0; i < TX_FD_NUM; i++) {
		lp->tfd_base[i].fd.FDNext = cpu_to_le32(fd_virt_to_bus(lp, &lp->tfd_base[i+1]));
		lp->tfd_base[i].fd.FDSystem = cpu_to_le32(0xffffffff);
		lp->tfd_base[i].fd.FDCtl = cpu_to_le32(0);
	}
	lp->tfd_base[TX_FD_NUM-1].fd.FDNext = cpu_to_le32(fd_virt_to_bus(lp, &lp->tfd_base[0]));
	lp->tfd_start = 0;
	lp->tfd_end = 0;

	/* Buffer List (for Receive) */
	lp->fbl_ptr = (struct FrFD *)fd_addr;
	lp->fbl_ptr->fd.FDNext = cpu_to_le32(fd_virt_to_bus(lp, lp->fbl_ptr));
	lp->fbl_ptr->fd.FDCtl = cpu_to_le32(RX_BUF_PAGES | FD_CownsFD);
	for (i = 0; i < RX_BUF_PAGES; i++) {
		lp->fbl_ptr->bd[i].BuffData = cpu_to_le32(lp->data_buf_dma[i]);
		/* BDID is index of FrFD.bd[] */
		lp->fbl_ptr->bd[i].BDCtl =
			cpu_to_le32(BD_CownsBD | (i << BD_RxBDID_SHIFT) | PAGE_SIZE);
	}
	lp->fbl_curid = 0;

	printk(KERN_DEBUG "%s: TxFD %p RxFD %p FrFD %p\n",
	       dev->name, lp->tfd_base, lp->rfd_base, lp->fbl_ptr);
	return 0;
}

static void
tc35815_clear_queues(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	int i;

	for (i = 0; i < TX_FD_NUM; i++) {
		struct sk_buff *skb =
			lp->tfd_base[i].fd.FDSystem != cpu_to_le32(0xffffffff) ?
			lp->tx_skbs[le32_to_cpu(lp->tfd_base[i].fd.FDSystem)].skb : NULL;
		if (lp->tx_skbs[i].skb != skb) {
			printk("%s: tx_skbs mismatch(%d).\n", dev->name, i);
			panic_queues(dev);
		}
		if (skb) {
			pci_unmap_single(lp->pci_dev, lp->tx_skbs[i].skb_dma, skb->len, PCI_DMA_TODEVICE);
			lp->tx_skbs[i].skb = NULL;
			lp->tx_skbs[i].skb_dma = 0;
			dev_kfree_skb_any(skb);
		}
		lp->tfd_base[i].fd.FDSystem = cpu_to_le32(0xffffffff);
	}

	tc35815_init_queues(dev);
}

static void
tc35815_free_queues(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	int i;

	if (lp->tfd_base) {
		for (i = 0; i < TX_FD_NUM; i++) {
			struct sk_buff *skb =
				lp->tfd_base[i].fd.FDSystem != cpu_to_le32(0xffffffff) ?
				lp->tx_skbs[le32_to_cpu(lp->tfd_base[i].fd.FDSystem)].skb : NULL;
			if (lp->tx_skbs[i].skb != skb) {
				printk("%s: tx_skbs mismatch(%d).\n", dev->name, i);
				panic_queues(dev);
			}
			if (skb) {
				dev_kfree_skb(skb);
				pci_unmap_single(lp->pci_dev, lp->tx_skbs[i].skb_dma, skb->len, PCI_DMA_TODEVICE);
				lp->tx_skbs[i].skb = NULL;
				lp->tx_skbs[i].skb_dma = 0;
			}
			lp->tfd_base[i].fd.FDSystem = cpu_to_le32(0xffffffff);
		}
	}

	lp->rfd_base = NULL;
	lp->rfd_base = NULL;
	lp->rfd_limit = NULL;
	lp->rfd_cur = NULL;
	lp->fbl_ptr = NULL;

	for (i = 0; i < RX_BUF_PAGES; i++) {
		if (lp->data_buf[i])
			pci_free_consistent(lp->pci_dev, PAGE_SIZE, lp->data_buf[i], lp->data_buf_dma[i]);
		lp->data_buf[i] = 0;
	}
	if (lp->fd_buf)
		pci_free_consistent(lp->pci_dev, PAGE_SIZE * FD_PAGE_NUM, lp->fd_buf, lp->fd_buf_dma);
	lp->fd_buf = NULL;
}

static void
dump_txfd(struct TxFD *fd)
{
	printk("TxFD(%p): %08x %08x %08x %08x\n", fd,
	       le32_to_cpu(fd->fd.FDNext),
	       le32_to_cpu(fd->fd.FDSystem),
	       le32_to_cpu(fd->fd.FDStat),
	       le32_to_cpu(fd->fd.FDCtl));
	printk("BD: ");
	printk(" %08x %08x",
	       le32_to_cpu(fd->bd.BuffData),
	       le32_to_cpu(fd->bd.BDCtl));
	printk("\n");
}

static int
dump_rxfd(struct RxFD *fd)
{
	int i, bd_count = (le32_to_cpu(fd->fd.FDCtl) & FD_BDCnt_MASK) >> FD_BDCnt_SHIFT;
	if (bd_count > 8)
		bd_count = 8;
	printk("RxFD(%p): %08x %08x %08x %08x\n", fd,
	       le32_to_cpu(fd->fd.FDNext),
	       le32_to_cpu(fd->fd.FDSystem),
	       le32_to_cpu(fd->fd.FDStat),
	       le32_to_cpu(fd->fd.FDCtl));
	if (le32_to_cpu(fd->fd.FDCtl) & FD_CownsFD)
	    return 0;
	printk("BD: ");
	for (i = 0; i < bd_count; i++)
		printk(" %08x %08x",
		       le32_to_cpu(fd->bd[i].BuffData),
		       le32_to_cpu(fd->bd[i].BDCtl));
	printk("\n");
	return bd_count;
}

static void
dump_frfd(struct FrFD *fd)
{
	int i;
	printk("FrFD(%p): %08x %08x %08x %08x\n", fd,
	       le32_to_cpu(fd->fd.FDNext),
	       le32_to_cpu(fd->fd.FDSystem),
	       le32_to_cpu(fd->fd.FDStat),
	       le32_to_cpu(fd->fd.FDCtl));
	printk("BD: ");
	for (i = 0; i < RX_BUF_PAGES; i++)
		printk(" %08x %08x",
		       le32_to_cpu(fd->bd[i].BuffData),
		       le32_to_cpu(fd->bd[i].BDCtl));
	printk("\n");
}

static void
panic_queues(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	int i;

	printk("TxFD base %p, start %u, end %u\n",
	       lp->tfd_base, lp->tfd_start, lp->tfd_end);
	printk("RxFD base %p limit %p cur %p\n",
	       lp->rfd_base, lp->rfd_limit, lp->rfd_cur);
	printk("FrFD %p\n", lp->fbl_ptr);
	for (i = 0; i < TX_FD_NUM; i++)
		dump_txfd(&lp->tfd_base[i]);
	for (i = 0; i < RX_FD_NUM; i++) {
		int bd_count = dump_rxfd(&lp->rfd_base[i]);
		i += (bd_count + 1) / 2;	/* skip BDs */
	}
	dump_frfd(lp->fbl_ptr);
	panic("%s: Illegal queue state.", dev->name);
}

static void print_eth(char *add)
{
	int i;

	printk("print_eth(%p)\n", add);
	for (i = 0; i < 6; i++)
		printk(" %2.2X", (unsigned char) add[i + 6]);
	printk(" =>");
	for (i = 0; i < 6; i++)
		printk(" %2.2X", (unsigned char) add[i]);
	printk(" : %2.2X%2.2X\n", (unsigned char) add[12], (unsigned char) add[13]);
}

static int tc35815_tx_full(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	return ((lp->tfd_start + 1) % TX_FD_NUM == lp->tfd_end);
}

static void tc35815_tx_timeout(struct net_device *dev)
{
	struct tc35815_local *np = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs *)dev->base_addr;

	printk(KERN_WARNING "%s: transmit timed out, status %#x\n",
	       dev->name, readl(&tr->Tx_Stat));

	/* Try to restart the adaptor. */
	del_timer(&np->timer);		/* Kill if running	*/
	tc35815_chip_reset(tr);
	tc35815_clear_queues(dev);
	tc35815_chip_init(dev);

	np->stats.tx_errors++;

	/* If we have space available to accept new transmit
	 * requests, wake up the queueing layer.  This would
	 * be the case if the chipset_init() call above just
	 * flushes out the tx queue and empties it.
	 *
	 * If instead, the tx queue is retained then the
	 * netif_wake_queue() call should be placed in the
	 * TX completion interrupt handler of the driver instead
	 * of here.
	 */
	if (!tc35815_tx_full(dev))
		netif_wake_queue(dev);
}

/*
 * Open/initialize the board. This is called (in the current kernel)
 * sometime after booting when the 'ifconfig' program is run.
 *
 * This routine should set everything up anew at each open, even
 * registers that "should" only need to be set once at boot, so that
 * there is non-reboot way to recover if something goes wrong.
 */
static int
tc35815_open(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;

	/*
	 * This is used if the interrupt line can turned off (shared).
	 * See 3c503.c for an example of selecting the IRQ at config-time.
	 */
	if (request_irq(dev->irq, &tc35815_interrupt, SA_SHIRQ, dev->name, dev)) {
		return -EAGAIN;
	}

	del_timer(&lp->timer);		/* Kill if running	*/
	tc35815_chip_reset((struct tc35815_regs*)dev->base_addr);

	if (tc35815_init_queues(dev) != 0) {
		free_irq(dev->irq, dev);
		return -EAGAIN;
	}

	/* Reset the hardware here. Don't forget to set the station address. */
	tc35815_chip_init(dev);

	/* We are now ready to accept transmit requeusts from
	 * the queueing layer of the networking.
	 */
	netif_start_queue(dev);

	return 0;
}

/* This will only be invoked if your driver is _not_ in XOFF state.
 * What this means is that you need not check it, and that this
 * invariant will hold if you make sure that the netif_*_queue()
 * calls are done at the proper times.
 */
static int tc35815_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs *)dev->base_addr;
	short length = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;
	struct TxFD *txfd;

	/* If some error occurs while trying to transmit this
	 * packet, you should return '1' from this function.
	 * In such a case you _may not_ do anything to the
	 * SKB, it is still owned by the network queueing
	 * layer when an error is returned.  This means you
	 * may not modify any SKB fields, you may not free
	 * the SKB, etc.
	 */

	/* This is the most common case for modern hardware.
	 * The spinlock protects this code from the TX complete
	 * hardware interrupt handler.  Queue flow control is
	 * thus managed under this lock as well.
	 */
	spin_lock_irq(&lp->lock);

	/*add to ring */
	txfd = &lp->tfd_base[lp->tfd_start];

	/* failsafe... */
	if (lp->tfd_start != lp->tfd_end)
		tc35815_txdone_nolock(dev);

	if (tc35815_debug > 3)
		print_eth(skb->data);
	if (lp->tx_skbs[lp->tfd_start].skb) {
		printk("%s: tx_skbs conflict.\n", dev->name);
		panic_queues(dev);
	}
	lp->tx_skbs[lp->tfd_start].skb = skb;
	lp->tx_skbs[lp->tfd_start].skb_dma = pci_map_single(lp->pci_dev, skb->data, skb->len, PCI_DMA_TODEVICE);
	txfd->bd.BuffData = cpu_to_le32(lp->tx_skbs[lp->tfd_start].skb_dma);
	txfd->bd.BDCtl = cpu_to_le32(length);
	txfd->fd.FDSystem = cpu_to_le32(lp->tfd_start);
	txfd->fd.FDCtl = cpu_to_le32(FD_CownsFD | (1 << FD_BDCnt_SHIFT));

	if (lp->tfd_start == lp->tfd_end) {
		/* Start DMA Transmitter. */
		txfd->fd.FDNext |= cpu_to_le32(FD_Next_EOL);
#ifdef GATHER_TXINT
		txfd->fd.FDCtl |= cpu_to_le32(FD_FrmOpt_IntTx);
#endif
		if (tc35815_debug > 2) {
			printk("%s: starting TxFD.\n", dev->name);
			dump_txfd(txfd);
		}
		writel(fd_virt_to_bus(lp, txfd), &tr->TxFrmPtr);
	} else {
		txfd->fd.FDNext &= cpu_to_le32(~FD_Next_EOL);
		if (tc35815_debug > 2) {
			printk("%s: queueing TxFD.\n", dev->name);
			dump_txfd(txfd);
		}
	}
	lp->tfd_start = (lp->tfd_start + 1) % TX_FD_NUM;

	dev->trans_start = jiffies;

	/* If we just used up the very last entry in the
	 * TX ring on this device, tell the queueing
	 * layer to send no more.
	 */
	if (tc35815_tx_full(dev)) {
		if (tc35815_debug > 1)
			printk(KERN_WARNING "%s: TxFD Exhausted.\n", dev->name);
		netif_stop_queue(dev);
	}

	/* When the TX completion hw interrupt arrives, this
	 * is when the transmit statistics are updated.
	 */

	spin_unlock_irq(&lp->lock);
	return 0;
}

#define FATAL_ERROR_INT \
	(Int_IntPCI | Int_DmParErr | Int_IntNRAbt)
#if 0  // for error end with DMA Parity error happened
static int tc35815_fatal_error_interrupt(struct net_device *dev, int status)
#else  // for error reset
static void tc35815_fatal_error_interrupt(struct net_device *dev, int status)
#endif
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	static int count;
	printk(KERN_WARNING "%s: Fatal Error Intterrupt (%#x):",
	       dev->name, status);
	if (status & Int_IntPCI)
		printk(" IntPCI");
	if (status & Int_DmParErr) 
#if 0  // for error end with DMA Parity error happened
	{
		struct tc35815_regs *tr = (struct tc35815_regs *)dev->base_addr;
		unsigned long tmp;
		tmp = readl(&tr->Int_En);
		tmp &= ~Int_DmParErrEn;
		writel(tmp, &tr->Int_En);
		printk(KERN_WARNING "Disable DMA Parity error Int.\n");
		return 1;
	}
#else  // for error reset
		printk(" DmParErr");
#endif
	if (status & Int_IntNRAbt)
		printk(" IntNRAbt");
	printk("\n");
	if (count++ > 100)
		panic("%s: Too many fatal errors.", dev->name);
	printk(KERN_WARNING "%s: Resetting ...\n", dev->name);
	/* Try to restart the adaptor. */
	del_timer(&lp->timer);		/* Kill if running	*/
	tc35815_chip_reset((struct tc35815_regs*)dev->base_addr);
	tc35815_clear_queues(dev);
	tc35815_chip_init(dev);
#if 0  // for error end with DMA Parity error happened
	return 0;
#else
	return;
#endif
}

/*
 * The typical workload of the driver:
 * Handle the network interface interrupts.
 */
static void tc35815_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct net_device *dev = dev_id;
	struct tc35815_regs *tr;
	struct tc35815_local *lp;
	int status;

	tr = (struct tc35815_regs*)dev->base_addr;
	lp = (struct tc35815_local *)dev->priv;

	status = readl(&tr->Int_Src);
	writel(status, &tr->Int_Src);	/* write to clear */

	/* Fatal errors... */
	if (status & FATAL_ERROR_INT) {
#if 0  // for error end with DMA Parity error happened
		if (tc35815_fatal_error_interrupt(dev, status) == 0)
#else  // for error reset
		tc35815_fatal_error_interrupt(dev, status);
#endif
			return;
	}
	/* recoverable errors */
	if (status & Int_IntFDAEx) {
		/* disable FDAEx int. (until we make rooms...) */
		writel(readl(&tr->Int_En) & ~Int_FDAExEn, &tr->Int_En);
		printk(KERN_WARNING
		       "%s: Free Descriptor Area Exhausted (%#x).\n",
		       dev->name, status);
		lp->stats.rx_dropped++;
	}
	if (status & Int_IntBLEx) {
		/* disable BLEx int. (until we make rooms...) */
		writel(readl(&tr->Int_En) & ~Int_BLExEn, &tr->Int_En);
		printk(KERN_WARNING
		       "%s: Buffer List Exhausted (%#x).\n",
		       dev->name, status);
		lp->stats.rx_dropped++;
	}
	if (status & Int_IntExBD) {
		printk(KERN_WARNING
		       "%s: Excessive Buffer Descriptiors (%#x).\n",
		       dev->name, status);
		lp->stats.rx_length_errors++;
	}

	/* normal notification */
	if (status & Int_IntMacRx) {
		/* Got a packet(s). */
		lp->lstats.rx_ints++;
		tc35815_rx(dev);
	}
	if (status & Int_IntMacTx) {
		/* Transmit complete. */
		lp->lstats.tx_ints++;
		tc35815_txdone(dev);
		netif_wake_queue(dev);
	}
}

/* We have a good packet(s), get it/them out of the buffers. */
static void
tc35815_rx(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	unsigned int fdctl;
	int i;
	int buf_free_count = 0;
	int fd_free_count = 0;

	while (!((fdctl = le32_to_cpu(lp->rfd_cur->fd.FDCtl)) & FD_CownsFD)) {
		int status = le32_to_cpu(lp->rfd_cur->fd.FDStat);
		int pkt_len = fdctl & FD_FDLength_MASK;
		struct RxFD *next_rfd;
		int bd_count = (fdctl & FD_BDCnt_MASK) >> FD_BDCnt_SHIFT;

		if (tc35815_debug > 2)
			dump_rxfd(lp->rfd_cur);
		if (status & Rx_Good) {
			/* Malloc up new buffer. */
			struct sk_buff *skb;
			unsigned char *data;
			int cur_bd, offset;

			skb = dev_alloc_skb(pkt_len + 2); /* +2: for reserve */
			if (skb == NULL) {
				printk(KERN_NOTICE "%s: Memory squeeze, dropping packet.\n",
				       dev->name);
				lp->stats.rx_dropped++;
				break;
			}
			skb_reserve(skb, 2);   /* 16 bit alignment */
			skb->dev = dev;

			data = skb_put(skb, pkt_len);

			/* copy from receive buffer */
			cur_bd = 0;
			offset = 0;
			while (offset < pkt_len && cur_bd < bd_count) {
				int len = le32_to_cpu(lp->rfd_cur->bd[cur_bd].BDCtl) &
					BD_BuffLength_MASK;
				void *rxbuf =
					rxbuf_bus_to_virt(lp, le32_to_cpu(lp->rfd_cur->bd[cur_bd].BuffData));
				memcpy(data + offset, rxbuf, len);
				offset += len;
				cur_bd++;
			}
			if (tc35815_debug > 3)
				print_eth(data);
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
			dev->last_rx = jiffies;
			lp->stats.rx_packets++;
			lp->stats.rx_bytes += pkt_len;
		} else {
			lp->stats.rx_errors++;
			printk(KERN_DEBUG "%s: Rx error (status %x)\n",
			       dev->name, status & Rx_Stat_Mask);
			/* WORKAROUND: LongErr and CRCErr means Overflow. */
			if ((status & Rx_LongErr) && (status & Rx_CRCErr)) {
				status &= ~(Rx_LongErr|Rx_CRCErr);
				status |= Rx_Over;
			}
			if (status & Rx_LongErr) lp->stats.rx_length_errors++;
			if (status & Rx_Over) lp->stats.rx_fifo_errors++;
			if (status & Rx_CRCErr) lp->stats.rx_crc_errors++;
			if (status & Rx_Align) lp->stats.rx_frame_errors++;
		}

		if (bd_count > 0) {
			/* put Free Buffer back to controller */
			int bdctl = le32_to_cpu(lp->rfd_cur->bd[bd_count - 1].BDCtl);
			unsigned char id =
				(bdctl & BD_RxBDID_MASK) >> BD_RxBDID_SHIFT;
			if (id >= RX_BUF_PAGES) {
				printk("%s: invalid BDID.\n", dev->name);
				panic_queues(dev);
			}
			/* free old buffers */
			while (lp->fbl_curid != id) {
				bdctl = le32_to_cpu(lp->fbl_ptr->bd[lp->fbl_curid].BDCtl);
				if (bdctl & BD_CownsBD) {
					printk("%s: Freeing invalid BD.\n",
					       dev->name);
					panic_queues(dev);
				}
				/* pass BD to controler */
				/* Note: BDLength was modified by chip. */
				lp->fbl_ptr->bd[lp->fbl_curid].BDCtl =
					cpu_to_le32(BD_CownsBD |
						    (lp->fbl_curid << BD_RxBDID_SHIFT) |
						    PAGE_SIZE);
				lp->fbl_curid =
					(lp->fbl_curid + 1) % RX_BUF_PAGES;
				if (tc35815_debug > 2) {
					printk("%s: Entering new FBD %d\n",
					       dev->name, lp->fbl_curid);
					dump_frfd(lp->fbl_ptr);
				}
				buf_free_count++;
			}
		}

		/* put RxFD back to controller */
		next_rfd = fd_bus_to_virt(lp, le32_to_cpu(lp->rfd_cur->fd.FDNext));
		if (next_rfd < lp->rfd_base || next_rfd > lp->rfd_limit) {
			printk("%s: RxFD FDNext invalid.\n", dev->name);
			panic_queues(dev);
		}
		for (i = 0; i < (bd_count + 1) / 2 + 1; i++) {
			/* pass FD to controler */
			lp->rfd_cur->fd.FDNext = cpu_to_le32(0xdeaddead);	/* for debug */
			lp->rfd_cur->fd.FDCtl = cpu_to_le32(FD_CownsFD);
			lp->rfd_cur++;
			fd_free_count++;
		}

		lp->rfd_cur = next_rfd;
	}

	/* re-enable BL/FDA Exhaust interupts. */
	if (fd_free_count) {
		writel(readl(&tr->Int_En) | Int_FDAExEn, &tr->Int_En);
		if (buf_free_count)
			writel(readl(&tr->Int_En) | Int_BLExEn, &tr->Int_En);
	}
}

#ifdef NO_CHECK_CARRIER
#define TX_STA_ERR	(Tx_ExColl|Tx_Under|Tx_Defer|Tx_LateColl|Tx_TxPar|Tx_SQErr)
#else
#define TX_STA_ERR	(Tx_ExColl|Tx_Under|Tx_Defer|Tx_NCarr|Tx_LateColl|Tx_TxPar|Tx_SQErr)
#endif

static void
tc35815_check_tx_stat(struct net_device *dev, int status)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	const char *msg = NULL;

	/* count collisions */
	if (status & Tx_ExColl)
		lp->stats.collisions += 16;
	if (status & Tx_TxColl_MASK)
		lp->stats.collisions += status & Tx_TxColl_MASK;

#ifdef WORKAROUND_LOSTCAR
	/* WORKAROUND: ignore LostCrS in full duplex operation */
	if ((lp->timer_state != asleep && lp->timer_state != lcheck)
	    || lp->fullduplex)
		status &= ~Tx_NCarr;
#endif

	if (!(status & TX_STA_ERR)) {
		/* no error. */
		lp->stats.tx_packets++;
		return;
	}

	lp->stats.tx_errors++;
	if (status & Tx_ExColl) {
		lp->stats.tx_aborted_errors++;
		msg = "Excessive Collision.";
	}
	if (status & Tx_Under) {
		lp->stats.tx_fifo_errors++;
		//msg = "Tx FIFO Underrun.";
		if (lp->lstats.tx_underrun < TX_THRESHOLD_KEEP_LIMIT) {
		  lp->lstats.tx_underrun++;
		  if (lp->lstats.tx_underrun >= TX_THRESHOLD_KEEP_LIMIT) {
		    struct tc35815_regs *tr = 
		      (struct tc35815_regs*)dev->base_addr;
		    writel(TX_THRESHOLD_MAX, &tr->TxThrsh);
		    msg = "Tx FIFO Underrun.Change Tx threshold to max.";
		  }
		}
	}
	if (status & Tx_Defer) {
		lp->stats.tx_fifo_errors++;
		msg = "Excessive Deferral.";
	}
#ifndef NO_CHECK_CARRIER
	if (status & Tx_NCarr) {
		lp->stats.tx_carrier_errors++;
		msg = "Lost Carrier Sense.";
	}
#endif
	if (status & Tx_LateColl) {
		lp->stats.tx_aborted_errors++;
		msg = "Late Collision.";
	}
	if (status & Tx_TxPar) {
		lp->stats.tx_fifo_errors++;
		msg = "Transmit Parity Error.";
	}
	if (status & Tx_SQErr) {
		lp->stats.tx_heartbeat_errors++;
		msg = "Signal Quality Error.";
	}
	if (msg)
		printk(KERN_WARNING "%s: %s (%#x)\n", dev->name, msg, status);
}

/* This handles TX complete events posted by the device
 * via interrupts.
 */
static void
tc35815_txdone(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	spin_lock(&lp->lock);
	tc35815_txdone_nolock(dev);
	spin_unlock(&lp->lock);
}
static void
tc35815_txdone_nolock(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	struct TxFD *txfd;
	unsigned int fdctl;

	txfd = &lp->tfd_base[lp->tfd_end];
	while (lp->tfd_start != lp->tfd_end &&
	       !((fdctl = le32_to_cpu(txfd->fd.FDCtl)) & FD_CownsFD)) {
		int status = le32_to_cpu(txfd->fd.FDStat);
		struct sk_buff *skb;
		unsigned long fdnext = le32_to_cpu(txfd->fd.FDNext);

		if (tc35815_debug > 2) {
			printk("%s: complete TxFD.\n", dev->name);
			dump_txfd(txfd);
		}
		tc35815_check_tx_stat(dev, status);

		skb = le32_to_cpu(txfd->fd.FDSystem) != cpu_to_le32(0xffffffff) ?
			lp->tx_skbs[le32_to_cpu(txfd->fd.FDSystem)].skb : NULL;
		if (lp->tx_skbs[lp->tfd_end].skb != skb) {
			printk("%s: tx_skbs mismatch.\n", dev->name);
			panic_queues(dev);
		}
		if (skb) {
			lp->stats.tx_bytes += skb->len;
			pci_unmap_single(lp->pci_dev, lp->tx_skbs[lp->tfd_end].skb_dma, skb->len, PCI_DMA_TODEVICE);
			lp->tx_skbs[lp->tfd_end].skb = NULL;
			lp->tx_skbs[lp->tfd_end].skb_dma = 0;
			dev_kfree_skb_irq(skb);
		}
		txfd->fd.FDSystem = cpu_to_le32(0xffffffff);

		lp->tfd_end = (lp->tfd_end + 1) % TX_FD_NUM;
		txfd = &lp->tfd_base[lp->tfd_end];
		if ((fdnext & ~FD_Next_EOL) != fd_virt_to_bus(lp, txfd)) {
			printk("%s: TxFD FDNext invalid.\n", dev->name);
			panic_queues(dev);
		}
		if (fdnext & FD_Next_EOL) {
			/* DMA Transmitter has been stopping... */
			if (lp->tfd_end != lp->tfd_start) {
				int head = (lp->tfd_start + TX_FD_NUM - 1) % TX_FD_NUM;
				struct TxFD* txhead = &lp->tfd_base[head];
				int qlen = (lp->tfd_start + TX_FD_NUM
					    - lp->tfd_end) % TX_FD_NUM;

				if (!(le32_to_cpu(txfd->fd.FDCtl) & FD_CownsFD)) {
					printk("%s: TxFD FDCtl invalid.\n", dev->name);
					panic_queues(dev);
				}
				/* log max queue length */
				if (lp->lstats.max_tx_qlen < qlen)
					lp->lstats.max_tx_qlen = qlen;


				/* start DMA Transmitter again */
				txhead->fd.FDNext |= cpu_to_le32(FD_Next_EOL);
#ifdef GATHER_TXINT
				txhead->fd.FDCtl |= cpu_to_le32(FD_FrmOpt_IntTx);
#endif
				if (tc35815_debug > 2) {
					printk("%s: start TxFD on queue.\n",
					       dev->name);
					dump_txfd(txfd);
				}
				writel(fd_virt_to_bus(lp, txfd), &tr->TxFrmPtr);
			}
			break;
		}
	}

	/* If we had stopped the queue due to a "tx full"
	 * condition, and space has now been made available,
	 * wake up the queue.
	 */
	if (netif_queue_stopped(dev) && ! tc35815_tx_full(dev))
		netif_wake_queue(dev);
}

/* The inverse routine to tc35815_open(). */
static int
tc35815_close(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	netif_stop_queue(dev);

	/* Flush the Tx and disable Rx here. */

	del_timer(&lp->timer);		/* Kill if running	*/
	tc35815_chip_reset((struct tc35815_regs*)dev->base_addr);
	free_irq(dev->irq, dev);

	tc35815_free_queues(dev);

	return 0;

}

/*
 * Get the current statistics.
 * This may be called with the card open or closed.
 */
static struct net_device_stats *tc35815_get_stats(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	/* Update the statistics from the device registers. */
	lp->stats.rx_missed_errors = readl(&tr->Miss_Cnt);

	return &lp->stats;
}

static void tc35815_set_cam_entry(struct tc35815_regs *tr, int index, unsigned char *addr)
{
	int cam_index = index * 6;
	unsigned long cam_data;
	unsigned long saved_addr;
	saved_addr = readl(&tr->CAM_Adr);

	if (tc35815_debug > 1) {
		int i;
		printk(KERN_DEBUG "%s: CAM %d:", MODNAME, index);
		for (i = 0; i < 6; i++)
			printk(" %02x", addr[i]);
		printk("\n");
	}
	if (index & 1) {
		/* read modify write */
		writel(cam_index - 2, &tr->CAM_Adr);
		cam_data = readl(&tr->CAM_Data) & 0xffff0000;
		cam_data |= addr[0] << 8 | addr[1];
		writel(cam_data, &tr->CAM_Data);
		/* write whole word */
		writel(cam_index + 2, &tr->CAM_Adr);
		cam_data = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
		writel(cam_data, &tr->CAM_Data);
	} else {
		/* write whole word */
		writel(cam_index, &tr->CAM_Adr);
		cam_data = (addr[0] << 24) | (addr[1] << 16) | (addr[2] << 8) | addr[3];
		writel(cam_data, &tr->CAM_Data);
		/* read modify write */
		writel(cam_index + 4, &tr->CAM_Adr);
		cam_data = readl(&tr->CAM_Data) & 0x0000ffff;
		cam_data |= addr[4] << 24 | (addr[5] << 16);
		writel(cam_data, &tr->CAM_Data);
	}

	if (tc35815_debug > 2) {
		int i;
		for (i = cam_index / 4; i < cam_index / 4 + 2; i++) {
			writel(i * 4, &tr->CAM_Adr);
			printk("CAM 0x%x: %08x",
			       i * 4, readl(&tr->CAM_Data));
		}
	}
	writel(saved_addr, &tr->CAM_Adr);
}


/*
 * Set or clear the multicast filter for this adaptor.
 * num_addrs == -1	Promiscuous mode, receive all packets
 * num_addrs == 0	Normal mode, clear multicast list
 * num_addrs > 0	Multicast mode, receive normal and MC packets,
 *			and do best-effort filtering.
 */
static void
tc35815_set_multicast_list(struct net_device *dev)
{
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;

	if (dev->flags&IFF_PROMISC)
	{
		/* Enable promiscuous mode */
		writel(CAM_CompEn | CAM_BroadAcc | CAM_GroupAcc | CAM_StationAcc, &tr->CAM_Ctl);
	}
	else if((dev->flags&IFF_ALLMULTI) || dev->mc_count > CAM_ENTRY_MAX - 3)
	{
		/* CAM 0, 1, 20 are reserved. */
		/* Disable promiscuous mode, use normal mode. */
		writel(CAM_CompEn | CAM_BroadAcc | CAM_GroupAcc, &tr->CAM_Ctl);
	}
	else if(dev->mc_count)
	{
		struct dev_mc_list* cur_addr = dev->mc_list;
		int i;
		int ena_bits = CAM_Ena_Bit(CAM_ENTRY_SOURCE);

		writel(0, &tr->CAM_Ctl);
		/* Walk the address list, and load the filter */
		for (i = 0; i < dev->mc_count; i++, cur_addr = cur_addr->next) {
			if (!cur_addr)
				break;
			/* entry 0,1 is reserved. */
			tc35815_set_cam_entry(tr, i + 2, cur_addr->dmi_addr);
			ena_bits |= CAM_Ena_Bit(i + 2);
		}
		writel(ena_bits, &tr->CAM_Ena);
		writel(CAM_CompEn | CAM_BroadAcc, &tr->CAM_Ctl);
	}
	else {
		writel(CAM_Ena_Bit(CAM_ENTRY_SOURCE), &tr->CAM_Ena);
		writel(CAM_CompEn | CAM_BroadAcc, &tr->CAM_Ctl);
	}
}

static unsigned long tc_phy_read(struct tc35815_regs *tr, int phy, int phy_reg)
{
	unsigned long data;
	int flags;
	save_flags(flags);
	cli();
	writel(MD_CA_Busy | (phy << 5) | phy_reg, &tr->MD_CA);
	while (readl(&tr->MD_CA) & MD_CA_Busy)
		;
	data = readl(&tr->MD_Data);
	restore_flags(flags);
	return data;
}

static void tc_phy_write(unsigned long d, struct tc35815_regs *tr, int phy, int phy_reg)
{
	int flags;
	save_flags(flags);
	cli();
	writel(d, &tr->MD_Data);
	writel(MD_CA_Busy | MD_CA_Wr | (phy << 5) | phy_reg, &tr->MD_CA);
	while (readl(&tr->MD_CA) & MD_CA_Busy)
		;
	restore_flags(flags);
}

/* Auto negotiation.  The scheme is very simple.  We have a timer routine
 * that keeps watching the auto negotiation process as it progresses.
 * The DP83840 is first told to start doing it's thing, we set up the time
 * and place the timer state machine in it's initial state.
 *
 * Here the timer peeks at the DP83840 status registers at each click to see
 * if the auto negotiation has completed, we assume here that the DP83840 PHY
 * will time out at some point and just tell us what (didn't) happen.  For
 * complete coverage we only allow so many of the ticks at this level to run,
 * when this has expired we print a warning message and try another strategy.
 * This "other" strategy is to force the interface into various speed/duplex
 * configurations and we stop when we see a link-up condition before the
 * maximum number of "peek" ticks have occurred.
 *
 * Once a valid link status has been detected we configure the BigMAC and
 * the rest of the Happy Meal to speak the most efficient protocol we could
 * get a clean link for.  The priority for link configurations, highest first
 * is:
 *                 100 Base-T Full Duplex
 *                 100 Base-T Half Duplex
 *                 10 Base-T Full Duplex
 *                 10 Base-T Half Duplex
 *
 * We start a new timer now, after a successful auto negotiation status has
 * been detected.  This timer just waits for the link-up bit to get set in
 * the BMCR of the DP83840.  When this occurs we print a kernel log message
 * describing the link type in use and the fact that it is up.
 *
 * If a fatal error of some sort is signalled and detected in the interrupt
 * service routine, and the chip is reset, or the link is ifconfig'd down
 * and then back up, this entire process repeats itself all over again.
 */
/* Note: Above comments are come from sunhme driver. */

static int tc35815_try_next_permutation(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short bmcr;

	bmcr = tc_phy_read(tr, pid, MII_BMCR);

	/* Downgrade from full to half duplex.  Only possible via ethtool.  */
	if (bmcr & BMCR_FULLDPLX) {
		bmcr &= ~BMCR_FULLDPLX;
		printk(KERN_DEBUG "%s: try next permutation (BMCR %x)\n", dev->name, bmcr);
		tc_phy_write(bmcr, tr, pid, MII_BMCR);
		return 0;
	}

	/* Downgrade from 100 to 10. */
	if (bmcr & BMCR_SPEED100) {
		bmcr &= ~BMCR_SPEED100;
		printk(KERN_DEBUG "%s: try next permutation (BMCR %x)\n", dev->name, bmcr);
		tc_phy_write(bmcr, tr, pid, MII_BMCR);
		return 0;
	}

	/* We've tried everything. */
	return -1;
}

static void
tc35815_display_link_mode(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short lpa;
	char *tmode = "";

	lpa = tc_phy_read(tr, pid, MII_LPA);
	/* WORKAROUND: MAC_FullDup is fixed to 0 on TX4938-builtin LANC */
	if (!(readl(&tr->MAC_Ctl) & MAC_FullDup))
		lpa &= ~(LPA_100FULL | LPA_10FULL);

	if (lpa & (LPA_100HALF | LPA_100FULL)) {
		if (lpa & LPA_100FULL)
			tmode = "100Mb/s, Full Duplex";
		else
			tmode = "100Mb/s, Half Duplex";
	} else {
		if (lpa & LPA_10FULL)
			tmode = "10Mb/s, Full Duplex";
		else
			tmode = "10Mb/s, Half Duplex";
	}

	printk(KERN_INFO "%s: Link is up at %s.\n", dev->name, tmode);
	printk(KERN_DEBUG "%s: MII BMCR %04lx BMSR %04lx LPA %04lx\n",
	       dev->name,
	       tc_phy_read(tr, pid, MII_BMCR),
	       tc_phy_read(tr, pid, MII_BMSR),
	       tc_phy_read(tr, pid, MII_LPA));
}

static void tc35815_display_forced_link_mode(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short bmcr;
	char *speed = "", *duplex = "";

	bmcr = tc_phy_read(tr, pid, MII_BMCR);
	if (bmcr & BMCR_SPEED100)
		speed = "100Mb/s, ";
	else
		speed = "10Mb/s, ";
	if (bmcr & BMCR_FULLDPLX)
		duplex = "Full Duplex.\n";
	else
		duplex = "Half Duplex.\n";

	printk(KERN_INFO "%s: Link has been forced up at %s%s", dev->name,
	       speed, duplex);
}

static int tc35815_set_link_modes(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short bmcr, lpa;

	if (lp->timer_state == arbwait) {
		printk(KERN_DEBUG "%s: MII BMCR %04lx BMSR %04lx LPA %04lx\n",
		       dev->name,
		       tc_phy_read(tr, pid, MII_BMCR),
		       tc_phy_read(tr, pid, MII_BMSR),
		       tc_phy_read(tr, pid, MII_LPA));
		lpa = tc_phy_read(tr, pid, MII_LPA);
		if (!(lpa & (LPA_10HALF | LPA_10FULL |
			     LPA_100HALF | LPA_100FULL)))
			goto no_response;
		if (lpa & (LPA_100FULL | LPA_100HALF))
			lp->linkspeed = 100;
		else
			lp->linkspeed = 10;
		if (lpa & (LPA_100FULL | LPA_10FULL))
			lp->fullduplex = 1;
		else
			lp->fullduplex = 0;
	} else {
		/* Forcing a link mode. */
		bmcr = tc_phy_read(tr, pid, MII_BMCR);
		if (bmcr & BMCR_SPEED100)
			lp->linkspeed = 100;
		else
			lp->linkspeed = 10;
		if (bmcr & BMCR_FULLDPLX)
			lp->fullduplex = 1;
		else
			lp->fullduplex = 0;
	}

	writel(readl(&tr->MAC_Ctl) | MAC_HaltReq, &tr->MAC_Ctl);
	if (lp->fullduplex) {
		writel(readl(&tr->MAC_Ctl) | MAC_FullDup, &tr->MAC_Ctl);
		/* WORKAROUND: MAC_FullDup is fixed to 0 on TX4938-builtin LANC */
		if (!(readl(&tr->MAC_Ctl) & MAC_FullDup)) {
			printk(KERN_INFO "%s: MAC_FullDup not supported.\n",
			       dev->name);
			lp->fullduplex = 0;
			bmcr = tc_phy_read(tr, pid, MII_BMCR);
			bmcr &= ~BMCR_FULLDPLX;
			/* power down to restart partner's negotiation */
			tc_phy_write(bmcr | BMCR_PDOWN,
				     tr, pid, MII_BMCR);
			/* power up again */
			tc_phy_write(bmcr, tr, pid, MII_BMCR);
			printk(KERN_DEBUG "%s: MII BMCR %04lx\n",
			       dev->name,
			       tc_phy_read(tr, pid, MII_BMCR));
		}
	} else {
		writel(readl(&tr->MAC_Ctl) & ~MAC_FullDup, &tr->MAC_Ctl);
	}
	writel(readl(&tr->MAC_Ctl) & ~MAC_HaltReq, &tr->MAC_Ctl);

#ifdef WORKAROUND_LOSTCAR
	/* WORKAROUND: enable LostCrS only if half duplex operation */
	if (!lp->fullduplex)
		writel(readl(&tr->Tx_Ctl) | Tx_EnLCarr, &tr->Tx_Ctl);
#endif

	return 0;
 no_response:
	return 1;
}

static void tc35815_timer(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short bmsr, bmcr, lpa;
	int restart_timer = 0;

	spin_lock_irq(&lp->lock);

	lp->timer_ticks++;
	switch (lp->timer_state) {
	case arbwait:
		/*
		 * Only allow for 5 ticks, thats 10 seconds and much too
		 * long to wait for arbitration to complete.
		 */
		/* TC35815 need more times... */
		if (lp->timer_ticks >= 10) {
			/* Enter force mode. */
	do_force_mode:
			printk(KERN_NOTICE "%s: Auto-Negotiation unsuccessful,"
			       " trying force link mode\n", dev->name);
			printk(KERN_DEBUG "%s: BMCR %x BMSR %x\n", dev->name,
			       (unsigned short)tc_phy_read(tr, pid, MII_BMCR),
			       (unsigned short)tc_phy_read(tr, pid, MII_BMSR));
			bmcr = BMCR_SPEED100;
			tc_phy_write(bmcr, tr, pid, MII_BMCR);

			/*
			 * OK, seems we need do disable the transceiver
			 * for the first tick to make sure we get an
			 * accurate link state at the second tick.
			 */

			lp->timer_state = ltrywait;
			lp->timer_ticks = 0;
			restart_timer = 1;
		} else {
			/* Anything interesting happen? */
			bmsr = tc_phy_read(tr, pid, MII_BMSR);
			if (bmsr & BMSR_ANEGCOMPLETE) {
				int ret;

				/* Just what we've been waiting for... */
				ret = tc35815_set_link_modes(dev);
				if (ret) {
					/* Ooops, something bad happened, go to
					 * force mode.
					 *
					 * XXX Broken hubs which don't support
					 * XXX 802.3u auto-negotiation make this
					 * XXX happen as well.
					 */
					goto do_force_mode;
				}

				/*
				 * Success, at least so far, advance our state
				 * engine.
				 */
				lp->timer_state = lupwait;
				restart_timer = 1;
			} else {
				restart_timer = 1;
			}
		}
		break;

	case lupwait:
		/*
		 * Auto negotiation was successful and we are awaiting a
		 * link up status.  I have decided to let this timer run
		 * forever until some sort of error is signalled, reporting
		 * a message to the user at 10 second intervals.
		 */
		bmsr = tc_phy_read(tr, pid, MII_BMSR);
		if (bmsr & BMSR_LSTATUS) {
			/*
			 * Wheee, it's up, display the link mode in use and put
			 * the timer to sleep.
			 */
			tc35815_display_link_mode(dev);
#if 1
			lp->saved_lpa = tc_phy_read(tr, pid, MII_LPA);
			lp->timer_state = lcheck;
			restart_timer = 1;
#else
			lp->timer_state = asleep;
			restart_timer = 0;
#endif
		} else {
			if (lp->timer_ticks >= 10) {
				printk(KERN_NOTICE "%s: Auto negotiation successful, link still "
				       "not completely up.\n", dev->name);
				lp->timer_ticks = 0;
				restart_timer = 1;
			} else {
				restart_timer = 1;
			}
		}
		break;

	case ltrywait:
		/*
		 * Making the timeout here too long can make it take
		 * annoyingly long to attempt all of the link mode
		 * permutations, but then again this is essentially
		 * error recovery code for the most part.
		 */
		bmsr = tc_phy_read(tr, pid, MII_BMSR);
		bmcr = tc_phy_read(tr, pid, MII_BMCR);
		if (lp->timer_ticks == 1) {
			/*
			 * Re-enable transceiver, we'll re-enable the
			 * transceiver next tick, then check link state
			 * on the following tick.
			 */
			restart_timer = 1;
			break;
		}
		if (lp->timer_ticks == 2) {
			restart_timer = 1;
			break;
		}
		if (bmsr & BMSR_LSTATUS) {
			/* Force mode selection success. */
			tc35815_display_forced_link_mode(dev);
			tc35815_set_link_modes(dev);  /* XXX error? then what? */
#if 1
			lp->saved_lpa = tc_phy_read(tr, pid, MII_LPA);
			lp->timer_state = lcheck;
			restart_timer = 1;
#else
			lp->timer_state = asleep;
			restart_timer = 0;
#endif
		} else {
			if (lp->timer_ticks >= 4) { /* 6 seconds or so... */
				int ret;

				ret = tc35815_try_next_permutation(dev);
				if (ret == -1) {
					/*
					 * Aieee, tried them all, reset the
					 * chip and try all over again.
					 */
					printk(KERN_NOTICE "%s: Link down, "
					       "cable problem?\n",
					       dev->name);

					/* Try to restart the adaptor. */
					tc35815_chip_reset(tr);
					tc35815_clear_queues(dev);
					tc35815_chip_init(dev);
					goto out;
				}
				lp->timer_ticks = 0;
				restart_timer = 1;
			} else {
				restart_timer = 1;
			}
		}
		break;

	case lcheck:
		bmcr = tc_phy_read(tr, pid, MII_BMCR);
		lpa = tc_phy_read(tr, pid, MII_LPA);
		if (bmcr & (BMCR_PDOWN | BMCR_ISOLATE | BMCR_RESET)) {
			printk(KERN_ERR "%s: PHY down? (BMCR %x)\n", dev->name,
			       bmcr);
		} else if ((lp->saved_lpa ^ lpa) &
			   (LPA_100FULL|LPA_100HALF|LPA_10FULL|LPA_10HALF)) {
			printk(KERN_NOTICE "%s: link status changed"
			       " (BMCR %x LPA %x->%x)\n", dev->name,
			       bmcr, lp->saved_lpa, lpa);
		} else {
			/* go on */
			restart_timer = 1;
			break;
		}
		/* Try to restart the adaptor. */
		tc35815_chip_reset(tr);
		tc35815_clear_queues(dev);
		tc35815_chip_init(dev);
		goto out;

	case asleep:
	default:
		/* Can't happens.... */
		printk(KERN_ERR "%s: Aieee, link timer is asleep but we got "
		       "one anyways!\n", dev->name);
		restart_timer = 0;
		lp->timer_ticks = 0;
		lp->timer_state = asleep; /* foo on you */
		break;
	}

	if (restart_timer) {
		lp->timer.expires = jiffies + ((12 * HZ)/10); /* 1.2s */
		add_timer(&lp->timer);
	}
out:
	spin_unlock_irq(&lp->lock);
}

static void tc35815_start_auto_negotiation(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short bmsr, bmcr, advertize;
	int timeout;
	__u32 mctl;

	bmsr = tc_phy_read(tr, pid, MII_BMSR);
	bmcr = tc_phy_read(tr, pid, MII_BMCR);
	advertize = tc_phy_read(tr, pid, MII_ADVERTISE);

	if (1 /* ethtool is not supported yet... */) {
		/* Advertise everything we can support. */
		if (bmsr & BMSR_10HALF)
			advertize |= ADVERTISE_10HALF;
		else
			advertize &= ~ADVERTISE_10HALF;
		if (bmsr & BMSR_10FULL)
			advertize |= ADVERTISE_10FULL;
		else
			advertize &= ~ADVERTISE_10FULL;
		if (bmsr & BMSR_100HALF)
			advertize |= ADVERTISE_100HALF;
		else
			advertize &= ~ADVERTISE_100HALF;
		if (bmsr & BMSR_100FULL)
			advertize |= ADVERTISE_100FULL;
		else
			advertize &= ~ADVERTISE_100FULL;

#if 1
		/* WORKAROUND: MAC_FullDup is fixed to 0 on TX4938-builtin LANC */
		mctl = readl(&tr->MAC_Ctl);
		if (!(mctl & MAC_FullDup)) {
			mctl |= MAC_HaltReq;
			writel(mctl, &tr->MAC_Ctl);
			writel(mctl | MAC_FullDup, &tr->MAC_Ctl);
			if (!(readl(&tr->MAC_Ctl) & MAC_FullDup)) {
				printk(KERN_INFO "%s: MAC_FullDup not supported.\n",
				       dev->name);
				advertize &= ~(ADVERTISE_10FULL | ADVERTISE_100FULL);
			}
			writel(mctl, &tr->MAC_Ctl);
			mctl &= ~MAC_HaltReq;
			writel(mctl, &tr->MAC_Ctl);
		}
#endif

		tc_phy_write(advertize, tr, pid, MII_ADVERTISE);

		/* Enable Auto-Negotiation, this is usually on already... */
		bmcr |= BMCR_ANENABLE;
		tc_phy_write(bmcr, tr, pid, MII_BMCR);

		/* Restart it to make sure it is going. */
		bmcr |= BMCR_ANRESTART;
		tc_phy_write(bmcr, tr, pid, MII_BMCR);
		printk(KERN_DEBUG "%s: ADVERTIZE %x BMCR %x\n", dev->name, advertize, bmcr);

		/* BMCR_ANRESTART self clears when the process has begun. */
		timeout = 64;  /* More than enough. */
		while (--timeout) {
			bmcr = tc_phy_read(tr, pid, MII_BMCR);
			if (!(bmcr & BMCR_ANRESTART))
				break; /* got it. */
			udelay(10);
		}
		if (!timeout) {
			printk(KERN_ERR "%s: TC35815 would not start auto "
			       "negotiation BMCR=0x%04x\n",
			       dev->name, bmcr);
			printk(KERN_NOTICE "%s: Performing force link "
			       "detection.\n", dev->name);
			goto force_link;
		} else {
			printk(KERN_DEBUG "%s: auto negotiation started.\n", dev->name);
			lp->timer_state = arbwait;
		}
	} else {
force_link:
		/* Force the link up, trying first a particular mode.
		 * Either we are here at the request of ethtool or
		 * because the Happy Meal would not start to autoneg.
		 */

		/* Disable auto-negotiation in BMCR, enable the duplex and
		 * speed setting, init the timer state machine, and fire it off.
		 */
#if 1	/* ethtool is not supported yet... */
		bmcr = BMCR_SPEED100;
#else
		if (ep == NULL || ep->autoneg == AUTONEG_ENABLE) {
			bmcr = BMCR_SPEED100;
		} else {
			if (ep->speed == SPEED_100)
				bmcr = BMCR_SPEED100;
			else
				bmcr = 0;
			if (ep->duplex == DUPLEX_FULL)
				bmcr |= BMCR_FULLDPLX;
		}
#endif
		tc_phy_write(bmcr, tr, pid, MII_BMCR);

		/* OK, seems we need do disable the transceiver for the first
		 * tick to make sure we get an accurate link state at the
		 * second tick.
		 */
		lp->timer_state = ltrywait;
	}

	del_timer(&lp->timer);
	lp->timer_ticks = 0;
	lp->timer.expires = jiffies + (12 * HZ)/10;  /* 1.2 sec. */
	lp->timer.data = (unsigned long) dev;
	lp->timer.function = &tc35815_timer;
	add_timer(&lp->timer);
}

static void tc35815_find_phy(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short id0, id1;

	/* find MII phy */
	for (pid = 31; pid >= 0; pid--) {
		id0 = tc_phy_read(tr, pid, MII_BMSR);
		if (id0 != 0xffff && id0 != 0x0000 &&
		    (id0 & BMSR_RESV) != (0xffff & BMSR_RESV) /* paranoia? */
			) {
			lp->phy_addr = pid;
			break;
		}
	}
	if (pid < 0) {
		printk(KERN_ERR "%s: No MII Phy found.\n",
		       dev->name);
		lp->phy_addr = pid = 0;
	}

	id0 = tc_phy_read(tr, pid, MII_PHYSID1);
	id1 = tc_phy_read(tr, pid, MII_PHYSID2);
	printk(KERN_INFO "%s: PHY(%02x) ID %04x %04x\n", dev->name,
	       pid, id0, id1);
}

static void tc35815_phy_chip_init(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int pid = lp->phy_addr;
	unsigned short bmcr;

	/* dis-isolate if needed. */
	bmcr = tc_phy_read(tr, pid, MII_BMCR);
	if (bmcr & BMCR_ISOLATE) {
		int count = 32;
		printk(KERN_DEBUG "%s: unisolating...", dev->name);
		tc_phy_write(bmcr & ~BMCR_ISOLATE, tr, pid, MII_BMCR);
		while (--count) {
			if (!(tc_phy_read(tr, pid, MII_BMCR) & BMCR_ISOLATE))
				break;
			udelay(20);
		}
		printk(" %s.\n", count ? "done" : "failed");
	}

	if (lp->option & TC35815_OPT_10M) {
		lp->linkspeed = 10;
		lp->fullduplex = (lp->option & TC35815_OPT_FULLDUP) != 0;
	} else if (lp->option & TC35815_OPT_100M) {
		lp->linkspeed = 100;
		lp->fullduplex = (lp->option & TC35815_OPT_FULLDUP) != 0;
	} else {
		tc35815_start_auto_negotiation(dev);
		return;
	}
	printk(KERN_DEBUG "%s: MII BMCR %04lx BMSR %04lx LPA %04lx\n",
	       dev->name,
	       tc_phy_read(tr, pid, MII_BMCR),
	       tc_phy_read(tr, pid, MII_BMSR),
	       tc_phy_read(tr, pid, MII_LPA));

	bmcr = tc_phy_read(tr, pid, MII_BMCR);
	if (lp->linkspeed == 100)
		bmcr |= BMCR_SPEED100;
	else
		bmcr &= ~BMCR_SPEED100;
	if (lp->fullduplex)
		bmcr |= BMCR_FULLDPLX;
	else
		bmcr &= ~BMCR_FULLDPLX;
	tc_phy_write(bmcr, tr, pid, MII_BMCR);

	lp->timer_state = asleep;
	tc35815_set_link_modes(dev);
}

static void tc35815_chip_reset(struct tc35815_regs *tr)
{
	int i;
	/* reset the controller */
	writel(MAC_Reset, &tr->MAC_Ctl);
	udelay(4); /* 3200ns */
	i = 0;
	while (readl(&tr->MAC_Ctl) & MAC_Reset) {
		if (i++ > 100) {
			printk(KERN_ERR "%s: MAC reset failed.\n", MODNAME);
			break;
		}
		mdelay(1);
	}
	writel(0, &tr->MAC_Ctl);

	/* initialize registers to default value */
	writel(0, &tr->DMA_Ctl);
	writel(0, &tr->TxThrsh);
	writel(0, &tr->TxPollCtr);
	writel(0, &tr->RxFragSize);
	writel(0, &tr->Int_En);
	writel(0, &tr->FDA_Bas);
	writel(0, &tr->FDA_Lim);
	writel(0xffffffff, &tr->Int_Src);	/* Write 1 to clear */
	writel(0, &tr->CAM_Ctl);
	writel(0, &tr->Tx_Ctl);
	writel(0, &tr->Rx_Ctl);
	writel(0, &tr->CAM_Ena);
	(void)readl(&tr->Miss_Cnt);	/* Read to clear */

	/* initialize internal SRAM */
	writel(DMA_TestMode, &tr->DMA_Ctl);
	for (i = 0; i < 0x1000; i += 4) {
		writel(i, &tr->CAM_Adr);
		writel(0, &tr->CAM_Data);
	}
	writel(0, &tr->DMA_Ctl);
}

static void tc35815_chip_init(struct net_device *dev)
{
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;
	struct tc35815_regs *tr = (struct tc35815_regs*)dev->base_addr;
	int flags;
	unsigned long txctl = TX_CTL_CMD;

	tc35815_phy_chip_init(dev);

	/* load station address to CAM */
	tc35815_set_cam_entry(tr, CAM_ENTRY_SOURCE, dev->dev_addr);

	/* Enable CAM (broadcast and unicast) */
	writel(CAM_Ena_Bit(CAM_ENTRY_SOURCE), &tr->CAM_Ena);
	writel(CAM_CompEn | CAM_BroadAcc, &tr->CAM_Ctl);

	save_flags(flags);
	cli();
#if 0	/* __BIG_ENDIAN? */
	writel(DMA_RxBigE | DMA_TxBigE | DMA_BURST_SIZE, &tr->DMA_Ctl);
#else
	writel(DMA_BURST_SIZE, &tr->DMA_Ctl);
#endif
	writel(RxFrag_EnPack | ETH_ZLEN, &tr->RxFragSize);	/* Packing */
	writel(0, &tr->TxPollCtr);	/* Batch mode */
	writel(TX_THRESHOLD, &tr->TxThrsh);
	writel(INT_EN_CMD, &tr->Int_En);

	/* set queues */
	writel(fd_virt_to_bus(lp, lp->rfd_base), &tr->FDA_Bas);
	writel((unsigned long)lp->rfd_limit - (unsigned long)lp->rfd_base,
		  &tr->FDA_Lim);
	/*
	 * Activation method:
	 * First, enable the MAC Transmitter and the DMA Receive circuits.
	 * Then enable the DMA Transmitter and the MAC Receive circuits.
	 */
	writel(fd_virt_to_bus(lp, lp->fbl_ptr), &tr->BLFrmPtr);	/* start DMA receiver */
	writel(RX_CTL_CMD, &tr->Rx_Ctl);	/* start MAC receiver */

	/* start MAC transmitter */
#ifdef WORKAROUND_LOSTCAR
	/* WORKAROUND: ignore LostCrS in full duplex operation */
	if ((lp->timer_state != asleep && lp->timer_state != lcheck) ||
	    lp->fullduplex)
		txctl = TX_CTL_CMD & ~Tx_EnLCarr;
#endif
#ifdef GATHER_TXINT
	txctl &= ~Tx_EnComp;	/* disable global tx completion int. */
#endif
	writel(txctl, &tr->Tx_Ctl);
#if 0	/* No need to polling */
	writel(fd_virt_to_bus(lp, lp->tfd_base), &tr->TxFrmPtr);	/* start DMA transmitter */
#endif
	restore_flags(flags);
}

#ifdef CONFIG_PROC_FS
static int tc35815_proc_info(char *buffer, char **start, off_t offset, int length, int *eof, void *data)
{
	int len = 0;
	struct net_device *dev = (struct net_device *)data;
	struct tc35815_local *lp = (struct tc35815_local *)dev->priv;

	len += sprintf(buffer, "TC35815 statistics (%s):\n", dev->name);
	len += sprintf(buffer + len,
		       "tx_ints %d, rx_ints %d, max_tx_qlen %d\n",
		       lp->lstats.tx_ints,
		       lp->lstats.rx_ints,
		       lp->lstats.max_tx_qlen);
	return len;
}
#endif

static struct pci_driver tc35815_pci_driver = {
	name:		MODNAME,
	id_table:	tc35815_pci_tbl,
	probe:		tc35815_init_one,
	remove:		__devexit_p(tc35815_remove_one),
#if 0
	suspend:	tc35815_suspend,
	resume:		tc35815_resume,
#endif
};


static int __init tc35815_init_module (void)
{
/* when a module, this is printed whether or not devices are found in probe */
#ifdef MODULE
	printk(version);
#endif
	return pci_module_init (&tc35815_pci_driver);
}


static void __exit tc35815_cleanup_module (void)
{
	pci_unregister_driver (&tc35815_pci_driver);
}


module_init(tc35815_init_module);
module_exit(tc35815_cleanup_module);

MODULE_DESCRIPTION("TOSHIBA TC35815 PCI 10M/100M Ethernet driver");
MODULE_LICENSE("GPL");
