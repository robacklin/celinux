/*
 * ibm_ocp_emac.h
 *
 *
 *      Armin Kuster akuster@mvista.com
 *      June, 2002
 *
 * Copyright 2002 MontaVista Softare Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR   IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  Version: 1.0: 06/02/02 - armin
 *  	pulled all emac defined out od enet.h
 *
 */

#ifndef _IBM_OCP_EMAC_H_
#define _IBM_OCP_EMAC_H_
/* General defines needed for the driver */

/* MODE REG 0 */
#define EMAC_M0_RXI			0x80000000
#define EMAC_M0_TXI			0x40000000
#define EMAC_M0_SRST			0x20000000
#define EMAC_M0_TXE			0x10000000
#define EMAC_M0_RXE			0x08000000
#define EMAC_M0_WKE			0x04000000

/* MODE Reg 1 */
#define EMAC_M1_FDE			0x80000000
#define EMAC_M1_ILE			0x40000000
#define EMAC_M1_VLE			0x20000000
#define EMAC_M1_EIFC			0x10000000
#define EMAC_M1_APP			0x08000000
#define EMAC_M1_AEMI			0x02000000
#define EMAC_M1_IST			0x01000000
#define EMAC_M1_MF_1000MBPS		0x00800000	/* 0's for 10MBPS */
#define EMAC_M1_MF_100MBPS		0x00400000
#define EMAC_M1_RFS_4K			0x00300000	/* ~4k for 512 byte */
#define EMAC_M1_RFS_2K			0x00200000
#define EMAC_M1_RFS_1K			0x00100000
#define EMAC_M1_TX_FIFO_2K		0x00080000	/* 0's for 512 byte */
#define EMAC_M1_TX_FIFO_1K		0x00040000
#define EMAC_M1_TR0_DEPEND		0x00010000	/* 0'x for single packet */
#define EMAC_M1_TR0_MULTI		0x00008000
#define EMAC_M1_TR1_DEPEND		0x00004000
#define EMAC_M1_TR1_MULTI		0x00002000
#define EMAC_M1_JUMBO_ENABLE		0x00001000

/* Transmit Mode Register 0 */
#define EMAC_TXM0_GNP0			0x80000000
#define EMAC_TXM0_GNP1			0x40000000
#define EMAC_TXM0_GNPD			0x20000000
#define EMAC_TXM0_FC			0x10000000

/* Receive Mode Register */
#define EMAC_RMR_SP			0x80000000
#define EMAC_RMR_SFCS			0x40000000
#define EMAC_RMR_ARRP			0x20000000
#define EMAC_RMR_ARP			0x10000000
#define EMAC_RMR_AROP			0x08000000
#define EMAC_RMR_ARPI			0x04000000
#define EMAC_RMR_PPP			0x02000000
#define EMAC_RMR_PME			0x01000000
#define EMAC_RMR_PMME			0x00800000
#define EMAC_RMR_IAE			0x00400000
#define EMAC_RMR_MIAE			0x00200000
#define EMAC_RMR_BAE			0x00100000
#define EMAC_RMR_MAE			0x00080000

/* Interrupt Status & enable Regs */
#define EMAC_ISR_OVR			0x02000000
#define EMAC_ISR_PP			0x01000000
#define EMAC_ISR_BP			0x00800000
#define EMAC_ISR_RP			0x00400000
#define EMAC_ISR_SE			0x00200000
#define EMAC_ISR_ALE			0x00100000
#define EMAC_ISR_BFCS			0x00080000
#define EMAC_ISR_PTLE			0x00040000
#define EMAC_ISR_ORE			0x00020000
#define EMAC_ISR_IRE			0x00010000
#define EMAC_ISR_DBDM			0x00000200
#define EMAC_ISR_DB0			0x00000100
#define EMAC_ISR_SE0			0x00000080
#define EMAC_ISR_TE0			0x00000040
#define EMAC_ISR_DB1			0x00000020
#define EMAC_ISR_SE1			0x00000010
#define EMAC_ISR_TE1			0x00000008
#define EMAC_ISR_MOS			0x00000002
#define EMAC_ISR_MOF			0x00000001

/* STA CONTROL REG */
#define EMAC_STACR_OC			0x00008000
#define EMAC_STACR_PHYE			0x00004000
#define EMAC_STACR_WRITE		0x00002000
#define EMAC_STACR_READ			0x00001000
#define EMAC_STACR_CLK_83MHZ		0x00000800	/* 0's for 50Mhz */
#define EMAC_STACR_CLK_66MHZ		0x00000400
#define EMAC_STACR_CLK_100MHZ		0x00000C00

/* Transmit Request Threshold Register */
#define EMAC_TRTR_256			0x18000000	/* 0's for 64 Bytes */
#define EMAC_TRTR_192			0x10000000
#define EMAC_TRTR_128			0x01000000


#define EMAC_TX_CTRL_GFCS		0x0200
#define EMAC_TX_CTRL_GP			0x0100
#define EMAC_TX_CTRL_ISA		0x0080
#define EMAC_TX_CTRL_RSA		0x0040
#define EMAC_TX_CTRL_IVT		0x0020
#define EMAC_TX_CTRL_RVT		0x0010

#define EMAC_TX_CTRL_DFLT ( \
	MAL_TX_CTRL_LAST | MAL_TX_CTRL_READY | MAL_TX_CTRL_INTR | \
	EMAC_TX_CTRL_GFCS | EMAC_TX_CTRL_GP )

/* madmal transmit status / Control bits */
#define EMAC_TX_ST_BFCS			0x0200
#define EMAC_TX_ST_BPP			0x0100
#define EMAC_TX_ST_LCS			0x0080
#define EMAC_TX_ST_ED			0x0040
#define EMAC_TX_ST_EC			0x0020
#define EMAC_TX_ST_LC			0x0010
#define EMAC_TX_ST_MC			0x0008
#define EMAC_TX_ST_SC			0x0004
#define EMAC_TX_ST_UR			0x0002
#define EMAC_TX_ST_SQE			0x0001

#define EMAC_TX_ST_DEFAULT		0x03F3

/* madmal receive status / Control bits */
#define EMAC_RX_ST_OE			0x0200
#define EMAC_RX_ST_PP			0x0100
#define EMAC_RX_ST_BP			0x0080
#define EMAC_RX_ST_RP			0x0040
#define EMAC_RX_ST_SE			0x0020
#define EMAC_RX_ST_AE			0x0010
#define EMAC_RX_ST_BFCS			0x0008
#define EMAC_RX_ST_PTL			0x0004
#define EMAC_RX_ST_ORE			0x0002
#define EMAC_RX_ST_IRE			0x0001
#define EMAC_BAD_RX_PACKET		0x02ff

/* all the errors we care about */
#define EMAC_RX_ERRORS			0x03FF

/* phy seed setup */
#define AUTO				99
#define _100BASET			100
#define _10BASET			10
#define HALF				22
#define FULL				44

#endif
