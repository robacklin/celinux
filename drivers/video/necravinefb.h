/*
 * BRIEF MODULE DESCRIPTION
 *	Hardware definitions for the NEC RavinE controller
 *
 *
 * Copyright 2003 mycable GmbH
 * Author: mycable GmbH
 *         Joerg Ritter jr@mycable.de
 *
 * 12-Jun-2003: first version, 640x480 only, RavinE DS2
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _NECRAVINE_H
#define _NECRAVINE_H

/********************************************************************/
#define uint32 unsigned long
/********************************************************************/

/*-------------------------------------------------------------------------*/
/* defines                                                                 */
/*-------------------------------------------------------------------------*/

#define RAVINE_FRAMEBUFFER_SIZE  0x007fffff /* display memory size         */
#define RAVINE_CPUWINDOW_SIZE    0x01fff000 /* CPU window size = 32 Mbyte  */
#define RAVINE_CTRLREGSIZE       0x00001000

#define RAVINE_TRASHPAGE         (RAVINE_FRAMEBUFFER_SIZE - 0x1000)

#define RGL_PALETTE_ENTRY(r,g,b) ((r) << 16 | (g) << 8 | (b))


/*-------------------------------------------------------------------------*/
/* registers                                                               */
/*-------------------------------------------------------------------------*/

//#define RAVINE_REG(offset) *(volatile unsigned long  *)(ravine_regbase + offset)

#define RAVINE_REG(offset) (ravine_regbase + offset) 
#define write_ravine(_r,_v) writel(_v,_r);
#define read_ravine(r)      readl(r)

/* video output [Vo] plane registers                                       */
/*=========================================================================*/

/*-------------------------------------------------------------------------*/
/* plane #0                                                                */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* memory registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin0StartAddr                RAVINE_REG(0x924)
#define RAVINE_VoWin0StrideX                  RAVINE_REG(0x568)
#define RAVINE_VoWin0StrideY                  RAVINE_REG(0x56c)
#define RAVINE_VoWin0VPStartX                 RAVINE_REG(0x5a8)
#define RAVINE_VoWin0VPStartY                 RAVINE_REG(0x5ac)

/*-------------------------------------------------------------------------*/
/* screen registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin0Height                   RAVINE_REG(0x5dc)
#define RAVINE_VoWin0Width                    RAVINE_REG(0x5d8)
#define RAVINE_VoWin0WStartX                  RAVINE_REG(0x608)
#define RAVINE_VoWin0WStartY                  RAVINE_REG(0x60c)

/*-------------------------------------------------------------------------*/
/* colorkey + const alpha                                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin0ColorKey                 RAVINE_REG(0x514)
#define RAVINE_VoAlpha0Const                  RAVINE_REG(0x624)

/*-------------------------------------------------------------------------*/
/* alpha and z plane                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoA0StartAddr                  RAVINE_REG(0x944)
#define RAVINE_VoZ0StartAddr                  RAVINE_REG(0x964)


/*-------------------------------------------------------------------------*/
/* plane #1                                                                */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* memory registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin1StartAddr                RAVINE_REG(0x920)
#define RAVINE_VoWin1StrideX                  RAVINE_REG(0x560)
#define RAVINE_VoWin1StrideY                  RAVINE_REG(0x564)
#define RAVINE_VoWin1VPStartX                 RAVINE_REG(0x5a0)
#define RAVINE_VoWin1VPStartY                 RAVINE_REG(0x5a4)

/*-------------------------------------------------------------------------*/
/* screen registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin1Height                   RAVINE_REG(0x5d4)
#define RAVINE_VoWin1Width                    RAVINE_REG(0x5d0)
#define RAVINE_VoWin1WStartX                  RAVINE_REG(0x600)
#define RAVINE_VoWin1WStartY                  RAVINE_REG(0x604)

/*-------------------------------------------------------------------------*/
/* colorkey + const alpha                                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin1ColorKey                 RAVINE_REG(0x510)
#define RAVINE_VoAlpha1Const                  RAVINE_REG(0x620)

/*-------------------------------------------------------------------------*/
/* alpha and z plane                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoA1StartAddr                  RAVINE_REG(0x940)
#define RAVINE_VoZ1StartAddr                  RAVINE_REG(0x960)


/*-------------------------------------------------------------------------*/
/* plane #2                                                                */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* memory registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin2StartAddr                RAVINE_REG(0x91c)
#define RAVINE_VoWin2StrideX                  RAVINE_REG(0x558)
#define RAVINE_VoWin2StrideY                  RAVINE_REG(0x55c)
#define RAVINE_VoWin2VPStartX                 RAVINE_REG(0x598)
#define RAVINE_VoWin2VPStartY                 RAVINE_REG(0x59c)

/*-------------------------------------------------------------------------*/
/* screen registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin2Height                   RAVINE_REG(0x5cc)
#define RAVINE_VoWin2Width                    RAVINE_REG(0x5c8)
#define RAVINE_VoWin2WStartX                  RAVINE_REG(0x5f8)
#define RAVINE_VoWin2WStartY                  RAVINE_REG(0x5fC)

/*-------------------------------------------------------------------------*/
/* colorkey + const alpha                                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin2ColorKey                 RAVINE_REG(0x50c)
#define RAVINE_VoAlpha2Const                  RAVINE_REG(0x61c)

/*-------------------------------------------------------------------------*/
/* alpha and z plane                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoA2StartAddr                  RAVINE_REG(0x93c)
#define RAVINE_VoZ2StartAddr                  RAVINE_REG(0x95c)


/*-------------------------------------------------------------------------*/
/* plane #3                                                                */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* memory registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin3StartAddr                RAVINE_REG(0x918)
#define RAVINE_VoWin3StrideX                  RAVINE_REG(0x550)
#define RAVINE_VoWin3StrideY                  RAVINE_REG(0x554)
#define RAVINE_VoWin3VPStartX                 RAVINE_REG(0x590) 
#define RAVINE_VoWin3VPStartY                 RAVINE_REG(0x594)

/*-------------------------------------------------------------------------*/
/* screen registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin3Height                   RAVINE_REG(0x5c4)
#define RAVINE_VoWin3Width                    RAVINE_REG(0x5c0)
#define RAVINE_VoWin3WStartX                  RAVINE_REG(0x5f0)
#define RAVINE_VoWin3WStartY                  RAVINE_REG(0x5f4)

/*-------------------------------------------------------------------------*/
/* colorkey + const alpha                                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin3ColorKey                 RAVINE_REG(0x508)
#define RAVINE_VoAlpha3Const                  RAVINE_REG(0x618)

/*-------------------------------------------------------------------------*/
/* alpha and z plane                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoA3StartAddr                  RAVINE_REG(0x938)
#define RAVINE_VoZ3StartAddr                  RAVINE_REG(0x958)


/*-------------------------------------------------------------------------*/
/* plane #4                                                                */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* memory registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin4StartAddr                RAVINE_REG(0x914)
#define RAVINE_VoWin4StrideX                  RAVINE_REG(0x548)
#define RAVINE_VoWin4StrideY                  RAVINE_REG(0x54c)
#define RAVINE_VoWin4VPStartX                 RAVINE_REG(0x588)
#define RAVINE_VoWin4VPStartY                 RAVINE_REG(0x58c)

/*-------------------------------------------------------------------------*/
/* screen registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin4Height                   RAVINE_REG(0x5bc)
#define RAVINE_VoWin4Width                    RAVINE_REG(0x5b8)
#define RAVINE_VoWin4WStartX                  RAVINE_REG(0x5e8)
#define RAVINE_VoWin4WStartY                  RAVINE_REG(0x5ec)

/*-------------------------------------------------------------------------*/
/* colorkey + const alpha                                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin4ColorKey                 RAVINE_REG(0x504)
#define RAVINE_VoAlpha4Const                  RAVINE_REG(0x614)

/*-------------------------------------------------------------------------*/
/* alpha and z plane                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoA4StartAddr                  RAVINE_REG(0x934)
#define RAVINE_VoZ4StartAddr                  RAVINE_REG(0x954)


/*-------------------------------------------------------------------------*/
/* plane #5                                                                */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* memory registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin5StartAddr                RAVINE_REG(0x910)
#define RAVINE_VoWin5StrideX                  RAVINE_REG(0x540)
#define RAVINE_VoWin5StrideY                  RAVINE_REG(0x544)
#define RAVINE_VoWin5VPStartX                 RAVINE_REG(0x580)
#define RAVINE_VoWin5VPStartY                 RAVINE_REG(0x584)

/*-------------------------------------------------------------------------*/
/* screen registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin5Height                   RAVINE_REG(0x5b4)
#define RAVINE_VoWin5Width                    RAVINE_REG(0x5b0)
#define RAVINE_VoWin5WStartX                  RAVINE_REG(0x5e0)
#define RAVINE_VoWin5WStartY                  RAVINE_REG(0x5e4)

/*-------------------------------------------------------------------------*/
/* colorkey + const alpha                                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWin5ColorKey                 RAVINE_REG(0x500)
#define RAVINE_VoAlpha5Const                  RAVINE_REG(0x610)

/*-------------------------------------------------------------------------*/
/* alpha and z plane                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoA5StartAddr                  RAVINE_REG(0x930)
#define RAVINE_VoZ5StartAddr                  RAVINE_REG(0x950)


/*-------------------------------------------------------------------------*/
/* plane work                                                              */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* memory registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoWorkStartAddr                RAVINE_REG(0x928)
#define RAVINE_VoWorkStrideX                  RAVINE_REG(0x570)
#define RAVINE_VoWorkStrideY                  RAVINE_REG(0x574)

/*-------------------------------------------------------------------------*/
/* alpha and z plane                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoAWorkStartAddr               RAVINE_REG(0x948)
#define RAVINE_VoZWorkStartAddr               RAVINE_REG(0x968)


/*-------------------------------------------------------------------------*/
/* video output control                                                    */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoColorLookUpTable             RAVINE_REG(0x000)
#define RAVINE_VoBgColor                      RAVINE_REG(0x518)
#define RAVINE_VoPlaneMode                    RAVINE_REG(0x520)
#define RAVINE_VoPlaneSelect                  RAVINE_REG(0x524)
#define RAVINE_VoPlaneOrder                   RAVINE_REG(0x528)
#define RAVINE_VoUpscale                      RAVINE_REG(0x530)
#define RAVINE_VoDisplayWidth                 RAVINE_REG(0x628)
#define RAVINE_VoDisplayHeight                RAVINE_REG(0x62c)

#define RAVINE_VoBufSwitchStatus              RAVINE_REG(0x90c)

#define RAVINE_VoControl                      RAVINE_REG(0xb00)
#define RAVINE_VoSyncStatus                   RAVINE_REG(0xb04)
#define RAVINE_VoActScanline                  RAVINE_REG(0xb08)
#define RAVINE_VoScanlineInt                  RAVINE_REG(0xb0c)
#define RAVINE_VoFrameCount                   RAVINE_REG(0xb60)

/*-------------------------------------------------------------------------*/
/* timing registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoHTotal                       RAVINE_REG(0xb10)
#define RAVINE_VoHSync1                       RAVINE_REG(0xb14)
#define RAVINE_VoHSync2                       RAVINE_REG(0xb18)
#define RAVINE_VoVTotal                       RAVINE_REG(0xb20)
#define RAVINE_VoVSync1                       RAVINE_REG(0xb24)
#define RAVINE_VoVSync2                       RAVINE_REG(0xb28)
#define RAVINE_VoExternalPllSet               RAVINE_REG(0xb30)
#define RAVINE_VoPixelClock                   RAVINE_REG(0xb34)
#define RAVINE_VoSyncSelect                   RAVINE_REG(0xb4c)

/*-------------------------------------------------------------------------*/
/* dim PWM output                                                          */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoDimSignal                    RAVINE_REG(0xb3c)
#define RAVINE_VoDimBase                      RAVINE_REG(0xb40)
#define RAVINE_VoDimCtrl                      RAVINE_REG(0xb44)
#define RAVINE_VoPwmCtrl                      RAVINE_REG(0xb48)


/*=========================================================================*/
/* host CPU interface [Host]                                               */
/*=========================================================================*/

#define RAVINE_HostControl                    RAVINE_REG(0x800)
#define RAVINE_HostStatus                     RAVINE_REG(0x804)
#define RAVINE_HostCmdPC                      RAVINE_REG(0x808)
#define RAVINE_HostCmdStop                    RAVINE_REG(0x80c)
#define RAVINE_HostIntThreshold               RAVINE_REG(0x810)
#define RAVINE_HostDma                        RAVINE_REG(0x828)
#define RAVINE_HostCache                      RAVINE_REG(0x82c)
#define RAVINE_HostIntEvent                   RAVINE_REG(0x830)
#define RAVINE_HostIntEnable                  RAVINE_REG(0x834)
#define RAVINE_HostIntClear                   RAVINE_REG(0x838)
#define RAVINE_HostReadyTimeout               RAVINE_REG(0x83c)
#define RAVINE_HostCmdFIFO                    RAVINE_REG(0x8e0)

#define RAVINE_HostCpuBaseAddr                RAVINE_REG(0x978)


/*=========================================================================*/
/* I2C interface [I2C]                                                     */
/*=========================================================================*/

#define RAVINE_I2CStatus                      RAVINE_REG(0x814)
#define RAVINE_I2CCommand                     RAVINE_REG(0x818)
#define RAVINE_I2CAddress                     RAVINE_REG(0x81c)
#define RAVINE_I2CData                        RAVINE_REG(0x820)
#define RAVINE_I2CTiming                      RAVINE_REG(0x824)


/*=========================================================================*/
/* memory interface [Mem]                                                  */
/*=========================================================================*/

#define RAVINE_MemControl                     RAVINE_REG(0x900)
#define RAVINE_MemSdramMode                   RAVINE_REG(0x904)
#define RAVINE_MemRefreshCycle                RAVINE_REG(0x908)
#define RAVINE_MemClockSet                    RAVINE_REG(0xb38)
#define RAVINE_MemClockDelay                  RAVINE_REG(0xb50)
#define RAVINE_MemClockDelayDDR               RAVINE_REG(0xb54)


/*=========================================================================*/
/* video input [vi]                                                        */
/*=========================================================================*/

#define RAVINE_ViStartAddr1                   RAVINE_REG(0x970)
#define RAVINE_ViStartAddr2                   RAVINE_REG(0x974)

#define RAVINE_ViStartX                       RAVINE_REG(0xa00)
#define RAVINE_ViStartY                       RAVINE_REG(0xa04)
#define RAVINE_ViScaledWidth                  RAVINE_REG(0xa08)
#define RAVINE_ViScaledHeight                 RAVINE_REG(0xa0C)
#define RAVINE_ViControl                      RAVINE_REG(0xa10)
#define RAVINE_ViFrameRate                    RAVINE_REG(0xa14)
#define RAVINE_ViDownscale                    RAVINE_REG(0xa18)
#define RAVINE_ViStatus                       RAVINE_REG(0xa20)
#define RAVINE_ViActScanline                  RAVINE_REG(0xa24)
#define RAVINE_ViScanlineInt                  RAVINE_REG(0xa28)
#define RAVINE_ViCSyncDecode                  RAVINE_REG(0xa2C)
#define RAVINE_ViStrideX                      RAVINE_REG(0xa30)
#define RAVINE_ViStrideY                      RAVINE_REG(0xa34)


/*=========================================================================*/
/* drawing engine [Draw]                                                   */
/*=========================================================================*/

/*-------------------------------------------------------------------------*/
/* common registers                                                        */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawControl                    RAVINE_REG(0xc00)
#define RAVINE_DrawStatus                     RAVINE_REG(0xcc8)
#define RAVINE_DrawPlaneSelect                RAVINE_REG(0xc04)
#define RAVINE_DrawOffset                     RAVINE_REG(0xc14)
#define RAVINE_DrawClipLeftTop                RAVINE_REG(0xc38)
#define RAVINE_DrawClipRightBottom            RAVINE_REG(0xc3c)
#define RAVINE_DrawFGColor                    RAVINE_REG(0xc64)
#define RAVINE_DrawBGColor                    RAVINE_REG(0xc68)

/*-------------------------------------------------------------------------*/
/* multiplexed registers                                                   */
/* used in Blt, Dda and Poly unit with different names                     */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawSetValue1                  RAVINE_REG(0xc08)
#define RAVINE_DrawSetValue2                  RAVINE_REG(0xc0c)
#define RAVINE_DrawSize                       RAVINE_REG(0xc10)

/*-------------------------------------------------------------------------*/
/* bitblt unit [DrawBlt]                                                   */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawBltSrc                     RAVINE_REG(0xc0c)
#define RAVINE_DrawBltDst                     RAVINE_REG(0xc08)
#define RAVINE_DrawBltSize                    RAVINE_REG(0xc10)
#define RAVINE_DrawBltPatControl              RAVINE_REG(0xc58)
#define RAVINE_DrawBltPatAddr                 RAVINE_REG(0xc98)
#define RAVINE_DrawBltAlphaConst              RAVINE_REG(0xcc0)
#define RAVINE_DrawBltSetAlphaPixel           RAVINE_REG(0xcc4)

/*-------------------------------------------------------------------------*/
/* line drawing unit [DrawDda]                                             */
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* basic registers                                                         */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawDdaStart                   RAVINE_REG(0xc08)
#define RAVINE_DrawDdaEnd                     RAVINE_REG(0xc0c)
#define RAVINE_DrawDdaLength                  RAVINE_REG(0xc10)
#define RAVINE_DrawDdaPels                    RAVINE_REG(0xc40)
#define RAVINE_DrawDdaMajor                   RAVINE_REG(0xc44)
#define RAVINE_DrawDdaMinor                   RAVINE_REG(0xc48)
#define RAVINE_DrawDdaEterm                   RAVINE_REG(0xc4c)
#define RAVINE_DrawDdaDirection               RAVINE_REG(0xc50)
#define RAVINE_DrawDdaAdjust                  RAVINE_REG(0xc54)

/*-------------------------------------------------------------------------*/
/* patterned lines [DrawDdaPat]                                            */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawDdaPatStart                RAVINE_REG(0xc5c)
#define RAVINE_DrawDdaPatEnd                  RAVINE_REG(0xc60)
#define RAVINE_DrawDdaPat0L                   RAVINE_REG(0xca0)
#define RAVINE_DrawDdaPat0H                   RAVINE_REG(0xca4)
#define RAVINE_DrawDdaPat1L                   RAVINE_REG(0xca8)
#define RAVINE_DrawDdaPat1H                   RAVINE_REG(0xcac)
#define RAVINE_DrawDdaPat2L                   RAVINE_REG(0xcb0)
#define RAVINE_DrawDdaPat2H                   RAVINE_REG(0xcb4)
#define RAVINE_DrawDdaPat3L                   RAVINE_REG(0xcb8)
#define RAVINE_DrawDdaPat3H                   RAVINE_REG(0xcbc)

/*-------------------------------------------------------------------------*/
/* shaded lines                                                            */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawDdaBlueStart               RAVINE_REG(0xc6c)
#define RAVINE_DrawDdaGreenStart              RAVINE_REG(0xc70)
#define RAVINE_DrawDdaRedStart                RAVINE_REG(0xc74)
#define RAVINE_DrawDdaBlueInc                 RAVINE_REG(0xc78)
#define RAVINE_DrawDdaGreenInc                RAVINE_REG(0xc7c)
#define RAVINE_DrawDdaRedInc                  RAVINE_REG(0xc80)

/*-------------------------------------------------------------------------*/
/* drawing lines with z values [DrawDdaZ]                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawDdaZStart                  RAVINE_REG(0xc84)
#define RAVINE_DrawDdaZStartFrac              RAVINE_REG(0xc88)
#define RAVINE_DrawDdaZInc                    RAVINE_REG(0xc8c)
#define RAVINE_DrawDdaZIncFrac                RAVINE_REG(0xc90)

/*-------------------------------------------------------------------------*/
/* polygon drawing unit [DrawPoly]                                         */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawPolyStart                  RAVINE_REG(0xc08)
#define RAVINE_DrawPolyEnd                    RAVINE_REG(0xc0c)
#define RAVINE_DrawPolyLength                 RAVINE_REG(0xc10)
#define RAVINE_DrawPolyXAxis                  RAVINE_REG(0xc18)
#define RAVINE_DrawPolySrc                    RAVINE_REG(0xc1c)

/*-------------------------------------------------------------------------*/
/* pattern sticking unit [DrawPs]                                          */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawPsDstA                     RAVINE_REG(0xc24)
#define RAVINE_DrawPsDstB                     RAVINE_REG(0xc28)
#define RAVINE_DrawPsDstC                     RAVINE_REG(0xc2c)
#define RAVINE_DrawPsDstD                     RAVINE_REG(0xc30)
#define RAVINE_DrawPsSize                     RAVINE_REG(0xc20)
#define RAVINE_DrawPsSrc                      RAVINE_REG(0xc9c)
#define RAVINE_DrawPsWorkOffset               RAVINE_REG(0xc34)


/*=========================================================================*/
/* system control [Sys]                                                    */
/*=========================================================================*/

#define RAVINE_SysControl                     RAVINE_REG(0xf00)
#define RAVINE_SysStatus                      RAVINE_REG(0xf04)
#define RAVINE_SysClockSet                    RAVINE_REG(0xf08)
#define RAVINE_SysResetEnable                 RAVINE_REG(0xf0c)


/*=========================================================================*/
/* register bit mask defines                                               */
/*=========================================================================*/

/*-------------------------------------------------------------------------*/
/* XY macros                                                               */
/*-------------------------------------------------------------------------*/

#define XY(x,y)                               ((y)    << 16) | ((x)    & 0xffff)
#define XYP(p)                                ((p).y  << 16) | ((p).x  & 0xffff)
#define XYP_P(p)                              ((p)->y << 16) | ((p)->x & 0xffff)


#define WH(x,y)                               XY((x)    -1, (y)    - 1) 
#define WHP(P)                                XY((P).x  -1, (P).y  - 1) 
#define WHP_P(P)                              XY((P)->x -1, (P)->y - 1) 

/*-------------------------------------------------------------------------*/
/* width height macro for Ravin-E                                          */
/* because width = 1 height = 1 draws a 2x2 square                         */
/*-------------------------------------------------------------------------*/

#define BITMASK(bits)                         ((1 << (bits)) - 1)

/*-------------------------------------------------------------------------*/
/* Ravin-E operating mode                                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_HostControl_DIRECT             0
#define RAVINE_HostControl_PATH               1
#define RAVINE_HostControl_LIST               2

/*-------------------------------------------------------------------------*/
/* Ravin-E interrupt bits: host interface                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_HostInt_Host_FIFO_EMPTY        (1 << 0)
#define RAVINE_HostInt_Host_FIFO_FULL         (1 << 1)
#define RAVINE_HostInt_Host_CMDLIST_FINISHED  (1 << 2)
#define RAVINE_HostInt_Host_NOTIFY            (1 << 3)
#define RAVINE_HostInt_Host_READY_TIMEOUT     (1 << 4)
#define RAVINE_HostInt_Host_CMDLIST_STOP      (1 << 5)

/*-------------------------------------------------------------------------*/
/* Ravin-E interrupt bits: video input                                     */
/*-------------------------------------------------------------------------*/

#define RAVINE_HostInt_Vi_SCANLINE            (1 << 6)
#define RAVINE_HostInt_Vi_NEXT_BUFFER         (1 << 7)
#define RAVINE_HostInt_Vi_BUFFER_OVERRUN      (1 << 8)

/*-------------------------------------------------------------------------*/
/* Ravin-E interrupt bits: video output                                    */
/*-------------------------------------------------------------------------*/

#define RAVINE_HostInt_Vo_SCANLINE            (1 << 12)
#define RAVINE_HostInt_Vo_NEXT_BUFFER         (1 << 13)
#define RAVINE_HostInt_Vo_BUFFER_UNDERRUN     (1 << 14)
#define RAVINE_HostInt_Vo_DIM_RAMP_FINISHED   (1 << 15)
#define RAVINE_HostInt_Vo_FRAMECOUNTER        (1 << 16)

/*-------------------------------------------------------------------------*/
/* Ravin-E interrupt bits: drawing engine                                  */
/*-------------------------------------------------------------------------*/

#define RAVINE_HostInt_Draw_IDLE              (1 << 18)

/*-------------------------------------------------------------------------*/
/* I2C bitmasks                                                            */
/*-------------------------------------------------------------------------*/

#define I2CSTS_BUSBUSY                        0x00000001
#define I2CSTS_ACK                            0x00000002
#define I2CSTS_SDA                            0x00000008
#define I2CSTS_SCL                            0x00000010
#define I2CCMD_START                          0x00000001
#define I2CCMD_STOP                           0x00000000
#define I2CADR_WRITE                          0x0000004a
#define I2CADR_READ                           0x0000004b

/*-------------------------------------------------------------------------*/
/* commands                                                                */
/*-------------------------------------------------------------------------*/

#define RAVINE_CMD_DIRECT                     0x00000000
#define RAVINE_CMD_NOTIFY                     0x00010000
#define RAVINE_CMD_STOP                       0x00020000
#define RAVINE_CMD_JUMP                       0x00030000
#define RAVINE_CMD_NOP                        0x00040000
#define RAVINE_CMD_RTS                        0x00050000
#define RAVINE_CMD_PSET                       0x10000000
#define RAVINE_CMD_LINE                       0x10010000
#define RAVINE_CMD_SLINE                      0x10020000
#define RAVINE_CMD_RLINE                      0x10030000
#define RAVINE_CMD_RECT                       0x10040000
#define RAVINE_CMD_COPY                       0x10050000
#define RAVINE_CMD_POLY                       0x10060000
#define RAVINE_CMD_SPOLY                      0x10070000
#define RAVINE_CMD_RPOLY                      0x10080000
#define RAVINE_CMD_PATSTICK                   0x10090000
#define RAVINE_CMD_LTSET                      0x100a0000
#define RAVINE_CMD_SDM                        0x100b0000
#define RAVINE_CMD_ZWM                        0x100c0000
#define RAVINE_CMD_SCOPE                      0x100d0000
#define RAVINE_CMD_PTSET                      0x100e0000
#define RAVINE_CMD_CFG                        0x100f0000
#define RAVINE_CMD_CBG                        0x10100000
#define RAVINE_CMD_SOFA                       0x10110000
#define RAVINE_CMD_DMODE                      0x10120000
#define RAVINE_CMD_LINE_R                     0x10130000
#define RAVINE_CMD_SLINE_R                    0x10140000
#define RAVINE_CMD_RLINE_R                    0x10150000
#define RAVINE_CMD_SET_PIXEL                  0x10160000
#define RAVINE_CMD_SET_PIXEL_R                0x10170000
#define RAVINE_CMD_SET_PIXEL_C                0x10180000
#define RAVINE_CMD_NONE                       0xffffffff

/*-------------------------------------------------------------------------*/
/* drawing control                                                         */
/*-------------------------------------------------------------------------*/

#define RAVINE_DrawControl_HDE                0x00000001
#define RAVINE_DrawControl_EDW                0x00000002
#define RAVINE_DrawControl_SEA                0x00000004
#define RAVINE_DrawControl_PEN                0x00000008
#define RAVINE_DrawControl_PET                0x00000010
#define RAVINE_DrawControl_PCE                0x00000020
#define RAVINE_DrawControl_SET                0x00000040
#define RAVINE_DrawControl_SRC                0x00000080
#define RAVINE_DrawControl_FNC                0x0000ff00
#define RAVINE_DrawControl_FNC_NPNSND         0x00000100
#define RAVINE_DrawControl_FNC_NPNSD          0x00000200
#define RAVINE_DrawControl_FNC_NPSND          0x00000400
#define RAVINE_DrawControl_FNC_NPSD           0x00000800
#define RAVINE_DrawControl_FNC_PNSND          0x00001000
#define RAVINE_DrawControl_FNC_PNSD           0x00002000
#define RAVINE_DrawControl_FNC_PSND           0x00004000
#define RAVINE_DrawControl_FNC_PSD            0x00008000

#define RAVINE_DrawControl_FNC_BLACKNESS      0x00000000    /* 0       */
#define RAVINE_DrawControl_FNC_NOTSRCERASE    0x00001100    /* DSon    */
#define RAVINE_DrawControl_FNC_NOTSRCCOPY     0x00003300    /* Sn      */
#define RAVINE_DrawControl_FNC_SRCERASE       0x00004400    /* SDna    */
#define RAVINE_DrawControl_FNC_DSTINVERT      0x00005500    /* Dn      */
#define RAVINE_DrawControl_FNC_PATINVERT      0x00005A00    /* DPx     */
#define RAVINE_DrawControl_FNC_SRCINVERT      0x00006600    /* DSx     */
#define RAVINE_DrawControl_FNC_SRCAND         0x00008800    /* DSa     */
#define RAVINE_DrawControl_FNC_MERGEPAINT     0x0000bb00    /* DSno    */
#define RAVINE_DrawControl_FNC_MERGECOPY      0x0000c000    /* PSa     */
#define RAVINE_DrawControl_FNC_SRCCOPY        0x0000cc00    /* S       */
#define RAVINE_DrawControl_FNC_SRCPAINT       0x0000ee00    /* DSo     */
#define RAVINE_DrawControl_FNC_PATCOPY        0x0000f000    /* P       */
#define RAVINE_DrawControl_FNC_PATPAINT       0x0000fb00    /* DPSnoo  */
#define RAVINE_DrawControl_FNC_WHITENESS      0x0000ff00    /* 1       */

#define RAVINE_DrawPlaneSelect_SRCG(src)      (src)
#define RAVINE_DrawPlaneSelect_DSTG(dst)      ((dst) << 4)
#define RAVINE_DrawPlaneSelect_DDAG(dst)      ((dst) << 8)

#define RAVINE_FNC_NPNSND                     0x00000001
#define RAVINE_FNC_NPNSD                      0x00000002
#define RAVINE_FNC_NPSND                      0x00000004
#define RAVINE_FNC_NPSD                       0x00000008
#define RAVINE_FNC_PNSND                      0x00000010
#define RAVINE_FNC_PNSD                       0x00000020
#define RAVINE_FNC_PSND                       0x00000040
#define RAVINE_FNC_PSD                        0x00000080
#define RAVINE_FNC_PSD                        0x00000080

#define RAVINE_FNC_BLACKNESS                  0x00000000    /* 0       */
#define RAVINE_FNC_NOTSRCERASE                0x00000011    /* DSon    */
#define RAVINE_FNC_NOTSRCCOPY                 0x00000033    /* Sn      */
#define RAVINE_FNC_SRCERASE                   0x00000044    /* SDna    */
#define RAVINE_FNC_DSTINVERT                  0x00000055    /* Dn      */
#define RAVINE_FNC_PATINVERT                  0x0000005A    /* DPx     */
#define RAVINE_FNC_SRCINVERT                  0x00000066    /* DSx     */
#define RAVINE_FNC_SRCAND                     0x00000088    /* DSa     */
#define RAVINE_FNC_MERGEPAINT                 0x000000bb    /* DSno    */
#define RAVINE_FNC_MERGECOPY                  0x000000c0    /* PSa     */
#define RAVINE_FNC_SRCCOPY                    0x000000cc    /* S       */
#define RAVINE_FNC_SRCPAINT                   0x000000ee    /* DSo     */
#define RAVINE_FNC_PATCOPY                    0x000000f0    /* P       */
#define RAVINE_FNC_PATPAINT                   0x000000fb    /* DPSnoo  */
#define RAVINE_FNC_WHITENESS                  0x000000ff    /* 1       */

#define RAVINE_DrawControl_SEN                0x00020000
#define RAVINE_DrawControl_SHM                0x00040000
#define RAVINE_DrawControl_ZWE                0x00080000
#define RAVINE_DrawControl_ZC                 0x00700000
#define RAVINE_DrawControl_HDS                0x07000000

#define RAVINE_DrawControl_HDS_Dda            0x00000000
#define RAVINE_DrawControl_HDS_Blt            0x01000000
#define RAVINE_DrawControl_HDS_Poly           0x02000000
#define RAVINE_DrawControl_HDS_Ps             0x04000000

#define RAVINE_DrawControl_AEN                (1 << 28)
#define RAVINE_DrawControl_ADT                (1 << 29)
#define RAVINE_DrawControl_BPP                (1 << 31)

#define RAVINE_DrawDdaPatStart_PID(arg)       ((arg) << 6)
#define RAVINE_DrawBltPatControl_PSI(arg)     ((arg) << 16)
#define RAVINE_DrawBltPatControl_PSX(arg)     ((arg) << 0)
#define RAVINE_DrawBltPatControl_PSY(arg)     ((arg) << 8)
#define RAVINE_DrawBltPatControl_PSIZE(arg)   ((arg) << 24)

/*-------------------------------------------------------------------------*/
/* video out control                                                       */
/*-------------------------------------------------------------------------*/

#define RAVINE_VoControl_DST                  (1 << 0)
#define RAVINE_VoControl_DDA                  (1 << 2)
#define RAVINE_VoControl_SWN                  (1 << 4)
#define RAVINE_VoControl_VBS                  (1 << 8)
#define RAVINE_VoControl_DAC                  (1 << 12)
#define RAVINE_VoControl_DIG                  (1 << 13)
#define RAVINE_VoControl_DPLL                 (1 << 14)
#define RAVINE_VoControl_CNT(arg)             ((arg) << 16)
#define RAVINE_VoControl_CLK(arg)             ((arg) << 24)

#define RAVINE_VoPlaneMode_HLDO(arg)          ((arg) << 0)
#define RAVINE_VoPlaneMode_INTER(arg)         ((arg) << 2)
#define RAVINE_VoPlaneMode_SYN                (1 << 4)
#define RAVINE_VoPlaneMode_WIN0(arg)          ((arg) << 8)
#define RAVINE_VoPlaneMode_WIN1(arg)          ((arg) << 10)
#define RAVINE_VoPlaneMode_WIN2(arg)          ((arg) << 12)
#define RAVINE_VoPlaneMode_WIN3(arg)          ((arg) << 14)
#define RAVINE_VoPlaneMode_WIN4(arg)          ((arg) << 16)
#define RAVINE_VoPlaneMode_WIN5(arg)          ((arg) << 18)

#define RAVINE_VoPlaneSelect_DE_WIN0          (1 << (0 + 0))
#define RAVINE_VoPlaneSelect_DE_WIN1          (1 << (1 + 0))
#define RAVINE_VoPlaneSelect_DE_WIN2          (1 << (2 + 0))
#define RAVINE_VoPlaneSelect_DE_WIN3          (1 << (3 + 0))
#define RAVINE_VoPlaneSelect_DE_WIN4          (1 << (4 + 0))
#define RAVINE_VoPlaneSelect_DE_WIN5          (1 << (5 + 0))
#define RAVINE_VoPlaneSelect_CKE_WIN0         (1 << (0 + 8))
#define RAVINE_VoPlaneSelect_CKE_WIN1         (1 << (1 + 8))
#define RAVINE_VoPlaneSelect_CKE_WIN2         (1 << (2 + 8))
#define RAVINE_VoPlaneSelect_CKE_WIN3         (1 << (3 + 8))
#define RAVINE_VoPlaneSelect_CKE_WIN4         (1 << (4 + 8))
#define RAVINE_VoPlaneSelect_CKE_WIN5         (1 << (5 + 8))
#define RAVINE_VoPlaneSelect_AE_WIN0          (1 << (0 + 16))
#define RAVINE_VoPlaneSelect_AE_WIN1          (1 << (1 + 16))
#define RAVINE_VoPlaneSelect_AE_WIN2          (1 << (2 + 16))
#define RAVINE_VoPlaneSelect_AE_WIN3          (1 << (3 + 16))
#define RAVINE_VoPlaneSelect_AE_WIN4          (1 << (4 + 16))
#define RAVINE_VoPlaneSelect_AE_WIN5          (1 << (5 + 16))
#define RAVINE_VoPlaneSelect_CON_WIN0         (1 << (0 + 24))
#define RAVINE_VoPlaneSelect_CON_WIN1         (1 << (1 + 24))
#define RAVINE_VoPlaneSelect_CON_WIN2         (1 << (2 + 24))
#define RAVINE_VoPlaneSelect_CON_WIN3         (1 << (3 + 24))
#define RAVINE_VoPlaneSelect_CON_WIN4         (1 << (4 + 24))
#define RAVINE_VoPlaneSelect_CON_WIN5         (1 << (5 + 24))

#define RAVINE_VoPlaneOrder_POWIN0(arg)       ((arg) << 0)
#define RAVINE_VoPlaneOrder_POWIN1(arg)       ((arg) << 4)
#define RAVINE_VoPlaneOrder_POWIN2(arg)       ((arg) << 8)
#define RAVINE_VoPlaneOrder_POWIN3(arg)       ((arg) << 12)
#define RAVINE_VoPlaneOrder_POWIN4(arg)       ((arg) << 16)
#define RAVINE_VoPlaneOrder_POWIN5(arg)       ((arg) << 20)

#define RAVINE_Get_VoPlaneOrder_POWIN0(arg)   (((arg) >> 0)  & 0x7)
#define RAVINE_Get_VoPlaneOrder_POWIN1(arg)   (((arg) >> 4)  & 0x7)
#define RAVINE_Get_VoPlaneOrder_POWIN2(arg)   (((arg) >> 8)  & 0x7)
#define RAVINE_Get_VoPlaneOrder_POWIN3(arg)   (((arg) >> 12) & 0x7)
#define RAVINE_Get_VoPlaneOrder_POWIN4(arg)   (((arg) >> 16) & 0x7)
#define RAVINE_Get_VoPlaneOrder_POWIN5(arg)   (((arg) >> 20) & 0x7)

#define RAVINE_VoCSyncDecode_14SCN(arg)       ((arg) << 0)
#define RAVINE_VoCSyncDecode_34SCN(arg)       ((arg) << 16)

#define RAVINE_VoBufSwitchStatus_WIN0         (1 << 0)
#define RAVINE_VoBufSwitchStatus_WIN1         (1 << 1)
#define RAVINE_VoBufSwitchStatus_WIN2         (1 << 2)
#define RAVINE_VoBufSwitchStatus_WIN3         (1 << 3)
#define RAVINE_VoBufSwitchStatus_WIN4         (1 << 4)
#define RAVINE_VoBufSwitchStatus_WIN5         (1 << 5)

#define RAVINE_VoBufSwitchStatus_A0           (1 << 6)
#define RAVINE_VoBufSwitchStatus_A1           (1 << 7)
#define RAVINE_VoBufSwitchStatus_A2           (1 << 8)
#define RAVINE_VoBufSwitchStatus_A3           (1 << 9)
#define RAVINE_VoBufSwitchStatus_A4           (1 << 10)
#define RAVINE_VoBufSwitchStatus_A5           (1 << 11)

#define RAVINE_VoBufSwitchStatus_VI1          (1 << 12)
#define RAVINE_VoBufSwitchStatus_VI2          (1 << 13)
#define RAVINE_VoBufSwitchStatus_CPU          (1 << 14)

#define RAVINE_VoUpscale_N(arg)               (arg)
#define RAVINE_VoUpscale_M(arg)               ((arg) << 8)
#define RAVINE_VoUpscale_UE_WIN0              (1 << 16)
#define RAVINE_VoUpscale_UE_WIN1              (1 << 17)
#define RAVINE_VoUpscale_UE_WIN2              (1 << 18)
#define RAVINE_VoUpscale_UE_WIN3              (1 << 19)
#define RAVINE_VoUpscale_UE_WIN4              (1 << 20)
#define RAVINE_VoUpscale_UE_WIN5              (1 << 21)
#define RAVINE_VoUpscale_CSWIN0(arg)          ((arg) << 24)

#define RAVINE_VoAlpha0Const_CON              (1<<16)
#define RAVINE_VoAlpha0Const_INV              (1<<16)
#define RAVINE_VoAlpha1Const_CON              (1<<16)
#define RAVINE_VoAlpha1Const_INV              (1<<16)                
#define RAVINE_VoAlpha2Const_CON              (1<<16)
#define RAVINE_VoAlpha3Const_INV              (1<<16)                
#define RAVINE_VoAlpha4Const_CON              (1<<16)
#define RAVINE_VoAlpha4Const_INV              (1<<16)                
#define RAVINE_VoAlpha5Const_CON              (1<<16)
#define RAVINE_VoAlpha5Const_INV              (1<<16)

#define RAVINE_VoFrameCount_INTF(arg)         ((arg) & 0x7ff)
#define RAVINE_Get_VoFrameCount_ACTF(arg)     (((arg) >> 16) & 0x7ff)
#define RAVINE_VoFrameCount_ENB               (1 << 28)
#define RAVINE_VoFrameCount_RST               (1 << 29)

#define RAVINE_NOWAIT                         (1 << 28)

#define RAVINE_MemSdramMode_ST                (1 << 16)

#define RAVINE_ViControl_DATA_CAPTURE         (1 << 0)
#define RAVINE_ViControl_INTERLACE_CAPTURE    (1 << 1)
#define RAVINE_ViControl_INVERT_INPUT_CLK     (1 << 2)
#define RAVINE_ViControl_CSYNC_IN_INV         (1 << 3)
#define RAVINE_ViControl_CAPTURE_MODE_8BIT    (1 << 4)
#define RAVINE_ViControl_ITU_IGNORE_00        (1 << 5)
#define RAVINE_ViControl_CAPTURE_MODE_RGB     (1 << 6)
#define RAVINE_ViControl_IGNORE_ERROR         (1 << 7)
#define RAVINE_ViControl_CAPTURE_FIRST_FIELD  (1 << 8)
#define RAVINE_ViControl_CAPTURE_SECOND_FIELD (1 << 9)
#define RAVINE_ViControl_DOUBLE_BUFFER_MODE   (1 << 10)
#define RAVINE_ViControl_USE_EXTERNAL_SYNC    (1 << 11)
#define RAVINE_ViControl_USE_EXTERNAL_HSYNC   (1 << 12)
#define RAVINE_ViControl_SECOND_FIELD_FIRST   (1 << 16)

#define RAVINE_ViDownScale_X_N(arg)           ((arg) << (3))
#define RAVINE_ViDownScale_X_M(arg)           ((arg) << (11))
#define RAVINE_ViDownScale_Y_N(arg)           ((arg) << (23))
#define RAVINE_ViDownScale_Y_M(arg)           ((arg) << (37))

#define RAVINE_PAL                            1
#define RAVINE_NTSC                           0

#define BMG0                                  0
#define BMG1                                  1
#define BMG2                                  2
#define BMG3                                  3
#define BMG4                                  4
#define BMG5                                  5
#define ALPHA0                                6
#define ALPHA1                                7
#define ALPHA2                                8
#define ALPHA3                                9
#define ALPHA4                                10
#define ALPHA5                                11
#define BMGWORK                               6
#define BMGZ                                  (1 << 3)



#undef EX                                   /* declaration in the module   */
#ifndef _AMM_RAVINE                         /* AMM_RavinE.c, extern in     */
#define EX extern                           /* others                      */
#else
#define EX
#endif


/*-------------------------------------------------------------------------*/
/*  variable type definitions                                              */
/*-------------------------------------------------------------------------*/

typedef struct {
  int 		total;
  int 		blank_width;
  int 		front_porch;
  int 		sync_width;
} tRGL_SUBTIMING;

typedef struct {
  const char     *name;
  int            screen_width;
  int            screen_height;
  tRGL_SUBTIMING subtiming_h;
  tRGL_SUBTIMING subtiming_v;
  unsigned long  vo_pixel_clock;
  int            v_freq;
} tRGL_TIMING;

typedef struct {
  u16 		x;
  u16 		y;
} tRGL_POINT;

typedef struct {
  u32 		x;
  u32 		y;
} tRGL_LPOINT;

typedef struct {
  int 		left;
  int 		top;
  int 		right;
  int 		bottom;
} tRGL_RECT;

typedef struct {
  u32 		lo;
  u32 		hi;
  u32 		length;
} tRGL_DDA_PATTERN;

typedef struct {
  u32     	address_offset;
  tRGL_POINT 	offset;
  unsigned int 	type;
} tRGL_BLT_PATTERN;


/*-------------------------------------------------------------------------*/
/*  display parameters                                                     */
/*-------------------------------------------------------------------------*/

tRGL_TIMING defd_timings[] = {
	/* 640x480 */
	{
		"VGA_640x480", 	/* name	*/
		640,		/* screen_width	*/
		480,		/* screen_height */
		{
			800,	/* subtiming_h.total        */
			160,	/* subtiming_h.blank_width  */
			16,     /* subtiming_h.front_porch  */
			34	/* subtiming_h.sync_width   */
		},
		{
			525,    /*   subtiming_v.total       */
			45,     /*   subtiming_v.blank_width */
			10,     /*   subtiming_v.front_porch */
			2       /*   subtiming_v.sync_width  */
		},
		0x00021003,     /*   vo_pixel_clock          */
		0               /*   v_freq                  */
	},
	{
		"XENARC_800x480", /* name  */
		800,              /* screen_width */
		480,              /* screen_height */
		{
			960,      /* subtiming_h.total        */
			160,      /* subtiming_h.blank_width  */
			24,       /* subtiming_h.front_porch  */
			34        /* subtiming_h.sync_width   */
		},
		{
			525,      /*   subtiming_v.total       */
			45,       /*   subtiming_v.blank_width */
			10,       /*   subtiming_v.front_porch */
			2         /*   subtiming_v.sync_width  */
		},
		0x00021003,       /*   vo_pixel_clock          */
		0                 /*   v_freq                  */
	},
	{
		"SVGA_800x600",   /* name  */
		800,              /* screen_width */
		600,              /* screen_height */
		{
			992,              /* subtiming_h.total */
			192,              /* subtiming_h.blank_width  */
			10,               /* subtiming_h.front_porch  */
			34                /* subtiming_h.sync_width   */
		},
		{
			645,              /*   subtiming_v.total       */
			45,               /*   subtiming_v.blank_width */
			10,               /*   subtiming_v.front_porch */
			2                 /*   subtiming_v.sync_width  */
		},
		0x00021003,               /*   vo_pixel_clock          */
		0                         /*   v_freq                  */
	}

};

#define NUM_TIMINGS		3

/*-------------------------------------------------------------------------*/
/*  function prototypes                                                    */
/*-------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

int amm_init_ravine      (u32, u32, u32, tRGL_TIMING *);
int amm_ravine_settiming (tRGL_TIMING *);

#ifdef __cplusplus
}
#endif


/*-------------------------------------------------------------------------*/
/*  global variables                                                       */
/*-------------------------------------------------------------------------*/

#undef EX                                   /* declaration in the module   */
#ifndef _AMM_RAVINE                         /* AMM_RavinE.c, extern in     */
#define EX extern                           /* others                      */
#else
#define EX
#endif

EX u32 ravine_mem_noburst;               /* Ravin-E memory base         */
EX u32 ravine_mem_burst;                 /* Ravin-E memory base, burst  */
                                            /* access, write burst only!   */

/*-------------------------------------------------------------------------*/
/* defines                                                                 */
/*-------------------------------------------------------------------------*/

//#define RAVINE_REG(offset) *(volatile unsigned long  *)(ravine_regbase + offset)

#define PCI_CONFIG_SPACE_BASE_ADDRESS 	0x600000800
#define PCI_RAVINE_VENDORID             0x1033
#define PCI_RAVINE_DEVICEID             0x0111
#define PCI_VENDORID_MASK               0x0000FFFF
#define PCI_DEVICEID_MASK               0xFFFF0000

//#define PRINT_FUNCTIONNAME printk("<1>necravinefb: %s\n", __FUNCTION__)
#define PRINT_FUNCTIONNAME 

// some offsets for Au1500 clock registers
#define sys_cntctrl          (0x0014/4)
#define sys_freqctrl0        (0x0020/4)
#define sys_freqctrl1        (0x0024/4)
#define sys_clksrc           (0x0028/4)
#define sys_auxpll           (0x0064/4)

#define pci_cmem             (0x0000/4)
#define pci_config           (0x0004/4)
#define pci_b2bmask_cch      (0x0008/4)
#define pci_b2bbase0_venid   (0x000C/4)
#define pci_b2bbase1_id      (0x0010/4)
#define pci_mwmask_dev       (0x0014/4)
#define pci_mwbase_rev_ccl   (0x0018/4)
#define pci_err_addr         (0x001C/4)
#define pci_spec_intack      (0x0020/4)
#define pci_id               (0x0100/4)
#define pci_statcmd          (0x0104/4)
#define pci_classrev         (0x0108/4)
#define pci_hdrtype          (0x010C/4)
#define pci_mbar             (0x0110/4)

#define RES_800x600
#undef RES_640x480  
#undef RES_800x480  



/*-------------------------------------------------------------------------*/
/* function prototypes                                                     */
/*-------------------------------------------------------------------------*/
#if 0
void ravine_module_info(disp_adapter_t *adapter, disp_module_info_t *info);

void ravine_flushrect(disp_draw_context_t *ctx, int x1, int y1, int x2, int y2);

/////
// core functions
/////

void ravine_draw_span(disp_draw_context_t *ctx, disp_color_t color, int x1, int x2, int y);
void ravine_draw_span_list(disp_draw_context_t *ctx, int count, disp_color_t color, 
       int x1[], int x2[], int y[]);
void ravine_draw_solid_rect(disp_draw_context_t *ctx, disp_color_t color, int x1, 
       int y1, int x2, int y2);
void ravine_draw_line_pat8x1(disp_draw_context_t *ctx, disp_color_t fgcolor, 
       disp_color_t bgcolor, int x1, int x2, int y, u8 pattern);
void ravine_draw_line_trans8x1(disp_draw_context_t *ctx, disp_color_t color, 
       int x1, int x2, int y, u8 pattern);
void ravine_draw_rect_pat8x8(disp_draw_context_t *ctx, disp_color_t fgcolor, 
       disp_color_t bgcolor, int x1, int y1, int x2, int y2);
void ravine_draw_rect_trans8x8(disp_draw_context_t *ctx, disp_color_t color, 
       int x1, int y1, int x2, int y2);
void ravine_blit1(disp_draw_context_t *ctx, int sx, int sy, int dx, int dy, int width, int height);
void ravine_blit2(disp_draw_context_t *ctx, disp_surface_t *src, 
       disp_surface_t *dst, int sx, int sy, int dx, int dy, int width, int height);
void ravine_draw_bitmap(disp_draw_context_t *ctx,
       u8 *image, int sstride, int bit0_offset,
       disp_color_t fgcolor, disp_color_t bgcolor,
       int transparent, int dx, int dy, int width, int height);
void ravine_update_draw_surface(disp_draw_context_t *ctx);
void ravine_update_pattern(disp_draw_context_t *ctx);
void ravine_load_pattern(disp_draw_context_t *ctx);

void ravine_wait_idle(disp_draw_context_t *context);
int  ravine_hw_idle(disp_adapter_t *context, void *ignored);

#endif
////////////////



#endif /* _NECRAVINE_H */
