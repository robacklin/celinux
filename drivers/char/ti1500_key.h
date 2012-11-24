/*
 *
 * FILE NAME ti1500_key.h
 *
 * BRIEF MODULE DESCRIPTION
 *  FPGA0 Driver for LCD, Buttons, GPIO and IRQ control for the XXS_1500 board
 *
 * Copyright 2003 mycable GmbH
 * Author: mycable GmbH
 *         linux@mycable.de
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
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
 * Notes:
 *
 *  Revision history
 *  Revision History:
 *  -----------------
 *  18.06.2003  TK  0.1   Initial release
 *  15.07.2003  TK  0.2   Update to FPGA 0.51
 *  21.07.2003  TK  0.21  Update to FPGA 0.52
 *  28.07.2003  TK  0.53.1  Update to FPGA 0.53  New version numbers
 */

/*-------------------------------------------------------------------------*/
/* defines                                                                 */
/*-------------------------------------------------------------------------*/

#define AMM_PB_NONE    0x00                 /* defines for amm_check_pb()  */

#define AMM_PB_LEFT    0x20
#define AMM_PB_RIGHT   0x04
#define AMM_PB_UP      0x10
#define AMM_PB_DOWN    0x08

#define AMM_PB_ESC     0x01
#define AMM_PB_RETURN  0x02

#define AMM_PB_ON      0x80

#define AMM_PB_STATUS  (uint8_t)*(volatile uint32_t *)amm_pb_base

#define AMM_PB_ON_GPIO AU1500_PGPIO, 2      /* primary GPIO #06, input,irq */
