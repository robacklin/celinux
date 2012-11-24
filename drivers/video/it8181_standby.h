/*  linux/drivers/video/it8181_standby.h --
 *                IT8181 console frame buffer driver
 * 
 * Copyright (C) 2003 MontaVista Software, Inc.
 *  Author: MontaVista Software, Inc.
 *     source@mvista.com
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
 */

#define IT8181_STANDBY_OSCI_STOP                0x00000001
#define IT8181_STANDBY_PLL1_STANDBY             0x00000004
#define IT8181_STANDBY_PLL2_STANDBY             0x00000008
#define IT8181_STANDBY_DAC_POWER_DOWN           0x00000010
#define IT8181_STANDBY_DAC_CLK_STOP             0x00000020
#define IT8181_STANDBY_LINE_DRAW_STANDBY        0x00000040
#define IT8181_STANDBY_BITBLT_STANDBY           0x00000080
#define IT8181_STANDBY_HARDWARECURSOR_STANDBY   0x00000100
#define IT8181_STANDBY_PALETTERAM_STANDBY       0x00000200
#define IT8181_STANDBY_PLL1_PDN                 0x00001000
#define IT8181_STANDBY_PLL2_PDN                 0x00002000

#define IT8181_EXA_INDEX_REG  0x03D6
#define IT8181_EXA_PORT_REG   0x03D7
#define IT8181_EXA98_INDEX    0x98
#define IT8181_EXA98_ESW      0x20
