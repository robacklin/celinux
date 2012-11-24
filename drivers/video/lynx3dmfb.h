/*
 * BRIEF MODULE DESCRIPTION
 *	Hardware definitions for the Lynx3DM+ controller
 *
 *
 * Copyright 2003 mycable GmbH
 * Author: mycable GmbH
 *         Joerg Ritter jr@mycable.de
 *
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


 
 
#define PCI_SMI_VENDORID                0x126F
#define PCI_LYNX3DMP_DEVICEID           0x0720

 
/*-------------------------------------------------------------------------*/
/* registers                                                               */
/*-------------------------------------------------------------------------*/

//#define RAVINE_REG(offset) *(volatile unsigned long  *)(ravine_regbase + offset)

#define LYNX_REG(offset) (ravine_regbase + offset) 
#define write_lynx(_r,_v) writel(_v,_r);
#define read_lynx(r)      readl(r)

#define RES_800x600
#undef RES_640x480  
#undef RES_800x480  


#define VPR32(vpr) *(volatile uint32_t*) &VPR[vpr]
#define DPR16(vpr) *(volatile uint16_t*) &DPR[vpr]
#define DPR32(vpr) *(volatile uint32_t*) &DPR[vpr]

