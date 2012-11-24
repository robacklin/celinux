/*
 * FILE NAME include/asm/it8172/pm.h
 *
 * BRIEF MODULE DESCRIPTION
 *
 * Author: MontaVista Software, Inc.
 *         support@mvista.com
 *
 * Copyright 2003 MontaVista Software Inc.
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

#ifndef __ASM_IT8172_PM_H
#define __ASM_IT8172_PM_H  

#ifndef __ASSEMBLER__
extern u64 jiffies_64;
extern void (*pm_idle)(void);
extern void it8172_pm_idle(void);
typedef enum idle_state { not_idle, idle } idle_state_t;
extern idle_state_t pm_idle_state;
extern idle_state_t vst_idle_state;
extern void vst_pm_idle(void);
extern unsigned long vst_threshold;
extern void update_jiffies_vst(void);

#ifndef CONFIG_HIGH_RES_RESOLUTION
!Err VST needs the HIGH_RES_RESOLUTION
#else
#define TIMER_TICKS_PER_JIFFIE	((CONFIG_HIGH_RES_RESOLUTION * 1000000) / 2 / HZ)
/*
 * MAX_TIMER_TICKS has some leeway to make sure the timer doesn't wrap
 * before we update jiffies.
 */
#define MAX_TIMER_TICKS (~(unsigned long)0 - (4 * TIMER_TICKS_PER_JIFFIE))
#endif /* CONFIG_HIGH_RES_RESOLUTION */
#endif	/* ASSEMBLER */
#endif
