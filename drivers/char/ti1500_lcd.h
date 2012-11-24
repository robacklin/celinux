/*
 *
 * FILE NAME ti1500_lcd.h
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
/* alphanumeric LCD commands                                               */
/*-------------------------------------------------------------------------*/

#define ALCD_CLEAR_DISPLAY    0x01
#define ALCD_CURSOR_HOME      0x02
#define ALCD_ENTRY_MODE       0x06      /* DD-RAM increment, shift cursor  */
#define ALCD_DISPLAY_CONTROL  0x08      /* display and cursor contol       */
#define ALCD_DISPLAY_ON       0x04      /* display on flag                 */
#define ALCD_DISPLAY_OFF      0x00      /* display off                     */
#define ALCD_CURSOR_ON        0x02      /* cursor on flag                  */
#define ALCD_CURSOR_OFF       0x00      /* cursor off                      */
#define ALCD_CBLINK_ON        0x01      /* cursor blink on flag            */
#define ALCD_CBLINK_OFF       0x00      /* cursor blink off                */
#define ALCD_CSHIFT_LEFT      0x10      /* shift cursor left               */
#define ALCD_CSHIFT_RIGHT     0x14      /* shift cursor right              */
#define ALCD_DSHIFT_LEFT      0x18      /* shift display left              */
#define ALCD_DSHIFT_RIGHT     0x1c      /* shift display right             */
#define ALCD_FUNCTION_SET     0x3c      /* 2 lines, 8bit data              */
#define ALCD_CGRAM_SET        0x40      /* set CG RAM address              */
#define ALCD_DDRAM_SET        0x80      /* set DD RAM address              */

#define ALCD_CPL              16

#define ALCD_ON               TRUE
#define ALCD_OFF              FALSE



/*-------------------------------------------------------------------------*/
/*  debug                                                                  */
/*-------------------------------------------------------------------------*/

#define PRINT_FUNCTIONNAME printk("<1>dev-dbglcd: %s\n", __FUNCTION__)

/*-------------------------------------------------------------------------*/
/*  function prototypes                                                    */
/*-------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

int  amm_init_alcd       (uint32_t);
void amm_clear_alcd      (void);
void amm_switch_alcd     (int);
void amm_alcd_set_cursor (int, int);
void amm_alcd_gotoxy     (uint8_t, uint8_t);
void amm_alcd_cmd_write  (uint8_t);
void amm_alcd_data_write (uint8_t);
void amm_alcd_putc       (uint8_t);
void amm_alcd_print      (const char *);

#ifdef __cplusplus
}
#endif





