/*
 *      e1356fb.h  --  Epson SED1356 Framebuffer Driver IOCTL Interface
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2001 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
 * IOCTLs to SED1356 fb driver. 0x45 is 'E' for Epson.
 */
#define FBIO_SED1356_BITBLT 0x4500

typedef struct {
    int   operation;
    int   rop;
    int   src_y;
    int   src_x;
    int   src_width;
    int   src_height;
    int   dst_y;
    int   dst_x;
    int   dst_width;
    int   dst_height;
    int   pattern_x;
    int   pattern_y;
    int   attribute;
    unsigned int bg_color;
    unsigned int fg_color;
    unsigned short* src;
    int   srcsize;
    int   srcstride;
} blt_info_t;

enum blt_attribute_t {
    BLT_ATTR_TRANSPARENT = 1
};

enum blt_operation_t {
    BLT_WRITE_ROP = 0,
    BLT_READ,
    BLT_MOVE_POS_ROP,
    BLT_MOVE_NEG_ROP,
    BLT_WRITE_TRANSP,
    BLT_MOVE_POS_TRANSP,
    BLT_PAT_FILL_ROP,
    BLT_PAT_FILL_TRANSP,
    BLT_COLOR_EXP,
    BLT_COLOR_EXP_TRANSP,
    BLT_MOVE_COLOR_EXP,
    BLT_MOVE_COLOR_EXP_TRANSP,
    BLT_SOLID_FILL
};
