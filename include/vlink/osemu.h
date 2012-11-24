#ifndef __VLINK_OSEMU_H__
#define __VLINK_OSEMU_H__

#define UBYTE unsigned char
#define UWORD unsigned short
#define ULONG unsigned



#define OSEMU		0x83
#define EMU_OPEN	0x0
#define EMU_CREAT	0x1
#define EMU_READ	0x2
#define EMU_WRITE	0x3
#define EMU_CLOSE	0x4
#define EMU_LSEEK	0x5
#define EMU_FILE_MODE	0x6

#define EMU_FIND_FIRST	0x8
#define EMU_FIND_NEXT	0x9
#define EMU_FIND_CLOSE	0xa
#define EMU_GET_IOCTL	0xb
#define EMU_GET_FILE_TM	0xc
#define EMU_GET_TIME	0xd
#define EMU_SET_FILE_TM	0xe
#define EMU_MKDIR	0xf
#define EMU_RMDIR	0x10
#define EMU_REMOVE	0x11
#define EMU_RENAME	0x12
#define EMU_GETCWD	0x13
#define EMU_CHDIR	0x14
#define EMU_GETCDRV	0x15


#define EMU_DUP		0x18
#define EMU_DUP2	0x19
#define EMU_VERSION	0x1a
#define EMU_EXIT	0x1b

#define EMU_SLEEP	0x1d
#define EMU_ARG_ENV	0x1e

static void putCMD_init(UBYTE c);
static int not_use_vlink(void);


void (*putCMD)(UBYTE);
void (*putPT)(UBYTE);
UBYTE (*getPT)(void);
UBYTE (*getPT_sync)(void);
void (*putPT_sync)(UBYTE);
void (*putPT_L)(ULONG);
ULONG (*getPT_L)(void);
void (*getPT_BLK)(UBYTE*,ULONG);
void (*putPT_BLK)(UBYTE*,ULONG);
UBYTE (*getSTAT)(void);

void (*putPT_flash)(void);

//unsigned int vl_cpu_fnc(unsigned int,unsigned int,unsigned int);

int __vlink_init(void);


int Read_VIO_pt(void *dramAddr, int nbytes);
int Write_VIO_pt(void *dramAddr,int nbytes);

void vl_get_flush_X(ULONG size);
int _dos_write(int fd,char *buf,int size);


#endif /* __VLINK_OSEMU_H__ */
