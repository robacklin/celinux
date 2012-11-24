/*
	OSEMU.C

    Copyright (c) 1995-2002 by Kyoto Micro Computer Co.,LTD.
    All Rights Reserved.

*/

#if CPU_MIPS /* { */
#define	VLINK_FAST		1
#endif /* } */

#if !VLINK_VXWORKS /* { */
#define	__asm__		asm
#endif /* } */

#define PORTSIZE unsigned char

int _dos_errno;

#define UBYTE unsigned char
#define UWORD unsigned short
#define ULONG unsigned

#define MONADR		__et2_vlink_monadr
#define MONADR_H	__et2_vlink_monadr_h
#define SHIFT_L		__et2_vlink_shift_l
#define SHIFT_R		__et2_vlink_shift_r
#define CMDWRS		__et2_vlink_cmdwrs
#define CMDRD		__et2_vlink_cmdrd
#define STAT		__et2_vlink_stat

ULONG MONADR=0;
ULONG SHIFT_L=0;
ULONG SHIFT_R=0;
ULONG MONADR_H=0;

volatile PORTSIZE *CMDWRS=0;
volatile PORTSIZE *CMDRD=0;
volatile PORTSIZE *STAT=0;


#if !LINUX /* { */
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

char *_dos_arg;
char *_dos_env;

struct  ffblk {
    char ff_reserved[23];
    char ff_attrib;
    unsigned short ff_ftime;
    unsigned short ff_fdate;
    struct {
	unsigned short low,high;
    } ff_fsize;
    char  ff_name[255+1];
} *ffblkp;

typedef struct {
unsigned year;
unsigned mon;
unsigned day;
unsigned hour;
unsigned min;
unsigned sec;
unsigned msec;
} _dos_time;


#endif /* } */

static void putCMD_init(UBYTE c);
static int not_use_vlink();

void (*putCMD)(UBYTE)		=putCMD_init;
void (*putPT)(UBYTE)		=(void (*)(UBYTE))not_use_vlink;
UBYTE (*getPT)(void)		=(UBYTE (*)(void))not_use_vlink;
UBYTE (*getPT_sync)(void)	=(UBYTE (*)(void))not_use_vlink;
void (*putPT_sync)(UBYTE)	=(void (*)(UBYTE))not_use_vlink;
void (*putPT_L)(ULONG)	 	=(void (*)(ULONG))not_use_vlink;
ULONG (*getPT_L)(void)		=(ULONG (*)(void))not_use_vlink;
void (*getPT_BLK)(UBYTE*,ULONG)	=(void (*)(UBYTE*,ULONG))not_use_vlink;
void (*putPT_BLK)(UBYTE*,ULONG)	=(void (*)(UBYTE*,ULONG))not_use_vlink;
UBYTE (*getSTAT)(void)		=(UBYTE (*)(void))not_use_vlink;

void (*putPT_flash)(void)	=(void (*)(void))not_use_vlink;


static unsigned vlink_sync_stat;

#if VL_CPU /* { */

static unsigned vlc_blk=512;	/* VLINK CPU 1 BLOCK SIZE */

#if CPU_MIPS || CPU_V800 || CPU_V850E || CPU_SH7 /* { */

UBYTE *vl_put_p;
UBYTE *vl_put_buf;

UBYTE *vl_get_p;
UBYTE *vl_get_pend;
UBYTE *vl_get_buf;
ULONG vlc_ret_val2;

#if VL_NMI /* { */
ULONG vl_nmi_flag;

volatile UBYTE *vl_host_ready_putp;
volatile UBYTE *vl_host_ready_getp;
volatile UBYTE *vl_target_ready_putp;
volatile UBYTE *vl_target_ready_getp;

static int vl_init_flg;

void vl_get_flush_X(ULONG size);

#endif /* } */


ULONG vl_put_flush_X(int fnc)
{
    ULONG len,size;

#if VL_NMI /* { */
    if(vl_nmi_flag){
	if(*vl_host_ready_getp!=0){
	    *vl_target_ready_putp=0;
	    while(*vl_host_ready_getp!=0){
		if(vl_get_p==vl_get_pend && *vl_host_ready_putp==0){
		    *vl_target_ready_getp=*vl_host_ready_putp=0xff;
		    vl_get_flush_X(vl_cpu_fnc(1,vl_get_buf));
		}
#if LINUX /* { */
		if(!is_lock_schedule()) schedule();
#endif /* } */
	    }
	}
	*vl_target_ready_putp=*vl_host_ready_getp=0xff;
	if(vl_init_flg){
	    *vl_host_ready_putp=0xff;
	    vl_init_flg=0;
	}
    }
#endif /* } */

    len=vl_put_p-vl_put_buf;

    if(len<=4){
	ULONG val;
	ULONG i;

	val=0;
	for(i=0;i<len;++i){
	    val |= (ULONG)(*(vl_put_buf+i))<<(i*8);
	}
	size=vl_cpu_fnc((len<<12) | fnc,vl_put_buf,val);
    }
    else{
	size=vl_cpu_fnc(fnc,vl_put_buf,len);
    }
    vl_put_p=vl_put_buf;
    return size;
}

void vl_put_flash(void)
{
    if(vl_put_p==vl_put_buf) return;
    vl_put_flush_X(0);
}


void vl_get_flush_X(size)
ULONG size;
{
    if(size<=4){
	ULONG i;
	ULONG val;

	val=vlc_ret_val2;
	for(i=0;i<size;++i){
	    *(vl_get_buf+i)=(UBYTE)(val>>(i*8));
	}
    }
    vl_get_pend=vl_get_buf+size;
    vl_get_p=vl_get_buf;
}


void vl_get_flash(void)
{
#if VL_NMI /* { */
    if(vl_nmi_flag){
	if(*vl_host_ready_putp!=0){
	    *vl_target_ready_getp=0;
	    while(*vl_host_ready_putp!=0){
#if LINUX /* { */
		if(!is_lock_schedule()) schedule();
#endif /* } */
		;
	    }
	}
	*vl_target_ready_getp=*vl_host_ready_putp=0xff;
	if(vl_init_flg){
	    *vl_host_ready_getp=0xff;
	    vl_init_flg=0;
	}
    }
#endif /* } */
    vl_get_flush_X(vl_cpu_fnc(1,vl_get_buf));
}

void vl_put_get_flash(void)
{
    vl_get_flush_X(vl_put_flush_X(2));
}


static UBYTE VLC_getSTAT(void)
{
    unsigned size;

    if(vl_get_p!=vl_get_pend) return 1;

#if VL_NMI /* { */
    if(vl_nmi_flag){
	return (*vl_host_ready_putp==0);
    }
#endif /* } */

    size=vl_cpu_fnc(3,vl_get_buf);
    if(size==0) return 0;
    vl_get_flush_X(size);
    return 1;
}


static void VLC_putPT(UBYTE data)
{
    *(vl_put_p++)=data;
    if(vl_put_p==(vl_put_buf+vlc_blk)){
	vl_put_flash();
    }
}

static UBYTE VLC_getPT(void)
{
    if(vl_put_p!=vl_put_buf){
	if(vl_get_p==vl_get_pend){
	    vl_put_get_flash();
	}
	else{
	    vl_put_flash();
	}
    }
    else if(vl_get_p==vl_get_pend){
	vl_get_flash();
    }
    return *(vl_get_p++);
}

static void VLC_putPT_L(ULONG data)
{
    VLC_putPT((UBYTE)data);
    VLC_putPT((UBYTE)(data>>8));
    VLC_putPT((UBYTE)(data>>16));
    VLC_putPT((UBYTE)(data>>24));
}

static ULONG VLC_getPT_L(void)
{
    return (ULONG)VLC_getPT() | ((ULONG)VLC_getPT()<<8) | ((ULONG)VLC_getPT()<<16) | ((ULONG)VLC_getPT()<<24);
}

static void VLC_putPT_BLK(UBYTE *bufp,ULONG size)
{
    ULONG r_size;

    r_size=(vl_put_buf+vlc_blk)-vl_put_p;
    while(size){
	if(r_size>size){
	    do{
		*(vl_put_p++)=*(bufp++);
		--size;
	    }while(size);
	    break;
	}
	size -= r_size;
	do{
	    *(vl_put_p++)=*(bufp++);
	    --r_size;
	}while(r_size);

	vl_put_flash();
	r_size=vlc_blk;
    }
}

static void VLC_getPT_BLK(UBYTE *bufp,ULONG size)
{
    ULONG len;
    ULONG r_size;

    if(vl_put_p!=vl_put_buf){
	if(vl_get_p==vl_get_pend){
	    vl_put_get_flash();
	}
	else{
	    vl_put_flash();
	}
    }
    else if(vl_get_p==vl_get_pend){
	vl_get_flash();
    }

    while(size){
	r_size=vl_get_pend-vl_get_p;
	if(size<=r_size){
	    len=size;
	    size=0;
	}
	else{
	    len=r_size;
	    size -= len;
	}
	do{
	    *(bufp++)= *(vl_get_p++);
	    --len;
	}while(len);
	if(size==0) break;


	vl_get_flash();
    }
}


#if CPU_MIPS /* { */

__asm__("
	.globl	__vlink_break_addr
	.ent	__vlink_break_addr
vl_cpu_fnc:
__vlink_break_addr:
	.long 0x7000003F	# dbreak
	sw	$3,vlc_ret_val2
	j	$31
	.end	__vlink_break_addr
");
#endif /* } */

#if CPU_V800 /* { */
asm("
	.globl	__vlink_break_addr
vl_cpu_fnc:
	mov	$2,$12
	mov	$6,$1
	mov	$7,$2
__vlink_break_addr:
	.short	0x6c00		# BRKPNT
	mov	$1,$10
	movhi	hi1(vlc_ret_val2),$zero,$1
	st.w	$2,lo(vlc_ret_val2)[$1]
	mov	$12,$2
	jmp	[$lp]
");
#endif /* } */

#if CPU_V850E /* { */
asm("
	.globl	___vlink_break_addr
_vl_cpu_fnc:
	mov	r2,r12
	mov	r30,r13
	mov	r6,r1
	mov	r7,r2

___vlink_break_addr:
	.short	0xf840		# DBTRAP

	mov	r1,r10
	mov	hilo(_vlc_ret_val2),r1
	st.w	r2,0[r1]
	mov	r12,r2
	mov	r13,r30
	jmp	[r31]
");
#endif /* } */

#if CPU_SH7 /* { */

#if LINUX /* { */
asm("
	.globl	_vlink_break_addr
vl_cpu_fnc:
_vlink_break_addr:
");
#else /* }{ */
asm("
	.globl	___vlink_break_addr
_vl_cpu_fnc:
___vlink_break_addr:
");
#endif /* } */


#if __SH4__ || __SH4L__ /* { */
asm("	.short	0x003b");
#else /* }{ */
asm("	.short	0x0000");
#endif /* } */

#if LINUX /* { */
asm("
	mov.l	.Lvlc_ret_val2,r2
	mov.l	r1,@r2
	rts
	nop

	.align 2
.Lvlc_ret_val2:
	.long	vlc_ret_val2
");
#else /* }{ */
asm("
	mov.l	.Lvlc_ret_val2,r2
	mov.l	r1,@r2
	rts
	nop

	.align 2
.Lvlc_ret_val2:
	.long	_vlc_ret_val2
");
#endif /* } */

#endif /* } */

#endif /* } */

#if CPU_ARM /* { */

jtag_com_stat()			/* bit0==1 受信でデータあり / bit1==0 送信可能 */
{
asm("	mrc	p14,0,r0,c0,c0");
}

jtag_com_put()
{
asm("	mcr	p14,0,r0,c1,c0");
}

jtag_com_get()
{
asm("	mrc	p14,0,r0,c1,c0");
}


static UBYTE VLC_getSTAT(void)
{
    return (jtag_com_stat() & 1);
}

static void VLC_putPT(UBYTE c)
{
    while((jtag_com_stat() & 2)!=0){
#if LINUX /* { */
	if(!is_lock_schedule()) schedule();
#endif /* } */
	;
    }
    jtag_com_put((ULONG)c);
}


static void VLC_putPT_L(ULONG c)
{
    while((jtag_com_stat() & 2)!=0){
#if LINUX /* { */
	if(!is_lock_schedule()) schedule();
#endif /* } */
	;
    }
    jtag_com_put(c);
}


static UBYTE VLC_getPT(void)
{
    UBYTE c;
    while((jtag_com_stat() & 1)==0){
#if LINUX /* { */
	if(!is_lock_schedule()) schedule();
#endif /* } */
	;
    }
    c=(jtag_com_get()>>24);
    return c;
}

static ULONG VLC_getPT_L(void)
{
    ULONG c;
    while((jtag_com_stat() & 1)==0){
#if LINUX /* { */
	if(!is_lock_schedule()) schedule();
#endif /* } */
	;
    }
    c=jtag_com_get();
    return c;
}

#define VLC_BLK		vlc_blk
#define EXT_CODE	0x12345678
#define EOF_CODE	0x87654321

static unsigned vlc_fast_disable_get;

static void set_longp(p,dt)
void *p;
ULONG dt;
{
    switch((ULONG)p&3) {
    case 0:
    	*((ULONG *)p)= dt;
	break;
    case 2:
	*((UWORD *)p)= dt;
	dt >>= 16;
	*((UWORD *)(p+2))= dt;
    	break;
    default:
	*(UBYTE *)p=dt;
	*((UBYTE *)p+1)=dt>>8;
	*((UBYTE *)p+2)=dt>>16;
	*((UBYTE *)p+3)=dt>>24;
    }
}

static void VLC_getPT_BLK(UBYTE *bufp,ULONG ct)
{
    ULONG data;
    ULONG blk;
    int index;
    int err;
    int err_ct;
    ULONG blk_count;

loop:
    if(VLC_BLK && ct>=16){
	if((err_ct=vlc_fast_disable_get)!=0) --vlc_fast_disable_get;
	if(err_ct>10) goto next;
	do{








	    blk=(VLC_BLK<=ct) ? VLC_BLK:(ct & ~0x3);
	    err=1;
	    for(index=0;index<blk;index+=4){
		data=getPT_L();








		if(data==EOF_CODE) goto error;
		if(data==EXT_CODE){
		    data=getPT_L();
		    if(data==EOF_CODE) goto error;
		    if(data==0) data=EOF_CODE;
		}
		set_longp(bufp+index,data);
	    }

	    if(getPT_L()==EOF_CODE){
		err=0;
	    }

error:






	    if(err==0){		/* OK */
		putPT(0);
		ct -= blk;
		bufp += blk;
		vlc_fast_disable_get=0;
		err_ct=0;



	    }
	    else{
		if(++err_ct>=10){
		    vlc_fast_disable_get=300*1024/VLC_BLK;
		    putPT(0xff);



		    break;
		}
		putPT(1);




	    }
	}while(ct>=16) ;
    }
next:
    blk_count=0;
    while(ct>=4){
	set_longp(bufp,getPT_L());
	bufp += 4;
	ct -= 4;
	blk_count += 4;
	if(blk_count==VLC_BLK){
	    goto loop;
	}
    }
    if(ct!=0){
	data=getPT_L();
	do{
	    *(bufp++)=(UBYTE)data;
	    data >>= 8;
	}while(--ct) ;
    }
}


static ULONG get_longp(p)
void *p;
{
    ULONG dt;

    switch((ULONG)p&3) {
    case 0:
    	dt= *((ULONG *)p) ;
	break;
    case 2:
    	dt=*(UWORD *)(p+2);
	dt<<=16;
	dt|=*(UWORD *)(p);
	break;
    default:
	dt=*((UBYTE *)p+3);
	dt<<=8;
	dt|=*((UBYTE *)p+2);
	dt<<=8;
	dt|=*((UBYTE *)p+1);
	dt<<=8;
	dt|=*(UBYTE *)p;
    }
    return dt;
}

static unsigned vlc_fast_disable_put;

static void VLC_putPT_BLK(UBYTE *bufp,ULONG ct)
{
    ULONG data;
    ULONG blk;
    int index;
    int err_ct;
    ULONG blk_count;

loop:
    if(VLC_BLK && ct>=16){
	if((err_ct=vlc_fast_disable_put)!=0) --vlc_fast_disable_put;
	if(err_ct>10) goto next;
	do{
	    unsigned stat;






	    blk=(VLC_BLK<=ct) ? VLC_BLK:(ct & ~0x3);

	    for(index=0;index<blk;index+=4){

		for(;;){
		    stat=jtag_com_stat();
		    if((stat & 2)==0) break;
		    if(stat & 1){
			jtag_com_get();
			goto error;
		    }
		}

		jtag_com_put(get_longp(bufp+index));







	    }



	    if(getPT()==0){
		ct -= blk;
		bufp += blk;
		vlc_fast_disable_put=0;
		err_ct=0;



	    }
	    else{
error:
		getPT();
		if(++err_ct>=10){
		    vlc_fast_disable_put=300*1024/VLC_BLK;



		    break;
		}




	    }
	}while(ct>=16) ;
    }
next:
    blk_count=0;
    while(ct>=4){
	data = get_longp(bufp);
	putPT_L(data);
	bufp += 4;
	ct -= 4;
	blk_count += 4;
	if(blk_count==VLC_BLK){
	    goto loop;
	}
    }
    if(ct!=0){
	do{
	    data <<= 8;
	    --ct;
	    data |= *(bufp+ct);
	}while(ct) ;
	putPT_L(data);
    }
}

#endif /* }  ARM */

#if CPU_MN103 /* { */


ULONG *__vl_dma_buf;

/* 通信ステータスレジスタ */
#define COMSR (unsigned long *)(0xc00001d4)
#define COMSR_mCOMOR	0x04		/* 1==データ送信可	*/
#define COMSR_mCOMIR	0x08		/* 1==受信データあり	*/

/* 通信データレジスタ */
#define JCCOM (volatile unsigned long *)(0xc00001e0)

/* デバッグコントロールレジスタ */
#define DCR   (volatile unsigned short *)(0xc0000030)
#define DCR_DE		0x01		/* デバッグレジスタ書き込み許可 */

jtag_com_stat()
{
    return *COMSR;
}

jtag_com_put(unsigned dt)
{
    unsigned short dcr;

    dcr=*DCR;
    *DCR=dcr|DCR_DE;
    *JCCOM=dt;
    *DCR=dcr;
}

static UBYTE VLC_getSTAT(void)
{
    return (jtag_com_stat() & COMSR_mCOMIR);
}

static void VLC_putPT(UBYTE c)
{
    while((jtag_com_stat() & COMSR_mCOMOR)!=0) ;
    jtag_com_put((ULONG)c);
}


static void VLC_putPT_L(ULONG c)
{
    while((jtag_com_stat() & COMSR_mCOMOR)!=0) ;
    jtag_com_put(c);
}


static UBYTE VLC_getPT(void)
{
    UBYTE c;
    while((jtag_com_stat() & COMSR_mCOMIR)==0) ;
    c=*JCCOM;
    return c;
}

static ULONG VLC_getPT_L(void)
{
    ULONG c;
    while((jtag_com_stat() & COMSR_mCOMIR)==0) ;
    c=*JCCOM;
    return c;
}

#define VLC_BLK		vlc_blk

static unsigned vlc_fast_disable_get;

static void set_longp(p,dt)
void *p;
ULONG dt;
{
    switch((ULONG)p&3) {
    case 0:
    	*((ULONG *)p)= dt;
	break;
    case 2:
	*((UWORD *)p)= dt;
	dt >>= 16;
	*((UWORD *)(p+2))= dt;
    	break;
    default:
	*(UBYTE *)p=dt;
	*((UBYTE *)p+1)=dt>>8;
	*((UBYTE *)p+2)=dt>>16;
	*((UBYTE *)p+3)=dt>>24;
    }
}



static void VLC_getPT_BLK(UBYTE *bufp,ULONG ct)
{
    ULONG data;
    ULONG blk;
    int index;
    ULONG *p;

    if(VLC_BLK && ct>=16){
	do{
	    blk=(VLC_BLK<=ct) ? VLC_BLK:(ct & ~0x3);

	    if(getPT_L()!=blk) for(;;) ;

	    p=__vl_dma_buf;
	    for(index=0;index<blk;index+=4){
		set_longp(bufp+index,*p);
		++p;
	    }

	    putPT_L(ct);
	    ct -= blk;
	    bufp += blk;

	}while(ct>=16) ;
    }

next:
    while(ct>=4){
	set_longp(bufp,getPT_L());
	bufp += 4;
	ct -= 4;
    }
    if(ct!=0){
	data=getPT_L();
	do{
	    *(bufp++)=(UBYTE)data;
	    data >>= 8;
	}while(--ct) ;
    }
}


static ULONG get_longp(p)
void *p;
{
    ULONG dt;

    switch((ULONG)p&3) {
    case 0:
    	dt= *((ULONG *)p) ;
	break;
    case 2:
    	dt=*(UWORD *)(p+2);
	dt<<=16;
	dt|=*(UWORD *)(p);
	break;
    default:
	dt=*((UBYTE *)p+3);
	dt<<=8;
	dt|=*((UBYTE *)p+2);
	dt<<=8;
	dt|=*((UBYTE *)p+1);
	dt<<=8;
	dt|=*(UBYTE *)p;
    }
    return dt;
}

static unsigned vlc_fast_disable_put;

static void VLC_putPT_BLK(UBYTE *bufp,ULONG ct)
{
    ULONG data;
    ULONG blk;
    int index;
    ULONG *p;

    if(VLC_BLK && ct>=16){
	do{

	    blk=(VLC_BLK<=ct) ? VLC_BLK:(ct & ~0x3);

	    p=__vl_dma_buf;
	    for(index=0;index<blk;index+=4){
		*(p++)=get_longp(bufp+index);
	    }

	    putPT_L(ct);
	    if(getPT_L()!=blk) for(;;) ;

	    ct -= blk;
	    bufp += blk;
	}while(ct>=16) ;
    }
    while(ct>=4){
	data = get_longp(bufp);
	putPT_L(data);
	bufp += 4;
	ct -= 4;
    }
    if(ct!=0){
	do{
	    data <<= 8;
	    --ct;
	    data |= *(bufp+ct);
	}while(ct) ;
	putPT_L(data);
    }
}

#endif /* }  MN103 */


#endif /* } VL_CPU */


#if KMC_BUF /* { */
static char *kmc_wp,*kmc_rp;
static char kmc_buffer[WDB_KMC_MTU];
#endif /* } */

void __et2_vlink_tbl(void)
{
    __et2_vlink_monadr=0;
    __et2_vlink_monadr_h=0;
    __et2_vlink_shift_l=0;
    __et2_vlink_shift_r=0;
    __et2_vlink_cmdwrs=0;
    __et2_vlink_cmdrd=0;
    __et2_vlink_stat=0;
}


static void ET2_putCMD(UBYTE c)
{
    UBYTE *dp;

    while((*STAT & 0x11)!=1) ;
    dp=(UBYTE *)(((c & 0xf)<<SHIFT_L)+MONADR);
    *(volatile PORTSIZE *)dp;
    dp=(UBYTE *)(((c & 0xf0)>>SHIFT_R)+MONADR_H);
    *(volatile PORTSIZE *)dp;
    *CMDWRS;
}

static void ET2_putPT(UBYTE c)
{
    UBYTE *dp;

#if KMC_BUF /* { */
    int count;
    count=0;
#endif /* } */
    while((*STAT & 0x1)==0){
#if KMC_BUF /* { */
	if(++count>=0x10000){
	    while((*STAT & 0x80)!=0){
		if(kmc_wp==0){
		    kmc_buffer[0]=*CMDRD;
		    kmc_wp=kmc_buffer+1;
		    kmc_rp=kmc_buffer;
		}
		else{
		    *(kmc_wp++)=*CMDRD;
		}
	    }
	    count=0;
	}
#endif /* } */
    }
    dp=(UBYTE *)(((c & 0xf)<<SHIFT_L)+MONADR);
    *(volatile PORTSIZE *)dp;
    dp=(UBYTE *)(((c & 0xf0)>>SHIFT_R)+MONADR_H);
    *(volatile PORTSIZE *)dp;
    *CMDWRS;
}

static void ET2_putPT_L(ULONG c)
{
    int i;
    UBYTE *p;
    p=(UBYTE *)&c;
    for(i=0;i<4;++i){
	putPT(*(p++));
    }
}

static UBYTE ET2_getPT(void)
{
#if KMC_BUF /* { */
    UBYTE c;

    if(kmc_rp!=0){
	c=*(kmc_rp++);
	if(kmc_wp==kmc_rp){
	    kmc_wp=kmc_rp=0;
	}
	return c;
    }
#endif /* } */
    while((*STAT & 0x80)==0){
#if LINUX /* { */
	if(!is_lock_schedule()) schedule();
#endif /* } */
	;
    }
    return *CMDRD;
}


static UBYTE ET2_getSTAT(void)
{
#if KMC_BUF /* { */
    if(kmc_rp!=0) return 0x80;
#endif /* } */
    return (*STAT & 0x80);
}

static ULONG ET2_getPT_L(void)
{
    int i;
    ULONG lval;
    UBYTE *p;

    p=(UBYTE *)&lval;
    for(i=0;i<4;++i){
	*(p++)=getPT();
    }
    return lval;
}

static void ET2_getPT_BLK(UBYTE *bufp,ULONG ct)
{
#if VLINK_FAST /* { */
    register UBYTE stat;
    register volatile PORTSIZE *pstat;
    register volatile PORTSIZE *pcmd;

    pstat=STAT;
    pcmd=CMDRD;
    while(ct){
#if KMC_BUF /* { */
	if(kmc_rp!=0){
	    *(bufp++)=*(kmc_rp++);
	    if(kmc_wp==kmc_rp){
		kmc_wp=kmc_rp=0;
	    }
	    --ct;
	}
	else
#endif /* } */
	{
	    while(((stat=*pstat) & 0x80)==0){
#if LINUX /* { */
		if(!is_lock_schedule()) schedule();
#endif /* } */
		;
	    }
	    *(bufp++)=*pcmd;
	    --ct;
	    if(stat & 0x40){
		*(bufp)=*pcmd;
		*(bufp+1)=*pcmd;
		*(bufp+2)=*pcmd;
		bufp += 3;
		ct -= 3;
	    }
	}
    }
#else /* }{ */
    while(ct){
	*(bufp++)=getPT();
	--ct;
    }
#endif /* } */
}

static void ET2_putPT_BLK(UBYTE *bufp,ULONG ct)
{
#if VLINK_FAST /* { */
    register volatile PORTSIZE *pstat;
    register volatile PORTSIZE *pcmdw;
    register UBYTE *dp;
    register UBYTE c;
    register ULONG mon;
    register ULONG mon_h;
    register int i;

#if KMC_BUF /* { */
    int count;
    count=0;
#endif /* } */
    pstat=STAT;
    pcmdw=CMDWRS;
    mon=MONADR;
    mon_h=MONADR_H;
    while(ct>16){
	ct -= 16;
	while((*pstat & 0x3)!=3){
#if KMC_BUF /* { */
	    if(++count>=0x10000){
		while((*STAT & 0x80)!=0){
		    if(kmc_wp==0){
			kmc_buffer[0]=*CMDRD;
			kmc_wp=kmc_buffer+1;
			kmc_rp=kmc_buffer;
		    }
		    else{
			*(kmc_wp++)=*CMDRD;
		    }
		}
		count=0;
	    }
#endif /* } */
	}
	for(i=0;i<16;++i){
	    c=*(bufp++);
	    dp=(UBYTE *)(((c & 0xf)<<SHIFT_L)+mon);
	    *(volatile PORTSIZE *)dp;
	    dp=(UBYTE *)(((c & 0xf0)>>SHIFT_R)+mon_h);
	    *(volatile PORTSIZE *)dp;
	    *pcmdw;
	}
    }
#endif /* } */
    while(ct){
	putPT(*(bufp++));
	--ct;
    }
}


#if !VLINK_VXWORKS && !LINUX /* { */

void putPTstring(char *p)
{
    char c;

    do{
	c=*(p++);
	putPT(c);
    } while (c!='\0') ;
}


int _dos_put_console(char c)
{
    putCMD(0);
    getPT_sync();
    putPT_BLK(&c,1);
    putPT_flash();
    return 0;
}

int _dos_putstring_console(char *str,int size)
{
    int len;
    int ct;
    int start;

    start=1;
    if(size==0){
	char *p;
	p=str;
	while(*p) ++p;
	size=p-str;
    }
    len=size;
    while(len){
	if(len>0x80){
	    ct=0x80;
	}
	else{
	    ct=len;
	}
	if(start){
	    putCMD(ct-1);
	    start=0;
	}
	else{
	    putPT(ct-1);
	}
	getPT_sync();
	putPT_BLK((UBYTE *)str,ct);
	len -= ct;
	str += ct;
    }
    putPT_flash();
    return size;
}


int _dos_stat_console(void)
{
    char c;

    putCMD(0x80);
    c=getPT();
    if(c=='\x1b'){
	int i;
	for(i=0;i<1000000;++i) ;
	c=0;
    }
    return c;
}

int _dos_get_console(void)
{
    char c;

    while((c=_dos_stat_console())=='\0') ;
    return c;
}


int _dos_open(char *fname,int mode)
{
    int fd;

    putCMD(OSEMU);
    putPT(EMU_OPEN);
    putPTstring(fname);
    putPT(mode);
    fd=getPT();
    if(fd==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return fd;
}

int _dos_creat(char *fname,int mode)
{
    int fd;

    putCMD(OSEMU);
    putPT(EMU_CREAT);
    putPTstring(fname);
    putPT(mode);
    fd=getPT();
    if(fd==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return fd;
}

int _dos_read(int fd,char *buf,int size)
{
    int rval;

    if(fd==0){	/* CON IN ? */
	if(size<0) return 0;
	*buf = _dos_get_console();
	_dos_write(1,buf,1);
	if(*buf=='\r'){
//	    *buf = 0;	/* END */
	    _dos_write(1,"\n",1);
	}
	return 1;
    }
    if(size<0) return -1;
    putCMD(OSEMU);
    putPT(EMU_READ);
    putPT(fd);
    putPT_L(size);
    getPT_BLK((UBYTE *)buf,size);
    putPT_sync(0);
    rval=getPT_L();
    if(rval==-1){
	_dos_errno=getPT();
    }
    return rval;
}

int _dos_write(int fd,char *buf,int size)
{
    int rval;

    if(size<0) return -1;
    if(fd==1){	/* CON OUT ? */
	return _dos_putstring_console(buf,size);
    }
    putCMD(OSEMU);
    putPT(EMU_WRITE);
    putPT(fd);
    putPT_L(size);
    putPT_BLK((UBYTE *)buf,size);
    rval=getPT_L();
    if(rval==-1){
	_dos_errno=getPT();
    }
    return rval;
}

int _dos_close(int fd)
{
    putCMD(OSEMU);
    putPT(EMU_CLOSE);
    putPT(fd);
    if(getPT()!=0){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}

int _dos_lseek(int fd,int ofs,int pos)
{
    int ret_pos;

    putCMD(OSEMU);
    putPT(EMU_LSEEK);
    putPT(fd);
    putPT_L(ofs);
    putPT(pos);
    if((ret_pos=getPT_L())==-1){
	_dos_errno=getPT();
    }
    return ret_pos;
}

int _dos_file_mode(char *fname,int mode,int action)
{
    putCMD(OSEMU);
    putPT(EMU_FILE_MODE);
    putPTstring(fname);
    putPT_L(mode);
    putPT(action);
    if((mode=getPT_L())==-1){
	_dos_errno=getPT();
    }
    return mode;
}

int _dos_set_dta(struct ffblk *fbp)
{
    ffblkp=fbp;
    return 0;
}


int _dos_find_first(char *fname,unsigned attrib)
{
    putCMD(OSEMU);
    putPT(EMU_FIND_FIRST);
    putPTstring(fname);
    if(getPT()!=0){
	return -1;
    }
    getPT_BLK((UBYTE *)ffblkp,sizeof(struct ffblk));
    return 0;
}

int _dos_find_next(void)
{
    putCMD(OSEMU);
    putPT(EMU_FIND_NEXT);
    if(getPT()!=0){
	return -1;
    }
    getPT_BLK((UBYTE *)ffblkp,sizeof(struct ffblk));
    return 0;
}

int _dos_find_close(void)
{
    putCMD(OSEMU);
    putPT(EMU_FIND_CLOSE);
    putPT_flash();
    return 0;
}

int _dos_get_ioctl(int fd)
{
    int rval;

    if(fd==1){	/* CON OUT ? */
	return 0x80;
    }
    putCMD(OSEMU);
    putPT(EMU_GET_IOCTL);
    putPT(fd);
    if((rval=getPT())==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return rval;
}

int _dos_get_file_time(int fd,unsigned short *timep)
{
    putCMD(OSEMU);
    putPT(EMU_GET_FILE_TM);
    putPT(fd);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    *timep=(unsigned short)getPT_L();
    *(timep+1)=(unsigned short)getPT_L();
    return 0;
}

int _dos_get_time(_dos_time *dtp)
{
    putCMD(OSEMU);
    putPT(EMU_GET_TIME);
    getPT_BLK((UBYTE *)dtp,sizeof(_dos_time));
    return 0;
}


int _dos_set_file_time(int fd,unsigned short *timep)
{
    putCMD(OSEMU);
    putPT(EMU_SET_FILE_TM);
    putPT(fd);
    putPT_L(*timep);
    putPT_L(*(timep+1));
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}

int _dos_mkdir(char *path)
{
    putCMD(OSEMU);
    putPT(EMU_MKDIR);
    putPTstring(path);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}

int _dos_rmdir(char *path)
{
    putCMD(OSEMU);
    putPT(EMU_RMDIR);
    putPTstring(path);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}

int _dos_remove(char *fname)
{
    putCMD(OSEMU);
    putPT(EMU_REMOVE);
    putPTstring(fname);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}

int _dos_rename(char *old,char *new)
{
    putCMD(OSEMU);
    putPT(EMU_RENAME);
    putPTstring(old);
    putPTstring(new);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}


int _dos_getcwd(char *path,int drvno)
{
    putCMD(OSEMU);
    putPT(EMU_GETCWD);
    putPT(drvno);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    do{
	*path=getPT();
    }while(*(path++)!='\0') ;
    return 0;
}

int _dos_chdir(char *path)
{
    putCMD(OSEMU);
    putPT(EMU_CHDIR);
    putPTstring(path);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}

int _dos_getcdrv(void)
{
    putCMD(OSEMU);
    putPT(EMU_GETCDRV);
    return getPT();
}

int _dos_dup(int fd)
{
    putCMD(OSEMU);
    putPT(EMU_DUP);
    putPT(fd);
    if((fd=getPT())==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return fd;
}

int _dos_dup2(int oldfd,int newfd)
{
    putCMD(OSEMU);
    putPT(EMU_DUP2);
    putPT(oldfd);
    putPT(newfd);
    if(getPT()==0xff){
	_dos_errno=getPT();
	return -1;
    }
    return 0;
}

int _dos_version(void)
{
    putCMD(OSEMU);
    putPT(EMU_VERSION);
    return getPT_L();
}

int _dos_exit(int retcode)
{
    putCMD(OSEMU);
    putPT(EMU_EXIT);
    putPT_L(retcode);
    for(;;) ;
}

int _dos_sleep(int ms)
{
    putCMD(OSEMU);
    putPT(EMU_SLEEP);
    putPT_L(ms);
    getPT();
    return 0;
}

int _dos_arg_env(void)
{
    int arg_size;
    int env_size;
    char *malloc();

    putCMD(OSEMU);
    putPT(EMU_ARG_ENV);
    arg_size=getPT_L();
    env_size=getPT_L();
    _dos_arg=malloc(arg_size+1);
    _dos_env=malloc(env_size+1);
    getPT_BLK((UBYTE *)_dos_arg,arg_size);
    putPT_sync(0);
    getPT_BLK((UBYTE *)_dos_env,env_size);
    *(_dos_arg+arg_size)='\0';
    *(_dos_env+env_size)='\0';
    return 0;
}

#endif /* } !VLINK_VXWORKS */


#define READ_VIO	0
#define WRITE_VIO	1

int Read_VIO_pt(void *dramAddr, int nbytes)
{
    int size;

    if(nbytes<=0) return nbytes;
    if(vlink_sync_stat==1){
	size=getPT_L();
	if(size>nbytes) return 0;
	nbytes=size;
    }
    else{
	putCMD(0x82);
	putPT(READ_VIO);
	putPT_L(nbytes);
    }
#if CPU_ARM && LINUX /* { */
    lock_schedule();
#endif /* } */
    getPT_BLK((UBYTE *)dramAddr,nbytes);
#if CPU_ARM && LINUX /* { */
    unlock_schedule();
#endif /* } */
    return nbytes;
}

int Write_VIO_pt(void *dramAddr,int nbytes)
{
    if(nbytes<=0) return nbytes;
    if(vlink_sync_stat!=1){
	putCMD(0x82);
	putPT(WRITE_VIO);
    }
    putPT_L(nbytes);
    if(vlink_sync_stat==2){
	getPT();
    }

#if CPU_ARM && LINUX /* { */
    lock_schedule();
#endif /* } */
    putPT_BLK((UBYTE *)dramAddr,nbytes);
#if CPU_ARM && LINUX /* { */
    unlock_schedule();
#endif /* } */
    putPT_flash();
    return nbytes;
}


int __vlink_init(void)
{
    ULONG *vtbl;
#if !VLINK_VXWORKS && !LINUX /* { */
    char *malloc();
#endif /* } */

    vtbl=(ULONG *)(((ULONG)__et2_vlink_tbl+3) & 0xfffffffc);
    if(*vtbl==0x12345678){
	ULONG vlink_offset;
#if CPU_ARM /* { */
	ULONG target_vlink_offset();

	vlink_offset=target_vlink_offset();
#else /*}{ */
	vlink_offset=0;
#endif /* } */

	__et2_vlink_monadr=*(vtbl+1)+vlink_offset;
	__et2_vlink_monadr_h=*(vtbl+2)+vlink_offset;
	__et2_vlink_shift_l=*(vtbl+3);
	__et2_vlink_shift_r=*(vtbl+4);
	__et2_vlink_cmdwrs=(PORTSIZE *)(*(vtbl+5)+vlink_offset);
	__et2_vlink_cmdrd=(PORTSIZE *)(*(vtbl+6)+vlink_offset);
	__et2_vlink_stat=(PORTSIZE *)(*(vtbl+7)+vlink_offset);

	vlink_sync_stat=*(vtbl+8);

	putCMD=ET2_putCMD;
	putPT=ET2_putPT;
	getPT=ET2_getPT;
	getPT_sync=ET2_getPT;
	putPT_sync=ET2_putPT;
	putPT_L=ET2_putPT_L;
	getPT_L=ET2_getPT_L;
	getPT_BLK=ET2_getPT_BLK;
	putPT_BLK=ET2_putPT_BLK;
	getSTAT=ET2_getSTAT;
	return 1;
    }
#if VL_CPU /* { */
    else if(*vtbl==0x87654321){

	vlink_sync_stat=*(vtbl+1);

	vlc_blk=*(vtbl+2);

	putCMD=VLC_putPT;
	putPT=VLC_putPT;
	getPT=VLC_getPT;
	putPT_L=VLC_putPT_L;
	getPT_L=VLC_getPT_L;
	getPT_BLK=VLC_getPT_BLK;
	putPT_BLK=VLC_putPT_BLK;
	getSTAT=VLC_getSTAT;
#if CPU_ARM /* { */
	if(VLC_getSTAT()){
	    getPT();
	}
#endif /* } */
#if CPU_MN103 /* { */
	if(VLC_getSTAT()){
	    getPT();
	}
	if(vlc_blk!=0){
	    __vl_dma_buf=(ULONG *)malloc(vlc_blk);
	}
#endif /* } */
#if CPU_MIPS || CPU_V800 || CPU_V850E || CPU_SH7 /* { */
	putPT_flash=vl_put_flash;
	vl_put_buf=(UBYTE *)malloc(vlc_blk*2+4);
	vl_put_p=vl_put_buf;
	vl_get_p=vl_get_pend=vl_get_buf=vl_put_buf+vlc_blk;

#if VL_NMI /* { */
	vl_nmi_flag=*(vtbl+3);
	vl_host_ready_putp=vl_get_p+vlc_blk;
	vl_host_ready_getp=vl_host_ready_putp+1; 
	vl_target_ready_putp=vl_host_ready_getp+1;
	vl_target_ready_getp=vl_target_ready_putp+1;
	*vl_host_ready_getp=*vl_host_ready_putp=0;
	*vl_target_ready_putp=*vl_target_ready_getp=0xff;
	vl_init_flg=1;
#endif /* } */

#endif /* } */
	return 2;
    }
#endif /* } */
    putCMD=(void (*)(UBYTE))not_use_vlink;
    return 0;
}


void putCMD_init(UBYTE c)
{
    __vlink_init();
    putCMD(c);
}

int not_use_vlink()
{
    return 0;
}
