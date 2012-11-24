#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/stddef.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>


#include <vlink/vlink.h>

#define MAJOR_NR VLINK_MAJOR

static char module_name[] = "pt_vlink";
static char sbuf[BUF_MAX];
static char rbuf[BUF_MAX];

static int lock_stat;

static int vlink_open(struct inode *inode,struct file *file)
{
    MOD_INC_USE_COUNT;
    return 0;
}

static int vlink_close(void)
{
    MOD_DEC_USE_COUNT;
    return 0;
}

static int vlink_ioctl(struct inode *inode,struct file *file,
	unsigned int cmd,unsigned long arg)
{
    switch(cmd)
    {
    case KMC_VLINK_NULL:
	break;
    case KMC_VLINK_SEND:
	copy_from_user(&sbuf,(void *)arg,BUF_MAX);
	printk(" arg=\"%s\"\n",sbuf);
	break;
    case KMC_VLINK_RECV:
	strcpy(rbuf,"kernel memory TEST");
	copy_to_user((void *)arg,&rbuf,BUF_MAX);
	break;


    case KMC_PTSVR_READ:
	{
	    PPTSVR_STR p=(PPTSVR_STR)&sbuf;

	    copy_from_user(&sbuf,(void *)arg,sizeof(PTSVR_STR));
	    if(p->size > BUF_MAX){
		printk("vlink: KMC_PTSVR_READ size=%d > %d",p->size,BUF_MAX);
	    }
	    Read_VIO_pt(rbuf,p->size);
	    copy_to_user((void *)p->buf,&rbuf,p->size);
	}
	break;
    case KMC_PTSVR_WRITE:
	{
	    PPTSVR_STR p=(PPTSVR_STR)&sbuf;
	    void *abuf;

	    copy_from_user(&sbuf,(void *)arg,sizeof(PTSVR_STR));
	    if(p->size > BUF_MAX){
		abuf = (void *)kmalloc(p->size,GFP_KERNEL);
	    }
	    else{
		abuf = (void *)rbuf;
	    }
	    copy_from_user(abuf,(void *)p->buf,p->size);
	    Write_VIO_pt(abuf,p->size);
	}
	break;

    case KMC_PTSVR_STAT:
	{
	    PPTSVR_STR p=(PPTSVR_STR)&sbuf;

	    copy_from_user(&sbuf,(void *)arg,sizeof(PTSVR_STR));
	    p->ret = getSTAT();
	    copy_to_user((void *)arg,&sbuf,sizeof(PTSVR_STR));
	}
	break;

    case KMC_PTSVR_LOCK:
	{
	    PPTSVR_STR p=(PPTSVR_STR)&sbuf;

	    copy_from_user(&sbuf,(void *)arg,sizeof(PTSVR_STR));
//	    lock_stat = (int)p->buf;
	}
	break;

    default:
	printk("%s:ioctl cmd=%d unknown !!\n",module_name,cmd);
	break;
    }
    return 0;
}


int is_lock_schedule()
{
    if(lock_stat & PTSVR_LOCK_SCHEDULE) return 1;
    return 0;
}

void lock_1char()
{
    if(lock_stat & PTSVR_LOCK1CHAR_SCHEDULE) lock_stat=PTSVR_LOCK_SCHEDULE;
}

int lock_schedule()
{
    lock_stat |= PTSVR_LOCK_SCHEDULE;
}

int unlock_schedule()
{
    lock_stat &= ~PTSVR_LOCK_SCHEDULE;
}

static struct file_operations vlink_fops = {
    owner:	THIS_MODULE,
    ioctl:	vlink_ioctl,
    open:	vlink_open,
    release:	vlink_close,
};

static int __init vlink_init(void)
{
    printk("%s: PARTNER vlink driver loaded.\n",module_name);
    if(register_chrdev(VLINK_MAJOR,device_name,&vlink_fops)){
	printk("%s: Device registration error.\n",module_name);
	return -EBUSY;
    }
    __vlink_init();
    return 0;
}

static void vlink_cleanup(void)
{
    printk("%s: PARTNER vlink driver removed.\n",module_name);
    if(unregister_chrdev(VLINK_MAJOR,device_name)){
	printk("%s: Device unregister error\n",module_name);
    }
}

module_init(vlink_init);
module_exit(vlink_cleanup);

MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;

#define	malloc(x)	kmalloc(x,GFP_KERNEL)

#define LINUX			1

#if defined(__mips__)
#define	CPU_MIPS		1
#define	VL_CPU			1
#define	VL_NMI			1
#endif
#if defined(__arm__)
#define	CPU_ARM			1
#define	VL_CPU			1
#define	VL_NMI			0

#include <asm/system.h>

#endif
#if defined(__sh__)
#define	CPU_SH7			1
#define	VL_CPU			1
#define	VL_NMI			1
#endif

#ifndef	VL_CPU
 #error "set CPU define !!"
#endif

#include <vlink/osemu.h>

#include "osemu.c"


ULONG target_vlink_offset()
{
    return 0xf9000000;		/* KZP ARM9EX */
}
