/*
 *	Video for Linux Two
 *
 *	A generic video device interface for the LINUX operating system
 *	using a set of device structures/vectors for low level operations.
 *
 *	This file replaces the videodev.c file that comes with the
 *	regular kernel distribution.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 * Author:	Bill Dirks <bdirks@pacbell.net>
 *		based on code by Alan Cox, <alan@cymru.net>
 *
 */

/*
 * Video capture interface for Linux
 *
 *	A generic video device interface for the LINUX operating system
 *	using a set of device structures/vectors for low level operations.
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 * Author:	Alan Cox, <alan@redhat.com>
 *
 * Fixes:
 */

/*
 * Video4linux 1/2 integration by Justin Schoeman
 * <justin@suntiger.ee.up.ac.za>
 * 2.4 PROCFS support ported from 2.4 kernels by 
 *  Iñaki García Etxebarria <garetxe@euskalnet.net>
 * Makefile fix by "W. Michael Petullo" <mike@flyn.org>
 * 2.4 devfs support ported from 2.4 kernels by
 *  Dan Merillat <dan@merillat.org>
 * Added Gerd Knorrs v4l1 enhancements (Justin Schoeman)
 */

#ifndef __KERNEL__
#define __KERNEL__
#endif


#include <linux/config.h>
#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif


#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <asm/div64.h>
#include <linux/videodev.h>


#if LINUX_VERSION_CODE >= 0x020100
#include <asm/uaccess.h>
#endif
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/io.h>

#ifdef CONFIG_KMOD
#include <linux/kmod.h>
#endif

#include <linux/proc_fs.h>
#if defined(CONFIG_UST) || defined(CONFIG_UST_MODULE)
#include <linux/ust.h>
#endif

#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

#define V4L2_NUM_DEVICES	256 

static int v4l2_major		= 81;
MODULE_PARM(v4l2_major, "i");

#define VIDEO_NUM_DEVICES	256 

/*
 *	Active devices 
 */

static struct v4l2_device *v4l2_device[V4L2_NUM_DEVICES];
static struct file_operations v4l2_fops;
 
static struct video_device *video_device[VIDEO_NUM_DEVICES];
static struct file_operations video_fops;


#ifndef MODULE
#ifdef CONFIG_VIDEO_BWQCAM
extern int init_bw_qcams(struct video_init *);
#endif
#ifdef CONFIG_VIDEO_CPIA
extern int cpia_init(struct video_init *);
#endif
#ifdef CONFIG_VIDEO_PLANB
extern int init_planbs(struct video_init *);
#endif
#ifdef CONFIG_VIDEO_ZORAN
extern int init_zoran_cards(struct video_init *);
#endif
#ifdef CONFIG_VIDEO_OMAP
extern int init_v4l2_omap1510(struct video_init *ignore);
#endif
#if LINUX_VERSION_CODE < 0x020400
#ifdef CONFIG_VIDEO_BT848
extern int init_bttv_cards(struct video_init *);
extern int i2c_tuner_init(struct video_init *);
#endif
#ifdef CONFIG_VIDEO_SAA5249
extern int init_saa_5249(struct video_init *);
#endif	
#ifdef CONFIG_VIDEO_CQCAM
extern int init_colour_qcams(struct video_init *);
#endif
#ifdef CONFIG_RADIO_AZTECH
extern int aztech_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_RTRACK
extern int rtrack_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_RTRACK2
extern int rtrack2_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_SF16FMI
extern int fmi_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_MIROPCM20
extern int pcm20_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_GEMTEK
extern int gemtek_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_TYPHOON
extern int typhoon_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_CADET
extern int cadet_init(struct video_init *);
#endif
#ifdef CONFIG_RADIO_TRUST
extern int trust_init(struct video_init *);
#endif
#ifdef CONFIG_VIDEO_PMS
extern int init_pms_cards(struct video_init *);
#endif
#endif /* not 2.4.x */
#endif /* compiled into kernel (not module) */


static struct video_init video_init_list[]={
#ifndef MODULE
#ifdef CONFIG_VIDEO_BWQCAM
	{"bw-qcam", init_bw_qcams},
#endif        
#ifdef CONFIG_VIDEO_CPIA
        {"cpia", cpia_init},
#endif	
#ifdef CONFIG_VIDEO_PLANB
	{"planb", init_planbs},
#endif
#ifdef CONFIG_VIDEO_ZORAN
	{"zoran", init_zoran_cards},
#endif	
#ifdef CONFIG_VIDEO_OMAP
	{"omap1510", init_v4l2_omap1510},
#endif
#if LINUX_VERSION_CODE < 0x020400
#ifdef CONFIG_VIDEO_BT848
	{"i2c-tuner", i2c_tuner_init},
	{"bttv", init_bttv_cards},
#endif	
#ifdef CONFIG_VIDEO_SAA5249
	{"saa5249", init_saa_5249},
#endif	
#ifdef CONFIG_VIDEO_CQCAM
	{"c-qcam", init_colour_qcams},
#endif	
#ifdef CONFIG_VIDEO_PMS
	{"PMS", init_pms_cards}, 
#endif	
#ifdef CONFIG_RADIO_AZTECH
	{"Aztech", aztech_init}, 
#endif	
#ifdef CONFIG_RADIO_RTRACK
	{"RTrack", rtrack_init}, 
#endif 
#ifdef CONFIG_RADIO_RTRACK2
	{"RTrack2", rtrack2_init}, 
#endif
#ifdef CONFIG_RADIO_SF16FMI
	{"SF16FMI", fmi_init}, 
#endif	
#ifdef CONFIG_RADIO_MIROPCM20
	{"PCM20", pcm20_init}, 
#endif
#ifdef CONFIG_RADIO_CADET
	{"Cadet", cadet_init},
#endif
#ifdef CONFIG_RADIO_GEMTEK
	{"GemTek", gemtek_init},
#endif
#ifdef CONFIG_RADIO_TYPHOON
	{"radio-typhoon", typhoon_init},
#endif
#ifdef CONFIG_RADIO_TRUST
	{"Trust", trust_init}, 
#endif	
#endif /* not 2.4.x */
#endif /* compiled into kernel (not module) */
	{"end", NULL}
};


#if LINUX_VERSION_CODE >= 0x020100
/*
 *	Read will do some smarts later on. Buffer pin etc.
 */
 
inline ssize_t video_read(struct file *file,
	char *buf, size_t count, loff_t *ppos)
{
	struct video_device *vfl=video_device[MINOR(file->f_dentry->d_inode->i_rdev)];
#if LINUX_VERSION_CODE >= 0x020300
	if (vfl->fops) {
		if (vfl->fops->read)
			return vfl->fops->read(file,buf,count,ppos);
		else
			return -EINVAL;
	}
#endif
	if(vfl->read)
		return vfl->read(vfl, buf, count, file->f_flags&O_NONBLOCK);
	else
		return -EINVAL;
}

/*
 *	Write for now does nothing. No reason it shouldnt do overlay setting
 *	for some boards I guess..
 */

inline ssize_t video_write(struct file *file, const char *buf, 
	size_t count, loff_t *ppos)
{
	struct video_device *vfl=video_device[MINOR(file->f_dentry->d_inode->i_rdev)];
#if LINUX_VERSION_CODE >= 0x020300
	if (vfl->fops) {
		if (vfl->fops->write)
			return vfl->fops->write(file,buf,count,ppos);
		else
			return 0;
	}
#endif
	if(vfl->write)
		return vfl->write(vfl, buf, count, file->f_flags&O_NONBLOCK);
	else
		return 0;
}

/*
 *	Poll to see if we're readable, can probably be used for timing on incoming
 *  frames, etc..
 */

inline unsigned int video_poll(struct file *file, poll_table * wait)
{
	struct video_device *vfl=video_device[MINOR(file->f_dentry->d_inode->i_rdev)];
#if LINUX_VERSION_CODE >= 0x020300
	if (vfl->fops) {
		if (vfl->fops->poll)
			return vfl->fops->poll(file,wait);
		else
			return 0;
	}
#endif
	if(vfl->poll)
		return vfl->poll(vfl, file, wait);
	else
		return 0;
}

#else
#warning This version of videodevX does not fully support pre 2.2.x kernels
inline int video_read(struct inode *ino,struct file *file,
			  char *buf, int count)
{
         int err;
	 struct video_device *vfl=video_device[MINOR(ino->i_rdev)];
	 if (vfl->read)
	   return vfl->read(vfl, buf, count, file->f_flags&O_NONBLOCK);
	 else
	   return -EINVAL;
}

inline int video_write(struct inode *ino,struct file *file, const char *buf, 
			int count)
{
	int err;
	struct video_device *vfl=video_device[MINOR(ino->i_rdev)];
	if (vfl->write)
	  return vfl->write(vfl, buf, count, file->f_flags&O_NONBLOCK);
	else
	  return 0;
}

#endif

/*
 *	Open a video device.
 */

inline int video_open(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	int err, retval = 0;
	struct video_device *vfl;
	
	if(minor>=VIDEO_NUM_DEVICES)
		return -ENODEV;
	lock_kernel();
	vfl=video_device[minor];
	if(vfl==NULL) {
		retval = -ENODEV;
		goto error_out;
	}
#if LINUX_VERSION_CODE >= 0x020300
	if (vfl->fops) {
		unlock_kernel();
		if (vfl->fops->open)
			return vfl->fops->open(inode,file);
		else
			return 0;
	}
#endif
	if(vfl->busy) {
		retval = -EBUSY;
		goto error_out;
	}
	vfl->busy=1;		/* In case vfl->open sleeps */

#if LINUX_VERSION_CODE >= 0x020403
	if (vfl->owner)
		__MOD_INC_USE_COUNT(vfl->owner);
#endif
	unlock_kernel();
	
	if(vfl->open)
	{
		err=vfl->open(vfl,0);	/* Tell the device it is open */
		if(err)
		{
			vfl->busy=0;
#if LINUX_VERSION_CODE >= 0x020403
			if (vfl->owner)
				__MOD_DEC_USE_COUNT(vfl->owner);
#endif
			return err;
		}
	}
	return 0;
error_out:
	unlock_kernel();
	return retval;
}

/*
 *	Last close of a video for Linux device
 */
	
inline int video_release(struct inode *inode, struct file *file)
{
	struct video_device *vfl;
	lock_kernel();
	vfl=video_device[MINOR(inode->i_rdev)];
#if LINUX_VERSION_CODE >= 0x020300
	if (vfl->fops) {
		unlock_kernel();
		if (vfl->fops->release)
			return vfl->fops->release(inode,file);
		else
			return 0;
	}
#endif
	if(vfl->close)
		vfl->close(vfl);
	vfl->busy=0;
#if LINUX_VERSION_CODE >= 0x020403
	if (vfl->owner)
		__MOD_DEC_USE_COUNT(vfl->owner);
#endif
	unlock_kernel();
	return 0;
}

inline int video_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct video_device *vfl=video_device[MINOR(inode->i_rdev)];
	int err;
#if LINUX_VERSION_CODE >= 0x020300
	if (vfl->fops) {
		if (vfl->fops->ioctl)
			return vfl->fops->ioctl(inode,file,cmd,arg);
		else
			return -EINVAL;
	}
#endif
	err=vfl->ioctl(vfl, cmd, (void *)arg);
	if(err!=-ENOIOCTLCMD)
		return err;
	
	switch(cmd)
	{
		default:
			return -EINVAL;
	}
}

/*
 *	We need to do MMAP support
 */
 
 
#if LINUX_VERSION_CODE >= 0x020100
inline int video_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret = -EINVAL;
	struct video_device *vfl=video_device[MINOR(file->f_dentry->d_inode->i_rdev)];
#if LINUX_VERSION_CODE >= 0x020300
	if (vfl->fops) {
		if (vfl->fops->mmap)
			return vfl->fops->mmap(file,vma);
		else
			return -EINVAL;
	}
#endif /* 2.3.x and up */
#else /* pre 2.1.x */
inline int video_mmap(struct inode * ino, struct file * file,
		      struct vm_area_struct * vma)
{
	int ret = -EINVAL;
	struct video_device *vfl=video_device[MINOR(ino->i_rdev)];
#endif
	if(vfl->mmap) {
		lock_kernel();
		ret = vfl->mmap(vfl, (char *)vma->vm_start, 
				(unsigned long)(vma->vm_end-vma->vm_start));
		unlock_kernel();
	}
	return ret;
}

#if (LINUX_VERSION_CODE >= 0x020300) && defined(CONFIG_PROC_FS)
 static void videodev_proc_create_dev (struct video_device *vfd, char *name);
 static void videodev_proc_destroy_dev (struct video_device *vfd);
 static void v4l2_proc_create_dev (struct v4l2_device *vfd, char *name);
 static void v4l2_proc_destroy_dev (struct v4l2_device *vfd);
#endif
 
/*
 *	Video For Linux device drivers request registration here.
 */
 
#if LINUX_VERSION_CODE >= 0x020405
int video_register_device(struct video_device *vfd, int type, int nr)
#else
int video_register_device(struct video_device *vfd, int type)
#endif
{
	int i=0;
	int base;
	int err;
	int end;
	char *name_base;
	
	switch(type)
	{
		case VFL_TYPE_GRABBER:
			base=0;
			end=64;
			name_base = "video";
			break;
		case VFL_TYPE_VTX:
			base=192;
			end=224;
			name_base = "vtx";
			break;
		case VFL_TYPE_VBI:
			base=224;
			end=240;
			name_base = "vbi";
			break;
		case VFL_TYPE_RADIO:
			base=64;
			end=128;
			name_base = "radio";
			break;
		default:
			return -1;
	}
	
	for(i=base;i<end;i++)
	{
		if((video_device[i]==NULL)&&(v4l2_device[i]==NULL))
		{
#if (defined(CONFIG_PROC_FS) && (LINUX_VERSION_CODE >= 0x020300)) \
     || defined(CONFIG_DEVFS_FS)
			char name[16];
#endif
			video_device[i]=vfd;
			vfd->minor=i;
			/* The init call may sleep so we book the slot out
			   then call */
			MOD_INC_USE_COUNT;
			if(vfd->initialize)
			{
				err=vfd->initialize(vfd);
				if(err<0)
				{
					video_device[i]=NULL;
					MOD_DEC_USE_COUNT;
					return err;
				}
			}
#ifdef CONFIG_DEVFS_FS
			sprintf (name, "v4l/%s%d", name_base, i - base);
			/*
			 * Start the device root only.  Anything else
			 * has serious privacy issues.
			 */
			vfd->devfs_handle = 
				devfs_register (NULL, name, DEVFS_FL_DEFAULT,
						VIDEO_MAJOR, vfd->minor,
						S_IFCHR | S_IRUSR | S_IWUSR,
#if LINUX_VERSION_CODE >= 0x020300
						vfd->fops ? vfd->fops :
#endif
						&video_fops, NULL);
#endif

#if (LINUX_VERSION_CODE >= 0x020300) && defined(CONFIG_PROC_FS)
			sprintf (name, "%s%d", name_base, i - base);
			videodev_proc_create_dev (vfd, name);
#endif

			return 0;
		}
	}
	return -ENFILE;
}

/*
 *	Unregister an unused video for linux device
 */
 
void video_unregister_device(struct video_device *vfd)
{
	if(video_device[vfd->minor]!=vfd)
		panic("vfd: bad unregister");

#if (LINUX_VERSION_CODE >= 0x020300) && defined(CONFIG_PROC_FS)
	videodev_proc_destroy_dev (vfd);
#endif
#ifdef CONFIG_DEVFS_FS
	devfs_unregister (vfd->devfs_handle);
#endif
	video_device[vfd->minor]=NULL;
	MOD_DEC_USE_COUNT;
}


/* V4L2 stuff starts here */


/*
 *	Active devices 
 */


static struct v4l2_clock *masterclock;
static struct v4l2_v4l_compat *v4l2_v4l_compat;

/*
 *	D E V I C E   O P E R A T I O N S
 *
 */

/*
 *	Open a video device.
 */
static int
v4l2_video_open(struct inode *inode, struct file *file)
{
	unsigned int		minor = MINOR(inode->i_rdev);
	struct v4l2_device	*vfl;
	int			err;

	if (minor >= V4L2_NUM_DEVICES)
		return -ENODEV;
	vfl = v4l2_device[minor];
#ifdef CONFIG_KMOD	/*  KMOD code by Erik Walthinsen  */
	if((vfl == NULL)&&(video_device[minor] == NULL))
	{
		char modname[20];

		sprintf(modname, "char-major-%d-%d", v4l2_major, minor);
		request_module(modname);
		vfl = v4l2_device[minor];
	}
#endif
	if (vfl == NULL)
		return video_open(inode, file);
	if (vfl->open == NULL)
		return -ENODEV;
	err = vfl->open(vfl, file->f_flags, &file->private_data);
	if (err == 0 && file->private_data == NULL)
	{
		printk(KERN_ERR"V4L2: Device returned NULL open id\n");
		err = -ENODEV;
	}
	if (err == 0)
		++vfl->busy;
	return err;
}

/*
 *	Last close of a struct file
 */	
static int
v4l2_video_release(struct inode *inode, struct file *file)
{
	struct v4l2_device *vfl = v4l2_device[MINOR(inode->i_rdev)];
	
	if(!vfl)
		return video_release(inode, file);
	
	if (vfl->close)
		vfl->close(file->private_data);
	file->private_data = NULL;
	if (vfl->busy)
		--vfl->busy;
	return 0;
}

/*
 *	Read from a video device
 */
static ssize_t
v4l2_video_read(struct file *file,
	   char *buf, size_t count, loff_t *ppos)
{
	struct v4l2_device *vfl =
		v4l2_device[MINOR(file->f_dentry->d_inode->i_rdev)];
		
	if(!vfl)  
		return video_read(file, buf, count, ppos);
		
	if (vfl->read)
		return vfl->read(file->private_data, buf, count,
				 file->f_flags & O_NONBLOCK);
	return -EINVAL;
}

/*
 *	Write to a video device
 */
static ssize_t
v4l2_video_write(struct file *file,
	    const char *buf, size_t count, loff_t *ppos)
{
	struct v4l2_device *vfl =
		v4l2_device[MINOR(file->f_dentry->d_inode->i_rdev)];
		
		
	if(!vfl)
		return video_write(file, buf, count, ppos);
		
	if (vfl->write)
		return vfl->write(file->private_data, buf, count,
				  file->f_flags & O_NONBLOCK);
	return 0;
}

/*
 *	IO Control
 */
static void
fill_ctrl_category(struct v4l2_queryctrl *qc);


static int
v4l2_video_ioctl(struct inode *inode, struct file *file,
	    unsigned int cmd, unsigned long arg)
{
	struct v4l2_device *vfl = v4l2_device[MINOR(inode->i_rdev)];
	char	targ[V4L2_MAX_IOCTL_SIZE];
	void	*parg	= (void *)arg;
	int	err	= -EINVAL;
	
	if (!vfl)
		return video_ioctl(inode, file, cmd, arg);

	if (vfl->ioctl == NULL)
		return -EINVAL;

	/*  Copy arguments into temp kernel buffer  */
	switch (_IOC_DIR(cmd))
	{
	case _IOC_NONE:
		parg = (void *)arg;
		break;
	case _IOC_WRITE:
	case (_IOC_WRITE | _IOC_READ):
	/* Some v4l ioctls that are read/write are flagged read-only  */
	case _IOC_READ:
		if (_IOC_SIZE(cmd) > sizeof(targ))
		{
			printk(KERN_ERR"V4L2: ioctl 0x%08x arguments are "
			       "too big, > %d\n", cmd, sizeof(targ));
			break;/*  Arguments are too big.  */
		}
		if (copy_from_user(targ, (void *)arg, _IOC_SIZE(cmd)))
		{
			printk(KERN_INFO"V4L2: Fault on write ioctl 0x%08x "
			       "copying data from user buffer\n", cmd);
			return -EFAULT;
		}
		parg = targ;
		break;
	}

	/*  Fill in the category for pre-defined controls  */
	if (cmd == VIDIOC_QUERYCTRL)
		fill_ctrl_category((struct v4l2_queryctrl *)parg);

	/*  Try passing it to the driver first  */
	err = vfl->ioctl(file->private_data, cmd, parg);

	/*  If the driver doesn't recognize it and it's an old ioctl,
	    pass it through the translation layer.  */
	if (err == -ENOIOCTLCMD && _IOC_TYPE(cmd) == 'v' &&
		v4l2_v4l_compat != NULL)
	{
		err = v4l2_v4l_compat->translate_ioctl(
				file, vfl, file->private_data, cmd, parg);
	}

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd))
	{
	case _IOC_READ:
	case (_IOC_WRITE | _IOC_READ):
		if (parg == targ &&
		    copy_to_user((void *)arg, parg, _IOC_SIZE(cmd)))
		{
			printk(KERN_INFO"V4L2: Fault on read ioctl 0x%08x "
			       "copying results to user buffer\n", cmd);
			return -EFAULT;
		}
		break;
	}

	if (err != -ENOIOCTLCMD)
		return err;

	/*  Handle ioctls not recognized by the driver  */
	return -EINVAL;
}

/*
 *	Memory mapping
 */ 
static int
v4l2_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct v4l2_device *vfl =
		v4l2_device[MINOR(file->f_dentry->d_inode->i_rdev)];
	int	err;
	
	if(!vfl)
		return video_mmap(file, vma);
	
	if (vfl->mmap)
	{
		/*  For v4l compatibility. v4l apps typically pass zero	*/
		/*  for the offset, needs fixup.			*/
		if (v4l2_v4l_compat != NULL)
			v4l2_v4l_compat->fix_offset(file, vfl, vma);

		vma->vm_file = file;
		err = vfl->mmap(file->private_data, vma);

#if LINUX_VERSION_CODE < 0x020203
		/* This is now done in the kernel, as it should be */
 		if (err == 0 && vma->vm_file != NULL)
			++vma->vm_file->f_count;
#endif
		return err;
	}
	return -ENODEV;
}

/*
 *	Poll (select()) support
 */
static unsigned int
v4l2_video_poll(struct file *file, poll_table *table)
{
	struct v4l2_device *vfl =
		v4l2_device[MINOR(file->f_dentry->d_inode->i_rdev)];
		
	if(!vfl)
		return video_poll(file, table);
		
	if (vfl->poll)
		return vfl->poll(file->private_data, file, table);
	return POLLERR;
}

/*
 *	Not used.
 */ 
static loff_t
v4l2_video_llseek(struct file *file,
	     loff_t offset, int origin)
{
	return -ESPIPE;
}

/*
 *	CONTROL CATEGORIES
 */
static void
fill_ctrl_category(struct v4l2_queryctrl *qc)
{
	if ((qc->id >= V4L2_CID_BRIGHTNESS &&
	     qc->id <= V4L2_CID_HUE) ||
	    (qc->id >= V4L2_CID_BLACK_LEVEL &&
	     qc->id <= V4L2_CID_LASTP1-1))
	{
		qc->category = V4L2_CTRL_CAT_VIDEO;
		strcpy(qc->group, "Video");
	} else
	if ((qc->id >= V4L2_CID_AUDIO_VOLUME &&
	     qc->id <= V4L2_CID_AUDIO_LOUDNESS))
	{
		qc->category = V4L2_CTRL_CAT_AUDIO;
		strcpy(qc->group, "Audio");
	} else
	if ((qc->id >= V4L2_CID_EFFECT_BASE &&
	     qc->id <= V4L2_CID_EFFECT_BASE + 10000))
	{
		qc->category = V4L2_CTRL_CAT_EFFECT;
		strcpy(qc->group, "Effect");
	} else
	{
		strcpy(qc->group, "Private");
	}
}


#if defined(CONFIG_PROC_FS) || defined(CONFIG_DEVFS_FS)
static char *device_types[] =
{
	"capture",	"codec",	"output",	"effects", 
	"vbi",		"vtr",		"teletext",	"radio", 
	"undef",	"undef",	"undef",	"undef",
};
#endif

/*
 *	D E V I C E   R E G I S T R A T I O N
 *
 *	Video for Linux Two device drivers request registration here.
 */
int
v4l2_register_device(struct v4l2_device *vfl)
{
	int	i	= 0;
	int	err;
#if (LINUX_VERSION_CODE >= 0x020300) && \
    (defined(CONFIG_PROC_FS) || defined(CONFIG_DEVFS_FS))
	char	*base_name;
	char	name[32];
#endif

	if (vfl == NULL)
	{
		printk(KERN_ERR"V4L2: v4l2_register_device() passed"
		       " a NULL pointer!\n");
		return -1;
	}
	i = vfl->minor;
	if (vfl->open == NULL)
	{
		printk(KERN_ERR "V4L2: Device %d has no open method\n", i);
		return -1;
	}
	if (i < 0 || i >= V4L2_NUM_DEVICES)
	{
		printk(KERN_ERR"V4L2: Minor value %d is out of range\n", i);
		return -1;
	}
	if (v4l2_device[i] != NULL)
	{
		printk(KERN_ERR"V4L2: %s and %s have both been assigned"
		       " minor %d\n", v4l2_device[i]->name,
		       vfl->name, i);
		return 1;
	}

  	if (video_device[i] != NULL)
	{
		printk(KERN_ERR"V4L2: %s(v4l1) and %s(v4l2) have both been assigned"
		       " minor %d\n", video_device[i]->name,
		       vfl->name, i);
		return 1;
	}

	v4l2_device[i] = vfl;
	/* The init call may sleep so we book the slot out then call */
	MOD_INC_USE_COUNT;
	err = 0;
	if (vfl->initialize)
		err = vfl->initialize(vfl);
	if (err < 0)
	{
		printk(KERN_ERR "V4L2: %s initialize method failed\n",
		       vfl->name);
		v4l2_device[i] = NULL;
		MOD_DEC_USE_COUNT;
		return err;
	}

	vfl->busy = 0;

	vfl->name[sizeof(vfl->name) - 1] = 0;
	printk(KERN_INFO"V4L2: Registered \"%s\" as char device %d, %d\n",
		 vfl->name, v4l2_major, vfl->minor);

#ifdef CONFIG_KMOD
	/*  Try loading the v4l compatibility layer  */
	if (v4l2_v4l_compat == NULL)
		request_module("v4l_compat");
#endif

#if (LINUX_VERSION_CODE >= 0x020300) && \
    defined(CONFIG_PROC_FS) || defined(CONFIG_DEVFS_FS)
	if (vfl->type >= 0 &&
	    vfl->type < sizeof(device_types)/sizeof(char*))
		base_name = device_types[vfl->type];
	else if (vfl->type >= V4L2_TYPE_PRIVATE)
		base_name = "private";
	else 
		base_name = "undef";
	
#if defined(CONFIG_PROC_FS)
	sprintf(name, "%s%i", base_name, i);
	v4l2_proc_create_dev (vfl, name);
#endif /* CONFIG_PROCFS */
#endif

#ifdef CONFIG_DEVFS_FS
	/*
	 * Start the device root only.  Anything else
	 * has serious privacy issues.
	 */
	if (vfl->devfs_devname[0]==0)
		sprintf (name, "v4l2/%s%d", base_name, i);
	else
		sprintf (name, "v4l2/%s", vfl->devfs_devname);
	vfl->devfs_handle = 
		devfs_register (NULL, name, DEVFS_FL_DEFAULT,
				v4l2_major, vfl->minor,
				S_IFCHR | S_IRUSR | S_IWUSR,
				&v4l2_fops, NULL);
#endif

	memset(vfl->v4l2_reserved, 0, sizeof(vfl->v4l2_reserved));
	vfl->v4l2_priv = NULL;
	return 0;
}

/*
 *	Unregister an unused video for linux device
 */
void
v4l2_unregister_device(struct v4l2_device *vfl)
{
	if (vfl->minor < 0 || vfl->minor >= V4L2_NUM_DEVICES ||
	    v4l2_device[vfl->minor] != vfl)
	{
		printk(KERN_ERR"V4L2: bad unregister\n");
		return;
	}

#if (LINUX_VERSION_CODE >= 0x020300) && defined(CONFIG_PROC_FS)
	/* Add the /proc entry */
	v4l2_proc_destroy_dev (vfl);
#endif

#ifdef CONFIG_DEVFS_FS
	devfs_unregister (vfl->devfs_handle);
#endif
	v4l2_device[vfl->minor] = NULL;
	MOD_DEC_USE_COUNT;
}


/*
 *	Register/unregister v4l back compatibility layer
 */
int
v4l2_v4l_compat_register(struct v4l2_v4l_compat *v4l_c)
{
	if (v4l_c == NULL)
	{
		printk(KERN_ERR"V4L2: v4l2_register_v4l_compat() passed"
		       " a NULL pointer!\n");
		return -1;
	}
	if (v4l2_v4l_compat != NULL)
	{
		printk(KERN_ERR"V4L2: Attempt to re-register v4l1 "
		       "compatibility layer.\n");
		return -1;
	}
	if (v4l_c->translate_ioctl == NULL)
	{
		printk(KERN_ERR "V4L2: v4l1 compatibility layer has no "
		       "translate_ioctl() method\n");
		return -1;
	}
	if (v4l_c->fix_offset == NULL)
	{
		printk(KERN_ERR "V4L2: v4l1 compatibility layer has no "
		       "fix_offset() method\n");
		return -1;
	}

	v4l2_v4l_compat = v4l_c;

	printk(KERN_INFO"V4L2: v4l1 backward compatibility enabled.\n");
	return 0;
}
void
v4l2_v4l_compat_unregister(struct v4l2_v4l_compat *v4l_c)
{
	if (v4l2_v4l_compat != v4l_c)
		return;
	v4l2_v4l_compat = NULL;
	printk(KERN_INFO"V4L2: v4l1 backward compatibility disabled.\n");
}


/*
 *	/ p r o c / v i d e o d e v   H A N D L E R
 */

#ifdef CONFIG_PROC_FS
/* The actual code is the same, it's the interface what changes */
#if 0
static int
video_build_proc(char *buf, char **start, off_t offset, int len, void *data)
{
	struct v4l2_device *vfl;
	struct video_device *vfl1;
	int	i;
	char	*t;

	len = 0;
	len += sprintf(buf, "Video for Linux Two (V%d.%d alpha)."
		       " Major device: %d\n",
		       V4L2_MAJOR_VERSION, V4L2_MINOR_VERSION,
		       v4l2_major);
	//len += sprintf(buf+len,"minor: type      busy name\n");
	for (i = 0; i < V4L2_NUM_DEVICES; i++)
	{
		vfl = v4l2_device[i];
		if (vfl == NULL)
			continue;
		if  (len > (PAGE_SIZE - 80))
			return len;
		if (vfl->type >= 0 &&
		    vfl->type < sizeof(device_types)/sizeof(char*))
			t = device_types[vfl->type];
		else if (vfl->type >= V4L2_TYPE_PRIVATE)
			t = "private";
		else 
			t = "undef";
		len += sprintf(buf+len, "%5d: %-9s %3d  %s\n",
			       vfl->minor, t, vfl->busy, vfl->name);
	}
	len += sprintf(buf+len, "Video for Linux One\n");
	for (i = 0; i < VIDEO_NUM_DEVICES; i++)
	{
		vfl1 = video_device[i];
		if (vfl1 == NULL)
			continue;
		if  (len > (PAGE_SIZE - 80))
			return len;
		len += sprintf(buf+len, "%5d: v4l1 capture device %3d  %s\n",
			       vfl1->minor, vfl1->busy, vfl1->name);
	}
	return len;
}
#endif

#if LINUX_VERSION_CODE < 0x020300

/*  Original /proc file code from Erik Walthinsen  */

static int
video_read_proc(char *buf, char **start, off_t offset, int len, int unused)
{
	return video_build_proc(buf, start, offset, len, NULL);
}

/* proc file for /proc/videodev */
static struct proc_dir_entry video_proc_entry =
{
	0, 8, "videodev", S_IFREG | S_IRUGO, 1, 0, 0, 0, NULL,
	&video_read_proc
};
#else /* 2.3 */

#if defined(CONFIG_PROC_FS)
struct videodev_proc_data {
	struct list_head proc_list;
	char name[16];
	struct video_device *vdev;
	struct v4l2_device *v2dev;
	struct proc_dir_entry *proc_entry;
};

static struct proc_dir_entry *video_dev_proc_entry = NULL;
struct proc_dir_entry *video_proc_entry = NULL;
EXPORT_SYMBOL(video_proc_entry);
LIST_HEAD(videodev_proc_list);

static int videodev_proc_read(char *page, char **start, off_t off,
			       int count, int *eof, void *data)
{
	int len = 0;
	struct videodev_proc_data *d = 0;
	struct list_head *tmp;

	list_for_each (tmp, &videodev_proc_list) {
		d = list_entry(tmp, struct videodev_proc_data, proc_list);
		if ((data == d->vdev) || (data == d->v2dev))
			break;
	}

	/* not found */
	if (tmp == &videodev_proc_list)
		return 0;

#if 1
/* Check whether we are opening a V4L1 or V4L2 device */
	if (d->vdev)
		len = sprintf(page,
			"Video4Linux1 device:\n"
			"  Name:\td->vdev->name\n"
			"  Minor:\td->vdev->minor\n");
	else if (d->v2dev)
		len = sprintf(page,
			"Video4Linux2 device:\n"
			"  Name: \t%s\n"
			"  Minor:\t%d\n"
			"  Type: \t%s\n", d->v2dev->name, d->v2dev->minor, 
				(d->v2dev->type > V4L2_TYPE_PRIVATE) ? "undef" :
				device_types[d->v2dev->type]);
	else
		panic("vdev proc: none of V4L1 or V4L2");
#else
	len = video_build_proc(page, start, off, count, data);
#endif
	/* fix the read count. Not so important, since usually we only
	 do cat /proc.. */
	len -= off;

	if (len < count) {
		*eof = 1;
		if (len <= 0)
			return 0;
	}
	else
		len = count;

	*start = page + off;

	return len;
}

static void videodev_proc_create(void)
{
	video_proc_entry = create_proc_entry("video", S_IFDIR, &proc_root);

	if (video_proc_entry == NULL) {
		printk("video_dev: unable to initialise /proc/video\n");
		return;
	}

	video_proc_entry->owner = THIS_MODULE;
	video_dev_proc_entry = create_proc_entry("dev", S_IFDIR, video_proc_entry);

	if (video_dev_proc_entry == NULL) {
		printk("video_dev: unable to initialise /proc/video/dev\n");
		return;
	}

	video_dev_proc_entry->owner = THIS_MODULE;
}

#ifdef MODULE
#if defined(CONFIG_PROC_FS)
static void videodev_proc_destroy(void)
{
	if (video_dev_proc_entry != NULL)
		remove_proc_entry("dev", video_proc_entry);

	if (video_proc_entry != NULL)
		remove_proc_entry("video", &proc_root);
}
#endif
#endif

static void videodev_proc_create_dev (struct video_device *vfd, char *name)
{
	struct videodev_proc_data *d;
	struct proc_dir_entry *p;

	if (video_dev_proc_entry == NULL)
		return;

	d = kmalloc (sizeof (struct videodev_proc_data), GFP_KERNEL);
	if (!d)
		return;

	p = create_proc_entry(name, S_IFREG|S_IRUGO|S_IWUSR, video_dev_proc_entry);
	p->data = vfd;
	p->read_proc = videodev_proc_read;

	d->proc_entry = p;
	d->vdev = vfd;
	d->v2dev = NULL; /* not a V4L2 device */
	strcpy (d->name, name);

	list_add (&d->proc_list, &videodev_proc_list);
}

static void videodev_proc_destroy_dev (struct video_device *vfd)
{
	struct list_head *tmp;
	struct videodev_proc_data *d;

	list_for_each (tmp, &videodev_proc_list) {
		d = list_entry(tmp, struct videodev_proc_data, proc_list);
		if (vfd == d->vdev) {
			remove_proc_entry(d->name, video_dev_proc_entry);
			list_del (&d->proc_list);
			kfree (d);
			break;
		}
	}
}

static void v4l2_proc_create_dev (struct v4l2_device *vfd, char *name)
{
	struct videodev_proc_data *d;
	struct proc_dir_entry *p;

	if (video_dev_proc_entry == NULL)
		return;

	d = kmalloc (sizeof (struct videodev_proc_data), GFP_KERNEL);
	if (!d)
		return;

	p = create_proc_entry(name, S_IFREG|S_IRUGO|S_IWUSR, video_dev_proc_entry);
	p->data = vfd;
	p->read_proc = videodev_proc_read;

	d->proc_entry = p;
	d->vdev = NULL; /* not a V4L device */
	d->v2dev = vfd;
	strcpy (d->name, name);

	list_add (&d->proc_list, &videodev_proc_list);
}

static void v4l2_proc_destroy_dev (struct v4l2_device *vfd)
{
	struct list_head *tmp;
	struct videodev_proc_data *d;

	list_for_each (tmp, &videodev_proc_list) {
		d = list_entry(tmp, struct videodev_proc_data, proc_list);
		if (vfd == d->v2dev) {
			remove_proc_entry(d->name, video_dev_proc_entry);
			list_del (&d->proc_list);
			kfree (d);
			break;
		}
	}
}
#endif
#endif
#endif

/*
 *	V I D E O   F O R   L I N U X   T W O   I N I T I A L I Z A T I O N
 */

static struct file_operations video_fops =
{
#if LINUX_VERSION_CODE >= 0x020403
	owner:		THIS_MODULE,
#endif
	llseek:		v4l2_video_llseek,
	read:		video_read,
	write:		video_write,
	poll:		video_poll,
	ioctl:		video_ioctl,
	mmap:		video_mmap,
	open:		video_open,
	release:	video_release
};
static struct file_operations v4l2_fops =
{
	llseek:		v4l2_video_llseek,
	read:		v4l2_video_read,
	write:		v4l2_video_write,
	poll:		v4l2_video_poll,
	ioctl:		v4l2_video_ioctl,
	mmap:		v4l2_video_mmap,
	open:		v4l2_video_open,
	release:	v4l2_video_release
};

/*
 *	Initialize Video for Linux Two
 */
	
#ifdef CONFIG_DEVFS_FS
#define REGISTER_CHRDEV devfs_register_chrdev
#define UNREGISTER_CHRDEV devfs_unregister_chrdev
#else
#define REGISTER_CHRDEV register_chrdev
#define UNREGISTER_CHRDEV unregister_chrdev
#endif

static int
videodev_register_chrdev(int interface /* 1 or 2 */)
{
	int			major[3];
	char			*name;
	struct file_operations	*fops;
	major[1] = VIDEO_MAJOR;
	major[2] = v4l2_major;
	name = (interface == 1) ? "v4l1" : "v4l2";
	fops = (interface == 1) ? &video_fops : &v4l2_fops;

	if (interface == 1)
		printk(KERN_INFO"Video for Linux One (2.2.16)."
		       " Major device: %d\n", major[1]);
	if (interface == 2)
		printk(KERN_INFO"Video for Linux Two (V%d.%d)."
		       " Major device: %d\n",
		       V4L2_MAJOR_VERSION, V4L2_MINOR_VERSION, major[2]);

	if (major[1] == major[2])
	{
		if (interface == 2)
			return 0;
		name = "v4l1/2";
		fops = &v4l2_fops;
	}

	if (REGISTER_CHRDEV(major[interface], name, fops))
	{
		printk("Unable to get major %d for %s\n",
		       major[interface], name);
		return -EIO;
	}
	return 0;
}

static int
videodev_unregister_chrdev(int interface /* 1 or 2 */)
{
	int	major[3];
	char	*name;
	major[1] = VIDEO_MAJOR;
	major[2] = v4l2_major;
	name = (interface == 1) ? "v4l1" : "v4l2";

	if (major[1] == major[2])
	{
		if (interface == 2)
			return 0;
		name = "v4l1/2";
	}

	UNREGISTER_CHRDEV(major[interface], name);
	return 0;
}

int videodev_init(void)
{
	int	i;
	struct video_init *vfli = video_init_list;

	i = videodev_register_chrdev(1);
	if (i == 0)
		i = videodev_register_chrdev(2);
	if (i)
		return i;

	/* make sure there's a way to tell if a device is not there */
	for (i = 0; i < V4L2_NUM_DEVICES; i++)
		v4l2_device[i] = NULL;
	for (i = 0; i < VIDEO_NUM_DEVICES; i++)
		video_device[i] = NULL;
#ifdef CONFIG_PROC_FS
#if  LINUX_VERSION_CODE < 0x020300
	proc_register(&proc_root, &video_proc_entry);
#else /* 2.3 */
	videodev_proc_create ();
#endif
#endif
	masterclock = NULL;

	while(vfli->init!=NULL)
	{
		vfli->init(vfli);
		vfli++;
	}

	return 0;
}

#ifdef MODULE		
int init_module(void)
{
	return videodev_init();
}

void cleanup_module(void)
{
#ifdef CONFIG_PROC_FS
#if  LINUX_VERSION_CODE < 0x020300
	proc_unregister(&proc_root, video_proc_entry.low_ino);
#else /* 2.3 */
	videodev_proc_destroy ();
#endif
#endif /* CONFIG_PROC_FS */
	videodev_unregister_chrdev(1);
	videodev_unregister_chrdev(2);
}
#endif


/*
 *
 *	V 4 L 2   D R I V E R   H E L P E R   A P I
 *
 */

void
v4l2_version(int *major, int *minor)
{
	*major = V4L2_MAJOR_VERSION;
	*minor = V4L2_MINOR_VERSION;
}

int
v4l2_major_number(void)
{
	return v4l2_major;
}

struct v4l2_device *
v4l2_device_from_minor(int minor)
{
	if (minor < 0 || minor >= V4L2_NUM_DEVICES)
		return NULL;
	return v4l2_device[minor];
}

struct v4l2_device *
v4l2_device_from_file(struct file *file)
{
	if (file == NULL)
		return NULL;
	return v4l2_device_from_minor(MINOR(file->f_dentry->d_inode->i_rdev));
}

void *
v4l2_openid_from_file(struct file *file)
{
	if (file == NULL)
		return NULL;
	return file->private_data;
}

#if  LINUX_VERSION_CODE >= 0x020300

struct page *kvirt_to_pa(unsigned long adr)
{
	struct page *ret = NULL;
	pmd_t *pmd;
	pte_t *pte;
	pgd_t *pgd;

	pgd = pgd_offset_k(adr);
	if (!pgd_none(*pgd)) {
		pmd = pmd_offset(pgd, adr);
		if (!pmd_none(*pmd)) {
			pte = pte_offset(pmd, adr);
			if (pte_present(*pte)) {
				ret = pte_page(*pte);
			}
		}
	}

	return ret;
}

/*  Useful for using vmalloc()ed memory as DMA target  */
unsigned long v4l2_vmalloc_to_bus(void *virt)
{
	struct page *page;
	unsigned long kva, ret;

	page = kvirt_to_pa((unsigned long) virt);
	kva = ((unsigned long)page_address(page)) | (((unsigned long) virt) & (PAGE_SIZE - 1));
	ret = virt_to_bus((void *) kva);

	return ret;
}

/*  Useful for a nopage handler when mmap()ing vmalloc()ed memory  */
struct page *v4l2_vmalloc_to_page(void *virt)
{
	struct page *page;

	page = kvirt_to_pa((unsigned long) virt);

	return page;
}

#else /* 2.2 */

static struct mm_struct *
find_init_mm(void)
{
	static struct mm_struct	*mm;
	struct task_struct	*p;
	if (mm)
		return mm;
	for (p = current; p && (p = p->next_task) != current; )
		if (p->pid == 0)
			break;
	mm = (p) ? p->mm : NULL;
	return mm;
}

/*  Useful for using vmalloc()ed memory as DMA target  */
unsigned long
v4l2_vmalloc_to_bus(void *virt)
{
	pgd_t		*pgd;
	pmd_t		*pmd;
	pte_t		*pte;
	unsigned long	a = (unsigned long)virt;
	struct mm_struct *mm = find_init_mm();

	if (mm == NULL ||
	    pgd_none(*(pgd = pgd_offset(mm,  a))) ||
	    pmd_none(*(pmd = pmd_offset(pgd, a))) ||
	    pte_none(*(pte = pte_offset(pmd, a))))
		return 0;
	return virt_to_bus((void *)pte_page(*pte))
		+ (a & (PAGE_SIZE - 1));
}

/*  Useful for a nopage handler when mmap()ing vmalloc()ed memory  */
unsigned long
v4l2_vmalloc_to_page(void *virt)
{
	pgd_t		*pgd;
	pmd_t		*pmd;
	pte_t		*pte;
	unsigned long	a = (unsigned long)virt;
	struct mm_struct *mm = find_init_mm();

	if (mm == NULL ||
	    pgd_none(*(pgd = pgd_offset(current->mm, a))) ||
	    pmd_none(*(pmd = pmd_offset(pgd,         a))) ||
	    pte_none(*(pte = pte_offset(pmd,         a))))
		return 0;
	return pte_page(*pte);
}
#endif /* 2.2 */


/*
 *  Simple queue management
 */
static rwlock_t rw_lock_unlocked = RW_LOCK_UNLOCKED;
void
v4l2_q_init(struct v4l2_queue *q)
{
	if (q == NULL)
		return;
	q->qlock = rw_lock_unlocked;
	q->forw = (struct v4l2_q_node *)q;
	q->back = (struct v4l2_q_node *)q;
}
void
v4l2_q_add_head(struct v4l2_queue *q, struct v4l2_q_node *node)
{
	unsigned long flags;
	if (q == NULL || node == NULL)
		return;
	if (q->forw == NULL || q->back == NULL)
		v4l2_q_init(q);
	write_lock_irqsave(&(q->qlock), flags);
	node->forw = q->forw;
	node->back = (struct v4l2_q_node *)q;
	q->forw->back = node;
	q->forw = node;
	write_unlock_irqrestore(&(q->qlock), flags);
}
void
v4l2_q_add_tail(struct v4l2_queue *q, struct v4l2_q_node *node)
{
	unsigned long flags;
	if (q == NULL || node == NULL)
		return;
	if (q->forw == NULL || q->back == NULL)
		v4l2_q_init(q);
	write_lock_irqsave(&(q->qlock), flags);
	node->forw = (struct v4l2_q_node *)q;
	node->back = q->back;
	q->back->forw = node;
	q->back = node;
	write_unlock_irqrestore(&(q->qlock), flags);
}
void *
v4l2_q_del_head(struct v4l2_queue *q)
{
	unsigned long flags;
	struct v4l2_q_node *node;
	if (q == NULL)
		return NULL;
	write_lock_irqsave(&(q->qlock), flags);
	if (q->forw == NULL || q->back == NULL ||
	    q->forw == (struct v4l2_q_node *)q ||
	    q->back == (struct v4l2_q_node *)q)
	{
		write_unlock_irqrestore(&(q->qlock), flags);
		return NULL;
	}
	node = q->forw;
	node->forw->back = (struct v4l2_q_node *)q;
	q->forw = node->forw;
	node->forw = NULL;
	node->back = NULL;
	write_unlock_irqrestore(&(q->qlock), flags);
	return node;
}
void *
v4l2_q_del_tail(struct v4l2_queue *q)
{
	unsigned long flags;
	struct v4l2_q_node *node;
	if (q == NULL)
		return NULL;
	write_lock_irqsave(&(q->qlock), flags);
	if (q->forw == NULL || q->back == NULL ||
	    q->forw == (struct v4l2_q_node *)q ||
	    q->back == (struct v4l2_q_node *)q)
	{
		write_unlock_irqrestore(&(q->qlock), flags);
		return NULL;
	}
	node = q->back;
	node->back->forw = (struct v4l2_q_node *)q;
	q->back = node->back;
	node->forw = NULL;
	node->back = NULL;
	write_unlock_irqrestore(&(q->qlock), flags);
	return node;
}
void *
v4l2_q_peek_head(struct v4l2_queue *q)
{
	unsigned long flags;
	struct v4l2_q_node *node;
	read_lock_irqsave(&(q->qlock), flags);
	if (q == NULL || q->forw == NULL || q->forw == (struct v4l2_q_node *)q)
	{
		read_unlock_irqrestore(&(q->qlock), flags);
		return NULL;
	}
	node = q->forw;
	read_unlock_irqrestore(&(q->qlock), flags);
	return node;
}
void *
v4l2_q_peek_tail(struct v4l2_queue *q)
{
	unsigned long flags;
	struct v4l2_q_node *node;
	read_lock_irqsave(&(q->qlock), flags);
	if (q == NULL || q->back == NULL || q->back == (struct v4l2_q_node *)q)
	{
		read_unlock_irqrestore(&(q->qlock), flags);
		return NULL;
	}
	node = q->back;
	read_unlock_irqrestore(&(q->qlock), flags);
	return node;
}
void *
v4l2_q_yank_node(struct v4l2_queue *q, struct v4l2_q_node *node)
{
	unsigned long flags;
	struct v4l2_q_node *t;
	if (v4l2_q_peek_head(q) == NULL || node == NULL)
		return NULL;
	write_lock_irqsave(&(q->qlock), flags);
	for (t = q->forw; t != (struct v4l2_q_node *)q; t = t->forw)
		if (t == node)
		{
			node->back->forw = node->forw;
			node->forw->back = node->back;
			node->forw = NULL;
			node->back = NULL;
			write_unlock_irqrestore(&(q->qlock), flags);
			return node;
		}
	write_unlock_irqrestore(&(q->qlock), flags);
	return NULL;
}
int
v4l2_q_last(struct v4l2_queue *q)
{
/*  This function by Olivier Carmona  */

	unsigned long flags;
	read_lock_irqsave(&(q->qlock), flags);
	if (q == NULL)
	{
		read_unlock_irqrestore(&(q->qlock), flags);
		return -1;
	}
	if (q->forw == NULL || q->back == NULL ||
	    q->forw == (struct v4l2_q_node *)q ||
	    q->back == (struct v4l2_q_node *)q)
	{
		read_unlock_irqrestore(&(q->qlock), flags);
		return -1;
	}
	if (q->forw == q->back)
	{
		read_unlock_irqrestore(&(q->qlock), flags);
		return 1;
	}
	read_unlock_irqrestore(&(q->qlock), flags);
	return 0;
}


/*
 *  Math functions
 */

u32
v4l2_math_div6432(u64 a, u32 d, u32 *r)
{
	u32 m = do_div(a, d);
	if (r) *r = m;
	return (u32)a;
}

unsigned long
v4l2_timestamp_divide(stamp_t t, unsigned long p_100ns)
{
	/*  Note: 't' is in 1ns units, 'p_100ns' is in 100ns units, */
	/*  and the quotient is rounded  */
	u64	p;

	p = (u64)p_100ns * 100;  /* 1ns units */
	t >>= 6;      /*  /64 to allow p_100ns longer than 4 secs. */
	p >>= 6;  /*  to keep quotient the same  */
	return v4l2_math_div6432((u64)t + (p >> 1), (u32)p, NULL);
}

/*  Force the timestamp to be an integer multiple of p_100ns  */
unsigned long
v4l2_timestamp_correct(stamp_t *t, unsigned long p_100ns)
{
	/*  Note: 't' is in 1ns units, 'p_100ns' is in 100ns units */
	unsigned long	n;

	n = v4l2_timestamp_divide((u64)*t, p_100ns);
	*t = (u64)p_100ns * n * 100;
	return n;
}

/*
 *	Master clock operations
 */

int
v4l2_masterclock_register(struct v4l2_clock *clock)
{
	if (clock == NULL || clock->gettime == NULL)
		return -1;
	if (masterclock != NULL)
		return -1;
	masterclock = clock;
	MOD_INC_USE_COUNT;
	return 0;
}
void
v4l2_masterclock_unregister(struct v4l2_clock *clock)
{
	if (clock != masterclock)
		return;
	masterclock = NULL;
	MOD_DEC_USE_COUNT;
}
void
v4l2_masterclock_gettime(stamp_t *curr)
{
	if (masterclock)
		masterclock->gettime(curr);
	else
	{
#if defined(CONFIG_UST) || defined(CONFIG_UST_MODULE)
		ust_gettime(curr);
#else
		struct timeval	t;
		stamp_t stamp;
		do_gettimeofday(&t);
		stamp = (stamp_t)t.tv_sec * 1000000 + t.tv_usec;
		stamp *= 1000;
		*curr = stamp;
#endif
	}
}

/*
 *  Video Standard Operations (contributed by Michael Schimek)
 */

/* This is the recommended method to deal with the framerate fields. More 
   sophisticated drivers will access the fields directly. */
unsigned int
v4l2_video_std_fps(struct v4l2_standard *vs)
{ 
	if (vs->framerate.numerator > 0)
		return (((vs->framerate.denominator << 8) / 
			 vs->framerate.numerator) + 
			(1 << 7)) / (1 << 8);
	return 0;
}

/*  Compute the time per frame in 100ns units  */
unsigned long
v4l2_video_std_tpf(struct v4l2_standard *vs)
{
	return v4l2_math_div6432(
		(u64)vs->framerate.numerator * 10000000
		+ vs->framerate.denominator / 2,
		vs->framerate.denominator,
		NULL);
}

/*  Used only in v4l2_video_std_confirm()  */
static void
catc1p2e6(__u8 *s, char c, int n)
{
	n /= 10000;
	sprintf(s + strlen(s), "%c%d.%02d", c, n / 100, n % 100);
}

/* Verify the validity of the parameters of a v4l2_standard structure and
   create the name and id from the other fields. It does not relieve a 
   driver from examining if it can fulfill the request.  Returns an 
   errno < 0 if inconsistent, 0 if an unknown but maybe usable format, 
   or the V4L2_STD_XXX_X value if a known standard. */
int
v4l2_video_std_confirm(struct v4l2_standard *vs)
{
	unsigned int	rate  = 0;
	unsigned int	lines = vs->framelines;
	int		std   = 0;

	strcpy(vs->name, "Unknown");
	if (vs->reserved1 || vs->reserved2)
		return -EINVAL;

	if (vs->framerate.numerator > 0 &&	
	    vs->framerate.denominator > 0)
		rate = v4l2_video_std_fps(vs);

	if (vs->framelines >= 624 && vs->framelines <= 626)
		lines = 625;
	else if (vs->framelines >= 524 && vs->framelines <= 526)
		lines = 525;

	if (rate == 0 || lines == 0 || rate > 200)
		return -EINVAL;

	switch (vs->colorstandard)
	{
	case V4L2_COLOR_STD_PAL:
		strcpy(vs->name, "PAL");
		if (rate == 25 && lines == 625)
			switch (vs->colorstandard_data.pal.colorsubcarrier)
			{
			case V4L2_COLOR_SUBC_PAL_N:
				strcpy(vs->name, "PAL-N");
				if (vs->transmission & ~V4L2_TRANSM_STD_N)
					return -EINVAL;
				return V4L2_STD_PAL_N;
			case V4L2_COLOR_SUBC_PAL:
				if (vs->transmission & 
				    ~(V4L2_TRANSM_STD_B | V4L2_TRANSM_STD_G |
				      V4L2_TRANSM_STD_H | V4L2_TRANSM_STD_I |
				      V4L2_TRANSM_STD_D))
					return -EINVAL;
				std = V4L2_STD_PAL;
				goto addtransm;
			}
		else if (rate == 30 && lines == 525)
			switch (vs->colorstandard_data.pal.colorsubcarrier)
			{
			case V4L2_COLOR_SUBC_PAL_M:
				strcpy(vs->name, "PAL-M");
				if (vs->transmission & ~V4L2_TRANSM_STD_M)
					return -EINVAL;
				return V4L2_STD_PAL_M;
			case V4L2_COLOR_SUBC_PAL:
				strcpy(vs->name, "PAL-60");
				if (vs->transmission)
					return -EINVAL;
				return V4L2_STD_PAL_60;
			}
		if (vs->transmission)
			return -EINVAL;
		catc1p2e6(vs->name, ' ', 
			  vs->colorstandard_data.pal.colorsubcarrier);
		break;

	case V4L2_COLOR_STD_NTSC:
		strcpy(vs->name, "NTSC");
		if (rate == 25 && lines == 625)
			switch (vs->colorstandard_data.ntsc.colorsubcarrier)
			{
			case V4L2_COLOR_SUBC_NTSC:
				strcpy(vs->name, "NTSC-N");
				if (vs->transmission & ~V4L2_TRANSM_STD_N)
					return -EINVAL;
				return V4L2_STD_NTSC_N;
			}
		else if (rate == 30 && lines == 525)
			switch (vs->colorstandard_data.ntsc.colorsubcarrier)
			{
			case V4L2_COLOR_SUBC_NTSC:
				if (vs->transmission & ~V4L2_TRANSM_STD_M)
					return -EINVAL;
				std = V4L2_STD_NTSC;
				goto addtransm;
			case V4L2_COLOR_SUBC_PAL:
				strcpy(vs->name, "NTSC-44");
				if (vs->transmission)
					return -EINVAL;
				return V4L2_STD_NTSC_44;
			}
		if (vs->transmission)
			return -EINVAL;
		catc1p2e6(vs->name, ' ', 
			  vs->colorstandard_data.ntsc.colorsubcarrier);
		break;

	case V4L2_COLOR_STD_SECAM:
		strcpy(vs->name, "SECAM");
		if (rate == 25 && lines == 625)
			if (vs->colorstandard_data.secam.f0b == 
			    V4L2_COLOR_SUBC_SECAMB &&
			    vs->colorstandard_data.secam.f0r == 
			    V4L2_COLOR_SUBC_SECAMR)
			{
				if (vs->transmission &
				    ~(V4L2_TRANSM_STD_B | V4L2_TRANSM_STD_D |
			              V4L2_TRANSM_STD_G | V4L2_TRANSM_STD_K |
			              V4L2_TRANSM_STD_K1 | V4L2_TRANSM_STD_L))
					return -EINVAL;
				std = V4L2_STD_SECAM;
				goto addtransm;
			}
		if (vs->transmission)
			return -EINVAL;
		catc1p2e6(vs->name, ' ', vs->colorstandard_data.secam.f0b);
		catc1p2e6(vs->name, '/', vs->colorstandard_data.secam.f0r);
		break;

	default:
		return -EINVAL;
	}

        sprintf(vs->name + strlen(vs->name), " %d/%d",
	        vs->framelines, rate);

	return std;

 addtransm:
	if (vs->transmission) strcat(vs->name, "-");

        if (vs->transmission & V4L2_TRANSM_STD_B) strcat(vs->name, "B/");
        if (vs->transmission & V4L2_TRANSM_STD_G) strcat(vs->name, "G/");
        if (vs->transmission & V4L2_TRANSM_STD_H) strcat(vs->name, "H/");
	if (vs->transmission & V4L2_TRANSM_STD_I) strcat(vs->name, "I/");
	if (vs->transmission & V4L2_TRANSM_STD_D) strcat(vs->name, "D/");
	if (vs->transmission & V4L2_TRANSM_STD_K) strcat(vs->name, "K/");
	if (vs->transmission & V4L2_TRANSM_STD_K1) strcat(vs->name, "K1/");
	if (vs->transmission & V4L2_TRANSM_STD_L) strcat(vs->name, "L/");
	if (vs->transmission & V4L2_TRANSM_STD_M) strcat(vs->name, "M/");
	if (vs->transmission & V4L2_TRANSM_STD_N) strcat(vs->name, "N/");

	if (vs->name[strlen(vs->name) - 1] == '/')
		vs->name[strlen(vs->name) - 1] = 0;    

	return std;
}

/* Fill in the fields of a v4l2_standard structure according to the
   'id' and 'transmission' parameters.  Returns negative on error.  */
int
v4l2_video_std_construct(struct v4l2_standard *vs,
			 int id, __u32 transmission)
{
	memset(vs, 0, sizeof(struct v4l2_standard));

	vs->framerate.numerator = 1;
	vs->framerate.denominator = 25;
	vs->framelines = 625;

	switch (id)
	{
	case V4L2_STD_PAL_60:
		vs->framerate.numerator = 1001;
		vs->framerate.denominator = 30000;
		vs->framelines = 525;
		/* fall thru */
	case V4L2_STD_PAL:
		vs->colorstandard = V4L2_COLOR_STD_PAL;
		vs->colorstandard_data.pal.colorsubcarrier =
			V4L2_COLOR_SUBC_PAL;
		break;
	case V4L2_STD_PAL_M:
		vs->framerate.numerator = 1001;
		vs->framerate.denominator = 30000;
		vs->framelines = 525;
		vs->colorstandard = V4L2_COLOR_STD_PAL;
		vs->colorstandard_data.pal.colorsubcarrier = 
			V4L2_COLOR_SUBC_PAL_M;
		break;
	case V4L2_STD_PAL_N:
		vs->colorstandard = V4L2_COLOR_STD_PAL;
		vs->colorstandard_data.pal.colorsubcarrier = 
			V4L2_COLOR_SUBC_PAL_N;
		break;

	case V4L2_STD_NTSC:
		vs->framerate.numerator = 1001;
		vs->framerate.denominator = 30000;
		vs->framelines = 525;
		/* fall thru */
	case V4L2_STD_NTSC_N:
		vs->colorstandard = V4L2_COLOR_STD_NTSC;
		vs->colorstandard_data.ntsc.colorsubcarrier = 
			V4L2_COLOR_SUBC_NTSC;
		break;
	case V4L2_STD_NTSC_44:
		vs->framerate.numerator = 1001;
		vs->framerate.denominator = 30000;
		vs->framelines = 525;
		vs->colorstandard = V4L2_COLOR_STD_NTSC;
		vs->colorstandard_data.ntsc.colorsubcarrier = 
			V4L2_COLOR_SUBC_PAL;
		break;

	case V4L2_STD_SECAM:
		vs->colorstandard = V4L2_COLOR_STD_SECAM;
		vs->colorstandard_data.secam.f0b = V4L2_COLOR_SUBC_SECAMB;
		vs->colorstandard_data.secam.f0r = V4L2_COLOR_SUBC_SECAMR;
		break;

	default:
		return -EINVAL;
	}

	vs->transmission = transmission;

	return v4l2_video_std_confirm(vs);
}


/*---------------------------------------*/

EXPORT_SYMBOL(v4l2_register_device);
EXPORT_SYMBOL(v4l2_unregister_device);
EXPORT_SYMBOL(v4l2_v4l_compat_register);
EXPORT_SYMBOL(v4l2_v4l_compat_unregister);

EXPORT_SYMBOL(v4l2_version);
EXPORT_SYMBOL(v4l2_major_number);
EXPORT_SYMBOL(v4l2_device_from_minor);
EXPORT_SYMBOL(v4l2_device_from_file);
EXPORT_SYMBOL(v4l2_openid_from_file);
EXPORT_SYMBOL(v4l2_vmalloc_to_bus);
EXPORT_SYMBOL(v4l2_vmalloc_to_page);
EXPORT_SYMBOL(v4l2_q_init);
EXPORT_SYMBOL(v4l2_q_add_head);
EXPORT_SYMBOL(v4l2_q_add_tail);
EXPORT_SYMBOL(v4l2_q_del_head);
EXPORT_SYMBOL(v4l2_q_del_tail);
EXPORT_SYMBOL(v4l2_q_peek_head);
EXPORT_SYMBOL(v4l2_q_peek_tail);
EXPORT_SYMBOL(v4l2_q_yank_node);
EXPORT_SYMBOL(v4l2_q_last);
EXPORT_SYMBOL(v4l2_math_div6432);
EXPORT_SYMBOL(v4l2_timestamp_divide);
EXPORT_SYMBOL(v4l2_timestamp_correct);
EXPORT_SYMBOL(v4l2_masterclock_register);
EXPORT_SYMBOL(v4l2_masterclock_unregister);
EXPORT_SYMBOL(v4l2_masterclock_gettime);
EXPORT_SYMBOL(v4l2_video_std_fps);
EXPORT_SYMBOL(v4l2_video_std_tpf);
EXPORT_SYMBOL(v4l2_video_std_confirm);
EXPORT_SYMBOL(v4l2_video_std_construct);
EXPORT_SYMBOL(video_register_device);
EXPORT_SYMBOL(video_unregister_device);

