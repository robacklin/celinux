/*
 * linux/drivers/block/nvprd.c
 *
 * Non-Volatile and H/W Write-Protected RAM disk.
 *
 * ----------------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 * originally created 22 Jan 2002
 *
 * original author: Erez Doron (erez@savan.com)
 *
 * Jamey Hicks JAN-30-2002 
 * Added nvrd_size cmdline option, made all routines static, reindented to
 * linux style, and added module_init and module_cleanup.  
 *
 * Erez Doron FEB-18-2002
 * Fixed offset problem.
 * IMPORTANT: the driver is not backward compatible
 *
 * Steve Longerbeam APR-08-2003 <stevel@mvista.com>
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Added h/w write protection, and ability to put NVRD anywhere in physical
 * address space, not just at end of system memory.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#ifndef I386
#include <asm/arch/memory.h>
#endif
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/blk.h>
#include <linux/blkpg.h>

#define DEBUG

#ifdef CONFIG_MODVERSIONS
#define MODVERSIONS
#include <linux/modversions.h>
#endif

#define NVRD_MAJOR	0       /* auto detect */
#define NVRD_NAME	"nvrd"
#define BLOCKSIZE	1024
#define SECTSIZE	512
#define SECTSIZE_SHIFT	9	/* log 2 of SECTSIZE */

#ifdef DEBUG
#define	dbg(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __func__, ## args)
#else
#define	dbg(fmt, args...)
#endif

#define PFX NVRD_NAME
#define err(format, arg...) printk(KERN_ERR PFX ": " format , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format , ## arg)
#define emerg(format, arg...) printk(KERN_EMERG PFX ": " format , ## arg)

static int nvrd_major;
static phys_addr_t nvrd_physaddr=0;     /* start physaddr */
static unsigned int nvrd_size=0;      /* size in bytes */
static int nvrd_nblocks;		/* size in blocks */
static int nvrd_blocksize;
static int nvrd_sectsize;
static int nvrd_sectsize_shift;	/* log 2 of nvrd_sectsize */
static long nvrd_nsectors;
static int nvrd_bytesize;	/* size in bytes */
static char *nvrd_dptr;		/* data pointer */
static devfs_handle_t devfs_handle;

MODULE_PARM(nvrd_size, "i");
MODULE_PARM_DESC(nvrd_size, "NVRD Size in bytes");
MODULE_PARM(nvrd_physaddr, "i");
MODULE_PARM_DESC(nvrd_physaddr, "NVRD Physical Start Address");

#ifndef MODULE
int __init setup_nvrd_size(char *cmdline)
{
        char *aftersizeptr;
        nvrd_size = memparse(cmdline, &aftersizeptr);
        return 1;
} 

int __init setup_nvrd_physaddr(char *cmdline)
{
        char *afteraddrptr;
        nvrd_physaddr = memparse(cmdline, &afteraddrptr);
        return 1;
} 

__setup("nvrd_size=", setup_nvrd_size);
__setup("nvrd_physaddr=", setup_nvrd_physaddr);
#endif

static int nvrd_open(struct inode *inode, struct file *file)
{
        if (MINOR(inode->i_rdev) > 0)
                return -ENXIO;

        MOD_INC_USE_COUNT;

        return 0;
}

static int nvrd_release(struct inode *inode, struct file *file)
{
        MOD_DEC_USE_COUNT;

        return 0;
}

static int nvrd_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{

        if (MINOR(inode->i_rdev)) return -EINVAL;
        switch (cmd) {
        case BLKFLSBUF :
                if (!capable(CAP_SYS_ADMIN))
                        return -EACCES;
                destroy_buffers(inode->i_rdev);
                break;

        case BLKGETSIZE :
                return put_user(nvrd_nsectors, (long *) arg);

        case BLKROSET :
        case BLKROGET :
        case BLKSSZGET :
                return blk_ioctl(inode->i_rdev, cmd, arg);    

        default :
                return -EINVAL;
        }

        return 0;
}

static struct block_device_operations nvrd_bd_op = {
        open:	        nvrd_open,
        release:	nvrd_release,
        ioctl:	        nvrd_ioctl,
};


// init_mm.page_table_lock must be held before calling!
static void page_writeable(void * vaddr, int rw)
{
        unsigned long addr = (unsigned long)vaddr;
	pgd_t *pgdp;
	pmd_t *pmdp;
	pte_t *ptep;
	
	pgdp = pgd_offset_k(addr);
	if (!pgd_none(*pgdp)) {
		pmdp = pmd_offset(pgdp, addr);
		if (!pmd_none(*pmdp)) {
			pte_t pte;
			ptep = pte_offset(pmdp, addr);
			pte = *ptep;
			if (pte_present(pte)) {
				pte = rw ? pte_mkwrite(pte) :
					pte_wrprotect(pte);
				set_pte(ptep, pte);
				// FIXME: I'd like to be able to flush
				// just this page, not the whole TLB,
				// but flush_tlb_page() only works for
				// user vm areas, and this is a kernel
				// mapping. Also should call update_mmu_cache()
				// after TLB flush, but can't for same
				// reason.
				flush_tlb_all();
			}
		}
	}
}

static int nvrd_make_request(request_queue_t *q, int rw,
			     struct buffer_head *sbh)
{
        char *ptr, *bdata;
        long offset, len, len1=0;
	unsigned long flags;

        if ((MINOR(sbh->b_rdev)) > 0) goto fail;


        len = sbh->b_size;

        /* i put the first block in end of mem and last block
         * in the beginning so when enlarging the ramdisk by 
         * starting it previuosly in the memory, will really
         * enlarge the end of the disk so one could use
         * programs like resize2fs
         */

	offset = (nvrd_nsectors - sbh->b_rsector - 1l) << nvrd_sectsize_shift;
	//offset = sbh->b_rsector << nvrd_sectsize_shift;
	bdata = bh_kmap(sbh);

	for ( ; len; len -= len1,
		      offset -= nvrd_sectsize,
		      bdata += nvrd_sectsize) {

		//len1=min(len,nvrd_sectsize)
		len1 = len < nvrd_sectsize ? len : nvrd_sectsize;

		if (((offset+len1)>nvrd_bytesize)||(offset<0)) {
			err("attempt to access beyond device, b_rsector=%lu\n",
			    (unsigned long)(sbh->b_rsector));
			dbg("offset=%li, len1=%li, nvrd_bytesize=%li\n",
			    offset,len1,(long)nvrd_bytesize);
			goto fail;
		}

		ptr = nvrd_dptr + offset;

		switch (rw) {
		case READ :
		case READA : 
			memcpy(bdata, ptr, len1);
			break;
		case WRITE :
			/*
			 * NOTE: a sector never crosses a page boundary,
			 * so when writing a sector, we only have to modify
			 * the writeable flag for a single page.
			 */
			spin_lock_irqsave(&init_mm.page_table_lock, flags);
			page_writeable(ptr, 1);
			memcpy(ptr, bdata, len1);
			page_writeable(ptr, 0);
			spin_unlock_irqrestore(&init_mm.page_table_lock,flags);
			break;
		default :
			err("bad command: %d\n", rw);
			goto fail;
		}
	}
   
        sbh->b_end_io(sbh, 1);
        return 0;
    
 fail:
        sbh->b_end_io(sbh, 0);
        return 0;
}

static int __init init_nvrd(void)
{
	if (nvrd_size == 0) {
		err("NVRD size in bytes must be specified\n");
		return -EINVAL;
	}

	if (nvrd_physaddr == 0) {
		/*
		 * if physaddr parameter not specified, assume some portion
		 * of the end of physical memory is unused (kernel booted
		 * with 'mem=').
		 */
		/* PHYS_OFFSET is physical address of start of DRAM */
		nvrd_physaddr = (num_physpages<<PAGE_SHIFT) + PHYS_OFFSET;
	}
	//nvrd_size <<= 20;

        nvrd_sectsize = SECTSIZE;
        nvrd_sectsize_shift = SECTSIZE_SHIFT;
        nvrd_blocksize = BLOCKSIZE;
        nvrd_nblocks = nvrd_size / nvrd_blocksize; // in blocks
        nvrd_bytesize = (nvrd_nblocks*nvrd_blocksize);
        nvrd_nsectors = nvrd_bytesize >> nvrd_sectsize_shift;

        dbg("nvrd_bytesize= %u\n", nvrd_bytesize);
        dbg("nvrd_nsectors= %lu\n", nvrd_nsectors);

        // remap physical memory to virtual one
        nvrd_dptr = __ioremap_readonly(nvrd_physaddr, nvrd_size, 0);
        if (!nvrd_dptr) {
                err("ioremap failed\n");
                return -EIO;
        }

        info("using adresses: 0x%lX..0x%lX\n",
	       nvrd_physaddr, nvrd_physaddr + nvrd_size - 1);
        info("mapped to: 0x%lX\n",(unsigned long)nvrd_dptr);
	
        if ((nvrd_major = devfs_register_blkdev(NVRD_MAJOR,
						NVRD_NAME,
						&nvrd_bd_op)) < 0) {
                err("device registration failed (%d)\n", nvrd_major);
                return -EIO;
        }

        info("major number %d assigned\n", nvrd_major);
    
        blk_queue_make_request(BLK_DEFAULT_QUEUE(nvrd_major),
			       &nvrd_make_request);

        devfs_handle = devfs_mk_dir(NULL, NVRD_NAME, NULL);
        devfs_register_series(devfs_handle, "%u", 1,
                              DEVFS_FL_DEFAULT, nvrd_major, 0,
                              S_IFBLK | S_IRUSR | S_IWUSR,
                              &nvrd_bd_op, NULL);

    
        hardsect_size[nvrd_major] = &nvrd_sectsize;
        blksize_size[nvrd_major] = &nvrd_blocksize;
        blk_size[nvrd_major] = &nvrd_nblocks;

        return 0;
}

static void __exit cleanup_nvrd(void)
{

        destroy_buffers(MKDEV(nvrd_major, 0));
        devfs_unregister(devfs_handle);
        devfs_unregister_blkdev(nvrd_major, NVRD_NAME);
        hardsect_size[nvrd_major] = NULL;
        blksize_size[nvrd_major] = NULL;
        blk_size[nvrd_major] = NULL;
	iounmap(nvrd_dptr);
	
	info("unloaded\n");
}

module_init(init_nvrd);
module_exit(cleanup_nvrd);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Erez Doron <erez@savan.com>");
MODULE_DESCRIPTION("Non-Volatile/Write-Protected Ram Disk");
