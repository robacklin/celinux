/*
 * mount.c - operations for initializing and mounting sysfs.
 */

#if 0 /* linux-pm */
#define DEBUG 
#endif

#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/pagemap.h>
#include <linux/init.h>

#include "sysfs.h"

#if 1 /* linux-pm */
extern struct file_operations simple_dir_operations;
extern struct inode_operations simple_dir_inode_operations;

extern int simple_statfs(struct super_block *, struct statfs *);
#endif /* linux-pm */

/* Random magic number */
#define SYSFS_MAGIC 0x62656572

struct vfsmount *sysfs_mount;
struct super_block * sysfs_sb = NULL;

static struct super_operations sysfs_ops = {
	.statfs		= simple_statfs,
#if 0 /* linux-pm */
	.drop_inode	= generic_delete_inode,
#endif /* linux-pm */
};

static int sysfs_fill_super(struct super_block *sb, void *data, int silent)
{
	struct inode *inode;
	struct dentry *root;

	sb->s_blocksize = PAGE_CACHE_SIZE;
	sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
	sb->s_magic = SYSFS_MAGIC;
	sb->s_op = &sysfs_ops;
	sysfs_sb = sb;

	inode = sysfs_new_inode(S_IFDIR | S_IRWXU | S_IRUGO | S_IXUGO);
	if (inode) {
		inode->i_op = &simple_dir_inode_operations;
		inode->i_fop = &simple_dir_operations;
		/* directory inodes start off with i_nlink == 2 (for "." entry) */
		inode->i_nlink++;	
	} else {
		pr_debug("sysfs: could not get root inode\n");
		return -ENOMEM;
	}

	root = d_alloc_root(inode);
	if (!root) {
		pr_debug("%s: could not get root dentry!\n",__FUNCTION__);
		iput(inode);
		return -ENOMEM;
	}
	sb->s_root = root;
	return 0;
}

#if 1 /* linux-pm */
struct super_block *sysfs_read_super(struct super_block *s, void *data, 
				     int silent)
{
	return (sysfs_fill_super(s, data, silent) == 0) ? s : NULL;
}

#else /* linux-pm */
static struct super_block *sysfs_get_sb(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
	return get_sb_single(fs_type, flags, data, sysfs_fill_super);
}
#endif /* linux-pm */

static struct file_system_type sysfs_fs_type = {
	.name		= "sysfs",
#if 1 /* linux-pm */
	.fs_flags	= FS_SINGLE,
	.read_super	= sysfs_read_super,
#else /* linux-pm */
	.get_sb		= sysfs_get_sb,
	.kill_sb	= kill_litter_super,
#endif /* linux-pm */
};

int __init sysfs_init(void)
{
	int err;

	err = register_filesystem(&sysfs_fs_type);
	if (!err) {
		sysfs_mount = kern_mount(&sysfs_fs_type);
		if (IS_ERR(sysfs_mount)) {
			printk(KERN_ERR "sysfs: could not mount!\n");
			err = PTR_ERR(sysfs_mount);
			sysfs_mount = NULL;
		}
	}
	return err;
}
