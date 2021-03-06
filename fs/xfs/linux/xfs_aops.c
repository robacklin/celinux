/*
 * Copyright (c) 2000-2002 Silicon Graphics, Inc.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Further, this software is distributed without any warranty that it is
 * free of the rightful claim of any third person regarding infringement
 * or the like.	 Any license provided herein, whether implied or
 * otherwise, applies only to this software file.  Patent licenses, if
 * any, provided herein do not apply to combinations of this program with
 * other software, or any other product whatsoever.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Contact information: Silicon Graphics, Inc., 1600 Amphitheatre Pkwy,
 * Mountain View, CA  94043, or:
 *
 * http://www.sgi.com
 *
 * For further information regarding this notice, see:
 *
 * http://oss.sgi.com/projects/GenInfo/SGIGPLNoticeExplan/
 */

#include <xfs.h>
#include <linux/mm.h>
#include <linux/iobuf.h>
#include <linux/locks.h>
#include <linux/pagemap.h>


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,9)
#define page_buffers(page)	((page)->buffers)
#define page_has_buffers(page)	((page)->buffers)
#endif

STATIC int linvfs_pb_bmap(struct inode *, loff_t, ssize_t,
			  struct page_buf_bmap_s *, int);
STATIC int delalloc_convert(struct inode *, struct page *, int, int);

/*
 * match_offset_to_mapping
 * Finds the corresponding mapping in block @map array of the
 * given @offset within a @page.
 */
STATIC page_buf_bmap_t *
match_offset_to_mapping(
	struct page		*page,
	page_buf_bmap_t		*map,
	unsigned long		offset)
{
	loff_t			full_offset;	/* offset from start of file */

	ASSERT(offset < PAGE_CACHE_SIZE);

	full_offset = page->index;		/* NB: using 64bit number */
	full_offset <<= PAGE_CACHE_SHIFT;	/* offset from file start */
	full_offset += offset;			/* offset from page start */

	if (full_offset < map->pbm_offset)
		return NULL;
	if (map->pbm_offset + map->pbm_bsize > full_offset)
		return map;
	return NULL;
}

STATIC void
map_buffer_at_offset(
	struct page		*page,
	struct buffer_head	*bh,
	unsigned long		offset,
	int			block_bits,
	page_buf_bmap_t		*mp)
{
	page_buf_daddr_t	bn;
	loff_t			delta;
	int			sector_shift;

	ASSERT(!(mp->pbm_flags & PBMF_HOLE));
	ASSERT(!(mp->pbm_flags & PBMF_DELAY));
	ASSERT(!(mp->pbm_flags & PBMF_UNWRITTEN));
	ASSERT(mp->pbm_bn != PAGE_BUF_DADDR_NULL);

	delta = page->index;
	delta <<= PAGE_CACHE_SHIFT;
	delta += offset;
	delta -= mp->pbm_offset;
	delta >>= block_bits;

	sector_shift = block_bits - 9;
	bn = mp->pbm_bn >> sector_shift;
	bn += delta;
	ASSERT((bn << sector_shift) >= mp->pbm_bn);

	lock_buffer(bh);
	bh->b_blocknr = bn;
	bh->b_dev = mp->pbm_target->pbr_kdev;
	set_bit(BH_Mapped, &bh->b_state);
	clear_bit(BH_Delay, &bh->b_state);
}

/*
 * Convert delalloc space to real space, do not flush the
 * data out to disk, that will be done by the caller.
 */
STATIC int
release_page(
	struct page		*page)
{
	struct inode		*inode = (struct inode*)page->mapping->host;
	unsigned long		end_index = inode->i_size >> PAGE_CACHE_SHIFT;
	int			ret;

	/* Are we off the end of the file ? */
	if (page->index >= end_index) {
		unsigned offset = inode->i_size & (PAGE_CACHE_SIZE-1);
		if ((page->index >= end_index+1) || !offset) {
			ret =  -EIO;
			goto out;
		}
	}

	ret = delalloc_convert(inode, page, 0, 0);

out:
	if (ret < 0) {
		block_flushpage(page, 0);
		ClearPageUptodate(page);

		return 0;
	}

	return 1;
}

/*
 * Convert delalloc or unmapped space to real space and flush out
 * to disk.
 */
STATIC int
write_full_page(
	struct page		*page,
	int			delalloc)
{
	struct inode		*inode = (struct inode*)page->mapping->host;
	unsigned long		end_index = inode->i_size >> PAGE_CACHE_SHIFT;
	int			ret;

	/* Are we off the end of the file ? */
	if (page->index >= end_index) {
		unsigned offset = inode->i_size & (PAGE_CACHE_SIZE-1);
		if ((page->index >= end_index+1) || !offset) {
			ret =  -EIO;
			goto out;
		}
	}

	if (!page_has_buffers(page)) {
		create_empty_buffers(page, inode->i_dev, 1 << inode->i_blkbits);
	}

	ret = delalloc_convert(inode, page, 1, delalloc == 0);

out:
	if (ret < 0) {
		/*
		 * If it's delalloc and we have nowhere to put it,
		 * throw it away.
		 */
		if (delalloc)
			block_flushpage(page, 0);
		ClearPageUptodate(page);
		unlock_page(page);
	}

	return ret;
}

/*
 * Look for a page at index which is unlocked and not mapped
 * yet - clustering for mmap write case.
 */
STATIC unsigned int
probe_unmapped_page(
	struct address_space	*mapping,
	unsigned long		index,
	unsigned int		pg_offset)
{
	struct page		*page;
	int			ret = 0;

	page = find_get_page(mapping, index);
	if (!page)
		return 0;
	if (TryLockPage(page)) {
		page_cache_release(page);
		return 0;
	}
	if (page->mapping && PageDirty(page)) {
		if (!page_has_buffers(page)) {
			ret = PAGE_CACHE_SIZE;
		} else {
			struct buffer_head	*bh, *head;
			bh = head = page_buffers(page);
			do {
				if (buffer_mapped(bh) || !buffer_uptodate(bh)) {
					break;
				}
				ret += bh->b_size;
				if (ret >= pg_offset)
					break;
			} while ((bh = bh->b_this_page) != head);
		}
	}

	unlock_page(page);
	page_cache_release(page);
	return ret;
}

STATIC unsigned int
probe_unmapped_cluster(
	struct inode		*inode,
	struct page		*startpage,
	struct buffer_head	*bh,
	struct buffer_head	*head)
{
	unsigned long		tindex, tlast;
	unsigned int		len, total = 0;
	struct address_space	*mapping = inode->i_mapping;

	/* First sum forwards in this page */
	do {
		if (buffer_mapped(bh))
			break;
		total += bh->b_size;
	} while ((bh = bh->b_this_page) != head);

	/* if we reached the end of the page, sum forwards in
	 * following pages.
	 */
	if (bh == head) {
		tlast = inode->i_size >> PAGE_CACHE_SHIFT;
		for (tindex = startpage->index + 1; tindex < tlast; tindex++) {
			len = probe_unmapped_page(mapping, tindex,
							PAGE_CACHE_SIZE);
			if (!len)
				break;
			total += len;
		}
		if ((tindex == tlast) && (inode->i_size & ~PAGE_CACHE_MASK)) {
			len = probe_unmapped_page(mapping, tindex,
					inode->i_size & ~PAGE_CACHE_MASK);
			total += len;
		}
	}
	return total;
}

/*
 * Probe for a given page (index) in the inode & test if it is delayed.
 * Returns page locked and with an extra reference count.
 */
STATIC struct page *
probe_page(
	struct inode		*inode,
	unsigned long		index)
{
	struct page		*page;

	page = find_get_page(inode->i_mapping, index);
	if (!page)
		return NULL;
	if (TryLockPage(page)) {
		page_cache_release(page);
		return NULL;
	}
	if (page->mapping && page_has_buffers(page)) {
		struct buffer_head	*bh, *head;

		bh = head = page_buffers(page);
		do {
			if (buffer_delay(bh))
				return page;
		} while ((bh = bh->b_this_page) != head);
	}
	unlock_page(page);
	page_cache_release(page);
	return NULL;
}

STATIC void
submit_page(
	struct page		*page,
	struct buffer_head	*bh_arr[],
	int			cnt)
{
	struct buffer_head	*bh;
	int			i;

	if (cnt) {
		for (i = 0; i < cnt; i++) {
			bh = bh_arr[i];
			set_buffer_async_io(bh);
			set_bit(BH_Uptodate, &bh->b_state);
			clear_bit(BH_Dirty, &bh->b_state);
		}

		for (i = 0; i < cnt; i++)
			submit_bh(WRITE, bh_arr[i]);
	} else
		unlock_page(page);
}

STATIC int
map_page(
	struct inode		*inode,
	struct page		*page,
	page_buf_bmap_t		*maps,
	struct buffer_head	*bh_arr[],
	int			startio,
	int			all_bh)
{
	struct buffer_head	*bh, *head;
	page_buf_bmap_t		*mp = maps, *tmp;
	unsigned long		end, offset, end_index;
	int			i = 0, index = 0;
	int			bbits = inode->i_blkbits;

	end_index = inode->i_size >> PAGE_CACHE_SHIFT;
	if (page->index < end_index) {
		end = PAGE_CACHE_SIZE;
	} else {
		end = inode->i_size & (PAGE_CACHE_SIZE-1);
	}
	bh = head = page_buffers(page);
	do {
		offset = i << bbits;
		if (!(Page_Uptodate(page) || buffer_uptodate(bh)))
			continue;
		if (buffer_mapped(bh) && !buffer_delay(bh) && all_bh) {
			if (startio && (offset < end)) {
				lock_buffer(bh);
				bh_arr[index++] = bh;
			}
			continue;
		}
		tmp = match_offset_to_mapping(page, mp, offset);
		if (!tmp)
			continue;
		ASSERT(!(tmp->pbm_flags & PBMF_HOLE));
		ASSERT(!(tmp->pbm_flags & PBMF_DELAY));
		map_buffer_at_offset(page, bh, offset, bbits, tmp);
		if (startio && (offset < end)) {
			bh_arr[index++] = bh;
		} else {
			unlock_buffer(bh);
		}
	} while (i++, (bh = bh->b_this_page) != head);

	return index;
}

/*
 * Allocate & map buffers for page given the extent map. Write it out.
 * except for the original page of a writepage, this is called on
 * delalloc pages only, for the original page it is possible that
 * the page has no mapping at all.
 */
STATIC void
convert_page(
	struct inode		*inode,
	struct page		*page,
	page_buf_bmap_t		*maps,
	int			startio,
	int			all_bh)
{
	struct buffer_head	*bh_arr[MAX_BUF_PER_PAGE];
	int			cnt;

	cnt = map_page(inode, page, maps, bh_arr, startio, all_bh);
	submit_page(page, bh_arr, cnt);
	page_cache_release(page);
}

/*
 * Convert & write out a cluster of pages in the same extent as defined
 * by mp and following the start page.
 */
STATIC void
cluster_write(
	struct inode		*inode,
	unsigned long		tindex,
	page_buf_bmap_t		*mp,
	int			startio,
	int			all_bh)
{
	unsigned long		tlast;
	struct page		*page;

	tlast = (mp->pbm_offset + mp->pbm_bsize) >> PAGE_CACHE_SHIFT;
	for (; tindex < tlast; tindex++) {
		if (!(page = probe_page(inode, tindex)))
			break;
		convert_page(inode, page, mp, startio, all_bh);
	}
}

/*
 * Calling this without allocate_space set means we are being asked to
 * flush a dirty buffer head. When called with async_write set then we
 * are coming from writepage. A writepage call with allocate_space set
 * means we are being asked to write out all of the page which is before
 * EOF and therefore need to allocate space for unmapped portions of the
 * page.
 *
 * When called with startio e.g. from
 * write page it is important that we write WHOLE page if possible. The
 * bh->b_state's can not know of any of the blocks or which block for
 * that matter are dirty due to map writes, and therefore bh uptodate is
 * only vaild if the pagei itself isn't completely uptodate. Some layers
 * may clear the page dirty flag prior to calling write page under the
 * assumption the entire page will be written out, by not writing out the
 * whole page the page can be reused before all vaild dirty data is
 * written out. Note: in the case of a page that has been dirty'd by
 * mapwrite and but partially setup by block_prepare_write the
 * bh->b_states's will not agree and only ones setup by BPW/BCW will have
 * valid state, thus the whole page must be written out thing.
 */

STATIC int
delalloc_convert(
	struct inode		*inode,		/* inode containing page */
	struct page		*page,		/* page to convert - locked */
	int			startio,	/* start io on the page */
	int			allocate_space)
{
	struct buffer_head	*bh, *head;
	struct buffer_head	*bh_arr[MAX_BUF_PER_PAGE];
	page_buf_bmap_t		*mp, map;
	int			i, cnt = 0;
	int			len, err;
	unsigned long		p_offset = 0;
	loff_t			offset;
	loff_t			end_offset;

	offset = (loff_t)page->index << PAGE_CACHE_SHIFT;
	end_offset = offset + PAGE_CACHE_SIZE;
	if (end_offset > inode->i_size)
		end_offset = inode->i_size;

	bh = head = page_buffers(page);
	mp = NULL;

	len = bh->b_size;
	do {
		if (!(Page_Uptodate(page) || buffer_uptodate(bh)) && !startio) {
			goto next_bh;
		}

		if (mp) {
			mp = match_offset_to_mapping(page, &map, p_offset);
		}

		if (buffer_delay(bh)) {
			if (!mp) {
				err = linvfs_pb_bmap(inode, offset, len, &map,
						PBF_WRITE|PBF_FILE_ALLOCATE);
				if (err)
					goto error;
				mp = match_offset_to_mapping(page, &map,
								p_offset);
			}
			if (mp) {
				map_buffer_at_offset(page, bh, p_offset,
					inode->i_blkbits, mp);
				if (startio) {
					bh_arr[cnt++] = bh;
				} else {
					unlock_buffer(bh);
				}
			}
		} else if ((buffer_uptodate(bh) || Page_Uptodate(page)) &&
			   (allocate_space || startio)) {
			if (!buffer_mapped(bh)){
				int	size;
				
				/* Getting here implies an unmapped buffer was found,
				 * and we are in a path where we need to write the
				 * whole page out.
				 */
				if (!mp) {
					size = probe_unmapped_cluster(inode, page,
								      bh, head);
					err = linvfs_pb_bmap(inode, offset, size, &map,
							     PBF_WRITE|PBF_DIRECT);
					if (err) {
						goto error;
					}
					mp = match_offset_to_mapping(page, &map,
								     p_offset);
				}
				if (mp) {
					map_buffer_at_offset(page, bh, p_offset,
							     inode->i_blkbits, mp);
					if (startio) {
						bh_arr[cnt++] = bh;
					} else {
						unlock_buffer(bh);
					}
				}
			} else if (startio && buffer_mapped(bh)) {
				if(buffer_uptodate(bh) && allocate_space) {
					lock_buffer(bh);
					bh_arr[cnt++] = bh;
				}
			}
		}

next_bh:
		offset += len;
		p_offset += len;
		bh = bh->b_this_page;
	} while (offset < end_offset);

	if (startio)
		submit_page(page, bh_arr, cnt);

	if (mp)
		cluster_write(inode, page->index + 1, mp,
				startio, allocate_space);

	return 0;

error:
	for (i = 0; i < cnt; i++) {
		unlock_buffer(bh_arr[i]);
	}

	return err;
}

STATIC int
linvfs_get_block_core(
	struct inode		*inode,
	long			iblock,
	struct buffer_head	*bh_result,
	int			create,
	int			direct,
	page_buf_flags_t	flags)
{
	vnode_t			*vp = LINVFS_GET_VP(inode);
	page_buf_bmap_t		pbmap;
	int			retpbbm = 1;
	int			error;
	ssize_t			size;
	loff_t			offset = (loff_t)iblock << inode->i_blkbits;

	/* If we are doing writes at the end of the file,
	 * allocate in chunks
	 */
	if (create && (offset >= inode->i_size) && !(flags & PBF_SYNC))
		size = 1 << XFS_WRITE_IO_LOG;
	else
		size = 1 << inode->i_blkbits;

	VOP_BMAP(vp, offset, size,
		create ? flags : PBF_READ, NULL,
		(struct page_buf_bmap_s *)&pbmap, &retpbbm, error);
	if (error)
		return -error;

	if (retpbbm == 0)
		return 0;

	if (pbmap.pbm_bn != PAGE_BUF_DADDR_NULL) {
		page_buf_daddr_t	bn;
		loff_t			delta;

		delta = offset - pbmap.pbm_offset;
		delta >>= inode->i_blkbits;

		bn = pbmap.pbm_bn >> (inode->i_blkbits - 9);
		bn += delta;

		bh_result->b_blocknr = bn;
		set_bit(BH_Mapped, &bh_result->b_state);
	}

	/* If we previously allocated a block out beyond eof and
	 * we are now coming back to use it then we will need to
	 * flag it as new even if it has a disk address.
	 */
	if (create &&
	    ((!buffer_mapped(bh_result) && !buffer_uptodate(bh_result)) ||
	     (offset >= inode->i_size))) {
		set_bit(BH_New, &bh_result->b_state);
	}

	if (pbmap.pbm_flags & PBMF_DELAY) {
		if (unlikely(direct))
			BUG();

		if (create) {
			set_bit(BH_Mapped, &bh_result->b_state);
		}
		set_bit(BH_Delay, &bh_result->b_state);
	}

	return 0;
}

int
linvfs_get_block(
	struct inode		*inode,
	long			iblock,
	struct buffer_head	*bh_result,
	int			create)
{
	return linvfs_get_block_core(inode, iblock, bh_result,
					create, 0, PBF_WRITE);
}

STATIC int
linvfs_get_block_sync(
	struct inode		*inode,
	long			iblock,
	struct buffer_head	*bh_result,
	int			create)
{
	return linvfs_get_block_core(inode, iblock, bh_result,
					create, 0, PBF_SYNC|PBF_WRITE);
}


STATIC int
linvfs_get_block_direct(
	struct inode		*inode,
	long			iblock,
	struct buffer_head	*bh_result,
	int			create)
{
	return linvfs_get_block_core(inode, iblock, bh_result,
					create, 1, PBF_WRITE|PBF_DIRECT);
}

STATIC int
linvfs_pb_bmap(
	struct inode		*inode,
	loff_t			offset,
	ssize_t			count,
	page_buf_bmap_t		*pbmapp,
	int			flags)
{
	vnode_t			*vp = LINVFS_GET_VP(inode);
	int			error, nmaps = 1;

retry:
	if (flags & PBF_FILE_ALLOCATE) {
		VOP_STRATEGY(vp, offset, count, flags, NULL,
				pbmapp, &nmaps, error);
	} else {
		VOP_BMAP(vp, offset, count, flags, NULL,
				pbmapp, &nmaps, error);
	}
	if (flags & PBF_WRITE) {
		if (unlikely((flags & PBF_DIRECT) && nmaps &&
		    (pbmapp->pbm_flags & PBMF_DELAY))) {
			flags = PBF_WRITE | PBF_FILE_ALLOCATE;
			goto retry;
		}
		VMODIFY(vp);
	}
	return -error;
}

STATIC int
linvfs_bmap(
	struct address_space	*mapping,
	long			block)
{
	struct inode		*inode = (struct inode *)mapping->host;
	vnode_t			*vp = LINVFS_GET_VP(inode);
	int			error;

	/* block	     - Linux disk blocks    512b */
	/* bmap input offset - bytes		      1b */
	/* bmap output bn    - XFS BBs		    512b */
	/* bmap output delta - bytes		      1b */

	vn_trace_entry(vp, "linvfs_bmap", (inst_t *)__return_address);

	VOP_RWLOCK(vp, VRWLOCK_READ);
	VOP_FLUSH_PAGES(vp, (xfs_off_t)0, -1, 0, FI_REMAPF, error);
	VOP_RWUNLOCK(vp, VRWLOCK_READ);
	return generic_block_bmap(mapping, block, linvfs_get_block_direct);
}

STATIC int
linvfs_read_full_page(
	struct file		*unused,
	struct page		*page)
{
	return block_read_full_page(page, linvfs_get_block);
}

STATIC int
count_page_state(
	struct page		*page,
	int			*nr_delalloc,
	int			*nr_unmapped)
{
	*nr_delalloc = *nr_unmapped = 0;

	if (page_has_buffers(page)) {
		struct buffer_head	*bh, *head;

		bh = head = page_buffers(page);
		do {
			if (buffer_uptodate(bh) && !buffer_mapped(bh)) {
				(*nr_unmapped)++;
				continue;
			}
			if (!buffer_delay(bh))
				continue;
			(*nr_delalloc)++;
		} while ((bh = bh->b_this_page) != head);
		return 1;
	}

	return 0;
}

STATIC int
linvfs_write_full_page(
	struct page		*page)
{
	int			flagset = 0;
	int			error;
	int			need_trans;
	int			nr_delalloc, nr_unmapped;

	if (count_page_state(page, &nr_delalloc, &nr_unmapped)) {
		need_trans = nr_delalloc + nr_unmapped;
	} else {
		need_trans = 1;
	}

	if ((current->flags & (PF_FSTRANS|PF_NOIO)) && need_trans)
		goto out_fail;

	if (need_trans) {
		current->flags |= PF_NOIO;
		flagset = 1;
	}

	error = write_full_page(page, nr_delalloc);

	if (flagset)
		current->flags &= ~PF_NOIO;
	return error;

out_fail:
	SetPageDirty(page);
	unlock_page(page);
	return 0;
}

STATIC int
linvfs_prepare_write(
	struct file		*file,
	struct page		*page,
	unsigned int		from,
	unsigned int		to)
{
	if (file && (file->f_flags & O_SYNC)) {
		return block_prepare_write(page, from, to,
						linvfs_get_block_sync);
	} else {
		return block_prepare_write(page, from, to,
						linvfs_get_block);
	}
}

/*
 * Initiate I/O on a kiobuf of user memory
 */
STATIC int
linvfs_direct_IO(
	int			rw,
	struct inode		*inode,
	struct kiobuf		*iobuf,
	unsigned long		blocknr,
	int			blocksize)
{
	struct page		**maplist;
	size_t			page_offset;
	page_buf_t		*pb;
	page_buf_bmap_t		map;
	int			error = 0;
	int			pb_flags, map_flags, pg_index = 0;
	size_t			length, total;
	loff_t			offset;
	size_t			map_size, size;

	total = length = iobuf->length;
	offset = blocknr;
	offset <<= inode->i_blkbits;

	maplist = iobuf->maplist;
	page_offset = iobuf->offset;

	map_flags = (rw ? PBF_WRITE : PBF_READ) | PBF_DIRECT;
	pb_flags = (rw ? PBF_WRITE : PBF_READ) | PBF_FORCEIO | _PBF_LOCKABLE;
	while (length) {
		error = linvfs_pb_bmap(inode, offset, length, &map, map_flags);
		if (error)
			break;

		map_size = map.pbm_bsize - map.pbm_delta;
		size = min(map_size, length);
		if (map.pbm_flags & PBMF_HOLE) {
			size_t	zero_len = size;

			if (rw == WRITE)
				break;

			/* Need to zero it all */
			while (zero_len) {
				struct page	*page;
				size_t		pg_len;

				pg_len = min((size_t)
						(PAGE_CACHE_SIZE - page_offset),
						zero_len);

				page = maplist[pg_index];

				memset(kmap(page) + page_offset, 0, pg_len);
				flush_dcache_page(page);
				kunmap(page);

				zero_len -= pg_len;
				if ((pg_len + page_offset) == PAGE_CACHE_SIZE) {
					pg_index++;
					page_offset = 0;
				} else {
					page_offset = (page_offset + pg_len) &
							~PAGE_CACHE_MASK;
				}
			}
		} else {
			int	pg_count;

			pg_count = (size + page_offset + PAGE_CACHE_SIZE - 1)
					>> PAGE_CACHE_SHIFT;
			if ((pb = pagebuf_lookup(map.pbm_target, inode, offset,
						size, pb_flags)) == NULL) {
				error = -ENOMEM;
				break;
			}
			/* Need to hook up pagebuf to kiobuf pages */
			pb->pb_pages = &maplist[pg_index];
			pb->pb_offset = page_offset;
			pb->pb_page_count = pg_count;

			pb->pb_bn = map.pbm_bn + (map.pbm_delta >> 9);
			error = pagebuf_iostart(pb, pb_flags);
			pb->pb_flags &= ~_PBF_LOCKABLE;
			pagebuf_rele(pb);
			if (error != 0) {
				if (error > 0)
					error = -error;
				break;
			}

			page_offset = (page_offset + size) & ~PAGE_CACHE_MASK;
			if (page_offset)
				pg_count--;
			pg_index += pg_count;
		}

		offset += size;
		length -= size;
	}

	return (error ? error : (int)(total - length));
}

/*
 * This gets a page into cleanable state - page locked on entry
 * kept locked on exit. If the page is marked dirty we should
 * not come this way.
 */
STATIC int
linvfs_release_page(
	struct page		*page,
	int			gfp_mask)
{
	int			need_trans;
	int			nr_delalloc, nr_unmapped;

	if (count_page_state(page, &nr_delalloc, &nr_unmapped)) {
		need_trans = nr_delalloc;
	} else {
		need_trans = 0;
	}

	if (need_trans == 0) {
		return 1;
	}

	if (gfp_mask & __GFP_FS) {
		return release_page(page);
	}
	return 0;
}


struct address_space_operations linvfs_aops = {
	.readpage		= linvfs_read_full_page,
	.writepage		= linvfs_write_full_page,
	.sync_page		= block_sync_page,
	.releasepage		= linvfs_release_page,
	.prepare_write		= linvfs_prepare_write,
	.commit_write		= generic_commit_write,
	.bmap			= linvfs_bmap,
	.direct_IO		= linvfs_direct_IO,
};
