/*-*- linux-c -*-
 *  linux/drivers/video/i810_iface.c -- Hardware Interface
 *
 *      Copyright (C) 2001 Antonino Daplas
 *      All Rights Reserved      
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/types.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,4,14)
#include <linux/malloc.h>
#else
#include <linux/slab.h>
#endif

#include "i810_regs.h"
#include "i810_common.h"
#include "i810_iface.h"

/*
 * Resource Management
 */

/**
 * i810fb_find_free_block - finds a block of unbound gart memory
 * @pgsize: size in pages (4096)
 *
 * DESCRIPTION:
 * This function finds a free block of gart memory as determined
 * by gtt_map
 *
 * RETURNS:
 * start page offset of block
 */
static int i810fb_find_free_block(u32 pgsize)
{
	u32 offset, cur_size = 0;

	offset = i810_iface->sarea_offset + (SAREA_SIZE >> 12);
	while (cur_size < pgsize && offset < (i810_iface->aper_size >> 12)) {
		offset++;
		if (!test_bit(offset, i810_iface->gtt_map)) 
			++cur_size;
		else if (cur_size < pgsize) 
			cur_size = 0;
	} 
	return (cur_size < pgsize) ? -1 : (int) offset;
}

static pid_t i810fb_find_pid_by_key(u32 key)
{
	struct list_head *list;
	struct i810fb_user_struct *usr_list;
	
	list_for_each(list, &i810_iface->usr_list_head) {
		usr_list = (struct i810fb_user_struct *) list;
		if (usr_list->key == key) 
			return usr_list->pid;

	}
	return 0;
}

static void i810fb_delete_user_list(u32 key, pid_t pid)
{
	struct list_head *list1;
	struct i810fb_user_struct *usr_list;
	
	spin_lock(&i810_iface->agp_lock);
	i810fb_list_for_each_safe(list1, &i810_iface->usr_list_head) {
		usr_list = (struct i810fb_user_struct *) list1;
		if (usr_list->pid == pid && usr_list->key == key) {
			usr_list = (struct i810fb_user_struct *) list1;
			list_del(list1);
			vfree(usr_list);
		}
	}
	spin_unlock(&i810_iface->agp_lock);
}

/**
 * i810fb_allocate_agpmemory - allocates and binds agp memory
 * @agp_mem: pointer to agp_mem_user
 *
 * DESCRIPTION:
 * Allocates a requested agp memory type and size, then writes the surface
 * key and page offset to @agp_mem, if successful.  This routine will
 * check that current->pid matches the the saved pid.
 */
int i810fb_allocate_agpmemory(agp_mem_user *agp_mem)
{
	pid_t pid;
	agp_mem_struct *new;

	if (!test_bit(agp_mem->user_key, i810_iface->user_key_list))
		return -EACCES;
	pid = i810fb_find_pid_by_key(agp_mem->user_key);
	if (pid != current->pid)
		return -EACCES;

	switch (agp_mem->type) {
	case AGP_DMA:
	case AGP_DMA_IRQ:
		if (agp_mem->pgsize > MAX_DMA_SIZE >> 12)
			return -EINVAL;
		if (!agp_mem->pgsize)
			agp_mem->pgsize = 4;
		break;
	case AGP_SURFACE:
		if (!agp_mem->pgsize)
			return -EINVAL;
		break;
	case AGP_SAREA:
		agp_mem->pgsize = SAREA_SIZE >> 12;
		agp_mem->surface_key = i810_iface->i810_sarea_memory->key;
		agp_mem->offset = ((i810_iface->fb_size + MMIO_SIZE + i810_iface->aper_size) >> 12) +
			agp_mem->user_key;
		return 0;
	default:
		return -EINVAL;
	}
	if (NULL == (new = vmalloc(sizeof(agp_mem_struct))))
		return -ENOMEM;
	memset((void *) new, 0, sizeof(new));
	agp_mem->offset = i810fb_find_free_block(agp_mem->pgsize);
	if (agp_mem->offset == -1)
		return -ENOMEM;
	new->surface = agp_allocate_memory(agp_mem->pgsize, AGP_NORMAL_MEMORY);
	if (new->surface == NULL) {
		vfree(new);
		return -ENOMEM;
	}
	if (agp_bind_memory(new->surface, agp_mem->offset)) {
		agp_free_memory(new->surface);
		vfree(new);
		return -EBUSY;
	}
	memset((void *) i810_iface->fb_base_virt + agp_mem->offset, 0, 4096);
	i810fb_set_gttmap(new->surface);
	new->surface_type = agp_mem->type;
	new->user_key = agp_mem->user_key;
	new->task = current;
	new->pid = pid;
	agp_mem->surface_key = new->surface->key;
	list_add(&new->agp_list, &i810_iface->agp_list_head);
	return 0;
}

/**
 * i810fb_free_agpmemory - allocates and binds agp memory
 * @agp_mem: pointer to agp_mem_user
 *
 * DESCRIPTION:
 * Free a previously requested agp memory. 
 */
int i810fb_free_agpmemory(agp_mem_user *agp_mem)
{
	struct list_head *list1;
	agp_mem_struct *agp_list;
	int ret = -EINVAL;

	i810fb_sync();
	spin_lock(&i810_iface->agp_lock);
	i810fb_list_for_each_safe(list1, &i810_iface->agp_list_head) {
		agp_list = (agp_mem_struct *) list1;
		if (agp_list->surface->key == agp_mem->surface_key && 
		    agp_list->pid == current->pid) {
			i810fb_clear_gttmap(agp_list->surface);
			agp_unbind_memory(agp_list->surface);
			agp_free_memory(agp_list->surface);
			list_del(list1);
			vfree(agp_list);
			ret = 0;
		}
	}
	spin_unlock(&i810_iface->agp_lock);
	return ret;
}

/**
 * i810fb_remove_stale_memory - releases unfreed resources
 *
 * DESCRIPTION:
 * Release resources which were not freed by clients.  It checks
 * if the pid of the client is still active in memory.
 */
void i810fb_remove_stale_memory(void) 
{
	struct list_head *list1;
	agp_mem_struct *agp_list;
	struct i810fb_user_struct *usr;

	i810fb_sync();
	spin_lock(&i810_iface->agp_lock);
	i810fb_list_for_each_safe(list1, &i810_iface->agp_list_head) {
		agp_list = (agp_mem_struct *) list1;
		if (agp_list->task && agp_list->task->pid != agp_list->pid) {
			printk("i810fb: releasing stale AGP resource #%d of process %d\n", 
			       agp_list->surface->key, agp_list->pid);
			i810fb_clear_gttmap(agp_list->surface);
			agp_unbind_memory(agp_list->surface);
			agp_free_memory(agp_list->surface);
			list_del(list1);
			vfree(agp_list);
		}
	}
	i810fb_list_for_each_safe(list1, &i810_iface->usr_list_head) {
		usr = (struct i810fb_user_struct *) list1;
		if (usr->task && usr->task->pid != usr->pid) {
			printk("i810fb: releasing stale record #%d of process %d\n", 
			       usr->key, usr->pid);
			list_del(list1);
			clear_bit(usr->key, i810_iface->user_key_list);
			i810fb_delete_user_list(usr->key, usr->pid);
			vfree(usr);
		}
	}
	spin_unlock(&i810_iface->agp_lock);
}

/**
 * i810fb_acquire_fb - acquires the framebuffer
 *
 * DESCRIPTION:
 * Acquires the framebuffer device.  If successful, returns a 
 * user key which should be passed to the fb  driver each time 
 * a service is requested. A single process can acquire the fb
 * multiple times, receiving a different key each time.
 */
int i810fb_acquire_fb(void)
{
	struct i810fb_user_struct *new;
	int key;

	key = find_first_zero_bit(i810_iface->user_key_list, MAX_KEY);
	if (key >= MAX_KEY)
		return -1;
	if (i810fb_bind_all())	return -1;
	set_bit(key, i810_iface->user_key_list);
	new = vmalloc(sizeof(struct i810fb_user_struct));
	if (new == NULL)
		return -1;
	new->key = key;
	new->pid = current->pid;
	new->task = current;
	list_add(&new->list, &i810_iface->usr_list_head);
	return key;
}


/**
 * i810fb_check_mmap - check if area to be mmaped is valid
 * @offset: offset to start of aperture space
 *
 * DESCRIPTION:
 * Checks if @offset matches any of the agp memory in the list.
 */
int i810fb_check_agp_mmap(u32 offset)
{
	struct list_head *list;
	agp_mem_struct *agp_list;
	
	list_for_each(list, &i810_iface->agp_list_head) {
		agp_list = (agp_mem_struct *) list;
		if (agp_list->pid == current->pid &&
		    offset >= agp_list->surface->pg_start  && 
		    offset < agp_list->surface->pg_start + 
		    agp_list->surface->page_count) {
			return ((agp_list->surface->pg_start + 
				 agp_list->surface->page_count) - 
				offset) << 12;
		}
	}
	return 0;

}

/**
 * i810fb_check_sarea - check if shared area can be mapped 
 * @offset: offset to map (equivalent to user key)
  * 
 * DESCRIPTION:
 * This function checks if the sarea can be mapped to user space.
 * Only for root users.
 */
int i810fb_check_sarea(u32 offset)
{
	pid_t pid;

	if (current->uid) 
		return 0;
	
	pid = i810fb_find_pid_by_key(offset);
	if (pid != current->pid)
		return 0;

	if (test_bit(offset, i810_iface->user_key_list)) {
		set_bit(offset, i810_iface->has_sarea_list);
		return SAREA_SIZE;
	}

	return 0;
}

/**
 * i810fb_release_fb - release the framebuffer device
 * @command: pointer to i810_command
 *
 * DESCRIPTION:
 * Release the framebuffer device.  All allocated resources 
 * will be released, and the user key will be removed from the list.  
 */
static int i810fb_release_fb(i810_command *command)
{
	pid_t pid;

	pid = i810fb_find_pid_by_key(command->user_key);
	if (pid != current->pid)
		return -1;

	i810fb_release_all(command->user_key);
	clear_bit(command->user_key, i810_iface->user_key_list);
	clear_bit(command->user_key, i810_iface->has_sarea_list);
	i810fb_delete_user_list(command->user_key, pid);
	i810_iface->cur_dma_buf_virt = NULL;
	i810_iface->cur_dma_buf_phys = NULL;
	i810_iface->sarea->cur_surface_key = MAX_KEY;
	i810_iface->sarea->cur_user_key = MAX_KEY;
	i810_iface->sarea->is_valid = 0;
	return 0;
}
			
/**
 * i810fb_update_dma - updates the current user DMA buffer pointer
 * @command: pointer to i810_command structure
 *
 */
static int i810fb_update_dma(i810_command *command)
{
	struct list_head *list;
	agp_mem_struct *agp_list;

	i810fb_sync();
	list_for_each(list, &i810_iface->agp_list_head) {
		agp_list = (agp_mem_struct *) list;
		if (agp_list->surface->key == command->surface_key && 
		    agp_list->pid == current->pid &&
		    (agp_list->surface_type == AGP_DMA ||
		     agp_list->surface_type == AGP_DMA_IRQ)) {
			i810_iface->cur_dma_buf_virt = 
				(u32 *) (i810_iface->fb_base_virt + 
					 (agp_list->surface->pg_start << 12));
			i810_iface->cur_dma_buf_phys = 
				(u32 *) (i810_iface->fb_base_phys + 
					 (agp_list->surface->pg_start << 12));
			i810_iface->sarea->cur_surface_key = command->surface_key;
			i810_iface->sarea->cur_user_key = command->user_key;
			i810_iface->sarea->is_valid = 1;
			i810_iface->cur_dma_size = 
				agp_list->surface->page_count << 12;
			return 0;
		}
	}
	return -EINVAL;
}

/**
 * i810fb_check_destination - check if blitting destination will overwrite critical area
 * @opcode: blit opcode
 * @pointer: current offset to buffer
 *
 * DESCRIPTION:
 * The hardware has no means of determining if a blit operation can overwrite 
 * the ringbuffers.  This is a potential security hole.  For nontrusted clients,
 * this routine will check if the operation will overlap with the framebuffer's 
 * critical areas (ringbuffer, overlay, cursor, fontcache and sarea)
 */
static int i810fb_check_destination(u32 opcode, u32 *pointer)
{
	u32 start = 0, end = 0, bpp;
	u32 crit_start, crit_end;

	crit_start = i810_iface->fb_base_phys + i810_iface->fb_size;
	crit_end = i810_iface->sarea_start_phys + SAREA_SIZE;
	
	crit_start &= 0xFFFFFF;
	crit_end &= 0xFFFFFF;
	bpp = ((i810_readl(BLTCNTL) >> 4) & 3) + 1;

	switch (opcode) {
	case 0:
	case 0x10:
		break;
	case 0x20:
		start  = pointer[1] + ((pointer[0] >> 6) & 0xFFF) * bpp;
		end = start + (1 * bpp);
		break;
	case 0x21:
		start = pointer[2] + (pointer[1] & 0xFFFF) * bpp;
		end = pointer[2] + (pointer[1] >> 16) * bpp;
		break;
	case 0x22:
	case 0x30:
		start = pointer[2] + (pointer[1] & 0xFFFF) * bpp;
		end = pointer[3] + (pointer[1] >> 16) * bpp;
		break;
	default:
		start = pointer[3];
		end = pointer[3] + ((pointer[2] >> 16) * (pointer[1] & 0xFFFF)) +
			(pointer[2] & 0xFFFF);
	}
	start &= 0xFFFFFF;
	end &= 0xFFFFFF;
	return ((crit_start >= start && crit_start <= end) ||
		(crit_end >= start && crit_end <= end)); 
}

/**
 * i810fb_parse_parser - verifies parser type instructions (opcode 00)
 * @pointer:  the offset to the current user DMA buffer
 * @dsize: the number of dwords from the offset to the end of the 
 * instruction packets
 *
 * DESCRIPTION:
 * Process parser-type instructions.  Verification is done by 
 * checking the size of the stream against the opcode type, and
 * disallowing potentially dangerous opcodes.
 */

static int i810fb_parse_parser(u32 *pointer)
{
	u32 cur_header, opcode;
	int i;
	
	cur_header = *pointer;
	opcode = (cur_header >> 23) & 0x3f;

	switch(opcode) {
	/* disallowed opcodes */
	case 0x08: /* ARB_ON_OFF */
	case 0x14: /* LOAD_FRONTBUFFER */
	case 0x15: /* LOAD_DESTINATIONBUFFER */
	case 0x16: /* LOAD_ZBUFFER */
	case 0x30: /* BATCH_BUFFER */
		printk("i810fb: disallowed PARSER opcode\n"
		       "        Offending Header: %x\n", cur_header);
		return -1;
	}
	switch (opcode) {
	case 0 ... 8:
		i = 1;
		break;
	case 9 ... 0x1F:
		i = (cur_header & 0x3F) ? -1 : 2;
		break;
	case 0x20 ... 0x2F:
		i = ((cur_header & 0x3F) != 1) ? -1 : 3;
		break;
	default:
		printk("i810fb: illegal PARSER opcode\n"
		       "        Offending Header: %x\n", cur_header);
		return -1;
	}

	if (i == -1) {
		printk("i810fb: illegal PARSER instruction length\n"
		       "        Offending Header: %x\n", cur_header); 
	}
	return (i);
}

/**
 * i810fb_parse_blitter - verifies blitter type instructions (opcode 02)
 * @pointer:  the offset to the current user DMA buffer
 * @dsize: the number of dwords from the offset to the end of 
 * the instruction packets
 *
 * DESCRIPTION:
 * Process blit-type instructions.  Verification is done by 
 * checking the size of the stream aginst the opcode type, and
 * verifying that the blit operation will not overwrite critical
 * regions of the framebuffer memory.
 */
static int i810fb_parse_blitter(u32 *pointer)
{
	u32 cur_header, code, opcode = -1 , opsize = -1, size;
	int i = 0;

	cur_header = *pointer;
	code = (cur_header >> 22) & 0x7f;
	size = cur_header & 0x1F;

	while (i < ARRAY_SIZE(blit_header)) {
		if (blit_header[i].opcode == code) {
			opsize = blit_header[i].size;
			opcode = code;
			break;
		}
		i++;
	}

	if (opcode == -1) {
		printk("i810fb: invalid BLIT opcode\n"
		       "        Offending Header: %x\n", cur_header);
		return -1;
	}
	if (opsize > 0xFFFF0000 && size < (opsize & 0xFFFF)) {
		printk("i810fb: illegal BLIT instruction length\n"
		       "        Offending Header: %x\n", cur_header);
		return -1;
	}
	else if (size != opsize) {
		printk("i810fb: illegal BLIT instruction length\n"
		       "        Offending Header: %x\n", cur_header);
		return -1;
	}
	if (i810fb_check_destination(opcode, pointer)) {
		printk("i810fb: BLIT instruction will overwrite critical graphics memory\n"
		       "        Offending Header: %x\n", cur_header);
		return -1;
	}

	return  (size + 2);
}

#if 0 /* too complicated to verify, so disallow 3D instructions for now */
/**
 * i810fb_parse_render - verifies render type instructions (opcode 03)
 * @pointer:  the offset to the current user DMA buffer
 * @dsize: the number of dwords from the offset to the end of 
 * the instruction packets
 *
 * DESCRIPTION:
 * Process render-type instructions. It verifies the size of the packets based
 * on the opcode. All invalid opcodes will result in an error.
 */
static int i810fb_parse_render(u32 *pointer)
{
	u32 cur_header, opcode;
	int i;

	cur_header = *pointer;
	opcode = cur_header & (0x1F << 24);
	
	switch(opcode) {
	case 0 ... (0x18 << 24):
	case (0x1C << 24):
		i = 1;
		break;
	case (0x1D << 24) ... (0x1E << 24):
		i = (cur_header & 0xFF) + 2;
		break;
	case (0x1F << 24):
		i = (cur_header & 0x3FF) + 2;
		break;
	default:
		return -1; 
	}
	return (i);
}
#endif

/**
 * process_buffer_with_verify - process command buffer contents
 * @v_pointer: virtual pointer to start of instruction;
 * @p: physical pointer to start of instruction
 * @dsize: length of instruction sequence in dwords
 * @command: pointer to i810_command
 *
 * DESCRIPTION:
 * Processes command buffer instructions prior to execution.  This 
 * includes verification of each instruction for validity.
 * This is reserved for clients which are not trusted.
 */
static inline u32 process_buffer_with_verify(u32 v_pointer, u32 p, u32 dsize,
					    i810_command *command)
{
	u32  i = 0, opcode, size;

	size = dsize;
	if (dsize & 1) {
		*((u32 *) (v_pointer + (dsize << 2))) = 0;
		dsize++;
	}

	do {
		opcode =  *((u32 *) v_pointer) & (0x7 << 29);
		switch (opcode) {
		case PARSER:
			i = i810fb_parse_parser((u32 *) v_pointer); 
			break;
		case BLIT:
			i = i810fb_parse_blitter((u32 *) v_pointer); 
			break;
		default:
			printk("i810fb: invalid or 3D instruction type\n"
				"       Offending Header: %x\n",  *(u32 *) v_pointer);
			i = -1;
		}
		if (i == -1)
			break;
		if (i > size) {
			printk("i810fb: invalid instruction stream length\n");
			break;
		}
		v_pointer += i << 2;
		size -= i;
	} while (size);
	if (!size) {
		emit_instruction(dsize, p, 1); 
		i810fb_sync();
	}

	return dsize;
}

/**
 * process_buffer_no_verify - process command buffer contents
 * @v_pointer: virtual pointer to start of instruction;
 * @p: physical pointer to start of instruction
 * @dsize: length of instruction sequence in dwords
 * @command: pointer to i810_command
 *
 * DESCRIPTION:
 * Processes command buffer instructions prior to execution.  If
 * client has shared area, will update current head and tail.
 * This is reserved for trusted clients.
 */
static inline u32 process_buffer_no_verify(u32 v_pointer, u32 p, u32 dsize,
					    i810_command *command)
{
	u32 tail_pointer, tail;
		
	if (!test_bit(command->user_key, i810_iface->has_sarea_list)) {
		if (dsize & 1) {
			*((u32 *) (v_pointer + (dsize << 2))) = 0;
			dsize++;
		}
		emit_instruction(dsize, p, 0);
		i810fb_sync();
		return 0;
	}

	if (!(dsize & 1)) {
		*((u32 *) (v_pointer + (dsize << 2))) = 0;
		dsize++;
	}
	dsize += 3;
	
	tail = (command->dma_cmd_start + dsize) << 2;
	i810_iface->sarea->tail = tail;
	tail_pointer = v_pointer + ((dsize - 3) << 2);
	*(u32 *) tail_pointer        = PARSER | STORE_DWORD_IDX | 1;
	*(u32 *) (tail_pointer + 4)  = 7 << 2;
	*(u32 *) (tail_pointer + 8)  = tail;

	emit_instruction(dsize, p, 0);

	return 0;
}

static inline u32 process_overlay(u32 v_pointer, u32 dsize,
				  i810_command *command)
{
	u32 tail, buffer[30];
	int sarea = 0;

	if (dsize != 30)
		return -EINVAL;
	if (test_bit(command->user_key, i810_iface->has_sarea_list)) 
		sarea = 1;
	
	if (sarea) {
		tail = (command->dma_cmd_start + dsize) << 2;
		i810_iface->sarea->tail = tail;
	}

	memcpy_fromio(buffer, v_pointer, 120);
	memcpy_toio(i810_iface->ovl_start_virtual, buffer, 120);

	i810fb_load_overlay(i810_iface->ovl_start_phys);

	if (sarea)
		i810_iface->sarea->head = i810_iface->sarea->tail;
	
	return 0;
}	

/**
 * i810fb_emit_dma - processes DMA instructions from client
 * @command: pointer to i810_command
 *
 * DESCRIPTION:
 * Clients cannot directly use the ringbuffer.  To instruct the hardware 
 * engine, the client writes all instruction packets to the current user 
 * DMA buffer, and tells the fb driver to process those instructions.  
 * The fb driver on the other hand, will verify if the packets are valid 
 * _if_ the instructions come from a nontrusted source (not root).  
 * Once verification is finished, the instruction sequence will be processed 
 * via batch buffers. Routine will exit immediately if any invalid instruction is 
 * encountered. Instruction sequences can be chained, resulting in faster 
 * performance.  
 *
 * If the source is trusted, the verfication stage is skipped, resulting 
 * in greater performance at the expense of increasing the chances of 
 * locking the machine.  If the client is using shared memory, the start (head)
 * and end (tail) of the currently processed instruction sequence will be 
 * written to the shared area.
 */
static int i810fb_emit_dma(i810_command *command)
{

	u32 cur_pointer, phys_pointer, dsize, ret;
	
	if (i810_iface->lockup) return -EINVAL;
	if (i810_iface->sarea->cur_surface_key != command->surface_key) { 
		if (i810fb_update_dma(command))
			return -EACCES;
	}
	else if (i810_iface->sarea->cur_user_key != command->user_key)
		return -EINVAL;

	dsize = command->dma_cmd_dsize;
	if (dsize + command->dma_cmd_start > 
	    (i810_iface->cur_dma_size >> 2) - 3 || 
	    !dsize)
		return -EINVAL;
	phys_pointer = (u32) i810_iface->cur_dma_buf_phys + 
		(command->dma_cmd_start << 2);
	cur_pointer = (u32) i810_iface->cur_dma_buf_virt + 
		(command->dma_cmd_start << 2);

	switch(command->command) {
	case EMIT_DMA:
		if (!current->uid) 
			ret = process_buffer_no_verify(cur_pointer, phys_pointer, dsize,
						       command);
		else
			ret = process_buffer_with_verify(cur_pointer, phys_pointer, dsize,
							 command);
		break;
	case EMIT_OVERLAY:
		ret = process_overlay(cur_pointer, dsize, command);
		break;
	default:
		ret = 1;
	}
	return (ret) ? -EINVAL : 0;
}


int i810fb_process_command(i810_command *command)
{
	switch (command->command) {
	case EMIT_DMA:
	case EMIT_OVERLAY:
		return i810fb_emit_dma(command);
	case RELEASE_FB:
		return i810fb_release_fb(command);
	default:
		return -EINVAL;
	}
}

