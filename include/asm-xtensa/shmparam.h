#ifndef __ASM_XTENSA_SHMPARAM_H
#define __ASM_XTENSA_SHMPARAM_H

/*
 * include/asm-xtensa/shmparam.h
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file "COPYING" in the main directory of
 * this archive for more details.
 */

#include <xtensa/config/core.h>
#include <asm/page.h>




/* 19feb2003 -- jn
 *
 * Xtensa can have variable size caches, and if
 * the size of single way is larger than the page size,
 * then we have to start worrying about cache aliasing
 * problems (didn't they fix this problem like back in 1978?)
 */
#if XTENSA_CACHE_ALIAS
  #define SHMLBA XCDCACHE_WAY_SIZE
#else
  #define SHMLBA PAGE_SIZE		 /* attach addr a multiple of this */
#endif


#endif /* __ASM_XTENSA_SHMPARAM_H */
