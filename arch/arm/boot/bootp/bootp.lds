/*
 *  linux/arch/arm/boot/bootp/bootp.lds
 *
 *  Copyright (C) 2000-2002 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
  . = 0;
  _text = .;
  .text : {
   _stext = .;
   *(.start)
   /*
    * The align below is to facilitate installing and booting the bootpImage
    * from ROM. When installing this image in ROM, build your zImage as usual,
    * CONFIG_ZBOOT_ROM=y, and add an offset of 0x100 bytes to your usual
    * CONFIG_ZBOOT_ROM_TEXT start address to account for the bootpImage header.
    * Now zImage will be built and installed at the correct ROM address. So
    * no need to worry about Global Offset Table fixups due to running at
    * an address which differs from our link address. Now GOT fixups will
    * not be required. Which is a good thing when you're running from ROM.
    */
   . = ALIGN(256);
   kernel.o
   . = ALIGN(32);
   initrd_start = .;
   initrd.o
   initrd_len = . - initrd_start;
   . = ALIGN(32);
   _etext = .;
  }
  
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }
}
