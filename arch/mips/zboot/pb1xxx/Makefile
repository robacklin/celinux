# arch/mips/zboot/pb1xxx/Makefile
# 
# Makefile for Alchemy Semiconductor Pb1[015]00 boards.
# All of the boot loader code was derived from the ppc
# boot code.
#
# Copyright 2001,2002 MontaVista Software Inc.
#
# Author: Mark A. Greer
#	  mgreer@mvista.com
# Ported and modified for mips support by 
# Pete Popov <ppopov@mvista.com>
#
# This program is free software; you can redistribute  it and/or modify it
# under  the terms of  the GNU General  Public License as published by the
# Free Software Foundation;  either version 2 of the  License, or (at your
# option) any later version.

.c.s:
	$(CC) $(CFLAGS) -S -o $*.s $<
.s.o:
	$(AS) -o $*.o $<
.c.o:
	$(CC) $(CFLAGS) -D__BOOTER__ -c -o $*.o $<
.S.s:
	$(CPP) $(AFLAGS) -o $*.o $<
.S.o:
	$(CC) $(AFLAGS) -c -o $*.o $<

#########################################################################
# START BOARD SPECIFIC VARIABLES
ifdef CONFIG_MIPS_PB1000
BNAME=pb1000
endif

ifdef CONFIG_MIPS_PB1100
BNAME=pb1100
endif

ifdef CONFIG_MIPS_PB1500
BNAME=pb1500
endif

ifdef CONFIG_MIPS_DB1000
BNAME=db1000
endif

ifdef CONFIG_MIPS_DB1100
BNAME=db1100
endif

ifdef CONFIG_MIPS_DB1500
BNAME=db1500
endif

ifdef CONFIG_MIPS_BOSPORUS
BNAME=bosporus
endif

# These two variables control where the zImage is stored
# in flash and loaded in memory.  It only controls how the srec
# file is generated, the code is the same.
RAM_RUN_ADDR = 0x81000000
FLASH_LOAD_ADDR = 0xBFD00000

# These two variables specify the free ram region
# that can be used for temporary malloc area
AVAIL_RAM_START=0x80400000
AVAIL_RAM_END=0x80800000

# This one must match the LOADADDR in arch/mips/Makefile!
LOADADDR=0x80100000
# END BOARD SPECIFIC VARIABLES
#########################################################################

OBJECTS := head.o ../common/misc-common.o ../common/misc-simple.o \
	../common/au1k_uart.o ../common/string.o ../common/ctype.o
LIBS := ../lib/zlib.a

ENTRY := ../utils/entry
OFFSET := ../utils/offset
SIZE := ../utils/size

LD_ARGS := -T ../ld.script -Ttext $(RAM_RUN_ADDR) -Bstatic
OBJCOPY_ARGS = -O elf32-tradlittlemips

all: zImage

clean:
	rm -rf *.o vmlinux* zvmlinux.* ../images/*.srec

head.o: head.S $(TOPDIR)/vmlinux
	$(CC) $(AFLAGS) \
	-DKERNEL_ENTRY=$(shell sh $(ENTRY) $(NM) $(TOPDIR)/vmlinux ) \
	-c -o $*.o $<

../common/misc-simple.o:
	$(CC) $(CFLAGS) -DINITRD_OFFSET=0 -DINITRD_SIZE=0 -DZIMAGE_OFFSET=0 \
		-DAVAIL_RAM_START=$(AVAIL_RAM_START) \
		-DAVAIL_RAM_END=$(AVAIL_RAM_END) \
		-DLOADADDR=$(LOADADDR) \
		-DZIMAGE_SIZE=0 -c -o $@ $*.c

zvmlinux: $(OBJECTS) $(LIBS) ../ld.script ../images/vmlinux.gz ../common/dummy.o
	$(OBJCOPY) \
		--add-section=.image=../images/vmlinux.gz \
		--set-section-flags=.image=contents,alloc,load,readonly,data \
		../common/dummy.o image.o
	$(LD) $(LD_ARGS) -o $@ $(OBJECTS) image.o $(LIBS)
	$(OBJCOPY) $(OBJCOPY_ARGS) $@ $@ -R .comment -R .stab -R .stabstr \
		-R .initrd -R .sysmap

# Here we manipulate the image in order to get it the necessary
# srecord file we need.
zImage: zvmlinux
	mv zvmlinux ../images/zImage.$(BNAME)
	$(OBJCOPY) -O srec ../images/zImage.$(BNAME) ../images/$(BNAME).srec

zImage.flash: zImage
	$(OBJCOPY) -O srec --adjust-vma 0x3ed00000 \
		../images/zImage.$(BNAME) ../images/$(BNAME).flash.srec

include $(TOPDIR)/Rules.make
