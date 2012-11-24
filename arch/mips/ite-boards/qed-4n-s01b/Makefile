# 
# Copyright (C) 2000, 2003 MontaVista Software, Inc.
# Author: MontaVista Software, Inc.
#     source@mvista.com
#
# Makefile for the ITE 8172 (qed-4n-s01b) board, board
# specific files.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

USE_STANDARD_AS_RULE := true

O_TARGET := ite.o

export-objs   := qed_pm.o qed_vst.o

obj-y := init.o

obj-$(CONFIG_PCI) += pci_fixup.o
obj-$(CONFIG_PM) += qed_pm.o
obj-$(CONFIG_DPM) += qed_dpm.o it8172_dpm.o
obj-$(CONFIG_VST) += qed_vst.o
obj-$(CONFIG_BLK_DEV_INITRD) += le_ramdisk.o

include $(TOPDIR)/Rules.make
