#
# Makefile for the Intel PXA250/210 PCMCIA driver
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

O_TARGET			:= pxa_cs.o

obj-y				:= pxa.o
obj-$(CONFIG_ARCH_LUBBOCK)	+= lubbock.o
obj-$(CONFIG_ARCH_PXA_IDP)	+= pxa_idp.o
obj-$(CONFIG_ARCH_TRIZEPS2)	+= trizeps2.o
obj-$(CONFIG_ARCH_PXA_CERF)	+= ../sa1100_cerf.o

obj-m				:= $(O_TARGET)

include $(TOPDIR)/Rules.make
