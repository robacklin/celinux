# Makefile for the Linux device tree

obj-y			:= core.o sys.o interface.o power.o bus.o \
			   driver.o class.o platform.o \
			   cpu.o firmware.o init.o map.o
# linux-pm: obj-$(CONFIG_NUMA)	+= node.o  memblk.o

# linux-pm: begin
export-objs	:= core.o power.o sys.o bus.o driver.o \
			class.o platform.o firmware.o

O_TARGET := base.o

include $(TOPDIR)/Rules.make
# linux-pm: end
