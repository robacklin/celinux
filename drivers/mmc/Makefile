#
# Makefile for the kernel mmc device drivers.
#

O_TARGET := mmc.o

export-objs := mmc_core.o

obj-$(CONFIG_MMC) += mmc_base.o
obj-$(CONFIG_OMAP_MMC) += omap_mmc.o
# Declare multi-part drivers.
list-multi := mmc_base.o
mmc_base-objs := mmc_protocol.o mmc_core.o mmc_media.o

include $(TOPDIR)/Rules.make

# Link rules for multi-part drivers.

mmc_base.o: $(mmc_base-objs)
	$(LD) -r -o $@ $(mmc_base-objs)
