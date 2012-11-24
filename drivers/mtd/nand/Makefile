#
# linux/drivers/nand/Makefile
#
# $Id: Makefile.common,v 1.1 2003/05/21 15:00:04 dwmw2 Exp $

ifeq ($(PATCHLEVEL),4)
O_TARGET			:= nandlink.o
ifeq ($(CONFIG_MTD_NAND_SMC_SMDK2440),y)
export-objs                     := s3c2440nand.o nand_ecc.o nand_ids.o
else
export-objs			:= nand.o nand_ecc.o nand_ids.o
endif
endif

nandobjs-$(CONFIG_MTD_NAND_SMC_SMDK2440) += s3c2440nand.o

ifeq ($(CONFIG_MTD_NAND_SMC_SMDK2440),y)
obj-$(CONFIG_MTD_NAND)          += $(nandobjs-y) nand_ecc.o
else
obj-$(CONFIG_MTD_NAND)		+= nand.o nand_ecc.o
endif
obj-$(CONFIG_MTD_NAND_SPIA)	+= spia.o
obj-$(CONFIG_MTD_NAND_AUTCPU12)	+= autcpu12.o
obj-$(CONFIG_MTD_NAND_EDB7312)  += edb7312.o
obj-$(CONFIG_MTD_NAND_TX4925NDFMC)  += tx4925ndfmc.o
obj-$(CONFIG_MTD_NAND_TX4938NDFMC)  += tx4938ndfmc.o
obj-$(CONFIG_MTD_NAND_SMC_SMDK2440) += smdk2440.o 
obj-$(CONFIG_MTD_NAND_IDS)	+= nand_ids.o

-include $(TOPDIR)/Rules.make
