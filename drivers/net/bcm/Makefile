
#
# Makefile for linux/drivers/net/bcm
#

O_TARGET := bcm5700.o
obj-y    := b57um.o b57proc.o tigon3.o autoneg.o 5701rls.o
obj-m    := $(O_TARGET)

EXTRA_CFLAGS = -DDBG=0 -DT3_JUMBO_RCV_RCB_ENTRY_COUNT=256 -DNICE_SUPPORT -DPCIX_TARGET_WORKAROUND=1 -DINCLUDE_TBI_SUPPORT -DINCLUDE_5701_AX_FIX=1

include $(TOPDIR)/Rules.make

