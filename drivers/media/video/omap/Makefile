
O_TARGET := v4l2-omap.o

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs :=

# Each configuration option enables a list of files.

obj-y := camera_ov6x30.o camera_sanyo.o sbus_old_i2c.o \
	  sbus_sccb.o v4l2.o

ifdef CONFIG_OMAP_INNOVATOR
obj-y += camif_innovator.o
else
obj-y += camif_evm.o
endif

obj-m := $(O_TARGET)

include $(TOPDIR)/Rules.make
