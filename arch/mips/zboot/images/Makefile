
include $(TOPDIR)/Rules.make

vmlinux.gz: $(TOPDIR)/vmlinux
	$(OBJCOPY) -S -O binary $(TOPDIR)/vmlinux vmlinux
	gzip -vf1 vmlinux

clean:
	rm -f vmlinux.*  zImage.* zImage.*

