include $(ROOTDIR)/modules/.config
EXTRA_CFLAGS += -I$(ROOTDIR)/$(LINUXDIR)/include/linux

obj-$(CONFIG_MODULES_KSZFEAT) += kszfeat.o

subdir-m := $(mydir-y) $(mydir-m)

obj-m  := $(obj-y) $(obj-m)
M_OBJS := $(obj-y) $(obj-m)
obj-y  :=

ifndef KBUILD_EXTMOD
ifdef TOPDIR
-include $(TOPDIR)/Rules.make
endif
endif

.PHONY: clean mrproper distclean
clean:
	-for i in $(mydir-y) $(mydir-m); do make -C $$i $@; done
	rm -f $(obj-m)
mrproper: clean
	        rm -f .depend
distclean: mrproper

