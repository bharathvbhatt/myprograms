
# An external module Makefile
# david.mccullough@opengear.com

include $(ROOTDIR)/modules/.config

obj-$(CONFIG_MODULES_GOBISERIAL) += GobiSerial.o

obj-m  := $(obj-y) $(obj-m)
M_OBJS := $(obj-y) $(obj-m)
obj-y  :=

#
# for sub directory support
#
# mydir-$(CONFIG_MODULES_MY_SUBDIRMODULE)+= my_subdir
# subdir-m := $(mydir-y) $(mydir-m)
#

ifndef KBUILD_EXTMOD
ifdef TOPDIR
-include $(TOPDIR)/Rules.make
endif
endif

.PHONY: clean mrproper distclean
clean:
	rm -f $(obj-m)
mrproper: clean
	rm -f .depend
distclean: mrproper

