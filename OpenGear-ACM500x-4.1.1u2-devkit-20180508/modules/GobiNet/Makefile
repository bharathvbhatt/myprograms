
# An external module Makefile
# david.mccullough@opengear.com

include $(ROOTDIR)/modules/.config

obj-$(CONFIG_MODULES_GOBINET) := GobiNet.o
GobiNet-objs :=GobiUSBNet.o QMIDevice.o QMI.o 
GobiNet-objs += usbnet_2_6_32.o
GobiNet-objs += usbnet_2_6_35.o
GobiNet-objs += usbnet_3_0_6.o
GobiNet-objs += usbnet_3_10_21.o

obj-m  := $(obj-y) $(obj-m)
M_OBJS := $(obj-y) $(obj-m)
obj-y  :=

# Calling the cellular interface "wwan*" instead of "eth*"
# made firewall rules easier to write.
EXTRA_CFLAGS += -DGOBINET_WWAN

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

