
.EXPORT_ALL_VARIABLES:

VIRTUAL_TOPDIR := $(ROOTDIR)/modules

-include .config

#
# for now we only build modules outside the linux source tree
#

subdir-y    :=
subdir-m    := ocf mymodule ralink asix kszfeat GobiNet GobiSerial

MOD_SUB_DIRS := $(subdir-m)

#
# if we are being invoked from linux build
#
ifdef TOPDIR
-include $(TOPDIR)/Rules.make
endif

include ../config/Makefile.conf

.PHONY: oldconfig
oldconfig: conf
	$(SCRIPTSDIR)/conf -o Kconfig

.PHONY: config
config: conf
	$(SCRIPTSDIR)/conf Kconfig

.PHONY: menuconfig
menuconfig: mconf
	$(SCRIPTSDIR)/mconf Kconfig

.PHONY: clean mrproper distclean
clean mrproper distclean:
	for i in $(MOD_SUB_DIRS); do make -C $$i $@; done

%_only:
	$(MAKE) -C $(@:_only=)

%_clean:
	$(MAKE) -C $(@:_clean=) clean

