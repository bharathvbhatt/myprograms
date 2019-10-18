################################################################################
#
# Makefile for CDK
#

include vendors/OpenGear/ACM500x/config.build

ROOTDIR    = $(shell pwd)
include vendors/config/common/config.arch

BUILD_START_STRING ?= $(shell date -R)

CONFIG_SHELL := $(shell if [ -x "$$BASH" ]; then echo $$BASH; \
	  else if [ -x /bin/bash ]; then echo /bin/bash; \
	  else echo sh; fi ; fi)

ARCH_CONFIG = $(ROOTDIR)/vendors/OpenGear/ACM500x/config.arch
include $(ARCH_CONFIG)

DIRS    = $(VENDOR_TOPDIRS) user

VERSIONPKG ?= 4.1.1u2
VERSIONINFO ?= Version 4.1.1u2
VERSIONSUFFIX ?= 594cb83d (CDK Build)
VERSIONSTR = OpenGear/ACM500x $(VERSIONINFO) $(VERSIONSUFFIX)

export VENDOR PRODUCT ROOTDIR LINUXDIR HOSTCC CONFIG_SHELL
export CONFIG_CONFIG LINUX_CONFIG MODULES_CONFIG ROMFSDIR SCRIPTSDIR
export VERSIONPKG VERSIONSTR ROMFSINST PATH IMAGEDIR RELDIR RELFILES TFTPDIR
export BUILD_START_STRING LIBTOOL ARCH_CONFIG

all: ucfront cksum subdirs romfs image

.PHONY: ucfront
ucfront: tools/ucfront/*.c
	$(MAKE) -C tools/ucfront
	ln -sf $(ROOTDIR)/tools/ucfront/ucfront tools/ucfront-gcc
	ln -sf $(ROOTDIR)/tools/ucfront/ucfront tools/ucfront-g++
	ln -sf $(ROOTDIR)/tools/ucfront/ucfront-ld tools/ucfront-ld
	ln -sf $(ROOTDIR)/tools/ucfront/jlibtool tools/jlibtool

.PHONY: cksum
cksum: tools/cksum
tools/cksum: tools/sg-cksum/*.c
	$(MAKE) -C tools/sg-cksum
	ln -sf $(ROOTDIR)/tools/sg-cksum/cksum tools/cksum

.PHONY: config
config:
	$(MAKEARCH) -C ${LINUXDIR} menuconfig

################################################################################
#
# normal make targets
#

.PHONY: romfs
romfs: romfs.newlog romfs.extract romfs.subdirs romfs.modules romfs.post

.PHONY: romfs.newlog
romfs.newlog:
	rm -f $(IMAGEDIR)/romfs-inst.log

.PHONY: romfs.extract
romfs.extract:
	[ -d romfs ] || tar xpzf romfs.tar.gz
	
.PHONY: romfs.subdirs
romfs.subdirs:
	for dir in $(DIRS) ; do [ ! -d $$dir ] || $(MAKEARCH) -C $$dir romfs || exit 1 ; done

.PHONY: romfs.modules
romfs.modules:
	. $(LINUX_CONFIG); \
	. $(CONFIG_CONFIG); \
	if [ "$$CONFIG_MODULES" = "y" ]; then \
		[ -d $(ROMFSDIR)/lib/modules ] || mkdir -p $(ROMFSDIR)/lib/modules; \
		$(MAKEARCH_KERNEL) -C $(LINUXDIR) INSTALL_MOD_CMD="$(ROMFSINST) -S -r \"\"" INSTALL_MOD_PATH=$(ROMFSDIR) DEPMOD="$(ROOTDIR)/tools/depmod.pl" modules_install; \
		rm -f $(ROMFSDIR)/lib/modules/*/build; \
		rm -f $(ROMFSDIR)/lib/modules/*/source; \
		find $(ROMFSDIR)/lib/modules -type f -name "*o" | xargs -r $(STRIP) -R .comment -R .note -g --strip-unneeded; \
		if [ "$$CONFIG_USER_BUSYBOX_FEATURE_MODPROBE_FANCY_ALIAS" = "y" ]; \
		then \
			find $(ROMFSDIR)/lib/modules -type f -name "*o" | \
			/bin/sh $(ROOTDIR)/tools/modules-alias.sh \
					$(ROMFSDIR)/etc/modprobe.conf;\
		fi; \
	fi

.PHONY: romfs.post
romfs.post:
	$(MAKEARCH) -C vendors romfs.post
	-find $(ROMFSDIR)/. -name CVS | xargs -r rm -rf
	echo "$(VERSIONSTR) CDK -- " `date` > $(ROMFSDIR)/etc/version

.PHONY: image
image:
	[ -d $(IMAGEDIR) ] || mkdir $(IMAGEDIR)
	$(MAKEARCH) -C vendors image

.PHONY: subdirs
subdirs:
	for dir in $(DIRS) ; do [ ! -d $$dir ] || $(MAKEARCH) -C $$dir || exit 1 ; done

.PHONY: linux
linux:
	@if expr "$(LINUXDIR)" : 'linux-2\.[0-4].*' > /dev/null && \
			 [ ! -f $(LINUXDIR)/.depend ] ; then \
		echo "ERROR: you need to do a 'make dep' first" ; \
		exit 1 ; \
	fi
	[ -f $(LINUXDIR)/net/ipsec/.linked ] && \
	rm -f $(LINUXDIR)/net/ipsec/.linked; \
	$(MAKEARCH_KERNEL) -j$(HOST_NCPU) -C $(LINUXDIR) $(LINUXTARGET) || exit 1
	if [ -f $(LINUXDIR)/vmlinux ]; then \
		ln -f $(LINUXDIR)/vmlinux $(LINUXDIR)/linux ; \
	fi

.PHONY: linux_%
linux_%:
	KCONFIG_NOTIMESTAMP=1 $(MAKEARCH_KERNEL) -C $(LINUXDIR) $(patsubst linux_%,%,$@)

dep:
	@if [ ! -f $(LINUXDIR)/.config ] ; then \
		echo "ERROR: you need to do a 'make config' first" ; \
		exit 1 ; \
	fi
	@if expr "$(LINUXDIR)" : 'linux-2\.[0-4].*' > /dev/null; then \
		$(MAKEARCH_KERNEL) -C $(LINUXDIR) symlinks || true; \
	fi
	$(MAKEARCH_KERNEL) -C $(LINUXDIR) dep

.PHONY: modules
modules:
	. $(LINUXDIR)/.config; if [ "$$CONFIG_MODULES" = "y" ]; then \
		[ -d $(LINUXDIR)/modules ] || mkdir $(LINUXDIR)/modules; \
		[ -f $(LINUXDIR)/net/ipsec/.linked ] && \
		rm -f $(LINUXDIR)/net/ipsec/.linked; \
		$(MAKEARCH_KERNEL) -j$(HOST_NCPU) -C $(LINUXDIR) modules || exit 1; \
	fi

.PHONY: automake
automake:
	$(MAKE) -C config automake

%_only:
	@case "$(@)" in \
	single*) $(MAKE) NON_SMP_BUILD=1 `expr $(@) : 'single[_]*\(.*\)'` ;; \
	*/*) d=`expr $(@) : '\([^/]*\)/.*'`; \
	     t=`expr $(@) : '[^/]*/\(.*\)'`; \
	     $(MAKEARCH) -C $$d $$t;; \
	*)   $(MAKEARCH) -C $(*);; \
	esac

%_clean:
	@case "$(@)" in \
	single*) $(MAKE) NON_SMP_BUILD=1 `expr $(@) : 'single[_]*\(.*\)'` ;; \
	*/*) d=`expr $(@) : '\([^/]*\)/.*'`; \
	     t=`expr $(@) : '[^/]*/\(.*\)'`; \
	     $(MAKEARCH) -C $$d $$t;; \
	*)   $(MAKEARCH) -C $(*) clean;; \
	esac

%_romfs:
	@case "$(@)" in \
	single*) $(MAKE) NON_SMP_BUILD=1 `expr $(@) : 'single[_]*\(.*\)'` ;; \
	*/*) d=`expr $(@) : '\([^/]*\)/.*'`; \
	     t=`expr $(@) : '[^/]*/\(.*\)'`; \
	     $(MAKEARCH) -C $$d $$t;; \
	*)   $(MAKEARCH) -C $(*) romfs;; \
	esac

clean:
	for dir in $(DIRS); do [ ! -d $$dir ] || $(MAKEARCH) -C $$dir clean ; done
	rm -rf $(ROMFSDIR)
	rm -rf $(IMAGEDIR)/*

help:
	@echo "Quick reference for various supported make commands."
	@echo "----------------------------------------------------"
	@echo ""
	@echo "make config                configure the kernel/modules"
	@echo "make dep                   2.4 and earlier kernels need this step"
	@echo "make                       build the entire tree and final images"
	@echo "make clean                 clean out compiled files, but not config"
	@echo "make linux                 compile the kernel only"
	@echo "make romfs                 install all files to romfs directory"
	@echo "make image                 combine romfs and kernel into final image"
	@echo "make modules               build all modules"
	@echo "make romfs.modules         install modules into romfs"
	@echo "make DIR_only              build just the directory DIR"
	@echo "make DIR_romfs             install files from directory DIR to romfs"
	@echo "make DIR_clean             clean just the directory DIR"
	@echo ""
	@echo "Typically you want the following sequence"
	@echo ""
	@echo "make config"
	@echo "make dep                   (for 2.4.x kernel)"
	@echo "make linux"
	@echo "make modules"
	@echo "make"
	@exit 0

############################################################################
