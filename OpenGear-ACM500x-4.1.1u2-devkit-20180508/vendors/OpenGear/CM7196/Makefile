#
#	Makefile -- Build instructions for OpenGear/CM7196
#	ken.wilson@opengear.com
#
.EXPORT_ALL_VARIABLES:

KERNELZ  = $(IMAGEDIR)/zImage
RAMDISK  = $(IMAGEDIR)/ramdisk
RAMDISKZ = $(IMAGEDIR)/ramdisk.gz
IMAGE    = $(IMAGEDIR)/image.bin

ROMFS_DIRS = \
	bin \
	dev \
	etc etc/config etc/default etc/default/users \
	home home/httpd home/httpd/cgi-bin \
	lib lib/modules \
	mnt \
	opt \
	proc \
	sbin \
	sys \
	usr usr/bin usr/sbin \
	var

ifndef CONFIG_USER_UDEV
  ROMFS_DIRS += dev/flash dev/pts
endif

all:

clean:
	-rm -f mkcramfs mksquashfs mksquashfs7z

.PHONY: romfs
romfs:
	[ -d $(ROMFSDIR)/$$i ] || mkdir -m 0755 -p $(ROMFSDIR)
	for i in $(ROMFS_DIRS); do \
		[ -d $(ROMFSDIR)/$$i ] || mkdir -m 0755 -p $(ROMFSDIR)/$$i; \
	done
ifndef CONFIG_USER_UDEV
	for i in $(DEVICES); do \
		touch $(ROMFSDIR)/dev/@$$i; \
	done
	port=1; while [ $$port -le 64 ]; do \
		pname=`printf "port%02d" $$port`; \
		$(ROMFSINST) -s /var/dev/$$pname /dev/$$pname; \
		port=`expr $$port + 1`; \
	done
	
	$(ROMFSINST) -s /var/dev/sercon /dev/sercon
	$(ROMFSINST) -s /var/dev/modem01 /dev/modem01
	$(ROMFSINST) -s /var/dev/usbmodem01 /dev/usbmodem01
	$(ROMFSINST) -s /var/dev/cellmodem01 /dev/cellmodem01
	$(ROMFSINST) -s /var/dev/cellcommand01 /dev/cellcommand01
	$(ROMFSINST) -s /var/dev/bus /dev/bus
	$(ROMFSINST) -s /var/dev/gps0 /dev/gps0
endif
	$(ROMFSINST) ../romfs /
	$(ROMFSINST) romfs /
	make -f ../common.mk romfs

romfs.post::

mkcramfs: $(ROOTDIR)/user/cramfs/mkcramfs.c
	$(HOSTCC) -o $@ -I$(ROOTDIR)/$(LINUXDIR)/include $< -lz

mksquashfs:
	CC=$(HOSTCC) CFLAGS=$(HOSTCFLAGS) EXTRA_CFLAGS= make -C $(ROOTDIR)/user/squashfs-new/squashfs-tools mksquashfs
	ln -fs $(ROOTDIR)/user/squashfs-new/squashfs-tools/mksquashfs .

mksquashfs7z:
	make -C $(ROOTDIR)/user/squashfs/squashfs-tools mksquashfs7z
	ln -fs $(ROOTDIR)/user/squashfs/squashfs-tools/mksquashfs7z .

ifeq (linux-3,$(findstring linux-3,$(LINUXDIR)))
SQUASHFS_TOOL = mksquashfs
EXTRA_SQUASH_ARGS =
else
SQUASHFS_TOOL = mksquashfs7z
EXTRA_SQUASH_ARGS = -le
endif

image: $(SQUASHFS_TOOL)
	[ -d $(IMAGEDIR) ] || mkdir -p $(IMAGEDIR)
	-$(STRIP) $(ROMFSDIR)/bin/*
	-$(STRIP) $(ROMFSDIR)/sbin/*
	-$(STRIP) $(ROMFSDIR)/home/httpd/cgi-bin/*
	rm -rf $(ROMFSDIR)/man[1-9]
	chmod a+rx $(ROMFSDIR)/bin/* || /bin/true
	chmod a+rx $(ROMFSDIR)/etc/rc* $(ROMFSDIR)/etc/ifup*
	rm -rf $(ROMFSDIR)/share
	cp $(ROOTDIR)/$(LINUXDIR)/arch/arm/boot/zImage $(KERNELZ)
	$(MAKE) -C $(ROOTDIR) linux_armada-370-cm7196.dtb
	cat $(ROOTDIR)/$(LINUXDIR)/arch/arm/boot/dts/armada-370-cm7196.dtb >> $(KERNELZ)
	-rm $(RAMDISK)
	MKSQUASHFS=`pwd`/$(SQUASHFS_TOOL) ; cd $(ROMFSDIR) ; \
	$$MKSQUASHFS . $(RAMDISK) -all-root -noappend $(EXTRA_SQUASH_ARGS)
	cp $(RAMDISK) $(RAMDISKZ)
	cp $(RAMDISKZ) $(IMAGE)
	cat $(KERNELZ) >> $(IMAGE)
	printf '\0%s\0%s\0%s' $(VERSIONPKG) $(HW_VENDOR) $(HW_PRODUCT) >>$(IMAGE)
	$(ROOTDIR)/tools/cksum -b -o 2 $(IMAGE) >> $(IMAGE)
	if [ -d /tftpboot ]; then \
		cp $(KERNELZ) /tftpboot/ 2> /dev/null; \
		cp $(RAMDISKZ) /tftpboot/ 2> /dev/null; \
		cp $(IMAGE) /tftpboot/ 2> /dev/null; \
	fi

