#############################################################################
#
#	Makefile -- Build instructions for OpenGear/Lighthouse
#	toby.smith@opengear.com
#
#############################################################################

include $(LINUX_CONFIG)
include $(CONFIG_CONFIG)
include $(ARCH_CONFIG)

#############################################################################

ZIMAGE    = $(IMAGEDIR)/bzImage
LINUXIMG  = $(ROOTDIR)/$(LINUXDIR)/arch/i386/boot/bzImage
ROMFSIMG  = $(IMAGEDIR)/initrd
ROMFSIMGZ = $(IMAGEDIR)/initrd.gz
PRDBASE   = $(shell echo $(CONFIG_PRODUCT) | tr '[A-Z]' '[a-z]')
IMGBASE   = $(IMAGEDIR)/$(PRDBASE)-$(VERSIONPKG)
ISODIR    = $(IMAGEDIR)/iso
EXTIMG    = $(IMAGEDIR)/ext.img
USBIMG    = $(IMAGEDIR)/lighthouse-boot-usb.img
USBROMFSIMGZ = $(IMAGEDIR)/initrdU.gz
USBDIR    = $(IMAGEDIR)/usb

VENDDIR       = $(ROOTDIR)/vendors/$(CONFIG_VENDOR)/$(CONFIG_PRODUCT)
SQUASH_ENDIAN = -le
ROMFS_DIRS    = $(DEFAULT_ROMFS_DIRS) boot

#############################################################################

DIRS = 

ROMFS_DIRS = \
	bin boot \
	dev \
	etc etc/config \
	home home/httpd home/httpd/cgi-bin \
	lib lib/modules \
	mnt \
	proc \
	sbin \
	sys \
	usr usr/bin usr/sbin \
	var var/run var/log var/tmp var/lock var/empty \
	share/pixmaps

ifndef CONFIG_USER_UDEV
  ROMFS_DIRS += dev/flash dev/input dev/pts
endif

#############################################################################

all:
	dirs=$(DIRS) ; \
	for i in $$dirs ; do  make -C $$i || exit $? ; done

clean:
	-dirs=$(DIRS) ; \
	for i in $$dirs; do [ ! -d $$i ] || make -C $$i clean; done
	rm -f romfs.img ext3.img cramfs.img mkcramfs lilo

romfs:
	[ -d $(ROMFSDIR)/$$i ] || mkdir -m 0755 -p $(ROMFSDIR)
	for i in $(ROMFS_DIRS); do \
		[ -d $(ROMFSDIR)/$$i ] || mkdir -m 0755 -p $(ROMFSDIR)/$$i; \
	done
ifndef CONFIG_USER_UDEV
	for i in $(DEVICES); do \
		touch $(ROMFSDIR)/dev/`dirname $$i`/@`basename $$i`; \
	done
	$(ROMFSINST) -s /var/dev/sercon /dev/sercon
	$(ROMFSINST) -s /dev/psaux /dev/mouse
endif
	$(ROMFSINST) $(LINUXIMG) /boot/vmlinuz
	$(ROMFSINST) ../romfs /
	$(ROMFSINST) rc.dev /etc/
	$(ROMFSINST) -s /etc/config/hosts /etc/hosts
	$(ROMFSINST) /etc/nsswitch.conf
	$(ROMFSINST) -s /etc/config/TZ /etc/TZ
	$(ROMFSINST) -A "tty1:" -a "tty1:vt100:/bin/agetty /dev/tty1 9600" /etc/inittab
	$(ROMFSINST) -A "tty2:" -a "tty2:vt100:/bin/agetty /dev/tty2 9600" /etc/inittab
	$(ROMFSINST) -A "tty3:" -a "tty3:vt100:/bin/agetty /dev/tty3 9600" /etc/inittab
	$(ROMFSINST) -A "initiallogin.root.001" -F initiallogin_root.script /etc/profile
	$(ROMFSINST) -A "initiallogin.root.001" -F initiallogin_root.script /etc/profile
	make -f ../common.mk --warn-undefined-variables romfs
	# must be done after the common romfs make above: this overrides the default config.xml
	$(ROMFSINST) /etc/default/config.xml
	$(ROMFSINST) dialout-up /etc/scripts/dialout-up
	chmod 555 $(ROMFSDIR)/etc/scripts/dialout-up

romfs.post::
	rm -rf $(ROMFSDIR)/man[1-9]
	rm -rf $(ROMFSDIR)/share/man*
	rm -rf $(ROMFSDIR)/share/info*
	# fix up permissions
	-chmod a+rx $(ROMFSDIR)/bin/*
	-chmod a+rx $(ROMFSDIR)/etc/rc* $(ROMFSDIR)/etc/ifup*

#############################################################################
#
# re copy the kernel so that "make linux image" works
#

mkcramfs: $(ROOTDIR)/user/cramfs/mkcramfs.c
	$(HOSTCC) -o $@ $< -lz

image.dir:
	[ -d $(IMAGEDIR) ] || mkdir -p $(IMAGEDIR)

image.i386.zimage:
	cp $(ROOTDIR)/$(LINUXDIR)/arch/i386/boot/bzImage $(ZIMAGE)

image.cramfs: mkcramfs
	./mkcramfs -z -r $(ROMFSDIR) $(ROMFSIMG)

# Create (possibly) mbr + cramfs + zimage/linuz
image.bin:
	cat $(MBRIMG) $(ROMFSIMG) $(SHIM) $(ZIMAGE) >$(IMAGE)

image.flash:
	[ ! -f $(ROOTDIR)/boot/boot.bin ] || $(MAKE) vendor_flashbin

image: image.dir image.i386.zimage image.cramfs hdd-images iso-images

#############################################################################

iso-images:
	gzip -9 -f $(ROMFSIMG)
	$(MAKE) IMAGE=$(IMGBASE).iso  DEFAULT=vga   iso-image

iso-image:
	mkdir -p $(ISODIR)/boot/isolinux
	cp isolinux.bin $(ISODIR)/boot/isolinux/
	cp boot.msg $(ISODIR)/boot/isolinux/
	echo "default $(DEFAULT)" > $(ISODIR)/boot/isolinux/isolinux.cfg
	cat isolinux.cfg >> $(ISODIR)/boot/isolinux/isolinux.cfg
	cp $(ZIMAGE) $(ISODIR)/boot/isolinux/
	cp $(ROMFSIMGZ) $(ISODIR)/boot/isolinux/initrd.gz
	mkisofs -J -o $(IMAGE) -b boot/isolinux/isolinux.bin \
		-c boot/isolinux/boot.cat -no-emul-boot \
		-boot-load-size 4 -boot-info-table $(ISODIR)
	rm -rf $(ISODIR)

#############################################################################


hdd-images:
	# the VM images are now all the same (instead of separate images for KVM
	# and VMware), because with the linux 3.x kernel both IDE and SCSI drives
	# come up as 'sda', so we don't need to differentiate between the two.

	# the VM has a fixed 4 GB disk size; because there are already VMs out there
	# using it, we can't change the partition table or it will break upgrades.
	# This is split with a 400 MB root filesystem (in a ~1.8 GB partition), then
	# a 200 MB config partition, a 1.8 GB nvlog partition, and a tiny partition
	# the old serial key (approx. 8 MB).
	# if users want to expand or add more storage later on (for future unknown
	# features), they can create a new virtual HDD and add that to the VM.
	# CYLS = 4 000 000 000 / HEADS / SECTORS / 512 sector size
	$(MAKE) EXT=vm HEADS=255 SECTORS=63 CYLS=486                                   \
		ROMFS_MB=400 ROMFS_PARTITION_SECTORS=3695524 CONFIG_MB=200 NVLOG_MB=1800   \
		EXTRA_FILES="etc/virt"  ext3-images  vmware-image

	# the hardware LH devices have (at least) a 500 GB HDD, so we'll tailor it
	# for that. We'll give a 20 GB partition for / (with a smaller filesystem,
	# since that space isn't needed yet), 50 GB for each of config and nvlog,
	# and the remaining 4th partition we'll mark as an extended partition that
	# can be used for unknown extended things in future (about 350 GB or so).
	# CYLS = 500 000 000 000 / HEADS / SECTORS / 512 sector size
	$(MAKE) EXT=hw HEADS=255 SECTORS=63 CYLS=60788   ROMFS_MB=400                  \
		ROMFS_PARTITION_SECTORS=41943040 CONFIG_MB=51200 NVLOG_MB=51200            \
		REMOVE_FILES="dev/flash/@ethmac,b,8,4" ext3-images

ext3-images:
	$(MAKE) IMAGE=$(IMGBASE)-$(EXT).hdd NFI=$(IMGBASE)-$(EXT).bin \
		FLASH_IMAGE=$(IMGBASE)-$(EXT).flash.gz APPEND= hdd-image

ifdef SECTORS
TOTAL_SECTORS := $(shell expr \( $(CYLS) - 1 \) \* $(HEADS) \* $(SECTORS))
TOTAL_ROMFS   := $(ROMFS_PARTITION_SECTORS)
TOTAL_CONFIG  := $(shell expr $(CONFIG_MB) \* 1024 \* 1024 / 512)
TOTAL_NVLOG   := $(shell expr $(NVLOG_MB) \* 1024 \* 1024 / 512)

ROMFS_BLOCKS  := $(shell expr $(ROMFS_MB) \* 1024)

# for the generated HDD images, we want to have the following files:
# TARGET.hdd ($IMAGE): the entire HDD image, raw. (bootsect, romfs, then 0s to the end).
#                      if it's prohibitively big, can be removed.
# TARGET.flash.gz: the used portion of the HDD image (bootsect, romfs), gzipped. Useful
#                  for factory deployment on HW devices.
# TARGET.bin ($NFI): the used potion of the HDD image, gzipped, with version and
#                    checksum data at the end. Normal netflash upgrade file.
hdd-image:
	rm -f $(IMAGE)
	rm -f $(EXTIMG) $(IMAGEDIR)/device.spec $(IMAGEDIR)/qemu.pid
	rm -rf $(IMAGEDIR)/ext3
	mkdir $(IMAGEDIR)/ext3
	(cd $(ROMFSDIR);tar cf - .)|(cd $(IMAGEDIR)/ext3;tar xpf -)
	cp lilo.conf $(IMAGEDIR)/ext3/etc/lilo.conf 
	cp lilo.message $(IMAGEDIR)/ext3/etc/lilo.message
	for i in $(EXTRA_FILES); do \
		touch $(IMAGEDIR)/ext3/$$i; \
	done
	for i in $(REMOVE_FILES); do \
		rm $(IMAGEDIR)/ext3/$$i; \
	done
	[ -z "$(APPEND)" ] || \
		echo "	append = \"$(APPEND)\"" >> $(IMAGEDIR)/ext3/etc/lilo.conf
	cd $(IMAGEDIR)/ext3/.; find dev -type f -name '@*' -print | \
		sed 's/[@,]/ /g' | while read path file type maj min; do \
			rm -f $(IMAGEDIR)/ext3/$$path/@$$file,$$type,$$maj,$$min; \
			echo "/$$path$$file	$$type 666 0 0 $$maj $$min - - -"; \
		done > $(IMAGEDIR)/device.spec
	/usr/bin/genext2fs -U -d $(IMAGEDIR)/ext3 -D $(IMAGEDIR)/device.spec \
		-b $(ROMFS_BLOCKS) $(EXTIMG)
	/sbin/tune2fs -i 0 -j $(EXTIMG)
	dd if=/dev/zero bs=512 count=1 of=$(IMAGE)
	dd if=$(EXTIMG) conv=sync bs=`expr 1024 \\* 1024 \\* $(ROMFS_MB)` \
		>> $(IMAGE)
	dd if=/dev/zero bs=512 count=0 \
		seek=`expr $(CYLS) \* $(HEADS) \* $(SECTORS)` of=$(IMAGE)
	-printf "1,$(TOTAL_ROMFS),L,*\n,$(TOTAL_NVLOG),61,-\n,$(TOTAL_CONFIG),61,-\n,,L,-\n" | \
		/sbin/sfdisk --force -uS -C $(CYLS) -H $(HEADS) -S $(SECTORS) $(IMAGE)
	yes "" | \
		qemu-system-i386 -net none -nographic -pidfile $(IMAGEDIR)/qemu.pid \
			-append "rw root=801 console=ttyS0,9600 init=/bin/lilo" \
			-kernel $(ZIMAGE) \
			-drive file=/dev/zero,if=ide \
			-drive file=$(IMAGE),if=scsi | \
		while read t; do \
			if [ "$${#t}" -gt 1 ]; \
			then \
				echo "$$t"; \
			fi; \
			case "$$t" in \
			*panic*) kill -INT `cat $(IMAGEDIR)/qemu.pid` ;; \
		esac; \
	done
	rm -f $(EXTIMG) $(IMAGEDIR)/device.spec $(IMAGEDIR)/qemu.pid
	rm -rf $(IMAGEDIR)/ext3
	dd if=$(IMAGE) conv=sync bs=`expr 1024 \\* 1024 \\* $(ROMFS_MB) + 512` \
		count=1 | gzip -9  > $(NFI)
	printf '\0%s\0%s\0%s' $(VERSIONPKG) $(HW_VENDOR) $(HW_PRODUCT) >>$(NFI)
	$(ROOTDIR)/tools/cksum -b -o 2 $(NFI) >> $(NFI)
	@if [ -d /tftpboot ]; then \
		echo "cp $(NFI) /tftpboot/"; \
		cp $(NFI) /tftpboot/ 2> /dev/null; \
	fi
	# make the .flash.gz file -- just the used portion, gzipped
	dd if=$(IMAGE) conv=sync bs=`expr 1024 \\* 1024 \\* $(ROMFS_MB) + 512` \
		count=1 | gzip -9  > $(FLASH_IMAGE)
	# if the HDD image is too big (> ~10GB), kill it. We can use the flash image.
	@if [ `stat -c %s $(IMAGE)` -gt 10000000000 ]; then \
		rm $(IMAGE); \
	fi 

vmware-image:
	( \
		echo "# VMWare Disk descriptor file"; \
		echo "version=1"; \
		echo "CID=ffffffff"; \
		echo "parentCID=ffffffff"; \
		echo "createType=\"monolithicFlat\""; \
		echo "# Extent description"; \
		echo "RW `expr $(CYLS) \\* $(HEADS) \\* $(SECTORS)` FLAT \"`basename $(IMGBASE)-$(EXT).hdd`\" 0"; \
		echo "ddb.virtualHWVersion = \"4\""; \
		echo "ddb.adapterType = \"buslogic\""; \
		echo "ddb.geometry.cylinders = \"$(CYLS)\""; \
		echo "ddb.geometry.heads = \"$(HEADS)\""; \
		echo "ddb.geometry.sectors = \"$(SECTORS)\""; \
	) > $(IMAGEDIR)/$(CONFIG_PRODUCT)-$(EXT).vmdk
	cp $(VENDDIR)/$(CONFIG_PRODUCT).vmx \
		$(IMAGEDIR)/$(CONFIG_PRODUCT)-$(EXT).vmx
	rm -f $(IMGBASE)-$(EXT).tar.gz
	(cd $(IMAGEDIR); tar cvzf `basename $(IMGBASE)-$(EXT).tar.gz` \
		$(CONFIG_PRODUCT)-$(EXT).vmdk \
		`basename $(IMGBASE)-$(EXT).hdd` \
		$(CONFIG_PRODUCT)-$(EXT).vmx)
	if [ -f /usr/bin/ovftool ]; then \
		rm -rf $(IMAGEDIR)/ovf; \
		rm -rf $(IMGBASE)-$(EXT)-ovf.zip; \
		mkdir $(IMAGEDIR)/ovf/; \
		ovftool $(IMAGEDIR)/$(CONFIG_PRODUCT)-$(EXT).vmx $(IMAGEDIR)/ovf/; \
		(cd $(IMAGEDIR); zip -r `basename $(IMGBASE)-$(EXT)-ovf.zip` ovf) \
	fi
endif

image.usb:
	dd if=/dev/zero of=$(USBIMG) bs=1M count=1000 && \
	OFFSET=`mkdiskimage -o -4 $(USBIMG) 0 64 32` && \
	syslinux -t $$OFFSET $(USBIMG) && \
	LOOPDEV=`sudo losetup -o $$OFFSET -f --show $(USBIMG)` && \
	mkdir -p $(USBDIR) && \
	sudo mount -t vfat $$LOOPDEV $(USBDIR) && \
	sudo cp boot.msg syslinux.cfg $(USBDIR) && \
	sudo cp install.sh $(USBDIR) && \
	sudo chmod a+x $(USBDIR)/install.sh && \
	sudo cp $(IMGDIR)/$(IMGBASE)-hw.bin $(USBDIR)/lighthouse-hw.bin && \
	sudo cp $(USBROMFSIMGZ) $(ZIMAGE) $(USBDIR) && \
	sudo umount $(USBDIR) && \
	sudo losetup -d $$LOOPDEV

image.cramfs.usb: mkcramfs
	./mkcramfs -z -r $(ROMFSDIR) $(USBROMFSIMGZ)
	
romfs.usb-image:
	mv $(ROMFSDIR)/etc/rc $(VENDDIR)/rc.romfs
	cp $(VENDDIR)/rc.boot-usb $(ROMFSDIR)/etc/rc
	chmod a+x $(ROMFSDIR)/etc/rc

romfs.usb-image.post:
	mv $(VENDDIR)/rc.romfs $(ROMFSDIR)/etc/rc
	chmod a+x $(ROMFSDIR)/etc/rc

installer: romfs.usb-image image.cramfs.usb image.usb romfs.usb-image.post
