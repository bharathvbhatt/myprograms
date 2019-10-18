#############################################################################
#
#	Makefile -- Build instructions for OpenGear/VACM
#
#############################################################################

include $(LINUX_CONFIG)
include $(ARCH_CONFIG)

#############################################################################

ZIMAGE    = $(IMAGEDIR)/bzImage
LINUXIMG  = $(ROOTDIR)/$(LINUXDIR)/arch/i386/boot/bzImage
ROMFSIMG  = $(IMAGEDIR)/initrd
ROMFSIMGZ = $(IMAGEDIR)/initrd.gz
PRDBASE   = $(shell echo $(CONFIG_PRODUCT) | tr '[A-Z]' '[a-z]')
IMGBASE   = $(IMAGEDIR)/$(PRDBASE)-$(VERSIONPKG)
EXTIMG    = $(IMAGEDIR)/ext.img

VENDDIR       = $(ROOTDIR)/vendors/$(CONFIG_VENDOR)/$(CONFIG_PRODUCT)
SQUASH_ENDIAN = -le
ROMFS_DIRS    = $(DEFAULT_ROMFS_DIRS) boot

#############################################################################

DIRS = 

ROMFS_DIRS = \
	bin boot \
	dev dev/flash dev/input dev/pts \
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

DEVICES = \
	tty,c,5,0      console,c,5,1      cua0,c,5,64      cua1,c,5,65  \
	mem,c,1,1      kmem,c,1,2         null,c,1,3 \
	ram0,b,1,0     ram1,b,1,1 \
	ram2,b,1,2     ram3,b,1,3 \
	\
	ptmx,c,5,2 \
	\
	ptyp0,c,2,0    ptyp1,c,2,1      ptyp2,c,2,2      ptyp3,c,2,3 \
	ptyp4,c,2,4    ptyp5,c,2,5      ptyp6,c,2,6      ptyp7,c,2,7 \
	ptyp8,c,2,8    ptyp9,c,2,9      ptypa,c,2,10     ptypb,c,2,11 \
	ptypc,c,2,12   ptypd,c,2,13     ptype,c,2,14     ptypf,c,2,15 \
	\
	ptyq0,c,2,16    ptyq1,c,2,17    ptyq2,c,2,18    ptyq3,c,2,19 \
	ptyq4,c,2,20    ptyq5,c,2,21    ptyq6,c,2,22    ptyq7,c,2,23 \
	ptyq8,c,2,24    ptyq9,c,2,25    ptyqa,c,2,26    ptyqb,c,2,27 \
	ptyqc,c,2,28    ptyqd,c,2,29    ptyqe,c,2,30    ptyqf,c,2,31 \
	\
	ptyr0,c,2,32    ptyr1,c,2,33    ptyr2,c,2,34    ptyr3,c,2,35 \
	ptyr4,c,2,36    ptyr5,c,2,37    ptyr6,c,2,38    ptyr7,c,2,39 \
	ptyr8,c,2,40    ptyr9,c,2,41    ptyra,c,2,42    ptyrb,c,2,43 \
	ptyrc,c,2,44    ptyrd,c,2,45    ptyre,c,2,46    ptyrf,c,2,47 \
	\
	ptys0,c,2,48    ptys1,c,2,49    ptys2,c,2,50    ptys3,c,2,51 \
	ptys4,c,2,52    ptys5,c,2,53    ptys6,c,2,54    ptys7,c,2,55 \
	ptys8,c,2,56    ptys9,c,2,57    ptysa,c,2,58    ptysb,c,2,59 \
	ptysc,c,2,60    ptysd,c,2,61    ptyse,c,2,62    ptysf,c,2,63 \
	\
	tty0,c,4,0     tty1,c,4,1         tty2,c,4,2       tty3,c,4,3 \
	tty4,c,4,4     tty5,c,4,5         tty6,c,4,6       tty7,c,4,7 \
	\
	ttyACM0,c,166,0 \
	\
	ttyS0,c,4,64	ttyS1,c,4,65	ttyS2,c,4,66	ttyS3,c,4,67 \
	ttyS4,c,4,68	ttyS5,c,4,69	ttyS6,c,4,70	ttyS7,c,4,71 \
	ttyS8,c,4,72	ttyS9,c,4,73	ttyS10,c,4,74 	ttyS11,c,4,75 \
	ttyS12,c,4,76	ttyS13,c,4,77	ttyS14,c,4,78	ttyS15,c,4,79 \
	ttyS16,c,4,80	ttyS17,c,4,81	ttyS18,c,4,82	ttyS19,c,4,83 \
	ttyS20,c,4,84	ttyS21,c,4,85	ttyS22,c,4,86	ttyS23,c,4,87 \
	ttyS24,c,4,88	ttyS25,c,4,89	ttyS26,c,4,90	ttyS27,c,4,91 \
	ttyS28,c,4,92	ttyS29,c,4,93	ttyS30,c,4,94	ttyS31,c,4,95 \
	ttyS32,c,4,96	ttyS33,c,4,97	ttyS34,c,4,98	ttyS35,c,4,99 \
	ttyS36,c,4,100	ttyS37,c,4,101	ttyS38,c,4,102	ttyS39,c,4,103 \
	ttyS40,c,4,104	ttyS41,c,4,105	ttyS42,c,4,106	ttyS43,c,4,107 \
	ttyS44,c,4,108	ttyS45,c,4,109	ttyS46,c,4,110	ttyS47,c,4,111 \
	ttyS48,c,4,112	ttyS49,c,4,113	\
	ttyS50,c,4,114	ttyS51,c,4,115	ttyS52,c,4,116	ttyS53,c,4,117 \
	ttyS54,c,4,118	ttyS55,c,4,119	ttyS56,c,4,120	ttyS57,c,4,121 \
	ttyS58,c,4,122	ttyS59,c,4,123	\
	ttyS60,c,4,124	ttyS61,c,4,125	ttyS62,c,4,126	ttyS63,c,4,127 \
	\
	sda,b,8,0 \
	sda1,b,8,1 sda2,b,8,2 sda3,b,8,3 sda4,b,8,4 sda5,b,8,5 \
	sda6,b,8,6 sda7,b,8,7 sda8,b,8,8 sda9,b,8,9 sda10,b,8,10 \
	sda11,b,8,11 sda12,b,8,12 sda13,b,8,13 sda14,b,8,14 sda15,b,8,15 \
	sdb,b,8,16 \
	sdb1,b,8,17 sdb2,b,8,18 sdb3,b,8,19 sdb4,b,8,20 sdb5,b,8,21 \
	sdb6,b,8,22 sdb7,b,8,23 sdb8,b,8,24 sdb9,b,8,25 sdb10,b,8,26 \
	sdb11,b,8,27 sdb12,b,8,28 sdb13,b,8,29 sdb14,b,8,30 sdb15,b,8,31 \
	sdc,b,8,32 \
	sdc1,b,8,33 sdc2,b,8,34 sdc3,b,8,35 sdc4,b,8,36 sdc5,b,8,37 \
	sdc6,b,8,38 sdc7,b,8,39 sdc8,b,8,40 sdc9,b,8,41 sdc10,b,8,42 \
	sdc11,b,8,43 sdc12,b,8,44 sdc13,b,8,45 sdc14,b,8,46 sdc15,b,8,47 \
	sdd,b,8,48 \
	sdd1,b,8,49 sdd2,b,8,50 sdd3,b,8,51 sdd4,b,8,52 sdd5,b,8,53 \
	sdd6,b,8,54 sdd7,b,8,55 sdd8,b,8,56 sdd9,b,8,57 sdd10,b,8,58 \
	sdd11,b,8,59 sdd12,b,8,60 sdd13,b,8,61 sdd14,b,8,62 sdd15,b,8,63 \
	sde,b,8,64 \
	sde1,b,8,65 sde2,b,8,66 sde3,b,8,67 sde4,b,8,68 sde5,b,8,69 \
	sde6,b,8,70 sde7,b,8,71 sde8,b,8,72 sde9,b,8,73 sde10,b,8,74 \
	sde11,b,8,75 sde12,b,8,76 sde13,b,8,77 sde14,b,8,78 sde15,b,8,79 \
	sdf,b,8,80 \
	sdf1,b,8,81 sdf2,b,8,82 sdf3,b,8,83 sdf4,b,8,84 sdf5,b,8,85 \
	sdf6,b,8,86 sdf7,b,8,87 sdf8,b,8,88 sdf9,b,8,89 sdf10,b,8,90 \
	sdf11,b,8,91 sdf12,b,8,92 sdf13,b,8,93 sdf14,b,8,94 sdf15,b,8,95 \
	sdg,b,8,96 \
	sdg1,b,8,97 sdg2,b,8,98 sdg3,b,8,99 sdg4,b,8,100 sdg5,b,8,101 \
	sdg6,b,8,102 sdg7,b,8,103 sdg8,b,8,104 sdg9,b,8,105 sdg10,b,8,106 \
	sdg11,b,8,107 sdg12,b,8,108 sdg13,b,8,109 sdg14,b,8,110 sdg15,b,8,111 \
	sdh,b,8,112 \
	sdh1,b,8,113 sdh2,b,8,114 sdh3,b,8,115 sdh4,b,8,116 sdh5,b,8,117 \
	sdh6,b,8,118 sdh7,b,8,119 sdh8,b,8,120 sdh9,b,8,121 sdh10,b,8,122 \
	sdh11,b,8,123 sdh12,b,8,124 sdh13,b,8,125 sdh14,b,8,126 sdh15,b,8,127 \
	\
	fd0,b,2,0 \
	fd0CompaQ,b,2,36 fd0d360,b,2,4 fd0h1200,b,2,8 fd0h1440,b,2,40 \
	fd0h1476,b,2,56 fd0h1494,b,2,72 fd0h1600,b,2,92 fd0h360,b,2,20 \
	fd0h410,b,2,48 fd0h420,b,2,64 fd0h720,b,2,24 fd0h880,b,2,80 \
	fd0u1040,b,2,84 fd0u1120,b,2,88 fd0u1440,b,2,28 fd0u1600,b,2,124 \
	fd0u1680,b,2,44 fd0u1722,b,2,60 fd0u1743,b,2,76 fd0u1760,b,2,96 \
	fd0u1840,b,2,116 fd0u1920,b,2,100 fd0u2880,b,2,32 fd0u3200,b,2,104 \
	fd0u3520,b,2,108 fd0u360,b,2,12 fd0u3840,b,2,112 fd0u720,b,2,16 \
	fd0u800,b,2,120 fd0u820,b,2,52 fd0u830,b,2,68 \
	fd1,b,2,1 \
	fd1CompaQ,b,2,37 fd1d360,b,2,5 fd1h1200,b,2,9 fd1h1440,b,2,41 \
	fd1h1476,b,2,57 fd1h1494,b,2,73 fd1h1600,b,2,93 fd1h360,b,2,21 \
	fd1h410,b,2,49 fd1h420,b,2,65 fd1h720,b,2,25 fd1h880,b,2,81 \
	fd1u1040,b,2,85 fd1u1120,b,2,89 fd1u1440,b,2,29 fd1u1600,b,2,125 \
	fd1u1680,b,2,45 fd1u1722,b,2,61 fd1u1743,b,2,77 fd1u1760,b,2,97 \
	fd1u1840,b,2,117 fd1u1920,b,2,101 fd1u2880,b,2,33 fd1u3200,b,2,105 \
	fd1u3520,b,2,109 fd1u360,b,2,13 fd1u3840,b,2,113 fd1u720,b,2,17 \
	fd1u800,b,2,121 fd1u820,b,2,53 fd1u830,b,2,69 \
	\
	ttyp0,c,3,0    ttyp1,c,3,1        ttyp2,c,3,2      ttyp3,c,3,3 \
	ttyp4,c,3,4    ttyp5,c,3,5        ttyp6,c,3,6      ttyp7,c,3,7 \
	ttyp8,c,3,8    ttyp9,c,3,9        ttypa,c,3,10     ttypb,c,3,11 \
	ttypc,c,3,12   ttypd,c,3,13       ttype,c,3,14     ttypf,c,3,15 \
	\
	zero,c,1,5     random,c,1,8       urandom,c,1,9 \
	\
	ipsec,c,36,10 \
	ledman,c,126,0 \
	hifn0,c,28,0 \
	watchdog,c,10,130 \
	ppp,c,108,0 \
	nvram,c,10,144 \
	\
	hda,b,3,0   hda1,b,3,1   hda2,b,3,2   hda3,b,3,3   hda4,b,3,4 \
	hdb,b,3,64  hdb1,b,3,65  hdb2,b,3,66  hdb3,b,3,67  hdb4,b,3,68 \
	hdc,b,22,0  hdc1,b,22,1  hdc2,b,22,2  hdc3,b,22,3  hdc4,b,22,4 \
	hdd,b,22,64 hdd1,b,22,65 hdd2,b,22,66 hdd3,b,22,67 hdd4,b,22,68 \
	\
	mtd0,c,90,0	mtdr0,c,90,1	mtdblock0,b,31,0 \
	mtd1,c,90,2	mtdr1,c,90,3	mtdblock1,b,31,1 \
	mtd2,c,90,4	mtdr2,c,90,5	mtdblock2,b,31,2 \
	mtd3,c,90,6	mtdr3,c,90,7	mtdblock3,b,31,3 \
	mtd4,c,90,8	mtdr4,c,90,9	mtdblock4,b,31,4 \
	mtd5,c,90,10	mtdr5,c,90,11	mtdblock5,b,31,5 \
	mtd6,c,90,12	mtdr6,c,90,13	mtdblock6,b,31,6

FLASH_DEVICES = \
	image,b,8,0 \
	rootfs,b,8,1 \
	ethmac,b,8,4

#############################################################################

all:
	dirs=$(DIRS) ; \
	for i in $$dirs ; do  make -C $$i || exit $? ; done

clean:
	-dirs=$(DIRS) ; \
	for i in $$dirs; do [ ! -d $$i ] || make -C $$i clean; done
	rm -f romfs.img ext3.img cramfs.img mkcramfs lilo

.PHONY: romfs
romfs:
	[ -d $(ROMFSDIR)/$$i ] || mkdir -m 0755 -p $(ROMFSDIR)
	for i in $(ROMFS_DIRS); do \
		[ -d $(ROMFSDIR)/$$i ] || mkdir -m 0755 -p $(ROMFSDIR)/$$i; \
	done
	for i in $(DEVICES); do \
		touch $(ROMFSDIR)/dev/`dirname $$i`/@`basename $$i`; \
	done
	port=1; while [ $$port -le 64 ]; do \
		pname=`printf "port%02d" $$port`; \
		$(ROMFSINST) -s /var/dev/$$pname /dev/$$pname; \
		port=`expr $$port + 1`; \
	done
	chmod 666 $(ROMFSDIR)/dev/@ptmx,*
	$(ROMFSINST) -s /var/dev/sercon /dev/sercon
	$(ROMFSINST) $(LINUXIMG) /boot/vmlinuz
	$(ROMFSINST) ../romfs /
	$(ROMFSINST) romfs /
	$(ROMFSINST) -s /etc/config/hosts /etc/hosts
	$(ROMFSINST) -s /etc/config/TZ /etc/TZ
	$(ROMFSINST) -A "tty1:" -a "tty1:vt100:/bin/agetty /dev/tty1 9600" /etc/inittab
	$(ROMFSINST) -A "tty2:" -a "tty2:vt100:/bin/agetty /dev/tty2 9600" /etc/inittab
	$(ROMFSINST) -A "tty3:" -a "tty3:vt100:/bin/agetty /dev/tty3 9600" /etc/inittab
	$(ROMFSINST) -A "ttyS0:" -a "ttyS0:vt100:/bin/agetty /dev/ttyS0 9600" /etc/inittab
	make -f ../common.mk --warn-undefined-variables romfs
	chmod 555 $(ROMFSDIR)/etc/default/scripts/*

romfs.post::
	rm -rf $(ROMFSDIR)/man[1-9]
	rm -rf $(ROMFSDIR)/share/man*
	rm -rf $(ROMFSDIR)/share/info*
	# fix up permissions
	-chmod a+rx $(ROMFSDIR)/bin/*
	-chmod a+rx $(ROMFSDIR)/etc/rc* $(ROMFSDIR)/etc/ifup*
	# Replace setfset with dummy script
	$(ROMFSINST) -s /etc/config/scripts/setfset /bin/setfset

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

image: image.dir image.i386.zimage image.cramfs hdd-images

#############################################################################


	# CHS size gives: 255 * 63 * 38 / 2 / 1024 = 298 MB or so
	# 272MB (exactly) for romfs partition, which contains a 270MB filesystem
	# (at time of writing, a built filesystem is about 240MB)
	# 20MB for config FS
	# 4 MB for nvlog FS (not needed, really)
	# this total uses about 296MB of our 298 size
hdd-images:
	$(MAKE) EXT=vm HEADS=255 SECTORS=63 CYLS=39 \
		ROMFS_MB=270 ROMFS_PARTITION_SECTORS=557056 CONFIG_MB=20 NVLOG_MB=4 \
		EXTRA_FILES="etc/virt" ext3-images vmware-image

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
	[ -z "$(APPEND)" ] || \
		echo "  append = \"$(APPEND)\"" >> $(IMAGEDIR)/ext3/etc/lilo.conf
	cd $(IMAGEDIR)/ext3/.; find dev -type f -name '@*' -print | \
		sed 's/[@,]/ /g' | while read path file type maj min; do \
			rm -f $(IMAGEDIR)/ext3/$$path/@$$file,$$type,$$maj,$$min; \
			echo "/$$path$$file     $$type 666 0 0 $$maj $$min - - -"; \
		done > $(IMAGEDIR)/device.spec
	/usr/bin/genext2fs -U -d $(IMAGEDIR)/ext3 -D $(IMAGEDIR)/device.spec \
		-b $(ROMFS_BLOCKS) $(EXTIMG)
	/sbin/tune2fs -i 0 -j $(EXTIMG)
	dd if=/dev/zero bs=512 count=1 of=$(IMAGE)
	dd if=$(EXTIMG) conv=sync bs=`expr 1024 \\* 1024 \\* $(ROMFS_MB)` >> $(IMAGE)
	dd if=/dev/zero bs=512 count=0 seek=`expr $(CYLS) \* $(HEADS) \* $(SECTORS)` of=$(IMAGE)
	-printf "1,$(TOTAL_ROMFS),L,*\n,$(TOTAL_NVLOG),61,-\n,$(TOTAL_CONFIG),61,-\n,,L,-\n" | \
		/sbin/sfdisk --force $(IMAGE)
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

