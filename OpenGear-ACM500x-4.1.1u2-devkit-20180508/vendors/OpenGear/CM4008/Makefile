#
#	Makefile -- Build instructions for OpenGear/CM4008
#	davidm@opengear.com
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
	proc \
	sbin \
	usr usr/bin usr/sbin \
	var

ifndef CONFIG_USER_UDEV
  ROMFS_DIRS += dev/flash dev/pts
  DEVICES = \
	mem,c,1,1,0640       kmem,c,1,2,0640      null,c,1,3 \
	zero,c,1,5      random,c,1,8    urandom,c,1,9 \
	ram0,b,1,0,0660      ram1,b,1,1,0660      ram2,b,1,2,0660 \
	ram3,b,1,3,0660 \
	\
	tty,c,5,0	console,c,5,1,0600 \
	ttyS0,c,4,64,0660	ttyS1,c,4,65,0660	ttyS2,c,4,66,0660 \
	ttyS3,c,4,67,0660	ttyS4,c,4,68,0660	ttyS5,c,4,69,0660 \
	ttyS6,c,4,70,0660	ttyS7,c,4,71,0660 \
	\
	cua0,c,5,64,0660	cua1,c,5,65,0660	cua2,c,5,66,0660 \
	cua3,c,5,67,0660	cua4,c,5,68,0660	cua5,c,5,69,0660 \
	cua6,c,5,70,0660	cua7,c,5,71,0660 \
	\
	ttyAM0,c,204,16,0660	cuamm0,c,205,16,0660 \
	\
	ptyp0,c,2,0     ptyp1,c,2,1     ptyp2,c,2,2     ptyp3,c,2,3 \
	ptyp4,c,2,4     ptyp5,c,2,5     ptyp6,c,2,6     ptyp7,c,2,7 \
	ptyp8,c,2,8     ptyp9,c,2,9     ptypa,c,2,10    ptypb,c,2,11 \
	ptypc,c,2,12    ptypd,c,2,13    ptype,c,2,14    ptypf,c,2,15 \
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
	ttyp0,c,3,0     ttyp1,c,3,1     ttyp2,c,3,2     ttyp3,c,3,3 \
	ttyp4,c,3,4     ttyp5,c,3,5     ttyp6,c,3,6     ttyp7,c,3,7 \
	ttyp8,c,3,8     ttyp9,c,3,9     ttypa,c,3,10    ttypb,c,3,11 \
	ttypc,c,3,12    ttypd,c,3,13    ttype,c,3,14    ttypf,c,3,15 \
	\
	ttyq0,c,3,16    ttyq1,c,3,17    ttyq2,c,3,18    ttyq3,c,3,19 \
	ttyq4,c,3,20    ttyq5,c,3,21    ttyq6,c,3,22    ttyq7,c,3,23 \
	ttyq8,c,3,24    ttyq9,c,3,25    ttyqa,c,3,26    ttyqb,c,3,27 \
	ttyqc,c,3,28    ttyqd,c,3,29    ttyqe,c,3,30    ttyqf,c,3,31 \
	\
	ttyr0,c,3,32    ttyr1,c,3,33    ttyr2,c,3,34    ttyr3,c,3,35 \
	ttyr4,c,3,36    ttyr5,c,3,37    ttyr6,c,3,38    ttyr7,c,3,39 \
	ttyr8,c,3,40    ttyr9,c,3,41    ttyra,c,3,42    ttyrb,c,3,43 \
	ttyrc,c,3,44    ttyrd,c,3,45    ttyre,c,3,46    ttyrf,c,3,47 \
	\
	ttys0,c,3,48    ttys1,c,3,49    ttys2,c,3,50    ttys3,c,3,51 \
	ttys4,c,3,52    ttys5,c,3,53    ttys6,c,3,54    ttys7,c,3,55 \
	ttys8,c,3,56    ttys9,c,3,57    ttysa,c,3,58    ttysb,c,3,59 \
	ttysc,c,3,60    ttysd,c,3,61    ttyse,c,3,62    ttysf,c,3,63 \
	\
	mtd0,c,90,0,0640     mtdr0,c,90,1,0640    mtdblock0,b,31,0,0640 \
	mtd1,c,90,2,0640     mtdr1,c,90,3,0640    mtdblock1,b,31,1,0640 \
	mtd2,c,90,4,0640     mtdr2,c,90,5,0640    mtdblock2,b,31,2,0640 \
	mtd3,c,90,6,0640     mtdr3,c,90,7,0640    mtdblock3,b,31,3,0640 \
	mtd4,c,90,8,0640     mtdr4,c,90,9,0640    mtdblock4,b,31,4,0640 \
	mtd5,c,90,10,0640    mtdr5,c,90,11,0640   mtdblock5,b,31,5,0640 \
	mtd6,c,90,12,0640    mtdr6,c,90,13,0640   mtdblock6,b,31,6,0640 \
	mtd7,c,90,14,0640    mtdr7,c,90,15,0640   mtdblock7,b,31,7,0640 \
	mtd8,c,90,16,0640    mtdr8,c,90,17,0640   mtdblock8,b,31,8,0640 \
	mtd9,c,90,18,0640    mtdr9,c,90,19,0640   mtdblock9,b,31,9,0640 \
	\
	watchdog,c,10,130 \
	ipsec,c,36,10 \
	ledman,c,126,0 \
	rtc,c,10,135,0660 \
	ppp,c,108,0,0600
endif # CONFIG_USER_UDEV

FLASH_DEVICES = \
	boot,c,90,0 \
	ethmac,c,90,0 \
	bootarg,c,90,0 \
	config,c,90,2,0640 \
	configblock,b,31,1,0640 \
	image,c,90,4 \
	all,c,90,6,0640

all:

clean:
	-rm -f mkcramfs mksquashfs7z

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
endif
	$(ROMFSINST) ../romfs /
	make -f ../common.mk romfs

mkcramfs: $(ROOTDIR)/user/cramfs/mkcramfs.c
	$(HOSTCC) -o $@ -I$(ROOTDIR)/$(LINUXDIR)/include $< -lz

mksquashfs7z:
	make -C $(ROOTDIR)/user/squashfs/squashfs-tools mksquashfs7z
	ln -fs $(ROOTDIR)/user/squashfs/squashfs-tools/mksquashfs7z .

image: mksquashfs7z
	[ -d $(IMAGEDIR) ] || mkdir -p $(IMAGEDIR)
	-$(STRIP) $(ROMFSDIR)/bin/*
	-$(STRIP) $(ROMFSDIR)/sbin/*
	-$(STRIP) $(ROMFSDIR)/home/httpd/cgi-bin/*
	rm -rf $(ROMFSDIR)/man[1-9]
	chmod a+rx $(ROMFSDIR)/bin/* || /bin/true
	chmod a+rx $(ROMFSDIR)/etc/rc* $(ROMFSDIR)/etc/ifup*
	rm -rf $(ROMFSDIR)/share
	cp $(ROOTDIR)/$(LINUXDIR)/arch/arm/boot/zImage $(KERNELZ)
	-rm $(RAMDISK)
	MKSQUASHFS7Z=`pwd`/mksquashfs7z ; cd $(ROMFSDIR) ; \
	$$MKSQUASHFS7Z . $(RAMDISK) -all-root -noappend -le 
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

