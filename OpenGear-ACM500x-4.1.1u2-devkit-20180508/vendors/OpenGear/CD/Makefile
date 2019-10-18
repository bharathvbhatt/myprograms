#############################################################################
#
#	Makefile -- Build instructions for OpenGear/CD
#	David McCullough <davidm@opengear.com>
#
#############################################################################

include $(LINUX_CONFIG)
include $(CONFIG_CONFIG)
include $(ARCH_CONFIG)

#############################################################################

LINUXFILE = bzImage
LINUXIMG  = $(ROOTDIR)/$(LINUXDIR)/arch/i386/boot/$(LINUXFILE)

VENDDIR    = $(ROOTDIR)/vendors/$(CONFIG_VENDOR)/$(CONFIG_PRODUCT)/.

# the number of 512 byte blocks in config partition (4096 = 2Mb)
CONFIG     = 4096

KERNELZ    = $(LINUXFILE)
RAMDISKZ   = cramfs.img.gz
ISOFILE    = $(IMAGEDIR)/CD.iso
ISODIR     = $(ROOTDIR)/iso

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
	usr usr/bin usr/sbin \
	var var/run var/log var/tmp var/lock var/empty

ifndef CONFIG_USER_UDEV
  ROMFS_DIRS += dev/flash dev/pts
  DEVICES = \
	tty,c,5,0      console,c,5,1      cua0,c,5,64      cua1,c,5,65  \
	mem,c,1,1      kmem,c,1,2         null,c,1,3 \
	ram0,b,1,0     ram1,b,1,1 \
	ram2,b,1,2     ram3,b,1,3 \
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
endif # CONFIG_USER_UDEV

FLASH_DEVICES = \
	image,b,22,0 \
	config,b,22,2

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
		touch $(ROMFSDIR)/dev/@$$i; \
	done
	port=1; while [ $$port -le 64 ]; do \
		pname=`printf "port%02d" $$port`; \
		$(ROMFSINST) -s /var/dev/$$pname /dev/$$pname; \
		port=`expr $$port + 1`; \
	done
	$(ROMFSINST) -s /var/dev/sercon /dev/sercon
	# these permissions are needed for openpty and family to work
	# on non-ptmx ptys
	chmod 620 $(ROMFSDIR)/dev/@[pt]ty[pqrsPQRS][0-9a-f],*
	chmod 666 $(ROMFSDIR)/dev/@tty,*

	# Make sure regular users get write access to the bit bucket
	chmod 666 $(ROMFSDIR)/dev/@null,*

	for i in $(FLASH_DEVICES); do \
		touch $(ROMFSDIR)/dev/flash/@$$i; \
	done
	$(ROMFSINST) -s /var/tmp/log /dev/log
endif
	$(ROMFSINST) $(LINUXIMG) /boot/vmlinuz
	$(ROMFSINST) -s /var/tmp /tmp
	$(ROMFSINST) -s /etc/config/passwd /etc/passwd
	$(ROMFSINST) -s /etc/config/group /etc/group
	$(ROMFSINST) -s /etc/config/ntpd.conf /etc/ntpd.conf
	$(ROMFSINST) -s /etc/config/TZ /etc/TZ
	$(ROMFSINST) ../romfs /
	$(ROMFSINST) -A "inet:" -a "inet:unknown:/bin/inetd" /etc/inittab
	$(ROMFSINST) -A "fltd:" -a "fltd:unknown:/bin/flatfsd" /etc/inittab
	$(ROMFSINST) -A "tty1:" -a "tty1:vt100:/bin/agetty /dev/tty1 9600" /etc/inittab
	$(ROMFSINST) -A "tty2:" -a "tty2:vt100:/bin/agetty /dev/tty2 9600" /etc/inittab
	$(ROMFSINST) -A "tty3:" -a "tty3:vt100:/bin/agetty /dev/tty3 9600" /etc/inittab
	> $(ROMFSDIR)/etc/default/inittab
	echo "$(VERSIONSTR) -- " `date` > $(ROMFSDIR)/etc/version

#############################################################################
#
# tool we need for build host based versions of the programs
#

mkcramfs: $(ROOTDIR)/user/cramfs/mkcramfs.c
	$(HOSTCC) -o $@ -I$(ROOTDIR)/$(LINUXDIR)/include $< -lz

LILO_CFLAGS = -I$(ROOTDIR)/user/lilo `cat $(ROOTDIR)/user/lilo/mylilo.h` -DLCF_IGNORECASE -DHAS_BOOT_H -DLCF_M386 -DLCF_READONLY -DLCF_VARSETUP

LILOSOURCE = lilo.c map.c geometry.c boot.c device.c common.c bsect.c cfg.c \
	temp.c partition.c identify.c probe.c shs2.c

.PHONY: lilo
lilo:
	gcc $(LILO_CFLAGS) -o $@ $(patsubst %.c,$(ROOTDIR)/user/lilo/%.c,$(LILOSOURCE))

#############################################################################
#
# re copy the kernel so that "make linux image" works
#

image: $(LOCAL_TARGET) mkcramfs
	[ -d $(IMAGEDIR) ] || mkdir -p $(IMAGEDIR)
#	-$(STRIP) $(ROMFSDIR)/bin/*
#	-$(STRIP) $(ROMFSDIR)/sbin/*
#	-$(STRIP) $(ROMFSDIR)/home/httpd/cgi-bin/*
#	rm -rf $(ROMFSDIR)/man[1-9]
#	./mkcramfs -z -r $(ROMFSDIR) cramfs.img
#	gzip -9 -f -c cramfs.img > $(RAMDISKZ) 
#	mkdir -p $(ISODIR)/boot/isolinux || true
	mkdir -p $(ISODIR)/www || true
#	mkdir -p $(ISODIR)/demo || true
#	cp ../CD/isolinux.bin $(ISODIR)/boot/isolinux/
#	cp boot.msg $(ISODIR)/boot/isolinux/
#	cp ../CD/isolinux.cfg $(ISODIR)/boot/isolinux/
#	cp $(LINUXIMG) $(ISODIR)/boot/isolinux/
#	cp $(RAMDISKZ) $(ISODIR)/boot/isolinux/initrd.gz
	:
	: mirror the website
	:
	cd $(ISODIR)/www; wget_1.12 --cache=off -pknH --restrict-file-names=windows --exclude-directories=firmware \
			--no-check-certificate \
			--mirror  http://www.opengear.com/
	:
	: mirror the manuals from the ftp site
	:
	cd $(ISODIR)/www; wget_1.12 --cache=off -pknH --restrict-file-names=windows \
			--no-check-certificate \
			--mirror  ftp://ftp.opengear.com/manual/
	:
	: mirror the SDT Connector downloads.
	:
	cd $(ISODIR)/www; wget_1.12 --cache=off -pknH --restrict-file-names=windows \
			--no-check-certificate \
			--mirror  ftp://ftp.opengear.com/sdt-connector/
	:
	: mirror the SDT for Nagios downloads.
	:
	cd $(ISODIR)/www; wget_1.12 --cache=off -pknH --restrict-file-names=windows \
			--no-check-certificate \
			--mirror  ftp://ftp.opengear.com/sdt-for-nagios/
#	:
#	: mirror the demo webpages
#	:
#	rm -f $(ISODIR)/demo/css/management.css
#	cd $(ISODIR)/demo; wget --cache=off -k -K -E --http-user=demo --http-passwd=demo \
			--restrict-file-names=windows --no-check-certificate \
			-nH --mirror https://cvs.opengear.com/cgi-bin/index.cgi
#	cd $(ISODIR)/demo; wget --cache=off -k -K -E --http-user=demo --http-passwd=demo \
			--restrict-file-names=windows --no-check-certificate \
			-nH --mirror https://cvs.opengear.com/cgi-bin/terminal.jar
#	cd $(ISODIR)/demo; grep /pixmap css/management.css | \
			sed 's|.*\(/pixmaps.*\.png\).*|https://cvs.opengear.com\1|' | \
			xargs wget_1.12 --cache=off -pknH --mirror -k -K -E --http-user=demo --http-passwd=demo --no-check-certificate
#	sed 's|/pixmaps|../pixmaps|g' < $(ISODIR)/demo/css/management.css > \
			$(ISODIR)/demo/css/management.css.fixed
	:
	: Change manual links to point to local directory
	:
	for i in $(ISODIR)/www/*.html; do \
		sed 's/ftp:\/\/ftp\.opengear\.com\/manual/manual/g' < $$i > $$i.fixed; \
		mv $$i.fixed $$i; \
	done
	:
	: Change SDT Connector links to point to local directory
	:
	for i in $(ISODIR)/www/*.html; do \
		sed 's/ftp:\/\/ftp\.opengear\.com\/sdt-connector/sdt-connector/g' < $$i > $$i.fixed; \
		mv $$i.fixed $$i; \
	done
	:
	: Change SDT for Nagios links to point to local directory
	:
	for i in $(ISODIR)/www/*.html; do \
		sed 's/ftp:\/\/ftp\.opengear\.com\/sdt-for-nagios/sdt-for-nagios/g' < $$i > $$i.fixed; \
		mv $$i.fixed $$i; \
	done
#	:
#	: remove submit buttons and make them reset buttons
#	:
#	for i in $(ISODIR)/demo/cgi-bin/*.html*; do \
		sed 's/type="submit"/type="reset"/g' < $$i > $$i.fixed; \
		mv $$i.fixed $$i; \
	done
#	:
#	: remove sensitive data from support report output
#	:
#	for i in $(ISODIR)/demo/cgi-bin/*form=support*; do \
		sed '/^[a-z]/d' < $$i > $$i.fixed; \
		mv $$i.fixed $$i; \
#	done
#	mv $(ISODIR)/demo/css/management.css.fixed $(ISODIR)/demo/css/management.css
	cp ../CD/AutorunPro.EXE ../CD/autorun.inf ../CD/index.html ../CD/banner.png $(ISODIR)/.
	:
	: make the filesystem
	:
	mkisofs -r -joliet-long -o $(ISOFILE) $(ISODIR)
#	mkisofs -r -joliet-long -o $(ISOFILE) -b boot/isolinux/isolinux.bin \
		-c boot/isolinux/boot.cat -no-emul-boot \
		-boot-load-size 4 -boot-info-table $(ISODIR)


#############################################################################
