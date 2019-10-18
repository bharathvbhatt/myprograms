# -*- makefile -*-
#
# 	common.mk -- common initialisation tasks on romfs for all targets
#	pchunt@opengear.com
#

.PHONY: romfs
romfs:
	$(ROMFSINST) -s /etc/config/bash_profile /.profile

	$(ROMFSINST) -s /var/tmp /tmp
ifndef CONFIG_USER_UDEV
        # ksyslogd will make a named socket at /dev/log; symlink avoids EROFS
	$(ROMFSINST) -s /var/tmp/log /dev/log
endif
	$(ROMFSINST) -s /etc/config/passwd /etc/passwd
	$(ROMFSINST) -s /etc/config/group /etc/group
	$(ROMFSINST) -s /etc/config/shadow /etc/shadow
	$(ROMFSINST) -s /etc/config/gshadow /etc/gshadow
	$(ROMFSINST) -s /etc/config/TZ /etc/TZ
	$(ROMFSINST) -s /etc/config /etc/Wireless
	$(ROMFSINST) -s /var/run /run 			# for udev
	$(ROMFSINST) -A "inet:" -a "inet:unknown:/bin/inetd" /etc/inittab
	$(ROMFSINST) -A "fltd:" -a "fltd:unknown:/bin/flatfsd" /etc/inittab

	# Setup correct permissions on /etc/ files
	chmod 660 $(ROMFSDIR)/etc/default/config.xml
	chmod 660 $(ROMFSDIR)/etc/default/pam.d/*

	# Make standard scripts executable
	find $(ROMFSDIR)/etc/scripts/ -type f -print0 | xargs -0 chmod 555

	# fix up permissions for scripts directory -- ssh doesn't like it being
	# group or world writable
	chmod g-w,o-w $(ROMFSDIR)/etc/scripts/
	chmod g-w,o-w $(ROMFSDIR)/etc/

	# Write the timestamp to /etc/version last
	echo "$(VERSIONSTR) -- " `date` > $(ROMFSDIR)/etc/version

