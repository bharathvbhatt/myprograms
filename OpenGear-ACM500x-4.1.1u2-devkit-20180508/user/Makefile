#
#	Makefile -- Build instructions for user level apps
#

.EXPORT_ALL_VARIABLES:
.PHONY: all romfs clean

#
# Include architecture specific build rules.
#

ifndef ROOTDIR
ROOTDIR=..
endif

UCLINUX_BUILD_USER=1
-include $(LINUX_CONFIG)
-include $(CONFIG_CONFIG)
-include $(ARCH_CONFIG)
-include $(MODULES_CONFIG)

DLIST=$(shell ls)

# add dirs of apps to be added to romfs
RLIST= Egpgm hello Serial run Server http_file timer AIS
 
all:
	for i in $(DLIST); do \
		if [ -d "$$i" ] && [ -f "$$i"/*akefile ]; then \
			make -C $$i || exit $$? ; \
		fi; \
	done

romfs:
	for i in $(RLIST) ; do \
		[ ! -d "$$i" ] || make -C $$i romfs || exit $$? ; \
	done
	
%_only:
	touch $(@:_only=)/.sgbuilt_user && $(MAKE) -j$(HOST_NCPU) $(SUBDIR_SMP_BUILD) -C $(@:_only=)

%_clean:
	$(MAKE) -j1 -C $(@:_clean=) clean; rm -f $(@:_clean=)/.sgbuilt_user; true

%_romfs:
	$(MAKE) $(SUBDIR_SMP_BUILD) -C $(@:_romfs=) romfs

clean:
	-for i in $(DLIST); do \
		if [ -d "$$i" ]; then \
			make -C $$i clean ; \
		fi;  \
	done

