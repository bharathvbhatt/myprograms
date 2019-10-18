include $(ROOTDIR)/modules/.config
EXTRA_CFLAGS += -I$(ROOTDIR)/$(LINUXDIR)/include/linux

obj-y += ax88172a.o

EXTRA_CFLAGS += -I$(ROOTDIR)/modules/asix/ax88172a/include -DEXPORT_SYMTAB

ifdef TOPDIR
-include $(TOPDIR)/Rules.make
endif

.PHONY: clean mrproper distclean
clean:
	rm -f $(obj-m) $(obj-y)
mrproper: clean
	rm -f .depend
distclean: mrproper

