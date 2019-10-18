CURRENT = $(shell uname -r)
TARGET = ax88172a
OBJS = ax88172a.o
MDIR = drivers/net/usb

ifneq (,$(findstring 2.6.22,$(CURRENT)))
MDIR = drivers/net/usb
else
MDIR = drivers/usb/net
endif

EXTRA_CFLAGS = -DEXPORT_SYMTAB
KDIR = /lib/modules/$(CURRENT)/build
PWD = $(shell pwd)
DEST = /lib/modules/$(CURRENT)/kernel/$(MDIR)

obj-m      := $(TARGET).o

default:
	make -C $(KDIR) SUBDIRS=$(PWD) modules
	gcc -pthread ioctl.c -o ioctl

$(TARGET).o: $(OBJS)
	$(LD) $(LD_RFLAG) -r -o $@ $(OBJS)

install:
	su -c "cp -v $(TARGET).ko $(DEST) && /sbin/depmod -a"

clean:
	-rm -f *.o *.ko .*.cmd .*.flags *.mod.c *.symvers *.markers *.order
	-rm -rf .tmp_versions .directory

-include $(KDIR)/Rules.make
