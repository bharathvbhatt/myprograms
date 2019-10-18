# for SGlinux builds
-include $(ROOTDIR)/modules/.config

obj-$(CONFIG_OCF_ARMADA370) += cesa_ocf.o

cesa_ocf-y := cesa/mvCesa.o cesa/mvLru.o cesa/mvMD5.o cesa/mvSHA1.o cesa/mvSHA256.o cesa/AES/mvAesAlg.o cesa/AES/mvAesApi.o cesa/mvCesaDebug.o cesa_ocf_drv.o cesa_if.o mvOsAbstract.o

obj ?= .
EXTRA_CFLAGS += -I$(obj)/.. -I$(obj)/
EXTRA_CFLAGS += -DMV_LINUX -DMV_CPU_LE -DMV_ARM -DMV_CESA_CHANNELS=1 -DMV_CESA_CHAIN_MODE -DMV_CESA_VERSION=3

ifdef TOPDIR
-include $(TOPDIR)/Rules.make
endif

