
obj-y += rt2860.o

EXTRA_CFLAGS += -DAGGREGATION_SUPPORT -DPIGGYBACK_SUPPORT -DWMM_SUPPORT -DLINUX
EXTRA_CFLAGS += -DSYSTEM_LOG_SUPPORT
EXTRA_CFLAGS += -DCONFIG_STA_SUPPORT
EXTRA_CFLAGS += -DRTMP_MAC_PCI -DRTMP_PCI_SUPPORT -DRT2860 -DRT28xx
EXTRA_CFLAGS += -DDOT11_N_SUPPORT
EXTRA_CFLAGS += -DDOT11N_DRAFT3
EXTRA_CFLAGS += -DSTATS_COUNT_SUPPORT
EXTRA_CFLAGS += -DWPA_SUPPLICANT_SUPPORT
EXTRA_CFLAGS += -DNATIVE_WPA_SUPPLICANT_SUPPORT
EXTRA_CFLAGS += -I$(ROOTDIR)/modules/ralink/rt2860/include

EXTRA_CFLAGS += -DINF_MAIN_DEV_NAME=\"wlan\"

rt2860-objs := \
	common/crypt_md5.o \
	common/crypt_sha2.o \
	common/crypt_hmac.o \
	common/crypt_aes.o \
	common/crypt_arc4.o \
	common/mlme.o \
	common/cmm_wep.o \
	common/action.o \
	common/cmm_data.o \
	common/rtmp_init.o \
	common/cmm_tkip.o \
	common/cmm_aes.o \
	common/cmm_sync.o \
	common/eeprom.o \
	common/cmm_sanity.o \
	common/cmm_info.o \
	common/cmm_cfg.o \
	common/cmm_wpa.o \
	common/dfs.o \
	common/spectrum.o \
	common/rtmp_timer.o \
	common/rt_channel.o \
	common/cmm_profile.o \
	common/cmm_asic.o \
	common/cmm_cmd.o \
	sta/assoc.o \
	sta/auth.o \
	sta/auth_rsp.o \
	sta/sync.o \
	sta/sanity.o \
	sta/rtmp_data.o \
	sta/connect.o \
	sta/wpa.o \
	sta/sta_cfg.o

rt2860-objs += \
	common/rtmp_init_inf.o \
	os/linux/rt_profile.o \
	os/linux/sta_ioctl.o \
	os/linux/rt_linux.o \
	os/linux/rt_main_dev.o

rt2860-objs += \
	common/ba_action.o

rt2860-objs += \
	common/cmm_mac_pci.o \
	common/cmm_data_pci.o \
	common/ee_prom.o \
	common/rtmp_mcu.o

rt2860-objs += \
	os/linux/rt_pci_rbus.o \
	os/linux/rt_rbus_pci_util.o \
	os/linux/pci_main_dev.o

obj-m  := $(obj-y) $(obj-m)
M_OBJS := $(obj-y) $(obj-m)
obj-y  :=

ifdef TOPDIR
-include $(TOPDIR)/Rules.make
endif

rt2860.o: $(rt2860-objs)
	$(LD) -r -o rt2860.o $(rt2860-objs)

.PHONY: clean mrproper distclean
clean:
	rm -f $(obj-m)
mrproper: clean
	rm -f .depend
distclean: mrproper

