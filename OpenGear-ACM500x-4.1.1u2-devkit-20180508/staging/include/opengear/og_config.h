#ifndef _OG_CONFIG
#define _OG_CONFIG

#include <vendor/autoconf.h>
#include <config/autoconf.h>
#include <linux/autoconf.h>
/* Opengear Configuration Header, use this for feature detection */

/* Product definitions */
#ifdef CONFIG_MACH_ACM550X
#define PRODUCT_ACM550x 1
#define HAVE_IOPORTS 1
#define HAVE_INTERNAL_TEMP 1
#define HAVE_RS485 1
#define MAX_NUM_ETH 2
#define HAVE_USB 1
#define HAVE_WIFI_AP_UI 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_IM72xx
#define PRODUCT_IM72xx 1
#define HAVE_SFP 1
#define MAX_NUM_ETH 2
#define HAVE_USB 1
#define HAVE_WIFI_AP_UI 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_SD_CARD 1
#define HAVE_FAILOVER 1
#define HAVE_GIGABIT 1
#endif

#ifdef CONFIG_DEFAULTS_MIOVISION_TRD
#define PRODUCT_TRD 1
#define MAX_NUM_ETH 1
#define HAVE_IOPORTS 1
#define HAVE_INTERNAL_TEMP 1
#define HAVE_USB 1
#define HAVE_SERIAL 1
#define HAVE_WIFI_AP_UI 1
#define HAVE_FAILOVER 1
#endif

#if defined(CONFIG_DEFAULTS_OPENGEAR_CM71XX) || defined(CONFIG_DEFAULTS_BLACKBOX_LES15XXA)
#define PRODUCT_CM71xx 1
#define MAX_NUM_ETH 2
#define HAVE_USB 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_SD_CARD 1
#define HAVE_FAILOVER 1
#define HAVE_GIGABIT 1
#endif

#if defined(CONFIG_DEFAULTS_OPENGEAR_CM7196)
#define PRODUCT_CM7196 1
#define MAX_NUM_ETH 2
#define HAVE_SFP 1
#define HAVE_INTERNAL_TEMP 1
#define HAVE_USB 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_FAILOVER 1
#define HAVE_GIGABIT 1
#endif

#if defined(CONFIG_DEFAULTS_OPENGEAR_ACM700X) || defined (CONFIG_DEFAULTS_OPENGEAR_ACM7004_5) || defined(CONFIG_DEFAULTS_BLACKBOX_LES160XA) || defined (CONFIG_DEFAULTS_TRIPPLITE_B093)
#define PRODUCT_ACM700x 1
#define MAX_NUM_ETH 2
#define HAVE_IOPORTS 1
#define HAVE_INTERNAL_TEMP 1
#define HAVE_USB 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_WIFI_AP_UI 1
#define HAVE_FAILOVER 1
#define HAVE_GIGABIT 1
#define HAVE_NAND_FLASH 1
#endif

#if defined (CONFIG_DEFAULTS_OPENGEAR_ACM7004_5)
#define PRODUCT_ACM7004_5 1
#endif

#ifdef CONFIG_MACH_ACM500X
#define PRODUCT_ACM500x
#define HAVE_IOPORTS 1
#define HAVE_INTERNAL_TEMP 1
#define HAVE_RS485 1
#define MAX_NUM_ETH 2
#define HAVE_USB 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_CM41xx
#define PRODUCT_CM41XX 1
#define MAX_NUM_ETH 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_SERIAL 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_IM42xx
#define PRODUCT_IM42XX 1
#define MAX_NUM_ETH 3
#define HAVE_USB 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_IM4004
#define PRODUCT_IM4004 1
#define MAX_NUM_ETH 2
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_SERIAL 1
#define HAVE_USB 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define HAVE_FAILOVER 1
#endif

/* Detect KCS via UCD stuff */
#ifdef CONFIG_PROP_UCD
#define PRODUCT_KCS61XX 1
#define MAX_NUM_ETH 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_USB 1
#define HAVE_SERIAL 1
#define HAVE_FAILOVER 1
#endif

/* Detect CMS via something else */
#ifdef CONFIG_PROP_CMS
#define PRODUCT_CMS 1
#define HAVE_CMS 1
#define HAVE_BROKEN_MULTICAST 1
#define WANT_FLASH_UPGRADE 1

#ifdef CONFIG_PRODUCT_LIGHTHOUSE
	#define MAX_NUM_ETH 2
	#define HAVE_BRIDGING 1
	#define HAVE_BONDING 1
#else
	#define MAX_NUM_ETH 1
#endif

#endif

/* Otherwise default to VACM */
#if defined(CONFIG_X86) && !defined(PRODUCT_CMS) && !defined(PRODUCT_KCS61XX)
#define PRODUCT_ACM500x
#define HAVE_IOPORTS 1
#define HAVE_INTERNAL_TEMP 1
#define HAVE_RS485 1
#define MAX_NUM_ETH 2
#define HAVE_USB 1
#define HAVE_SERIAL 1
#define HAVE_BRIDGING 1
#define HAVE_BONDING 1
#define WANT_FLASH_UPGRADE 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_CM4008
#define PRODUCT_CM4008 1
#define MAX_NUM_ETH 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_SERIAL 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_SD4008
#define PRODUCT_SD4008 1
#define HAVE_RS485 1
#define MAX_NUM_ETH 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_SERIAL 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_CM4002
#define PRODUCT_CM4002 1
#define MAX_NUM_ETH 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_SERIAL 1
#define HAVE_FAILOVER 1
#endif

#ifdef CONFIG_MACH_SD4002
#define PRODUCT_SD4002 1
#define PRODUCT_SD4001 1
#define MAX_NUM_ETH 1
#define HAVE_RS485 1
#define HAVE_BROKEN_MULTICAST 1
#define HAVE_SERIAL 1
#define HAVE_FAILOVER 1
#endif

/* Feature Definitions */
#ifdef HAVE_USB
/* 3G Support */
#ifdef CONFIG_LIB_SIERRA
#define HAVE_SIERRA 1
#define HAVE_CELLMODEM 1
#endif /* CONFIG_LIB_SIERRA */

/* USB Storage */
#ifdef CONFIG_USB_STORAGE
#define HAVE_USB_STORAGE 1
/* FTPD */
#ifdef CONFIG_USER_FTPD_FTPD
#define HAVE_FTPD 1
#endif
/* TFTPD */
#ifdef CONFIG_USER_TFTP_HPA
#define HAVE_TFTPD 1
#endif

#endif /* CONFIG_USB_STORAGE */

/* WIFI */
#if defined(CONFIG_USER_IW) || defined(CONFIG_USER_WIRELESS_TOOLS_IWPRIV)
#define HAVE_WIFI 1

#if defined(CONFIG_USER_HOSTAPD)
#define HAVE_WIFI_AP 1
#endif

#endif /* CONFIG_USER IW || CONFIG_USER_WIRELESS_TOOLS_IWPRIV */
#endif /* HAVE_USB */

/* KCS Features */
#ifdef CONFIG_PROP_UCD
#define HAVE_VNC 1
#define WANT_FLASH_UPGRADE 1
#endif


/* Software Features */
#ifdef CONFIG_LIB_OPENSSL_FIPS
#define OPENGEAR_FIPS 1
#endif

#ifdef CONFIG_USER_NUT
#define HAVE_NUT 1
#ifndef HAVE_CMS
#define HAVE_CGI_CONFIGURABLE_NUT 1
#endif
#endif

#ifdef CONFIG_USER_NUT_WITH_OTHERS
#define ALL_NUT_DRIVERS 1
#endif

#ifdef CONFIG_USER_PAM_KRB5
#define HAVE_KRB5 1
#endif

#ifdef CONFIG_PROP_POWERALERT
#define HAVE_POWERALERT 1
#endif

#ifdef CONFIG_PROP_EMD_EMD
#define HAVE_EMD 1

#ifdef CONFIG_PROP_EMD_KS8692
#define HAVE_EMD_KS8692 1
#endif

#ifdef CONFIG_PROP_EMD_EXTERNAL
#define HAVE_EMD_EXTERNAL 1
#endif

#endif /* CONFIG_PROP_EMD_EMD */

#ifdef CONFIG_USER_DHCP_ISC_SERVER_DHCPD
#define HAVE_DHCPD 1
#endif

#ifdef CONFIG_USER_DNSMASQ2_DNSMASQ2
#define HAVE_DNSMASQ 1
#endif

#ifdef CONFIG_USER_ZIP_ZIP
#define HAVE_ZIP 1
#endif

#ifdef CONFIG_USER_PPTPD_PPTPD
#define HAVE_PPTPD 1
#endif

#ifdef CONFIG_USER_PPPD_WITH_RADIUS
#define HAVE_PPPD_WITH_RADIUS 1
#endif

#ifdef CONFIG_USER_OPENSWAN
#define HAVE_IPSEC 1
#endif

#ifdef CONFIG_USER_OPENVPN_OPENVPN
#define HAVE_OPENVPN 1
#endif

#ifdef CONFIG_USER_BUSYBOX_FEATURE_SHADOWPASSWDS
#define HAVE_SHADOW 1
#endif

#ifdef CONFIG_PROP_PSMON
#define HAVE_PSMON 1
#endif

#ifdef CONFIG_PROP_RPCD
#define HAVE_RPCD 1
#endif

#ifdef CONFIG_USER_CURL_CURL
#define HAVE_CURL 1
#endif

#ifdef CONFIG_USER_EZIPUPDATE_EZIPUPDATE
#define DYNDNS 1
#define HAVE_DYNDNS 1
#endif

#ifdef CONFIG_USER_LLDPD
#define HAVE_LLDP 1
#endif

/* SNMPD */
#if defined(CONFIG_USER_NETSNMP_AGENT_SNMPD) || defined(CONFIG_USER_NETSNMP_NETSNMP) || defined(CONFIG_USER_NETSNMP_SNMPD) \
	|| defined(CONFIG_USER_SNMPD_SNMPD)
#define HAVE_SNMPD 1
#ifndef HAVE_CMS
#define HAVE_CGI_CONFIGURABLE_SNMPD 1
#endif
#endif

#ifdef CONFIG_PROP_CGI_RECOVERY
#define RECOVERY_IMAGE 1
#endif

/* Services/features on the box: allow them to be CGI-configurable, unless it's on CMS */
#ifdef CONFIG_USER_TELNETD_TELNETD
#ifndef HAVE_CMS
#define HAVE_CGI_CONFIGURABLE_TELNETD 1
#endif
#endif

#ifdef CONFIG_PROP_NAGIOS
#ifndef HAVE_CMS
#define HAVE_CGI_CONFIGURABLE_NAGIOS 1
#endif
#endif

#ifndef HAVE_CMS
#define HAVE_CGI_CONFIGURABLE_SERIAL_PORTBASES 1
#endif

#ifdef CONFIG_PROP_OPENGEAR_SERIAL_CLIENT
#ifdef HAVE_CMS
#define HAVE_RFC2217_DIALER 1
#endif
#endif

// Dialpool support. Currently only on CMS.
#ifdef HAVE_CMS
#define HAVE_DIALPOOL 1

#ifdef CONFIG_PRODUCT_LIGHTHOUSE
//Periodic health checking of the HAVE_DIALPOOL stuff
#define HAVE_DIALPOOL_HEALTH_TEST 1

//Serial concentrator support in portmanager for
//connecting to remote cms nodes's serial ports
#define HAVE_SERIAL_CONCENTRATOR 1
#endif

#endif

// Cherokee Webserver
#ifdef CONFIG_USER_CHEROKEE
#define HAVE_CHEROKEE
#endif

// Rebrand
#ifdef CONFIG_DEFAULTS_BLACKBOX
#define HAVE_REBRAND_BLACKBOX
#endif

#ifdef CONFIG_DEFAULTS_TRIPPLITE
#define HAVE_REBRAND_TRIPPLITE
#endif

#ifdef CONFIG_DEFAULTS_BLACKBOX
#define CMS_LABEL "VCMS"
#else
#define CMS_LABEL "Lighthouse"
#endif

#endif /* _OG_CONFIG */
