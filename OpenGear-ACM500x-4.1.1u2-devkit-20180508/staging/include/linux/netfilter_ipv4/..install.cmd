cmd_/home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv4/.install := /bin/sh scripts/headers_install.sh /home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv4   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ip_tables.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_CLUSTERIP.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_ECN.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_LOG.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_REJECT.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_TTL.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_ULOG.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_ah.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_ecn.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv4/ipt_ttl.h ; for F in ; do echo "\#include <asm-generic/$$F>" > /home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv4/$$F; done; touch /home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv4/.install
