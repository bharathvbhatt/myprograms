cmd_/home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv6/.install := /bin/sh scripts/headers_install.sh /home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv6   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6_tables.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_HL.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_LOG.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_NPT.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_REJECT.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_ah.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_frag.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_hl.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_ipv6header.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_mh.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_opts.h   /home/build/opengear/linux-3.x/include/uapi/linux/netfilter_ipv6/ip6t_rt.h ; for F in ; do echo "\#include <asm-generic/$$F>" > /home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv6/$$F; done; touch /home/build/opengear/linux-3.x/usr/include/linux/netfilter_ipv6/.install
