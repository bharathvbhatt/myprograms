cmd_/home/build/opengear/linux-3.x/usr/include/xen/.install := /bin/sh scripts/headers_install.sh /home/build/opengear/linux-3.x/usr/include/xen   /home/build/opengear/linux-3.x/include/uapi/xen/evtchn.h   /home/build/opengear/linux-3.x/include/uapi/xen/privcmd.h ; for F in ; do echo "\#include <asm-generic/$$F>" > /home/build/opengear/linux-3.x/usr/include/xen/$$F; done; touch /home/build/opengear/linux-3.x/usr/include/xen/.install
