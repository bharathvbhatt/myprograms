cmd_/home/build/opengear/linux-3.x/usr/include/linux/isdn/.install := /bin/sh scripts/headers_install.sh /home/build/opengear/linux-3.x/usr/include/linux/isdn   /home/build/opengear/linux-3.x/include/uapi/linux/isdn/capicmd.h ; for F in ; do echo "\#include <asm-generic/$$F>" > /home/build/opengear/linux-3.x/usr/include/linux/isdn/$$F; done; touch /home/build/opengear/linux-3.x/usr/include/linux/isdn/.install