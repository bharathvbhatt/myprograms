cmd_/home/build/opengear/linux-3.x/usr/include/uapi/.install := /bin/sh scripts/headers_install.sh /home/build/opengear/linux-3.x/usr/include/uapi  ; for F in ; do echo "\#include <asm-generic/$$F>" > /home/build/opengear/linux-3.x/usr/include/uapi/$$F; done; touch /home/build/opengear/linux-3.x/usr/include/uapi/.install