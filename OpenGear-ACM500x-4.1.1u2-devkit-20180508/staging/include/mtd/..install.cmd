cmd_/home/build/opengear/linux-3.x/usr/include/mtd/.install := /bin/sh scripts/headers_install.sh /home/build/opengear/linux-3.x/usr/include/mtd   /home/build/opengear/linux-3.x/include/uapi/mtd/inftl-user.h   /home/build/opengear/linux-3.x/include/uapi/mtd/mtd-abi.h   /home/build/opengear/linux-3.x/include/uapi/mtd/mtd-user.h   /home/build/opengear/linux-3.x/include/uapi/mtd/nftl-user.h   /home/build/opengear/linux-3.x/include/uapi/mtd/ubi-user.h ; for F in ; do echo "\#include <asm-generic/$$F>" > /home/build/opengear/linux-3.x/usr/include/mtd/$$F; done; touch /home/build/opengear/linux-3.x/usr/include/mtd/.install