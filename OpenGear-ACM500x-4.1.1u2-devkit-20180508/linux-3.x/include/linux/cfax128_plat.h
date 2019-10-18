#ifndef _LINUX_CFAX128_PLAT_H
#define _LINUX_CFAX128_PLAT_H

#include <linux/types.h>
#include <linux/nt7534.h>

struct cfax128_platform_data {
	struct nt7534_chip 	ctrl_chip;
	struct fb_info 		*info;
};

#endif /* _LINUX_CFAX128_PLAT_H */

