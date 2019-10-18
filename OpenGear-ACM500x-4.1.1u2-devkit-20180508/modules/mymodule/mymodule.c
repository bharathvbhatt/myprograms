/*****************************************************************************/
/*
 * Example module for external inclusion in the uClinux-dist
 * davidm@snapgear.com
 */

#include <linux/module.h>
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/kernel.h>
#include <linux/init.h>

/*****************************************************************************/

static int __init
mymodule_init(void)
{
	printk("My Module init\n");
	return(0);
}

void mymodule_fini(void)
{
	printk("My Module fini\n");
}

/*****************************************************************************/

module_init(mymodule_init);
module_exit(mymodule_fini);

/*****************************************************************************/
