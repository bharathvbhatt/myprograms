/*****************************************************************************/
/*
 * Hack the KSZ8692 Feature register
 * davidm@opengear.com
 *
 * power down/clock off most things in the core (USB/WAN/UART1-3)
 * can still turn off SPI and SDIO interface with some more thought
 * insmod /tmp/kszfeat.ko kszfeat_or=0x31cab
 * insmod /tmp/kszfeat.ko kszfeat_or=0x37caf
 *
 * CPU @ 166MHz
 * insmod /tmp/kszfeat.ko kszfeat_reg=4 kszfeat_and=0xff8f kszfeat_or=0x20
 *
 * PHY off
 * insmod /tmp/kszfeat.ko kszfeat_phyoff=1
 *
 * turn off LEDs in ledman next
 * turn spi stuff
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <mach/platform.h>

/*****************************************************************************/

unsigned int kszfeat_and = ~0;
module_param(kszfeat_and, int, 0444);
MODULE_PARM_DESC(kszfeat_and, "Keep only the bits set in the parameter (&=).");

unsigned int kszfeat_or = 0;
module_param(kszfeat_or, int, 0444);
MODULE_PARM_DESC(kszfeat_or, "Add in the bits set in the parameter (+=).");

unsigned int kszfeat_reg = KS8692_FEATURE_INTER_CFG;
module_param(kszfeat_reg, int, 0444);
MODULE_PARM_DESC(kszfeat_or, "Which register to change (default feature reg).");

unsigned int kszfeat_phyoff = 0;
module_param(kszfeat_phyoff, int, 0444);
MODULE_PARM_DESC(kszfeat_phyoff, "Turn off phy if 1 turn on if -1");

unsigned int kszfeat_ddroff = 0;
module_param(kszfeat_phyoff, int, 0444);
MODULE_PARM_DESC(kszfeat_phyoff, "Turn off ddr if 1 turn on if -1");

/*****************************************************************************/

static int __init
kszfeat_init(void)
{
	register int kszfeat = KS8692_READ_REG( kszfeat_reg );
	printk("KSZ8692 reg %d before  = 0x%08x\n", kszfeat_reg, kszfeat);
	kszfeat &= kszfeat_and;
	kszfeat |= kszfeat_or;
	KS8692_WRITE_REG( kszfeat_reg, kszfeat );
	kszfeat = KS8692_READ_REG( kszfeat_reg );
	printk("KSZ8692 reg %d after  = 0x%08x\n", kszfeat_reg, kszfeat);

	if (kszfeat_phyoff) {
		kszfeat = KS8692_READ_REG( KS8692_STA_CONF );
		printk("KSZ8692 reg %d before  = 0x%08x\n", KS8692_STA_CONF, kszfeat);
		if (kszfeat_phyoff > 0)
			kszfeat |= STA_MDC_SHUT_DOWN;
		else
			kszfeat &= ~STA_MDC_SHUT_DOWN;
		KS8692_WRITE_REG( KS8692_STA_CONF, kszfeat );
		kszfeat = KS8692_READ_REG( KS8692_STA_CONF );
		printk("KSZ8692 reg %d after  = 0x%08x\n", KS8692_STA_CONF, kszfeat);
	}

	if (kszfeat_ddroff) {
		register pass = 0;
		cli();
again:
		kszfeat = KS8692_READ_REG( KS8692_DDR_CTL_21 );
		if (kszfeat_ddroff > 0)
			kszfeat |= pass ? (DDR_POWER_DOWN | DDR_SREFRESH) : 0;
		else
			kszfeat &= pass ? ~(DDR_POWER_DOWN | DDR_SREFRESH) : ~0;
		KS8692_WRITE_REG( KS8692_DDR_CTL_21, kszfeat );
		kszfeat = KS8692_READ_REG( KS8692_DDR_CTL_21 );
		printk("KSZ8692 reg %d after  = 0x%08x\n", KS8692_STA_CONF, kszfeat);
		sti();
	}


	return -EINVAL;
}

void kszfeat_fini(void)
{
	printk("KSZ8692 Feature Driver unloaded\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David McCullough <david.mccullough@opengear.com>");
MODULE_DESCRIPTION("KSZ8692 Feature hacking driver");

module_init(kszfeat_init);
module_exit(kszfeat_fini);

/*****************************************************************************/

