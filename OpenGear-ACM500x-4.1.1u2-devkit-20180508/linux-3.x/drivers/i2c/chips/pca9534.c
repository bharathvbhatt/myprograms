/*
 * drivers/i2c/chips/pca9534.c
 *
 * I2C client driver for the Philips 9534 I/O expander.
 *
 * (C) Copyright (C) 2009, Greg Ungerer <greg.ungerer@opengear.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <mach/platform.h>

struct trigger_mode_setting { 
	int pin;
	int trigger_mode;
};

#define PCA_NUMPINS 8
#define	PCA9534_DRV_NAME	"pca9534"
#define PCA9534_MINOR 		145
#define PCA9534_MAGIC		'p' 
#define PCA9534_MIN_MINOR 	0xF0
#define PCA9534_MAX_MINOR	0xF1
#define PCA9534_GET_DIR		_IOR(PCA9534_MAGIC, PCA9534_MIN_MINOR, int)
#define PCA9534_SET_DIR		_IOW(PCA9534_MAGIC, PCA9534_MIN_MINOR + 1, int)
#define PCA9534_GET_COUNTERS	_IOR(PCA9534_MAGIC, PCA9534_MIN_MINOR + 2, int)
#define PCA9534_SET_PIN_TRIGGER	_IOW(PCA9534_MAGIC, PCA9534_MIN_MINOR + 3, struct trigger_mode_setting)
#define PCA9534_GET_PIN_TRIGGER _IOWR(PCA9534_MAGIC, PCA9534_MIN_MINOR + 4, int)
#define PCA9534_SET_DEBOUNCE_PERIOD	_IOW(PCA9534_MAGIC, PCA9534_MIN_MINOR + 5, int)
#define PCA9534_GET_DEBOUNCE_PERIOD	_IOR(PCA9534_MAGIC, PCA9534_MIN_MINOR + 6, int)

#define PCA9534_TRIGGER_RISING_EDGE 0x1
#define PCA9534_TRIGGER_FALLING_EDGE 0x2

#ifdef CONFIG_SERIAL_OG_RS4xx
#define	PCA9534_NUM		2
#else
#define	PCA9534_NUM		1
#endif

/*
 * We keep a copy of the client device found, since we are really only
 * need one device for the real misc device interface.
 */
static struct i2c_client *client[PCA9534_NUM];
static u8 pca9534_inpins[PCA9534_NUM];
static u8 pca9534_outpins[PCA9534_NUM];
/* Pulse counter guff for main PCA */

static u64 pca9534_counters[PCA_NUMPINS];
static int pca9534_trigger_mode[PCA_NUMPINS];
static int pca9534_debounce_period;

#define DEFAULT_DEBOUNCE_MS 10
#define DEFAULT_TRIGGER_MODE (PCA9534_TRIGGER_RISING_EDGE | PCA9534_TRIGGER_FALLING_EDGE)

static u32 devs_detected = 0;
#if defined(CONFIG_MACH_ACM500X)
/*
 * On the OpenGear ACM500x platform spi and i2c devies share the
 * same physical clock and data pins. So we need to lock users of
 * these types of external devices so they don't stomp on each other.
 */
extern struct semaphore acm500x_spilocki2c;
extern int acm500x_spilocki2c_inited;

u8 pca9534_readbyte(u32 idx, u32 a)
{
	u8 v = 0xff;
	if (idx < devs_detected) {
		down(&acm500x_spilocki2c);
		v = i2c_smbus_read_byte_data(client[idx], a);
		up(&acm500x_spilocki2c);
	}
	return v;
}

void pca9534_writebyte(u32 idx, u32 a, u8 v)
{
	if (idx < devs_detected) {
		down(&acm500x_spilocki2c);
		i2c_smbus_write_byte_data(client[idx], a, v);
		up(&acm500x_spilocki2c);

		/* Keep track of the output pin state */
		if (a == 1)
			pca9534_outpins[idx] = v;
	}
}

void pca9534_setoutpin(u32 idx, u32 pin, u32 v)
{
	if ((idx < devs_detected) && (pin < 8)) {
		u8 bit = (0x1 << pin);
		u8 reg = pca9534_outpins[idx];
		if (v)
			reg |= bit;
		else
			reg &= ~bit;
		pca9534_writebyte(idx, 1, reg);
	}
}

void pca9534_setdir(u32 idx, u32 pin, u32 d) 
{
	if ((idx < devs_detected) && (pin < 8)) {
		u8 bit = (0x1 << pin);
		u8 pin_dir = pca9534_readbyte(idx, 3);
		if (d) {
			pin_dir |= bit;
		} else {
			pin_dir &= ~bit; 
		}
		pca9534_writebyte(idx, 3, pin_dir);
	}
}
#else
#define	pca9534_readbyte(idx,a)		i2c_smbus_read_byte_data(client[idx],a)
#define	pca9534_writebyte(idx,a,v)	i2c_smbus_write_byte_data(client[idx],a,v);
#endif

#if 0
static void pca9534_dumpregs(u32 idx)
{
	printk("PCA9534 registers[%d]:\n", idx);
	printk("  reg[0]=0x%x\n", pca9534_readbyte(idx, 0));
	printk("  reg[1]=0x%x\n", pca9534_readbyte(idx, 1));
	printk("  reg[2]=0x%x\n", pca9534_readbyte(idx, 2));
	printk("  reg[3]=0x%x\n", pca9534_readbyte(idx, 3));
}
#endif


/*
 *****************************************************************************
 *
 *	Interrupt handler and work queue stuff
 *
 *****************************************************************************
 */

#define	PCA9534_IRQ	(LOW_IRQS + KS8692_INT_EXT_INT0)

static void pca_do_work(struct work_struct *unused) {

	int newpins;
	int i = 0;
	if (pca9534_debounce_period > 0) {
		msleep(pca9534_debounce_period);
	}
	newpins = pca9534_readbyte(0, 0);

	for (i = 0; i < PCA_NUMPINS; i++) {
		/* Check for rising edge */
		if ((pca9534_inpins[0] & (1 << i)) != (newpins & (1 << i))) {
			if (((newpins & (1 << i)) != 0) && (pca9534_trigger_mode[i] & PCA9534_TRIGGER_RISING_EDGE)) {
				pca9534_counters[i]++;
			}
			/* Check for falling edge */
			if (((newpins & (1 << i)) == 0) && (pca9534_trigger_mode[i] & PCA9534_TRIGGER_FALLING_EDGE)) {
				pca9534_counters[i]++;
			} 
		}
	}

#if PCA9534_NUM > 1
	if (devs_detected > 1) {
		pca9534_inpins[1] = pca9534_readbyte(1, 0);
	}
#endif
#if defined(CONFIG_LEDMAN) && defined(CONFIG_MACH_ACM500X)
	if ((pca9534_inpins[0] & 0x10) && ((newpins & 0x10) == 0)) {
		irqreturn_t ledman_interrupt(int irq, void *devid);
		ledman_interrupt(PCA9534_IRQ, NULL);
	}
#endif
	pca9534_inpins[0] = newpins;
	enable_irq(PCA9534_IRQ);	
}

static DECLARE_WORK(pca_wq, pca_do_work);

irqreturn_t pca_irq_handler(int irq, void *dev) 
{
	int scheduled = schedule_work(&pca_wq);	
	
#ifdef DBG
	if (scheduled == 0) {
		printk(KERN_ERR "Work already scheduled");
	}
#else
	(void)scheduled;
#endif
	disable_irq_nosync(PCA9534_IRQ);
	return IRQ_HANDLED;
}

/*
 *****************************************************************************
 *
 *	I2C Driver Interface
 *
 *****************************************************************************
 */


static int pca9534_probe(struct i2c_client *c, const struct i2c_device_id *id)
{
	s32 detect;

	if (devs_detected >= PCA9534_NUM)
		return -ENODEV;

	/* We should actually do a quick read here,
	   to make sure that the device exists 
	 */
#ifdef CONFIG_MACH_ACM500X
	down(&acm500x_spilocki2c);
#endif
	detect = i2c_smbus_read_byte_data(c, 2);
#ifdef CONFIG_MACH_ACM500X
	up(&acm500x_spilocki2c);
#endif
	if (detect < 0) {
		/* No device found */
		return -ENODEV;
	}
	
	client[devs_detected] = c;
	devs_detected++;
	/* Second 9534 pins control RS232/422 port modes on ACM5004-i */
	if (devs_detected == 2) {
		printk("PCA9534: mapping RS232/422 control lines\n");
		pca9534_writebyte(1, 1, 0x00);
		pca9534_writebyte(1, 3, 0xf0);
	}
	
	if (devs_detected == 1) {
		int i;
		printk("PCA9534: I2C I/O expander driver registered\n");
		/* Setup the pulse counter defaults */
		pca9534_debounce_period = DEFAULT_DEBOUNCE_MS;
		for (i = 0; i < PCA_NUMPINS; i++) {
			pca9534_trigger_mode[i] = DEFAULT_TRIGGER_MODE;
		}
		memset(pca9534_counters, '\0', sizeof(u64) * PCA_NUMPINS);
		/* Get the initial state, then set us up for interrupts */
		pca9534_inpins[0] = pca9534_readbyte(0, 0);
		printk("Requesting interrupt %d\n", PCA9534_IRQ);
		return request_irq(PCA9534_IRQ, pca_irq_handler, 0, "PCA9534 ExtIO", NULL);
	}

	return 0;
}

/*
 *****************************************************************************
 *
 *	Char/Misc Driver Interface
 *
 *****************************************************************************
 */

static ssize_t pca9534_read(struct file *fp, char __user *buf, size_t count, loff_t *ptr)
{
	ssize_t total = 0;

	if (! client[0])
		return -ENODEV;

	put_user(pca9534_inpins[0], buf++);
	*ptr += 1;
	total++;

#if PCA9534_NUM > 1
	if (count > 1 && (devs_detected > 1)) {
		put_user(pca9534_inpins[1], buf++);
		*ptr += 1;
		total++;
	}
#endif

	return total;
}

/* 
 * We only support writes to the first PCA - the second is twiddled directly by the
 * serial driver
 */

static ssize_t pca9534_write(struct file *fp, const char __user *buf, size_t count, loff_t *ptr)
{
	u8 out_pins_new; 
	u8 out_pins_old;
	
	if (! client[0])
		return -ENODEV;

	get_user(out_pins_new, buf++);
	*ptr += 1;
	
	out_pins_old = pca9534_readbyte(0, 1);
	/* Only allow us to set pins 0 to 3 inclusive */
	out_pins_old &= ~0xF;
	out_pins_old |= (out_pins_new & 0xF);
	pca9534_writebyte(0, 1, out_pins_old);
	/* Force a read to update our inpins register */
	pca9534_inpins[0] = pca9534_readbyte(0,0);
	return 1;
}

/*
 * We only support ioctl access to the first PCA - the second is twiddled directly by the 
 * serial driver 
 */
static long pca9534_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u8 old_pin_dir;
	u8 pin_dir;

	if (! client[0])
		return -ENODEV;

	switch (cmd) {
	case PCA9534_GET_DIR:
		/* Get Pin Direction */
		pin_dir = pca9534_readbyte(0, 3);
		/* Mask out the top 4 bits, since we don't want the user thinking
		 * they can do stuff with them 
		 */
		pin_dir &= 0xF; 
		put_user(pin_dir, (u8 *)arg);
		break;
	case PCA9534_SET_DIR:
		/* Set the Pin Direction */
		get_user(pin_dir, (u8*) arg);
		/* Mask out the top 4 bits, since the user can't touch them */
		pin_dir &= 0xF;
		old_pin_dir = pca9534_readbyte(0, 3);
		pin_dir |= (old_pin_dir & ~0x0F);
		pca9534_writebyte(0, 3, pin_dir);
		break;
	
	case PCA9534_GET_COUNTERS:
		if (copy_to_user((u8*)arg, &pca9534_counters, sizeof(pca9534_counters)) != 0) {
			return -EINVAL;
		}
		break;

	case PCA9534_SET_DEBOUNCE_PERIOD:
		{
			int debounce;
			if (copy_from_user(&debounce, (u8*)arg, sizeof(int)) != 0) {
				printk("Could not read debounce\n"); 
				return -EINVAL;
			}
			if (debounce < 0 || debounce > 200) {
				printk("Debounce %d out of spec\n", debounce); 
				return -EINVAL;
			}
			pca9534_debounce_period = debounce;
			printk("PCA9534 - setting debounce to %d ms\n", debounce);	
		}
		break;
	case PCA9534_GET_DEBOUNCE_PERIOD:
		if (copy_to_user((u8*)arg, &pca9534_debounce_period, sizeof(int)) != 0)
			return -EINVAL;
		break;

	case PCA9534_SET_PIN_TRIGGER:
		{
			struct trigger_mode_setting setting;
			if (copy_from_user(&setting, (u8*)arg, sizeof(setting)) != 0) {
				return -EINVAL;
			}
			if (setting.pin >= 0 && setting.pin < 8) {
				if (setting.trigger_mode & ~(PCA9534_TRIGGER_RISING_EDGE | PCA9534_TRIGGER_FALLING_EDGE)) {
					pca9534_trigger_mode[setting.pin] = setting.trigger_mode;	
				} else {
					return -EINVAL;
				}	
			} else {
				return -EINVAL;
			}
		}
		break;
	case PCA9534_GET_PIN_TRIGGER:
		{
			int pin;
			if (copy_from_user(&pin, (u8*)arg, sizeof(int)) != 0) {
				return -EINVAL;
			}
			if (pin >= 0 && pin < 8) {
				if (copy_to_user((u8*)arg, &pca9534_trigger_mode[pin], sizeof(int)) != 0) {
					return -EINVAL;
				}	
			} else {
				return -EINVAL;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
} 
/*
 *****************************************************************************
 *
 *	Driver Interface
 *
 *****************************************************************************
 */

static const struct i2c_device_id pca9534_idtable[] = {
	{ "pca9534", 0, },
	{ },
};

MODULE_DEVICE_TABLE(i2c, pca9534_idtable);

static struct i2c_driver pca9534_i2cdrv = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= PCA9534_DRV_NAME,
	},
	.id_table	= pca9534_idtable,
	.probe		= pca9534_probe,
};

static struct file_operations pca9534_fops = {
        .owner          = THIS_MODULE,
        .read           = pca9534_read,
        .write          = pca9534_write,
        .unlocked_ioctl          = pca9534_ioctl,
};

static struct miscdevice pca9534_miscdrv = {
        .minor          = PCA9534_MINOR,
        .name           = "extio",
        .fops           = &pca9534_fops,
};

static int __init pca9534_init(void)
{
	int rc;

#if defined(CONFIG_MACH_ACM500X)
	if (acm500x_spilocki2c_inited == 0) {
        	sema_init(&acm500x_spilocki2c, 1);
        	acm500x_spilocki2c_inited = 1;
	}
#endif /* CONFIG_MACH_ACM500X */

	if ((rc = i2c_add_driver(&pca9534_i2cdrv)) < 0)
		return rc;
        if ((rc = misc_register(&pca9534_miscdrv)) < 0) {
                i2c_del_driver(&pca9534_i2cdrv);
                return rc;
        }


	return 0;
}

static void __exit pca9534_exit(void)
{
	i2c_del_driver(&pca9534_i2cdrv);
	return;
}

module_init(pca9534_init);
module_exit(pca9534_exit);

MODULE_AUTHOR("Greg Ungerer <greg.ungerer@opengerar.com>");
MODULE_DESCRIPTION("Simple NXP PCA9534 I2C I/O Client Driver");
MODULE_LICENSE("GPL");
