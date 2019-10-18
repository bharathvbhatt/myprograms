/*
 * lm70.c
 *
 * The LM70 is a temperature sensor chip from National Semiconductor (NS).
 * Copyright (C) 2006 Kaiwan N Billimoria <kaiwan@designergraphix.com>
 *
 * The LM70 communicates with a host processor via an SPI/Microwire Bus
 * interface. The complete datasheet is available at National's website
 * here:
 * http://www.national.com/pf/LM/LM70.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/hwmon.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#if defined(CONFIG_MACH_ACM500X)
#include <linux/semaphore.h>
#endif


#define DRVNAME		"lm70"

#define LM70_CHIP_LM70		0	/* original NS LM70 */
#define LM70_CHIP_TMP121	1	/* TI TMP121/TMP123 */
#define LM70_CHIP_LM71		2	/* NS LM71 */
#define LM70_CHIP_LM74		3	/* NS LM74 */

struct lm70 {
	struct device *hwmon_dev;
	struct mutex lock;
	unsigned int chip;
};

#if defined(CONFIG_MACH_ACM500X) || defined(CONFIG_MACH_ACM550X)
/*
 * On the OpenGear ACM500x platform spi and i2c devies share the
 * same physical clock and data pins. So we need to lock users of
 * these types of external devices so they don't stomp on each other.
 */
DEFINE_SEMAPHORE(acm500x_spilocki2c);
int acm500x_spilocki2c_inited;
#endif /* CONFIG_MACH_ACM500X */

/* sysfs hook function */
static ssize_t lm70_sense_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	int status, val = 0;
	u8 rxbuf[2];
	s16 raw = 0;
	struct lm70 *p_lm70 = spi_get_drvdata(spi);

	if (mutex_lock_interruptible(&p_lm70->lock))
		return -ERESTARTSYS;

#if defined(CONFIG_MACH_ACM500X) || defined(CONFIG_MACH_ACM550X)
	down(&acm500x_spilocki2c);
#endif /* CONFIG_MACH_ACM500X */

	/*
	 * spi_read() requires a DMA-safe buffer; so we use
	 * spi_write_then_read(), transmitting 0 bytes.
	 */
	status = spi_write_then_read(spi, NULL, 0, &rxbuf[0], 2);
	if (status < 0) {
		pr_warn("spi_write_then_read failed with status %d\n", status);
		goto out;
	}
	raw = (rxbuf[0] << 8) + rxbuf[1];
	dev_dbg(dev, "rxbuf[0] : 0x%02x rxbuf[1] : 0x%02x raw=0x%04x\n",
		rxbuf[0], rxbuf[1], raw);

	/*
	 * LM70:
	 * The "raw" temperature read into rxbuf[] is a 16-bit signed 2's
	 * complement value. Only the MSB 11 bits (1 sign + 10 temperature
	 * bits) are meaningful; the LSB 5 bits are to be discarded.
	 * See the datasheet.
	 *
	 * Further, each bit represents 0.25 degrees Celsius; so, multiply
	 * by 0.25. Also multiply by 1000 to represent in millidegrees
	 * Celsius.
	 * So it's equivalent to multiplying by 0.25 * 1000 = 250.
	 *
	 * LM74 and TMP121/TMP123:
	 * 13 bits of 2's complement data, discard LSB 3 bits,
	 * resolution 0.0625 degrees celsius.
	 *
	 * LM71:
	 * 14 bits of 2's complement data, discard LSB 2 bits,
	 * resolution 0.0312 degrees celsius.
	 */
	switch (p_lm70->chip) {
	case LM70_CHIP_LM70:
		val = ((int)raw / 32) * 250;
		break;

	case LM70_CHIP_TMP121:
	case LM70_CHIP_LM74:
		val = ((int)raw / 8) * 625 / 10;
		break;

	case LM70_CHIP_LM71:
		val = ((int)raw / 4) * 3125 / 100;
		break;
	}

	status = sprintf(buf, "%d\n", val); /* millidegrees Celsius */
out:
#if defined(CONFIG_MACH_ACM500X) || defined(CONFIG_MACH_ACM550X)
	up(&acm500x_spilocki2c);
#endif /* CONFIG_MACH_ACM500X */
	mutex_unlock(&p_lm70->lock);
	return status;
}

static DEVICE_ATTR(temp1_input, S_IRUGO, lm70_sense_temp, NULL);

static ssize_t lm70_show_name(struct device *dev, struct device_attribute
			      *devattr, char *buf)
{
	return sprintf(buf, "%s\n", to_spi_device(dev)->modalias);
}

static DEVICE_ATTR(name, S_IRUGO, lm70_show_name, NULL);

/*----------------------------------------------------------------------*/

static int lm70_probe(struct spi_device *spi)
{
	int chip = spi_get_device_id(spi)->driver_data;
	struct lm70 *p_lm70;
	int status;

	/* signaling is SPI_MODE_0 */
	if (spi->mode & (SPI_CPOL | SPI_CPHA))
		return -EINVAL;

	/* NOTE:  we assume 8-bit words, and convert to 16 bits manually */
#if defined(CONFIG_MACH_ACM500X) 
	/*
	 * According to the data sheet I have the LM70 uses 16bit words.
	 * And we certainly need to set this for the KZS8292 SPI controler.
	 */
	spi->bits_per_word = 16;
#endif

	p_lm70 = devm_kzalloc(&spi->dev, sizeof(*p_lm70), GFP_KERNEL);
	if (!p_lm70)
		return -ENOMEM;

	mutex_init(&p_lm70->lock);
	p_lm70->chip = chip;

	spi_set_drvdata(spi, p_lm70);

	status = device_create_file(&spi->dev, &dev_attr_temp1_input);
	if (status)
		goto out_dev_create_temp_file_failed;
	status = device_create_file(&spi->dev, &dev_attr_name);
	if (status)
		goto out_dev_create_file_failed;

	/* sysfs hook */
	p_lm70->hwmon_dev = hwmon_device_register(&spi->dev);
	if (IS_ERR(p_lm70->hwmon_dev)) {
		dev_dbg(&spi->dev, "hwmon_device_register failed.\n");
		status = PTR_ERR(p_lm70->hwmon_dev);
		goto out_dev_reg_failed;
	}

	return 0;

out_dev_reg_failed:
	device_remove_file(&spi->dev, &dev_attr_name);
out_dev_create_file_failed:
	device_remove_file(&spi->dev, &dev_attr_temp1_input);
out_dev_create_temp_file_failed:
	spi_set_drvdata(spi, NULL);
	return status;
}

static int lm70_remove(struct spi_device *spi)
{
	struct lm70 *p_lm70 = spi_get_drvdata(spi);

	hwmon_device_unregister(p_lm70->hwmon_dev);
	device_remove_file(&spi->dev, &dev_attr_temp1_input);
	device_remove_file(&spi->dev, &dev_attr_name);
	spi_set_drvdata(spi, NULL);

	return 0;
}


static const struct spi_device_id lm70_ids[] = {
	{ "lm70",   LM70_CHIP_LM70 },
	{ "tmp121", LM70_CHIP_TMP121 },
	{ "lm71",   LM70_CHIP_LM71 },
	{ "lm74",   LM70_CHIP_LM74 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lm70_ids);

static struct spi_driver lm70_driver = {
	.driver = {
		.name	= "lm70",
		.owner	= THIS_MODULE,
	},
	.id_table = lm70_ids,
	.probe	= lm70_probe,
	.remove	= lm70_remove,
};

static int __init init_lm70(void)
{
#if defined(CONFIG_MACH_ACM500X)
	if (acm500x_spilocki2c_inited == 0) {
		sema_init(&acm500x_spilocki2c, 1);
		acm500x_spilocki2c_inited = 1;
	}
#endif /* CONFIG_MACH_ACM500X */
	return spi_register_driver(&lm70_driver);
}

static void __exit cleanup_lm70(void)
{
	spi_unregister_driver(&lm70_driver);
}

module_init(init_lm70);
module_exit(cleanup_lm70);

MODULE_AUTHOR("Kaiwan N Billimoria");
MODULE_DESCRIPTION("NS LM70 and compatibles Linux driver");
MODULE_LICENSE("GPL");
