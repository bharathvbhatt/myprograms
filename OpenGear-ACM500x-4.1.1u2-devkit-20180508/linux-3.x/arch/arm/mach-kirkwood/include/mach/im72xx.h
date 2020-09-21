

#define IM72XX_I2C_GPIO_BASE_22 		100
#define IM72XX_I2C_GPIO_BASE_23			150
#define IM72XX_GPIO_FLAG_NC			0x0
#define IM72XX_GPIO_FLAG_INPUT 			0x1
#define IM72XX_GPIO_FLAG_OUTPUT			0x2
#define IM72XX_GPIO_FLAG_INVERTED		0x4
#define IM72XX_GPIO_FLAG_INTERRUPT_LEVEL	0x8
#define IM72XX_GPIO_FLAG_OUTPUT_HIGH		0x10

struct im72xx_gpio_setting {
	char *name;
	int flags;
};

enum im72xx_i2c_gpios {
	GPIO_KEY_1 = IM72XX_I2C_GPIO_BASE_22,
	GPIO_KEY_2,
	GPIO_KEY_3, 
	GPIO_KEY_4,
	GPIO_NC0,
	GPIO_NC1,
	GPIO_SD_CARD,
	GPIO_SIM_CARD,
	GPIO_POWER_LED,
	GPIO_HEART_BEAT_LED,
	GPIO_SERIAL_ACTIVITY_LED,
	GPIO_NETWORK_ACTIVITY_LED,
	GPIO_NC2,
	GPIO_NC3,
	GPIO_LCD_BACKLIGHT,
	GPIO_CONFIG_RESET,
	GPIO_MODEM_RESET,
	GPIO_NC4,
	GPIO_I2C_SFP_SELECT,
	GPIO_NC5,
	GPIO_NC6,
	GPIO_NC7,	
	GPIO_NC8,	
	GPIO_NC9,	
};

static struct im72xx_gpio_setting im72xx_tca6424_gpio_settings[] = 
{
	{ "Key 1", IM72XX_GPIO_FLAG_NC },
	{ "Key 2", IM72XX_GPIO_FLAG_NC },
	{ "Key 3", IM72XX_GPIO_FLAG_NC },
	{ "Key 4", IM72XX_GPIO_FLAG_NC },
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "SD Card Detect", IM72XX_GPIO_FLAG_NC},
	{ "SIM Card Detect", IM72XX_GPIO_FLAG_INPUT },
	{ "Power LED", IM72XX_GPIO_FLAG_OUTPUT},
	{ "Heart Beat LED", IM72XX_GPIO_FLAG_OUTPUT},
	{ "Serial Activity LED", IM72XX_GPIO_FLAG_OUTPUT},
	{ "Network Activity LED", IM72XX_GPIO_FLAG_OUTPUT},
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "LCD Backlight", IM72XX_GPIO_FLAG_OUTPUT | IM72XX_GPIO_FLAG_OUTPUT_HIGH},
	{ "Config Reset", IM72XX_GPIO_FLAG_NC},
	{ "Modem Reset", IM72XX_GPIO_FLAG_OUTPUT | IM72XX_GPIO_FLAG_OUTPUT_HIGH},
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "I2C SFP Select", IM72XX_GPIO_FLAG_OUTPUT | IM72XX_GPIO_FLAG_OUTPUT_HIGH},
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "NC", IM72XX_GPIO_FLAG_NC },
	{ "NC", IM72XX_GPIO_FLAG_NC },
};
