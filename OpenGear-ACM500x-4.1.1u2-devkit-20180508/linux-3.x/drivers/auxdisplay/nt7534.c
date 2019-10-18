#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/nt7534.h>

#define bit(n) (((unsigned char)1)<<(n))

#undef BUSY_CHECK

#define CMD_DISPLAY_ON 	0xAF
#define CMD_DISPLAY_OFF 0xAE
#define CMD_DISPLAY_START_LINE_SET(line) (0x40 + line)
#define CMD_PAGE_ADDRESS_SET(page) (0xB0 + page)
#define CMD_COLUMN_ADDRESS_SET_LO(column) (column & 0x0F)
#define CMD_COLUMN_ADDRESS_SET_HI(column) (0x10 + ((column & 0xF0) >> 4))
#define CMD_WRITE_DISPLAY_DATA(data) (data)
#define CMD_RESET 0xE2
#define CMD_NOP	0xE3

static struct nt7534_chip chip;

void nt7534_init(struct nt7534_chip *new_chip) {
	/* Copy all the pin numbers */
	int i;
	chip = *new_chip;
	gpio_request_one(chip.a0, 0, "NT7534 A0 Pin");
	gpio_direction_output(chip.a0, 0);
	gpio_request_one(chip.e, 0, "NT7534 E Pin");
	gpio_direction_output(chip.e, 0);
	gpio_request_one(chip.rw, 0, "NT7534 R/W Pin");
	gpio_direction_output(chip.rw, 0);
	gpio_request_one(chip.cs1, 0, "NT7534 CS1 Pin");
	gpio_direction_output(chip.cs1, 0);
	for (i = 0; i < 8; i++) {
		char buf[64];
		sprintf(buf, "NT7534 Data %d pin", i);
		gpio_request_one(chip.data_bus[i], 0, buf);
		gpio_direction_output(chip.data_bus[i], 0);
	}
	chip.chip_inited = true;
}

static void nt7534_data_bus_set_data(unsigned char byte) {
	int i;

	if (!chip.chip_inited)
		return;
	
	for (i = 0; i < 8; i++) {
		gpio_direction_output(chip.data_bus[i], ((byte & (1 << i)) ? 1 : 0));
	}
}


static void nt7534_data_bus_read_data(unsigned char *byte) {
	int i;
	*byte = 0;

	if (!chip.chip_inited)
		return;
	
	for (i = 0; i < 8; i++) {
		gpio_direction_input(chip.data_bus[i] );
		if (gpio_get_value(chip.data_bus[i])) {
			*byte |= (1 << i);
		}
	}
}

static void nt7534_write(unsigned char byte, bool control)
{
	if (!chip.chip_inited)
		return;

	/* Delay to make sure we don't bust the cycle time */
	udelay(1);
	/* Put Data on the Bus */
	nt7534_data_bus_set_data(byte);
	if (control) {
		/* Command Mode */
		gpio_set_value(chip.a0, 0);
	} else {
		/* ! Command Mode */
		gpio_set_value(chip.a0, 1);
	}	
	/* Chip Select */
	gpio_set_value(chip.cs1, 0);
	/* Set RW Lo */
	gpio_set_value(chip.rw, 0);	
	/* Set E High */
	gpio_set_value(chip.e, 1);
	/* Keep this high for a while to make sure the data is read */
	udelay(1);
	/* Set E Low */
	gpio_set_value(chip.e, 0);
	/* Set RW High again */
	gpio_set_value(chip.rw, 1); 
	udelay(1);
	/* Turn off Chip Select */
	gpio_set_value(chip.cs1, 1);

}

static void nt7534_read(unsigned char *byte, bool control)
{
	
	/* Delay to make sure we don't bust the cycle time */
	udelay(1);
	if (control) {
		/* Command Mode */
		gpio_set_value(chip.a0, 0);
	} else {
		/* ! Command Mode */
		gpio_set_value(chip.a0, 1);
	}	
	/* Set RW HI */
	gpio_set_value(chip.rw, 1);
	/* Chip Select */
	gpio_set_value(chip.cs1, 0);

	
	/* Set E High */
	gpio_set_value(chip.e, 1);
	/* Keep this high for a while to make sure the data is read */
	udelay(1);
	/* Put Data on the Bus */
	nt7534_data_bus_read_data(byte);
	/* Set E Low */
	gpio_set_value(chip.e, 0);
	udelay(1);;
	/* Turn off Chip Select */
	gpio_set_value(chip.cs1, 1);


}

void nt7534_write_data(unsigned char byte) {
#ifdef BUSY_CHECK
	/* Wait for busy */
	u8 status = 0;
	do {
		nt7534_read(&status, true);
	} while (status & 1 << 7);
#endif		
	return nt7534_write(byte, false);
}

void nt7534_write_control(unsigned char byte) {
#ifdef BUSY_CHECK
	/* Wait for busy */
	u8 status = 0;
	do {
		nt7534_read(&status, true);
	} while (status & 1 << 7);
#endif
	//printk(KERN_ERR "Writing control byte %02x\n", byte);
	return nt7534_write(byte, true);
}

void nt7534_read_data(unsigned char *byte) {
#ifdef BUSY_CHECK
	/* Wait for busy */
	u8 status = 0;
	do {
		nt7534_read(&status, true);
	} while (status & 1 << 7);
#endif	

	return nt7534_read(byte, false);
}
void nt7534_display_state(bool on) {
	if (on) {
		nt7534_write_control(CMD_DISPLAY_ON);
	} else {
		nt7534_write_control(CMD_DISPLAY_OFF);
	}	
}

void nt7534_start_line(unsigned char line) {
	nt7534_write_control(CMD_DISPLAY_START_LINE_SET(min(line, 63)));
}

void nt7534_column_address(unsigned char column) {
	//printk(KERN_ERR "Writing column %02x", column);
	nt7534_write_control(CMD_COLUMN_ADDRESS_SET_HI(column));
	nt7534_write_control(CMD_COLUMN_ADDRESS_SET_LO(column));
}

void nt7534_page(unsigned char page) {
	//page = min(page, 7);
	nt7534_write_control(CMD_PAGE_ADDRESS_SET(page));
}

void nt7534_reset() {
	nt7534_write_control(CMD_RESET);
}
void nt7534_nop() {
	nt7534_write_control(CMD_NOP);
}


