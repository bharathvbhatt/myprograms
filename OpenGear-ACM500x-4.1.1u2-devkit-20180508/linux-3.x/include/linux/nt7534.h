#ifndef _LINUX_NT7534_H
#define _LINUX_NT7534_H

/* This structure contains the GPIOs for the nt7534 chip */
struct nt7534_chip {
	int a0;
	int e;
	int rw;
	int cs1;
	int data_bus[8];
	
	bool chip_inited;
};

void nt7534_init(struct nt7534_chip *new_chip);

void nt7534_reset(void);

void nt7534_nop(void);

void nt7534_read_data(unsigned char *byte);

void nt7534_write_data(unsigned char byte);

void nt7534_write_control(unsigned char byte);

void nt7534_display_state(bool on);

void nt7534_start_line(unsigned char line);

void nt7534_column_address(unsigned char column);

void nt7534_page(unsigned char page);

#endif /* _LINUX_NT7534_H */
