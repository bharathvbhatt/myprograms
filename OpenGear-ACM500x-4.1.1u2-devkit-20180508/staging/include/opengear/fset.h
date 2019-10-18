#ifndef _OPENGEAR_INTERNALEMD_H_
#define _OPENGEAR_INTERNALEMD_H_

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


__BEGIN_DECLS

#ifndef DEFAULTOFFS
#define DEFAULTOFFS 0x000
#endif

#define FLASHLEN	(1024 * 128)

#define	DEFAULTFLASH	"/dev/flash/bootarg"

typedef struct F_SET {
#define MODEL_LEN 32
	char model[MODEL_LEN];
#define SERIAL_COUNT_LEN 4
	char serial_count[SERIAL_COUNT_LEN];
#define SERIAL_NO_LEN 24
	char serial_no[SERIAL_NO_LEN];
#define FACTORY_OPTS_LEN 48
	char factory_opts[FACTORY_OPTS_LEN];
#define SENSORS_LEN 8
	char sensors[SENSORS_LEN];
#define MAC_LEN 18
	char ethaddr[MAC_LEN];
	char eth1addr[MAC_LEN];

	char *power_supplies;
	char *power_input;
	char *pinout;
	char *console;
	char *ethernet_count;

} fset;

typedef struct V_SET {
	char *name;
	char *value;
	struct V_SET *next;
} vset;

/*
 *	Search for a mtd partition in /proc/mtd.
 *	Assumes that each line starts with the device name followed
 *	by a ':', and the partition name is enclosed by quotes.
 *	Returns a pointer into a temporary buffer that will be
 *	overwritten on the next call.
 */
char *findmtddevice(const char *mtdname);

/**
 * @return 1 for error and 2 to display usage()
*/
size_t readfsetflash(const char *flash, int offset);

/**
 * The calling program is responsible for calling freeVars()
 * in order to free the global variable called "head".
 * "head" is a ponter to the start of the flash memory
 * where the fset strings are stored.
*/
void freeVars(void);

void printfset(const char *query);
vset * findValue(const char *value);
void parseFactorySetup(fset *features);
void printraw(void);
void saveFlash(const char * flash, off_t offset);
int updateflash(const char * flash, off_t offset, const char * write);
int updateflasharray(const char *flash, off_t offset, const char * const *write, int num_ents);
void deleteflash(const char * flash, off_t offset, const char * name);
void deleteflasharray(const char *flash, off_t offset, const char * const *name, int num_ents);
void getfset(fset* features);
void resetDefaults(const char * flash, off_t offset);


__END_DECLS

#endif /* _OPENGEAR_INTERNALEMD_H_ */

