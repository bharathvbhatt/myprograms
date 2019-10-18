
#ifndef _CFAX128_H_
#define _CFAX128_H_

#define CFAX128_WIDTH	(128)
#define CFAX128_HEIGHT	(64)
#define CFAX128_PAGES	(8)
#define CFAX128_ADDRESSES	(128)
#define CFAX128_SIZE		((CFAX128_PAGES) * \
				(CFAX128_ADDRESSES))

/*
 * The driver will blit this buffer to the LCD
 *
 * Its size is CFAG12864B_SIZE.
 */
extern unsigned char * cfax128_buffer;

/*
 * Get the refresh rate of the LCD
 *
 * Returns the refresh rate (hertz).
 */
extern unsigned int cfax128_getrate(void);

/*
 * Enable refreshing
 *
 * Returns 0 if successful (anyone was using it),
 * or != 0 if failed (someone is using it).
 */
extern unsigned char cfax128_enable(void);

/*
 * Disable refreshing
 *
 * You should call this only when you finish using the LCD.
 */
extern void cfax128_disable(void);

/*
 * Is enabled refreshing? (is anyone using the module?)
 *
 * Returns 0 if refreshing is not enabled (anyone is using it),
 * or != 0 if refreshing is enabled (someone is using it).
 *
 * Useful for buffer read-only modules.
 */
extern unsigned char cfax128_isenabled(void);

/*
 * Is the module inited?
 */
extern unsigned char cfax128_isinited(void);

#endif /* _CFAX128_H_ */

