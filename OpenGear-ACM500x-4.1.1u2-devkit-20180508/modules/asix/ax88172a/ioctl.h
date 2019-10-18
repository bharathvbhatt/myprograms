
/* Definition for IOCTL */
#define IOCTL_SIGNATURE			(SIOCDEVPRIVATE + 0)
#define IOCTL_I2C_SET_CONFIGURATION	(SIOCDEVPRIVATE + 1)
#define IOCTL_I2C_SEND_COMMAND		(SIOCDEVPRIVATE + 2)

#define IOCTL_SPI_SET_CONFIGURATION	(SIOCDEVPRIVATE + 3)
#define IOCTL_SPI_GET_SLAVE_STATUS	(SIOCDEVPRIVATE + 4)
#define IOCTL_SPI_SEND_WRITE_COMMAND	(SIOCDEVPRIVATE + 5)
#define IOCTL_SPI_SEND_READ_COMMAND	(SIOCDEVPRIVATE + 6)
#define IOCTL_SPI_GET_SLAVE_DATA	(SIOCDEVPRIVATE + 7)

#define IOCTL_UART_SET_CONFIGURATION	(SIOCDEVPRIVATE + 8)
#define IOCTL_UART_RECEIVE		(SIOCDEVPRIVATE + 9)
#define IOCTL_UART_SEND			(SIOCDEVPRIVATE + 10)

const unsigned char	ASIX_GID[8] = {'A','S','I','X','X','I','S','A'};

inline unsigned long STR_TO_U32(const char *cp,char **endp,unsigned int base)
{
	unsigned long result = 0,value;

	if (*cp == '0') {
		cp++;
		if ((*cp == 'x') && isxdigit(cp[1])) {
			base = 16;
			cp++;
		}
		if (!base) {
			base = 8;
		}
	}
	if (!base) {
		base = 10;
	}
	while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp-'0' : (islower(*cp)
	    ? toupper(*cp) : *cp)-'A'+10) < base) {
		result = result*base + value;
		cp++;
	}
	if (endp)
		*endp = (char *)cp;
	return result;
}

struct ioctl_i2c_cfg
{
	unsigned char	MSS;	/* 0:master mode, 1:slave mode */
	unsigned char	SIE;	/* reserved */
	unsigned char	TE;	/* 0:7 bit address, 1:10 bit address */
	unsigned char	SD;	/* reserved */
	unsigned char	I2CEN;	/* 0: disable I2C, 1: enable I2C */
	unsigned char	MIE;	/* 0: disable interrupt, 1: enable interrupt */
	unsigned short	Clock;	/* I2C master active clock */
	unsigned char	SDA;	/* reserved */
};

struct ioctl_i2c_cmd
{
	/* slave device address */
	unsigned long	Address;

	/* slave device address used 7 or 10 bits */
	unsigned long	AddressBits;

	/* 0 : read, 1 : write */
	unsigned long	Direction;

	/* command buffer offset from the beginning of Data */
	void		*CmdBufferOffset;

	/* command buffer length */
	unsigned long	CmdBufferLength;

	/* data buffer offset from the beginning of Data, for write command */
	void		*DataBufferOffset;

	/* data buffer length, for read command, this means read data length */
	unsigned long	DataBufferLength;

	unsigned char	Data[4];
};

struct ioctl_spi_cfg
{
	unsigned char	SPICR;	/* SPICR value */
	unsigned char	SPIBRR;	/* SPIBRR value */
	unsigned char	SPISSR;	/* SPISSR value, always set this to 0xFE */
	unsigned char	SPIIER;	/* SPIIER value */
	unsigned char	SPISCR;	/* SPISCR value, always set this to 0x01 */
};

struct ioctl_spi_cmd
{
	/* OpCode buffer offset from the beginning of Data */
	void		*OpCodeBufferOffset;

	/* OpCode buffer length */
	unsigned long	OpCodeBufferOffsetlength;

	/* for write command, read command set this to 0 */
	void		*DataBufferOffset;

	/*
	 * for get status, read and get data command,
	 * it means the status length, read data length.
	 */
	unsigned long	DataBufferLength;

	/*
	 * 0: no need OID_SPI_GET_SLAVE_DATA request,
	 * 1: need OID_SPI_GET_SLAVE_DATA request
	 */
	unsigned long	ReadType;

	unsigned char	Data[4];
};

struct ioctl_uart_cfg
{
	unsigned char	DLLR;		/* DLLR value */
	unsigned char	DLHR;		/* DLHR value */
	unsigned char	LCR;		/* LCR value */
	unsigned char	IER;		/* IER value */
	unsigned char	FCR;		/* FCR value */
	unsigned char	Reserved;
};

struct ioctl_uart_data
{
	unsigned long	data_len;
	void		*Data;
};

union ioctl_cmd {
	struct ioctl_spi_cfg	spi_cfg;
	struct ioctl_spi_cmd	spi_cmd;

	struct ioctl_i2c_cfg	i2c_cfg;
	struct ioctl_i2c_cmd	i2c_cmd;

	struct ioctl_uart_cfg	uart_cfg;
	struct ioctl_uart_data	uart_data;
};
