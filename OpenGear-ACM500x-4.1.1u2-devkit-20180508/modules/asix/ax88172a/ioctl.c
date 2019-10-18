/* INCLUDE FILE DECLARATIONS */
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <linux/sockios.h>
#include <linux/types.h>
#include <pthread.h>
#include "ioctl.h"

/* LOCAL VARIABLES DECLARATIONS */
static struct ifreq ifr_ax;
static int sockfd;

/* LOCAL SUBPROGRAM DECLARATIONS */


/* LOCAL SUBPROGRAM DECLARATIONS  BODYS */
void *check_terminate (void *arg)
{
	*((unsigned char *)arg) = 0;
	unsigned char buf, buf1;

	buf = getchar ();

	while (1) {
		buf = getchar ();
		if (buf == 0x0A)
			break;
	}

	*((unsigned char *)arg) = 1;
}

static void spi_test (void)
{
	union ioctl_cmd ioctl_cfg;
	int sel;
	unsigned char CmdBuffer[4];
	unsigned char DataBuffer[12];
	int i;

	memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

	ioctl_cfg.spi_cfg.SPICR = 0xF6;
	ioctl_cfg.spi_cfg.SPIBRR = 0x09;
	ioctl_cfg.spi_cfg.SPISSR = 0xFE;
	ioctl_cfg.spi_cfg.SPIIER = 0x01;
	ioctl_cfg.spi_cfg.SPISCR = 0x00;

	printf ("Configure AX88172A SPI interface : "
		"Master mode, Mode 3, 1.5Mhz\n");

	ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
	if (ioctl(sockfd, IOCTL_SPI_SET_CONFIGURATION, &ifr_ax) < 0) {
		printf("Failed to configure AX88172A SPI interface!!\n");
		return;
	}

	while (1) {
		printf ("\nPlease specify the number of the Test item:\n");
		printf ("1 : Read data from Slave device\n");
		printf ("2 : Write data to Slave device\n");
		printf ("3 : Exit\n");
		printf (":");
		scanf("%d", &sel);

		if (sel == 1) {

			CmdBuffer[0] = 0x40 | (12 - 1);	/* Memory read command 0x40 and length 12bytes */
			CmdBuffer[1] = 0x00;	/* address hi */
			CmdBuffer[2] = 0x00;	/* address med */
			CmdBuffer[3] = 0x00;	/* address low */

			memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

			ioctl_cfg.spi_cmd.OpCodeBufferOffset = (caddr_t)CmdBuffer;
			ioctl_cfg.spi_cmd.OpCodeBufferOffsetlength = 4;

			ioctl_cfg.spi_cmd.DataBufferOffset = (caddr_t)DataBuffer;
			ioctl_cfg.spi_cmd.DataBufferLength = 12;

			ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
			if (ioctl(sockfd, IOCTL_SPI_SEND_WRITE_COMMAND, &ifr_ax) < 0) {
				printf("Failed to issue AX88172A SPI write command!!\n");
				return;
			}

			for (i = 0; i < 12; i++) {
				printf ("DATA[%d] = 0x%02x\n", i, DataBuffer[i]);
			}

		} else if (sel == 2) {
			int data;

			printf ("Please specify the data to write:\n");
			for (i = 0; i < 12; i++) {
				while (1) {
					printf ("DATA[%d] = 0x", i);
					scanf("%x", &data);
					if ((data >= 0) && (data < 256))
						break;
					else
						printf ("Wrong input, please try again!!\n");
				}

				fflush (stdin);
				DataBuffer[i] = (unsigned char)data;

			}

			CmdBuffer[0] = 0xC0 | (12 - 1);	/* Memory write command 0xC0 and length 12bytes */
			CmdBuffer[1] = 0x00;	/* address hi */
			CmdBuffer[2] = 0x00;	/* address med */
			CmdBuffer[3] = 0x00;	/* address low */

			memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

			ioctl_cfg.spi_cmd.OpCodeBufferOffset = (caddr_t)CmdBuffer;
			ioctl_cfg.spi_cmd.OpCodeBufferOffsetlength = 4;

			ioctl_cfg.spi_cmd.DataBufferOffset = (caddr_t)DataBuffer;
			ioctl_cfg.spi_cmd.DataBufferLength = 12;

			ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
			if (ioctl(sockfd, IOCTL_SPI_SEND_WRITE_COMMAND, &ifr_ax) < 0) {
				printf("Failed to issue AX88172A SPI write command!!\n");
				return;
			}

		} else if (sel == 3) {
			break;
		}
	}
}

static void i2c_test (void)
{
	union ioctl_cmd ioctl_cfg;
	int sel;
	int address;
	int offset;
	int data;
	unsigned char CmdBuffer[10];
	unsigned char DataBuffer[100];

	memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

	ioctl_cfg.i2c_cfg.MIE = 1;
	ioctl_cfg.i2c_cfg.I2CEN = 1;
	ioctl_cfg.i2c_cfg.MSS = 1;
	ioctl_cfg.i2c_cfg.Clock = 0x0F;

	printf ("Configure AX88172A I2C : Master mode. ");

	ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
	if (ioctl(sockfd, IOCTL_I2C_SET_CONFIGURATION, &ifr_ax) < 0) {
		printf("Failed to configure AX88172A I2C interface!!\n");
		return;
	}

	printf ("done\n");

	while (1) {
		printf ("\nPlease specify the number of the Test item:\n");
		printf ("1 : Read data from 24c02\n");
		printf ("2 : Write data to 24c02\n");
		printf ("3 : Exit\n");
		printf (":");
		scanf("%d", &sel);

		if (sel == 1) {

			while (1) {
				printf ("Please specify the device address (hex): 0x");
				scanf("%x", &address);

				if ((address > 0) || (address < 50)) {
					break;
				}
			}

			while (1) {
				printf ("Please specify the offset (hex): 0x");
				scanf("%x", &offset);
				if ((offset > 0) || (offset < 96)) {
					break;
				} else {
					printf ("Wrong address input!\n");
				}
			}

			CmdBuffer[0] = offset;

			memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

			ioctl_cfg.i2c_cmd.Address = address;
			ioctl_cfg.i2c_cmd.Direction = 0;
			ioctl_cfg.i2c_cmd.CmdBufferOffset = (caddr_t)CmdBuffer;
			ioctl_cfg.i2c_cmd.CmdBufferLength = 1;
			ioctl_cfg.i2c_cmd.DataBufferOffset = (caddr_t)DataBuffer;
			ioctl_cfg.i2c_cmd.DataBufferLength = 1;
			ioctl_cfg.i2c_cmd.Data[0] = offset;

			ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
			if (ioctl(sockfd, IOCTL_I2C_SEND_COMMAND, &ifr_ax) < 0) {
				printf("Failed to issue AX88172A I2C command!!\n");
				return;
			}

			printf ("Read Slave device(0x%02x): 0x%02x = 0x%02x\n", address, offset,
				DataBuffer[0]);

		} else if (sel == 2) {

			while (1) {
				printf ("Please specify the device address (hex): 0x");
				scanf("%x", &address);

				if ((address > 0) || (address < 256)) {
					break;
				}
			}

			while (1) {
				printf ("Please specify the offset (hex): 0x");
				scanf("%x", &offset);
				if ((offset > 0) || (offset < 256)) {
					break;
				} else {
					printf ("Wrong address input!\n");
				}
			}

			while (1) {
				printf ("Please specify the data (hex): 0x");
				scanf("%x", &data);
				if ((data > 0) || (data < 256)) {
					break;
				} else {
					printf ("Wrong data input!\n");
				}
			}

			CmdBuffer[0] = offset;
			DataBuffer[0] = (unsigned char) data;

			memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

			ioctl_cfg.i2c_cmd.Address = address;
			ioctl_cfg.i2c_cmd.Direction = 1;
			ioctl_cfg.i2c_cmd.Data[0] = offset;
			ioctl_cfg.i2c_cmd.Data[1] = data;
			ioctl_cfg.i2c_cmd.CmdBufferOffset = (caddr_t)CmdBuffer;
			ioctl_cfg.i2c_cmd.CmdBufferLength = 1;
			ioctl_cfg.i2c_cmd.DataBufferOffset = (caddr_t)DataBuffer;
			ioctl_cfg.i2c_cmd.DataBufferLength = 1;

			ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
			if (ioctl(sockfd, IOCTL_I2C_SEND_COMMAND, &ifr_ax) < 0) {
				printf("Failed to issue AX88172A I2C command!!\n");
				return;
			}

		} else if (sel == 3) {
			break;
		}
	}
}

static void uart_test (void)
{
	union ioctl_cmd ioctl_cfg;
	char buf[512];
	int sel;

	while (1) {
		printf ("Please specify the number of the baud rate:\n");
		printf ("1 : 3600\n");
		printf ("2 : 4800\n");
		printf ("3 : 7200\n");
		printf ("4 : 9600\n");
		printf ("5 : 19200\n");
		printf ("6 : 38400\n");
		printf ("7 : 57600\n");
		printf (":");
		scanf("%d", &sel);
		if ((sel > 0) && (sel < 8))
			break;
	}

	memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

	switch (sel) {
	case 1:
		ioctl_cfg.uart_cfg.DLLR = 0x80;
		break;
	case 2:
		ioctl_cfg.uart_cfg.DLLR = 0x60;
		break;
	case 3:
		ioctl_cfg.uart_cfg.DLLR = 0x40;
		break;
	case 4:
		ioctl_cfg.uart_cfg.DLLR = 0x30;
		break;
	case 5:
		ioctl_cfg.uart_cfg.DLLR = 0x18;
		break;
	case 6:
		ioctl_cfg.uart_cfg.DLLR = 0x0c;
		break;
	case 7:
		ioctl_cfg.uart_cfg.DLLR = 0x08;
		break;
	default:
		ioctl_cfg.uart_cfg.DLLR = 0x30;
	}

	ioctl_cfg.uart_cfg.DLHR = 0;
	ioctl_cfg.uart_cfg.LCR = 3;
	ioctl_cfg.uart_cfg.IER = 0;
	ioctl_cfg.uart_cfg.FCR = 7;
	ioctl_cfg.uart_cfg.Reserved = 0;

	ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
	if (ioctl(sockfd, IOCTL_UART_SET_CONFIGURATION, &ifr_ax) < 0) {
		printf("Failed to configure AX88172A Uart interface!!\n\r");
		return;
	}

	while (1) {

		while (1) {
			printf ("\nPlease select test item\n");
			printf ("1 : Uart send\n");
			printf ("2 : Uart receive\n");
			printf ("3 : Exit UART test\n");
			printf (":");
			scanf("%d", &sel);
			if ((sel > 0) && (sel < 4))
				break;
		}
	
		if (sel == 3) {
			break;
		} else if (sel == 1) {
	
			int i;
			int length;
			int loop;
			int left;
	
			memset (buf, 0, 512);
			printf ("Please input the string:");
			scanf("%s", buf);

			memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

			length = strlen(buf);

			ioctl_cfg.uart_data.data_len = length;
			ioctl_cfg.uart_data.Data = (caddr_t)buf;
			ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
			if (ioctl(sockfd, IOCTL_UART_SEND, &ifr_ax) < 0) {
				printf("Failed to write AX88172A Uart data!!\n\r");
				return;
			}

		} else if (sel == 2) {

			unsigned char i;
			unsigned long cnt = 0;
			unsigned long t = 0;
			pthread_t pth;
			unsigned char input = 0;

			printf ("Start UART receive (Press Enter to stop)\n");
			fflush (stdout);

			fflush (stdin);
			if (pthread_create (&pth, NULL, check_terminate, &input) != 0) {
				printf ("Failed to create thread for UART test!\n");
				return;
			}

			do {
				usleep (10000);

				memset(&ioctl_cfg, 0, sizeof (ioctl_cfg));

				ifr_ax.ifr_data = (caddr_t)&ioctl_cfg;
				ioctl_cfg.uart_data.Data = (caddr_t)buf;
	
				if (ioctl(sockfd, IOCTL_UART_RECEIVE, &ifr_ax) < 0) {
					printf("\nFailed to write AX88172A Uart data!!\n\r");
					return;
				}

				if (ioctl_cfg.uart_data.data_len) {

					for (i = 0; i < ioctl_cfg.uart_data.data_len; i++) {
						printf ("%c", buf[i]);
					}
					fflush (stdout);
				}

			} while (input == 0);
		}

	}
}

int main(void)
{
	struct sockaddr_in servaddr, cliaddr;
	unsigned long ipa;
	char buf[16];
	char sig[8];
	int sel;

	union ioctl_cmd ioctl_cfg;

	sockfd = socket(AF_INET, SOCK_DGRAM, 0); /* create a socket */

	while(1) {
		printf("Please input the interface of the AX88172A. (ex. eth1)::");
		scanf("%s", buf);
		if ( memcmp(buf, "eth", 3) != 0 ) {
			printf("Wrong input !!\n");
		} else {
			strcpy(ifr_ax.ifr_name, buf);

#if 1
			memset(sig, 0, 8);
			ifr_ax.ifr_data = (caddr_t)sig;
			if (ioctl(sockfd, IOCTL_SIGNATURE, &ifr_ax) < 0) {
				printf("No AX88172A found at %s\n", buf);
			} else {
				if ( memcmp(ASIX_GID, sig, sizeof(ASIX_GID)) == 0 ) {
					printf("AX88172A :: %s , IP :: %s\n",
						ifr_ax.ifr_name ,
						inet_ntoa(((struct sockaddr_in*)&(ifr_ax.ifr_addr))->sin_addr));
					break;
				} else {
					printf ("Wrong GID : %s\n", sig);
				}
			}
#else
			/* Get IP address */
			if (ioctl(sockfd, SIOCGIFADDR, &ifr_ax) < 0) {
				printf("The interface %s does not valid!\n\r", buf);
			} else {
				memset(sig, 0, 8);
				ifr_ax.ifr_data = (caddr_t)sig;
				if (ioctl(sockfd, IOCTL_SIGNATURE, &ifr_ax) < 0) {
					printf("No AX88172A found at %s!!\n\r", buf);
				} else {
					if ( memcmp(ASIX_GID, sig, sizeof(ASIX_GID)) == 0 ) {
						printf("AX88172A :: %s , IP :: %s\n",
							ifr_ax.ifr_name ,
							inet_ntoa(((struct sockaddr_in*)&(ifr_ax.ifr_addr))->sin_addr));
						break;
					} else {
						printf ("Wrong GID : %s\n", sig);
					}
				}
			}
#endif
		}
	}

	while (1) {
		printf ("Please specify the number of the Test item:\n");
		printf ("1 : UART\n");
		printf ("2 : I2C\n");
		printf ("3 : SPI\n");
		printf ("4 : Exit\n");
		printf (":");
		scanf("%d", &sel);

		if (sel == 1) {
			uart_test ();
		} else if (sel == 2) {
			i2c_test ();
		} else if (sel == 3) {
			spi_test ();
		} else if (sel == 4) {
			break;
		}
	}

	return 0;
}
