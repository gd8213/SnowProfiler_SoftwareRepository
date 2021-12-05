#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>

// Change this to match the machine's serial port
// has to be changed at integrator board to ttyS0
// Possible Bugs:
//				-no permission ->sudo chmod a+rw /dev/serial0
//				-wrong port	
// see https://raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart for further info and functions

// ToDO for RPB3B
// disable console on AMA0 (delete in /boot/config.txt)
// add line uart_enable=1 in /boot/cmdline.txt)
// reboot
// see https://www.circuits.dk/setup-raspberry-pi-3-gpio-uart/ for further info

// this function only is used to read bytes from the STM32



#define SERDEV "/dev/serial0"

void initComPort(int* sfd, char* device)
{
	struct termios options;

	*sfd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (*sfd == -1)
	{
		fprintf(stderr, "unable to open %s\n", device);
		exit(1);
	}
	else
	{
		fcntl(*sfd, F_SETFL, FNDELAY);
	}

	tcgetattr(*sfd, &options);

	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	cfmakeraw(&options);

	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
	options.c_cflag |= (CLOCAL | CREAD | CS8);

	options.c_oflag &= ~OPOST;
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 0;

	tcsetattr(*sfd, TCSANOW, &options);
}


int main(void)
{
	int sfd;
	FILE *ofd;
	int32_t n, i;
	u_int32_t bytes;
	u_int8_t buff[25];

	// Initialize the serial port
	initComPort(&sfd, SERDEV);
	/*
	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'H';
	*p_tx_buffer++ = 'e';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'o';

	if (sfd != -1)
	{
	int count = write(sfd, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
	if (count < 0)
	{
	printf("UART TX error\n");
	}
	else
	{
	printf("[BYTES TX: %i]  %s\n",count,tx_buffer);
	}
	}*/
	/*
	// Bytes senden
	unsigned char BUF_TX[20];
	unsigned char *TX;

	TX = &BUF_TX[0];
	*TX++ = 'S';
	*TX++ = 't';
	*TX++ = 'o';
	*TX++ = 'p';


	printf("%i",buff);
	if (sfd != -1) {
	int out = write(sfd, &BUF_TX[0], (TX - &BUF_TX[0])); //
	if (out < 0) {
	printf("[ERROR] UART TX\n");
	}
	else {
	printf("[STATUS: TX %i Bytes] %s\n", out, BUF_TX);
	}
	} // if uart0
	*/
	while (1)
	{
		// Check if there's any data available to read
		ioctl(sfd, FIONREAD, &bytes);
		if (bytes > 0)
		{
			// printf("Bytes in Receive: %i\n", bytes);
			// Read what we can
			n = read(sfd, buff, 10000);

			if (n < 0)
			{
				printf("Read failed");
				//				fprintf(stderr, "read failed\n");
			}
			if (n > 0)
			{
				printf(buff);

				// file write begin
				/*
				ofd = fopen("data.bin", "a");
				if (ofd == NULL)
				{
				fprintf(stderr, "unable to open output file\n");
				exit(2);
				}

				fwrite(buff, 1, n, ofd);

				fclose(ofd);
				*/
			}
		}
		// usleep(1000);
	}
}
