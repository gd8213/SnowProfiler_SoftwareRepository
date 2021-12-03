#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {

	int uart0_filestream = -1;
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (uart0_filestream == -1) {
		printf("[ERROR] UART open()\n");
	}

	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

	//---------------------------------------------------------------------
	// start this transmission if PWM is finished
	// Bytes senden
		unsigned char BUF_TX[20];
		unsigned char *TX;

		TX = &BUF_TX[0];
		*TX++ = 's';
		*TX++ = 't';
		*TX++ = 'o';
		*TX++ = 'p';
	

	if (uart0_filestream != -1) {
		int out = write(uart0_filestream, &BUF_TX[0], (TX - &BUF_TX[0])); // 
		if (out < 0) {
			printf("[ERROR] UART TX\n");
		}
		else {
			printf("[STATUS: TX %i Bytes] %s\n", out, BUF_TX);
		}
	} // if uart0
	//---------------------------------------------------------------------
	sleep(1);
	//---------------------------------------------------------------------
	// start reading bytes after "stop" is transmitted to STM32
	// Bytes empfangen
	if (uart0_filestream != -1) {
		unsigned char BUF_RX[24003];
		int rx_length = read(uart0_filestream, (void*)BUF_RX, 24003);

		if (rx_length < 0) {
			printf("[ERROR] UART RX\n");
		}
		else if (rx_length == 0) {
			printf("[ERROR] UART RX - no data\n");
		}
		else {
			BUF_RX[rx_length] = '\0';
			printf("[STATUS: RX %i Bytes] %s\n", rx_length, BUF_RX);
		} //rx_length check
	} //if uart0

	//---------------------------------------------------------------------
	// save data from UART
	FILE *file1;
	const char *filename1 = "data.txt";

	//Write new file
	file1 = fopen(filename1, "wb");
	if (file1)
	{
		printf("Writing new file\n");

		fwrite(&BUF_RX[0], sizeof(unsigned char), 24003, file1);

		fclose(file1);
		file1 = NULL;
	}
	else
	{
		printf("No such file exists\n");
	}
	//---------------------------------------------------------------------
	close(uart0_filestream);
	return 0;
}