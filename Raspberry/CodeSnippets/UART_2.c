#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>

// Change this to match the machine's serial port
#define SERDEV "/dev/ttyS0"

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
	u_int8_t buff[10000];

	// Initialize the serial port
	initComPort(&sfd, SERDEV);

	while (1)
	{
		// Check if there's any data available to read
		ioctl(sfd, FIONREAD, &bytes);
		if (bytes > 0)
		{
			// Read what we can
			n = read(sfd, buff, 10000);
			
			if (n < 0)
			{
				print("Read failed);
//				fprintf(stderr, "read failed\n");
			}
			if (n > 0)
			{
				print(buff);

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