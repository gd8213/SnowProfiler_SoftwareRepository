#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>     // for read/write
#include <errno.h>


int file_i2c = -1;
int length;
int i;
unsigned char buffer[6] = {0};


//----- OPEN THE I2C BUS -----
int openi2cbus()
{
        char *filename = (char*)"/dev/i2c-1";
        if ((file_i2c = open(filename, O_RDWR)) < 0)
        {
                //ERROR HANDLING: you can check errno to see what went wrong
                printf("Failed to open the i2c bus");
                return -1;
        }

        return 0;
}

int readi2cbus()
{
        float buff[6] = {0};  
        int addr = 0x05;          //<<<<<The I2C address of the slave
        if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
        {
                printf("Failed to acquire bus access and/or talk to  slave.\n");
                //ERROR HANDLING; you can check errno to see what went wrong
                return -1;
        }


        //----- READ BYTES -----
        length = sizeof(buff);                        //<<< Number of bytes to read
        if (read(file_i2c, buff, length) != length)           //read() returns the number of bytes actually read, if it doesn't match then an error oc$
        {
                //ERROR HANDLING: i2c trcccansaction failed
                printf("Failed to read from the i2c bus.\n");
        }
        else
        {
                for(i=0;i<=length;i++)                 {
                printf("Data read: %x\n", buff[i]);
                }
        }

        return 0;
}

//----- WRITE BYTES -----

int writei2cbus()
{
        int addr = 0x05;          //<<<<<The I2C address of the slave
        if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
        {
                printf("Failed to acquire bus access and/or talk to  slave.\n");
                //ERROR HANDLING; you can check errno to see what went wrong
                return -1;
        }

        buffer[0] = 0x00;
        buffer[1] = 0x01;
        length = 1;                     //<<< Number of bytes to write

        int actual = write(file_i2c, buffer, length);
        if (actual != length)
//write() returns the number of bytes actually written, if it doesn't match then an erro$
        {
                // ERROR HANDLING: i2c transaction failed
                printf("Failed to write to the i2c bus.\n");
                printf("%d, %d \r\n", actual, errno);
        }

        return 0;
}



int main()
{
        printf("Hello, World! \r\n");
        openi2cbus();
        writei2cbus();
        readi2cbus();

        printf("Goodbye \r\n");
        return 0;
}



