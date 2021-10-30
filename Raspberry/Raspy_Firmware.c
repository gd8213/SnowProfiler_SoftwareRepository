#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>     // for read/write of I2C

#define FORCE_SIZE 4096
#define ARDUINO_I2C_ADDR 0x05

enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };
enum ProbeState state = probeInit;

int fd_i2c = -1;
int length;
int i;

float forceVec[FORCE_SIZE] = { 0 };


//----- OPEN THE I2C BUS -----
int OpenI2C() {
    char *filename = (char*)"/dev/i2c-1";
    if ((fd_i2c = open(filename, O_RDWR)) < 0) {
            //ERROR HANDLING
            printf("Failed to open the i2c bus");
            return -1;
    }

    return 0;
}

int ReadI2C() {
    float buff[6] = {0};  
    int addr = 0x05;          //<<<<<The I2C address of the slave
    if (ioctl(fd_i2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING
            return -1;
    }


    // Read Force Vector
    length = sizeof(forceVec[0])*FORCE_SIZE;                        //<<< Number of bytes to read
    if (read(fd_i2c, forceVec, length) != length)           //read() returns the number of bytes actually read, if it doesn't match then an error oc$
    {
            //ERROR HANDLING: i2c trcccansaction failed
            printf("Failed to read from the i2c bus.\n");
    } else {
            for(i=0;i<=length;i++) {
            printf("Data read: %x\n", buff[i]);
            }
    }

    return 0;
}

//----- WRITE BYTES -----
int WriteI2C() {
    if (ioctl(fd_i2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING; you can check errno to see what went wrong
            return -1;
    }

    unsigned char buffer[6] = {0};
    buffer[0] = 0x00;
    buffer[1] = 0x01;
    length = 1;                     //<<< Number of bytes to write

    int actual = write(fd_i2c, buffer, length);
    if (actual != length)
//write() returns the number of bytes actually written, if it doesn't match then an erro$
    {
            // ERROR HANDLING: i2c transaction failed
            printf("Failed to write to the i2c bus.\n");
    }

    return 0;
}



int main() {
    printf("Hello, World! \r\n");

    switch (state) {
        case probeInit:    
            printf("--- State Init. \r\n");
            state = probeMoving;
            break;
        case probeMoving:   
            printf("--- State Probe Moving to Location. \r\n");
            state = freeFall;
            break;
        case freeFall:      
            printf("--- State Free Fall. \r\n");
            state = deceleration;
            break;
        case deceleration:  
            printf("--- State Hit Surface. Deceleration of Probe. \r\n");
            state = stop;
            break;
        case stop:          
            printf("--- State Probe stoped moving. Prepare for Recovery. \r\n");
            state = probeRecovery;
            break;
        case probeRecovery: 
            printf("--- State recovering Probe. \r\n");
            state = probeMoving;
            break;
        default:            break;
    }

    OpenI2C();
    WriteI2C();
    ReadI2C();

    printf("Goodbye \r\n");
    return 0;
}



