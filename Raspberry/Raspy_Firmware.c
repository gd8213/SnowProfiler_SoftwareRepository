#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>     // for read/write of I2C
#include <stdlib.h>     // USB cam system call
#include <time.h>       // Get date and time to save recording
#include <string.h>     // Modify File name

#define DEBUG
#define FORCE_SIZE 4096
#define ARDUINO_I2C_ADDR 0x05

enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };
enum ProbeState state = probeInit;

int fd_i2c = -1;
int length;
int i;

float forceVec[FORCE_SIZE] = { 0 };
float accelVec[FORCE_SIZE] = { 0 };


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

int ReadForceVector() {
    // Open Bus to Arduino
    if (ioctl(fd_i2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING
            return -1;
    }

    // Read Force Vector
    length = sizeof(forceVec[0])*FORCE_SIZE;                        //<<< Number of bytes to read
    int actualRead = read(fd_i2c, forceVec, length);
    if (actualRead != length) {          //read() returns the number of bytes actually read, if it doesn't match then an error oc$
            //ERROR HANDLING: i2c trcccansaction failed
            printf("Failed to read from the i2c bus.\n");
            return -1;
    } 

    // Show received Data
    for(i=0; i <= length; i++) {
        printf("Data read: %f\n", forceVec[i]);
    }
    
    return 0;
}

//----- WRITE BYTES -----
int WriteToArduino() {
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

int StartCamRecording() {
    // Prepare Filename
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    char fileName[] = "yyyy-mm-dd_hh-mm-ss.mjpg";
    sprintf(fileName, "%d-%02d-%02d_%02d-%02d-%02d.mjpg", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);


    // Prepare Command
    // ffmpeg -s 1600x1200 -i /dev/video0 -c:v copy -c:a copy -y -t 5 output.avi 		// This one is the best; decrease resolution to get better fps
    // ffmpeg -t 30 -f v4l2 -framerate 50 -video_size 1600x1200 -y -i /dev/video0 
    char command[] = "ffmpeg -i /dev/video0 -c:v copy -c:a copy -y -t 30 ";
    strcat(command,fileName);
    strcat(command," &");
    
    // Take Video
	// & ... run in background
	// -y ... overwrite existing file
	// -t ... duration of video
    //int status = system("ffmpeg -t 30 -f v4l2 -framerate 50 -video_size 1600x1200 -y -i /dev/video0 output.mjpg &"); 
    int status = system(command); 
    return status;
}



int main() {
    printf("Hello, World! \r\n");

    StartCamRecording();

    

    

#ifndef DEBUG
    switch (state) {
        case probeInit:    
            printf("--- State Init. \r\n");
            OpenI2C();
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
			ReadForceVector();
            
            state = probeRecovery;
            break;
        case probeRecovery: 
            printf("--- State recovering Probe. \r\n");
            WriteToArduino();
            StartCamRecording();
            state = probeMoving;
            break;
        default:            break;
    }
#endif



    printf("Goodbye \r\n");
    return 0;
}



