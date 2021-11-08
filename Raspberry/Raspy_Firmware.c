#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>     // for read/write of I2C
#include <stdlib.h>     // USB cam system call
#include <time.h>       // Get date and time to save recording
#include <string.h>     // Modify File name
#include <wiringPi.h>   // GPIO -> see GPIO_Raspy.c for needed Setup
#include <stdbool.h>    // use bool

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define DEBUG
#define RASPY_4         // Remove on CM3 !!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define FORCE_SIZE 4096
#define ARDUINO_I2C_ADDR 0x05

#ifdef RASPY_4
    #define PWM_CLOCK 54000000  // For Raspy 4
#else
    #define PWM_CLOCK 19200000  // For Raspy 3/3++
#endif

#define PWM_PIN_IR 26            // wiring pi pin -> PWM0
#define PWM_PIN_MEAS 23         // wiring pi pin -> PWM1


enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };
enum ProbeState state = probeInit;

int fd_i2c = -1;

float forceVec[FORCE_SIZE] = { 0 };
float accelVec[FORCE_SIZE] = { 0 };
float timeVec[FORCE_SIZE] = { 0 };

struct tm tm;       // Time struct

// Function Prototypes
int OpenI2C();
int ReadForceVector();
int WriteToArduino();

int DetectFreeFallIMU();
int ReadAccelVector();

int PrepareDataFileName();
int StartCamRecording();
int SaveDataToCSV();

int InitPWM();
int SetDutyCyclePWM(int pin, int dutyCycle);





int OpenI2C() {
    // Open I2C Bus
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
    int shouldLength = sizeof(forceVec[0])*FORCE_SIZE;                        //<<< Number of bytes to read
    int readLength = 0;
    for(int i = 0; i < FORCE_SIZE; i++) {
        int actualRead = read(fd_i2c, &forceVec[i], sizeof(forceVec[i]));
        usleep(500);        // Wait some time for communication
        readLength = readLength + actualRead;
    } 

    printf("Received Byte from Arduino %d/%d \r\n", readLength, shouldLength);
    if (shouldLength != readLength) {
        return -1;
    } 

    return 0;

    /*
    length = sizeof(forceVec[0])*FORCE_SIZE;                        //<<< Number of bytes to read
    int actualRead = read(fd_i2c, forceVec, length);
    if (actualRead != length) {          //read() returns the number of bytes actually read, if it doesn't match then an error oc$
            //ERROR HANDLING: i2c trcccansaction failed
            printf("Failed to read from the i2c bus.\n");
            return -1;
    } 
    */
}

int ReadAccelVector() {
    // ToDo Get Acceleration from IMU
    return 0;
}

int WriteToArduino() {
    // Write Bytes to Arduino 
    if (ioctl(fd_i2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING; you can check errno to see what went wrong
            return -1;
    }

    unsigned char buffer[6] = {0};
    buffer[0] = 0x00;
    buffer[1] = 0x01;
    int length = 1;                     //<<< Number of bytes to write

    int actual = write(fd_i2c, buffer, length);
    if (actual != length)
//write() returns the number of bytes actually written, if it doesn't match then an erro$
    {
            // ERROR HANDLING: i2c transaction failed
            printf("Failed to write to the i2c bus.\n");
    }

    return 0;
}

int PrepareDataFileName() { 
    time_t t = time(NULL);
    tm = *localtime(&t);
    //struct tm tm = *localtime(&t);

    return 0;
} 

int StartCamRecording() {
    printf("Start Video Recording...");

    // Prepare Filename
    char fileName[] = "Data/yyyy-mm-dd_hh-mm-ss.mjpg";
    sprintf(fileName, "Data/%d-%02d-%02d_%02d-%02d-%02d.mjpg", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);


    // Prepare Command
    // ffmpeg -s 640x480 -i /dev/video0 -c:v copy -c:a copy -y -t 5 output.avi 		// This one is the best; decrease resolution to get better fps
    // ffmpeg -t 30 -f v4l2 -framerate 50 -video_size 1600x1200 -y -i /dev/video0 
    char command[] = "ffmpeg -s 640x480 -i /dev/video0 -c:v copy -c:a copy -y -t 30 ";
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

int InitPWM() {
    // Initialize PWM for 2kHz measurement -> pwm carrier 1kHz
    // Use system call because library has problems with root
    printf("Initialize PWM... \r\n");

    // Set pin to PWM
    char command[] = "gpio mode XX pwm";
    sprintf(command, "gpio mode %2d pwm", PWM_PIN_MEAS);
    system(command);
    sprintf(command, "gpio mode %2d pwm", PWM_PIN_IR);
    system(command);

    // Pin mode to PWM
    system("sudo gpio pwm-ms");

    // Set PWM carrier frequency with resolution
    #ifdef RASPY_4
        // Range=2000, Clock=27, f_pwm=1kHz
        system("sudo gpio pwmc 27"); 
        system("sudo gpio pwmr 2000");
        for (int i = 0; i < FORCE_SIZE; i++) {
            timeVec[i] = (float) 1/(1000*2)*i; 
        } 
    #else
        // Range=2000, Clock=9, f_pwm=1.0666kHz
        system("gpio pwmc 9"); 
        system("gpio pwmr 2000");
        for (int i = 0; i < FORCE_SIZE; i++) {
            timeVec[i] = (float) 1/(1066.666*2)*i;
        } 
    #endif

    return 0;
} 

int SetDutyCyclePWM(int pin, int dutyCycle){  
    // dutyCycle 0...100
    // Set PWM on Raspy
    // https://raspberrypi.stackexchange.com/questions/4906/control-hardware-pwm-frequency
    if (dutyCycle > 100){
        dutyCycle = 100;
    } else if (dutyCycle < 0){
        dutyCycle = 0;
    } 

    printf("Set Duty Cycle for pin %d to %d percent. \r\n", pin, dutyCycle);

    char command[] = "sudo gpio pwm xx xxxx";
    sprintf(command, "sudo gpio pwm %2d %4d", pin, 2000/100*dutyCycle);
    
    return system(command);
}


int SaveDataToCSV() {

   printf("Save data to file... \r\n");

    // Prepare File name
    char fileName[] = "Data/yyyy-mm-dd_hh-mm-ss.csv";
    sprintf(fileName, "Data/%d-%02d-%02d_%02d-%02d-%02d.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);


    // Write to CSV-file -> https://stackoverflow.com/questions/14916527/writing-to-a-csv-file-in-c/14916559
    FILE *filePtr;
    filePtr = fopen(fileName, "w");

    if (filePtr == NULL){ 
        // File was not opened
        printf("ERROR: Failed to open file. \r\n");
        return -1;
    } 

    fprintf(filePtr, "Time[s],Acceleration [m/s2],Force [N]  \r\n");
    for(int i = 0; i < FORCE_SIZE; i++) { 
        fprintf(filePtr, "%.4f,%.3f,%.3f \r\n", timeVec[i], accelVec[i], forceVec[i]);
    } 

    fclose(filePtr);
    return 0;
} 


int DetectFreeFallIMU() {
    // ToDo Get Acceleration State from IMU
    // Return 1 on freefall, 0 on g, -1 else
    return 0;
} 


int main() {
    printf("Hello, World! \r\n");
    
    // WiringPi Setup
    int res = wiringPiSetup();
    if (res < 0){ 
        printf("ERROR: GPIO init failed.");
    } 
    

    // Testing the Workflow with Arduino
    OpenI2C();
    res = InitPWM();
    res = SetDutyCyclePWM(PWM_PIN_MEAS, 50);
    
    sleep(2);
    res = SetDutyCyclePWM(PWM_PIN_MEAS, 0);
    ReadForceVector();

    PrepareDataFileName();
    SaveDataToCSV();

    

#ifndef DEBUG
    switch (state) {
        case probeInit:    
            printf("--- State Init. \r\n");
            OpenI2C();
            InitPWM();
            SetDutyCyclePWM(PWM_PIN_IR, 50);        // PWM for IR-LED
            SetDutyCyclePWM(PWM_PIN_MEAS, 0);
            state = probeMoving;
            break;

        case probeMoving:   
            printf("--- State Probe Moving to Location. \r\n");
            int freefall = DetectFreeFallIMU();
            if (freefall > 0){
                // Freefall detected
                state = freeFall;
            } else { 
                state = probeMoving;
            }             
            break;

        case freeFall:      
            printf("--- State Free Fall. \r\n");
            SetDutyCyclePWM(PWM_PIN_MEAS, 50);        // PWM for Measurement
            state = deceleration;
            break;

        case deceleration:  
            printf("--- State Hit Surface. Deceleration of Probe. \r\n");
            int freefall = DetectFreeFallIMU();
            if (freefall == 0){
                // Stop detected
                state = stop;
            } else { 
                // Still moving
                state = deceleration;
            }   
            break;

        case stop:          
            printf("--- State Probe stoped moving. Prepare for Recovery. \r\n");
            SetDutyCyclePWM(PWM_PIN_MEAS, 0);

			ReadForceVector();
            ReadAccelVector();
            
            int freefall = DetectFreeFallIMU();
            if (freefall == -1){
                // Started moving upwards
                state = probeRecovery;
            } else { 
                // Still stop
                state = stop;
            }  
            break;

        case probeRecovery: 
            printf("--- State recovering Probe. \r\n");
            StartCamRecording();
            SaveDataToCSV();
            state = probeMoving;
            break;
        default:            break;
    }
#endif



    printf("It is geting dark... \r\n");
    return 0;
}



