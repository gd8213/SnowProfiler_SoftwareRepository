#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>         // for read/write of I2C
#include <stdlib.h>         // USB cam system call
#include <math.h>           // Sqrt

#include <wiringPi.h>       // GPIO -> see GPIO_Raspy.c for needed Setup
#include <wiringPiI2C.h>    // I2C functions to read Registers -> Arduino IMU

#include <time.h>           // Get date and time to save recording
#include <stdbool.h>        // use bool
#include <string.h>         // Modify File name

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define DEBUG
#define RASPY_4         // Remove on CM3 !!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define FORCE_SIZE 4096
#define ARDUINO_I2C_ADDR 0x05

#ifdef RASPY_4      // Domes private Raspy 4
    #define PWM_CLOCK 54000000  // For Raspy 4
	#define PWM_PIN_IR 26            // wiring pi pin -> PWM0
	#define PWM_PIN_MEAS 23         // wiring pi pin -> PWM1
#else               // Compute Module or Raspby 3
    #define PWM_CLOCK 19200000  // For Raspy 3/3++, CM3/3+
	#define PWM_PIN_IR 12            // wiring pi pin -> PWM0
	#define PWM_PIN_MEAS 13         // wiring pi pin -> PWM1
#endif



// Arduino IMU
#define ARDUINO_IMU_ADDR 0x6A

#define ARDUINO_IMU_WHO_AM_I_REG       0X0F
#define ARDUINO_IMU_CTRL1_XL           0X10
#define ARDUINO_IMU_CTRL2_G            0X11

#define ARDUINO_IMU_STATUS_REG         0X1E

#define ARDUINO_IMU_CTRL6_C            0X15
#define ARDUINO_IMU_CTRL7_G            0X16
#define ARDUINO_IMU_CTRL8_XL           0X17

#define ARDUINO_IMU_OUTX_L_G           0X22
#define ARDUINO_IMU_OUTX_H_G           0X23
#define ARDUINO_IMU_OUTY_L_G           0X24
#define ARDUINO_IMU_OUTY_H_G           0X25
#define ARDUINO_IMU_OUTZ_L_G           0X26
#define ARDUINO_IMU_OUTZ_H_G           0X27

#define ARDUINO_IMU_OUTX_L_XL          0X28
#define ARDUINO_IMU_OUTX_H_XL          0X29
#define ARDUINO_IMU_OUTY_L_XL          0X2A
#define ARDUINO_IMU_OUTY_H_XL          0X2B
#define ARDUINO_IMU_OUTZ_L_XL          0X2C
#define ARDUINO_IMU_OUTZ_H_XL          0X2D



enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };
enum ProbeState state = probeInit;

int recordingLength = 5;       // in sec

int fd_i2c = -1;
int fd_arduinoIMU = -1;

float forceVec[FORCE_SIZE] = { -1 };
float accelVec[FORCE_SIZE] = { -1 };
float timeVec[FORCE_SIZE] = { -1 };

struct tm tm;       // Time struct
unsigned int pwmRange = 0;

// Function Prototypes
int OpenI2C();
int ReadForceVecFromArduino();
int SetCamLightOnArduino();

int InitArduinoIMU();
float ReadAccelFromArduino();     // Return g
int ReadAccelVector();

int PrepareDataFileName();
int StartCamRecording();
int SaveDataToCSV();

int InitPWM();
int SetDutyCyclePWM(int pin, int dutyCycle);





int OpenI2C() {
    printf("Open I2C bus to arduino... \r\n");
    // Open I2C Bus
    char *filename = (char*)"/dev/i2c-1";
    if ((fd_i2c = open(filename, O_RDWR)) < 0) {
            //ERROR HANDLING
            printf("ERROR: Failed to open the i2c bus \r\n");
            return -1;
    }

    return 0;
}


int ReadForceVecFromArduino() {
    float tempVec[FORCE_SIZE] = { -1 };
    int valuesPerPacket = 32;

    int shouldLength = sizeof(tempVec[0])*FORCE_SIZE;                        //<<< Number of bytes to read
    int readLength = 0;
    int startIndex = -1;        // Last Valid index from arduino -> Sort vector
    int actualRead = -1;

    // Open Bus to Arduino
    printf("Read force data from Arduion... \r\n");

    if (fd_i2c < 0) {
        printf("ERROR: File descriptor is wrong \r\n");
        return -1;
    }

    if (ioctl(fd_i2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING
            return -1;
    }



    // Actual I2C Communication
    actualRead = read(fd_i2c, &startIndex, sizeof(startIndex));

    for (int i = 0; i < FORCE_SIZE/valuesPerPacket; i++){
        actualRead = read(fd_i2c, &tempVec[i*valuesPerPacket], sizeof(tempVec[0])*valuesPerPacket);
        readLength = readLength + actualRead;
        usleep(1000);
    }
    
    
    printf("Received Data from Arduino %d/%d.\r\n", readLength, shouldLength);
    if(shouldLength != readLength){
        printf("ERROR: Not all data received. \r\n");
        return -1;
    } 

    // Sort Vector
    for (int i = 0; i < FORCE_SIZE; i++){
        if(startIndex >= FORCE_SIZE){
            startIndex = 0;
        } 

        forceVec[i] = tempVec[startIndex];
        startIndex++;  
    } 

    return 0;
}

int ReadAccelVector() {
    // ToDo Get Acceleration from IMU
    return 0;
}

int SetCamLightOnArduino(unsigned int value) {
    // Set light with value = 0...255
    printf("Set light to %d... \r\n", value);

    if (fd_i2c < 0) {
        printf("ERROR: File descriptor is wrong \r\n");
        return -1;
    }

    // Write Bytes to Arduino 
    if (ioctl(fd_i2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING; you can check errno to see what went wrong
            return -1;
    }

    
    int actual = write(fd_i2c, &value, sizeof(value));
    if (actual != sizeof(value)) {
//write() returns the number of bytes actually written, if it doesn't match then an erro$
            // ERROR HANDLING: i2c transaction failed
            printf("ERROR: Failed to Set endoscope light.\r\n");
    }

    return 0;
}

int PrepareDataFileName() { 
    printf("Prepare File name with current time...\r\n");
    time_t t = time(NULL);
    tm = *localtime(&t);
    //struct tm tm = *localtime(&t);

    return 0;
} 

int StartCamRecording() {
    printf("Start Video Recording...\r\n");

    // Prepare Filename
    char fileName[] = "Data/yyyy-mm-dd_hh-mm-ss.mjpg";
    sprintf(fileName, "Data/%d-%02d-%02d_%02d-%02d-%02d.mjpg", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);


    // Prepare Command
    // ffmpeg -s 640x480 -i /dev/video0 -c:v copy -c:a copy -y -t 5 output.avi 		// This one is the best; decrease resolution to get better fps
    // ffmpeg -t 30 -f v4l2 -framerate 50 -video_size 1600x1200 -y -i /dev/video0 
    char command[] = "ffmpeg -loglevel quiet -nostdin -s 640x480 -i /dev/video0 -c:v copy -c:a copy -y -t XX ";
    sprintf(command, "ffmpeg -loglevel quiet -nostdin -s 640x480 -i /dev/video0 -c:v copy -c:a copy -y -t %2d ", recordingLength);
    strcat(command,fileName);
    strcat(command," &");       // Run in Background

    int status = system(command); 
    return status;
}

int InitPWM() {
    // Initialize PWM for 2kHz measurement -> pwm carrier 2kHz
    // Use system call because library has problems with root
    // https://raspberrypi.stackexchange.com/questions/4906/control-hardware-pwm-frequency
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
        // f_pwm = 54000000 / Clock / Range
        // Range=2000, Clock=27, f_pwm=1kHz
        // Range=3000, Clock=9, f_pwm=2kHz
        pwmRange = 3000;
        system("sudo gpio pwmc 9");        // Clock
        system("sudo gpio pwmr 3000");      // Range
    #else
        // f_pwm = 19200000 / Clock / Range
        // Range=1920, Clock=10, f_pwm=1kHz
        // Range=1920, Clock=5, f_pwm=2kHz
        pwmRange = 1920;
        system("gpio pwmc 5"); 
        system("gpio pwmr 1920");
    #endif

    // Prepare Time vector with theoretical values
    for (int i = 0; i < FORCE_SIZE; i++) {
        timeVec[i] = (float) 1.0/2000.0*i; 
    } 

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
    sprintf(command, "sudo gpio pwm %2d %4d", pin, pwmRange/100*dutyCycle);
    
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

int InitArduinoIMU(){
    printf("Init of Arduino's IMU... \r\n");
    // Generate file descriptor
    fd_arduinoIMU = wiringPiI2CSetup(ARDUINO_IMU_ADDR);
    if (fd_arduinoIMU < 0) {
        printf("ERROR: File descriptor is wrong\r\n");
        return -1;
    }

    // Set the Accelerometer control register to work at 104 Hz, 4G,and in bypass mode and enable ODR/4
    // low pass filter(check figure9 of LSM6DS3's datasheet)
    int res = wiringPiI2CWriteReg8(fd_arduinoIMU, ARDUINO_IMU_CTRL1_XL, 0x4A);
    if (res != 0)    printf("ERROR: Failed to set accelerometer on Arduino's IMU... \r\n");

    // Set the ODR config register to ODR/4
    res = wiringPiI2CWriteReg8(fd_arduinoIMU, ARDUINO_IMU_CTRL8_XL, 0x09);
    if (res != 0)    printf("ERROR: Failed to set ODR config on Arduino's IMU... \r\n");

    return res;
} 


float ReadAccelFromArduino() {
    // Return accel in g
    printf("Read Arduino's IMU... \r\n");
    if (fd_arduinoIMU < 0) {
        printf("ERROR: Arduino IMU is not initialized with I2C. \r\n");
        return -1;
    }

    float accel[3];        // Acceleration in g

    int16_t data[3];
    float x,y,z;
    for (int i = 0; i < 3; i++) {
        data[0] = wiringPiI2CReadReg16(fd_arduinoIMU, ARDUINO_IMU_OUTX_L_XL);
        data[1] = wiringPiI2CReadReg16(fd_arduinoIMU, ARDUINO_IMU_OUTY_L_XL);
        data[2] = wiringPiI2CReadReg16(fd_arduinoIMU, ARDUINO_IMU_OUTZ_L_XL);

        x = data[0] * 4.0 / 32768.0;
        y = data[1] * 4.0 / 32768.0;
        z = data[2] * 4.0 / 32768.0;
        accel[i]  = sqrt(x*x + y*y + z*z);
    } 
    
    return (accel[0] + accel[1] + accel[2] )/3;
} 


int main() {
    printf("-----------------------------\r\n");
    printf("Raspy Firmware is starting... \r\n");
    printf("-----------------------------\r\n");
    printf("\r\n");
    printf("\r\n");

#ifdef DEBUG


// -----------------------------------------------------------------------------------------
 // Complete Flow

    if (wiringPiSetup() < 0)        printf("ERROR: wiringPiSetup Failed! \r\n");
    if (OpenI2C() < 0)              printf("ERROR: OpenI2C failed \r\n");
    if (InitPWM() < 0)              printf("ERROR: InitPWM failed \r\n");

    if (SetDutyCyclePWM(PWM_PIN_MEAS, 0) < 0)           printf("ERROR: Failed to set Meas PWM to 0 \r\n");
    if (SetCamLightOnArduino(0) < 0)                    printf("ERROR: Failed to set cam light 0 \r\n");
    printf("Raspy Initialization is done \r\n ");
    
    printf("Start PWM for Measurement \r\n");
    if (PrepareDataFileName() < 0)                      printf("ERROR: Failed to prepare File name \r\n");
    if (SetDutyCyclePWM(PWM_PIN_MEAS, 50) < 0)          printf("ERROR: Failed to set Meas PWM to 50 \r\n");
    sleep(3);
    
    printf("Measurement Done \r\n");
    if (SetDutyCyclePWM(PWM_PIN_MEAS, 0) < 0)           printf("ERROR: Failed to set Meas PWM to 0 \r\n");
    if (ReadForceVecFromArduino() < 0)                  printf("ERROR: Failed to read force vec from arduino \r\n");
    if (SaveDataToCSV() < 0)                            printf("ERROR: Failed to save data to CSV \r\n");
    
    printf("Start Cam Recording \r\n");
    if (SetCamLightOnArduino(0) < 0)                    printf("ERROR: Failed to set cam light 0 \r\n");
    if (StartCamRecording() < 0)                        printf("ERROR: Failed to start cam recording \r\n");

    sleep(recordingLength);
    printf("Finished cam Recording \r\n");   


// -----------------------------------------------------------------------------------------


 #else
int freefallDelay = 100;        // in ms
    switch (state) {
        case probeInit:  
            printf("Raspy Firmware is starting up... \r\n");  
            printf("--- State Init. \r\n");
            int res = wiringPiSetup();
            if (res < 0){ 
                printf("ERROR: GPIO init failed.");
            } 
            OpenI2C();
            InitPWM();
            SetDutyCyclePWM(PWM_PIN_IR, 50);        // PWM for IR-LED
            SetDutyCyclePWM(PWM_PIN_MEAS, 0);
            state = probeMoving;
            break;

        case probeMoving:   
            printf("--- State Probe Moving to Location. \r\n");
            state = probeMoving;
            float accel = ReadAccelFromArduino();
            if (accel >= 0 && accel <= 0.2){
                delay(freefallDelay);
                accel = ReadAccelFromArduino();
                if (accel >= 0 && accel <= 0.2) {
                    delay(freefallDelay);
                    accel = ReadAccelFromArduino();
                    if (accel >= 0 && accel <= 0.2) {
                        // Freefall detected
                        state = freeFall;
                    } 
                } 
            }            
            break;

        case freeFall:      
            printf("--- State Free Fall. \r\n");
            PrepareDataFileName();
            SetDutyCyclePWM(PWM_PIN_MEAS, 50);        // PWM for Measurement
            state = deceleration;
            break;

        case deceleration:  
            printf("--- State Hit Surface. Deceleration of Probe. \r\n");
            state = deceleration;
            float accel = ReadAccelFromArduino();
            if (accel >= 0.95 && accel <= 1.05){
                delay(freefallDelay);
                accel = ReadAccelFromArduino();
                if (accel >= 0.95 && accel <= 1.05){
                    delay(freefallDelay);
                    accel = ReadAccelFromArduino();
                    if (accel >= 0.95 && accel <= 1.05){
                        // Stop detected
                        state = stop;
                    }
                }  
            } 
            break;

        case stop:          
            printf("--- State Probe stoped moving. Prepare for Recovery. \r\n");
            SetDutyCyclePWM(PWM_PIN_MEAS, 0);

			ReadForceVecFromArduino();
            ReadAccelVector();
            
            float accel = ReadAccelFromArduino();
            if (accel <= 0.95|| accel >= 1.05){
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
            sleep(recordingLength);      // Wait until recording is over
            state = probeMoving;            
            break;
        default:       state = probeInit;     break;
    }
#endif



    printf("It is getting dark... \r\n");
    return 0;
}



