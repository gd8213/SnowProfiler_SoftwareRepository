#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>         // for read/write of I2C
#include <stdlib.h>         // USB cam system call
#include <math.h>           // Sqrt

#include <termios.h>		// for UART
#include <fcntl.h>			// for UART
#include <unistd.h>			// for UART
#include <sys/ioctl.h>		// for UART
#include <sys/types.h>		// for UART

#include <wiringPi.h>       // GPIO -> see GPIO_Raspy.c for needed Setup
#include <wiringPiI2C.h>    // I2C functions to read Registers -> Arduino IMU

#include <time.h>           // Get date and time to save recording
#include <stdbool.h>        // use bool
#include <string.h>         // Modify File name

#include "LibArduino.c"

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define DEBUG
//#define RASPY_4         // Remove on CM3 !!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define device "/dev/ttyS0" // define UART port to STM32 IMU

#define FORCE_SIZE 4096
#define SAMPLING_FREQUENCY 2000.0
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



enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };
enum ProbeState state = probeInit;
enum ProbeState oldState = probeRecovery;

int recordingLength = 2;       // in sec


int fd_ArduinoI2c = -1;         // I2C to Arduino -> read force
int fd_arduinoIMU = -1;         // I2C to Arduino IMU -> read Acc for state machine
int sfd;						// for UART STM IMU

float forceVec[FORCE_SIZE] = { -1 };    // forceVec[0] => oldest value at t=0
float accelVec[FORCE_SIZE] = { -1 };    // accelVec[0] => oldest value at t=0
float timeVecAcc[FORCE_SIZE] = { -1 };     // timeVec[0] = 0 and increasing
float timeVecForce[FORCE_SIZE] = { -1 };     // timeVec[0] = 0 and increasing


struct tm tm;       // Time struct
unsigned int pwmRange = 0;

// Function Prototypes
int OpenI2C();
int ReadForceVecFromArduino();
int SetCamLightOnArduino(int);

int InitArduinoIMU();
float ReadAccelFromArduino();     // Return g

int InitIMU();                      // Felbi: Init of your IMU
int ReadAccelVectorFromIMU();              // Felbi: This function is called to get the whole acceleration data from your IMU

int PrepareDataFileName();
int StartCamRecording();
int SaveDataToCSV();

int InitPWM();
int SetDutyCyclePWM(int pin, int dutyCycle);

void DebugPrintState();



int OpenI2C() {
    printf("Open I2C bus to arduino... \r\n");
    // Open I2C Bus
    char *filename = (char*)"/dev/i2c-1";
    if ((fd_ArduinoI2c = open(filename, O_RDWR)) < 0) {
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

    if (fd_ArduinoI2c < 0) {
        printf("ERROR: File descriptor is wrong \r\n");
        return -1;
    }

    if (ioctl(fd_ArduinoI2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING
            return -1;
    }



    // Actual I2C Communication
    actualRead = read(fd_ArduinoI2c, &startIndex, sizeof(startIndex));

    for (int i = 0; i < FORCE_SIZE/valuesPerPacket; i++) {
        actualRead = read(fd_ArduinoI2c, &tempVec[i*valuesPerPacket], sizeof(tempVec[0])*valuesPerPacket);
        readLength = readLength + actualRead;
        usleep(1000);
    }
    
    
    printf("Received Data from Arduino %d/%d.\r\n", readLength, shouldLength);
    if (shouldLength != readLength) {
        printf("ERROR: Not all data received. \r\n");
        return -1;
    } 

    // Sort Vector
    for (int i = 0; i < FORCE_SIZE; i++) {
        if (startIndex >= FORCE_SIZE) {
            startIndex = 0;
        } 

        forceVec[i] = tempVec[startIndex];
        startIndex++;  
    } 

    // Remove Starting -66
    int indFirstGood = 0;
    bool foundGood = false;
    for (int i = 0; i < FORCE_SIZE; ) {
        if (forceVec[i] == -66) {
            i++;
        } else {
            if (foundGood == false) {
                indFirstGood = i;
                foundGood = true;
            }
            if (indFirstGood == 0) {
                break;
            }
            forceVec[i-indFirstGood] = forceVec[i];
            forceVec[i] = -66;
        }
    }

   

    return 0;
}

int InitIMU() {
    // ToDo initialization of IMU
    // Paste your code here Felbi

	//###############################################################################
	// start init UART to STM32
	struct termios options;

	sfd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (sfd == -1)
	{
		fprintf(stderr, "unable to open %s\n", device);
		exit(1);
	}
	else
	{
		fcntl(sfd, F_SETFL, FNDELAY);
	}

	tcgetattr(sfd, &options);

	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	cfmakeraw(&options);

	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
	options.c_cflag |= (CLOCAL | CREAD | CS8);
	// options.c_lflag |= (ICANON | ECHO | ECHOE | ISIG );

	options.c_oflag &= ~OPOST;
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 0;

	tcsetattr(sfd, TCSANOW, &options);
	// end init UART to STM32
	//###############################################################################

    return 0;
}

int ReadAccelVectorFromIMU() {
    // ToDo Get Acceleration from IMU
    // Paste your code here Felbi
    // Sort Vector
	
	FILE *ofd;
	int32_t n, i=0;
	u_int8_t k = 0;
	u_int32_t bytes;
	u_int8_t buff[256];
	u_int8_t accelTemp[7];
	int sizeofRxData = 24576; // 4096 * 6;

	// Initialize the serial port
	// initComPort(&sfd, SERDEV);

	//##########################################
	// Write to STM32 (IMU) 
	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'S';
	*p_tx_buffer++ = 't';
	*p_tx_buffer++ = 'o';
	*p_tx_buffer++ = 'p';
	*p_tx_buffer++ = 'p';
	*p_tx_buffer++ = '\r';
	*p_tx_buffer++ = '\n';

		if (sfd != -1)
		{
			int count = write(sfd, &tx_buffer[0], 7);		//Filestream, bytes to write, number of bytes to write
			if (count < 0)
			{
				printf("UART TX error\n");
			}
			else
			{
				printf("TX Bytes send: %i\n", count);
			}
		}

	// ##########################################
	// read STM32 UART data
	// Check if there's any data available to read
		while (i<4096)
		{
			ioctl(sfd, FIONREAD, &bytes);
			if (bytes>=6 )
			{
				// Read what we can
				n = read(sfd, buff, 6);

				if (n < 0)
				{
					printf("Read failed\r\n");
				}
				if (n > 0)
				{
					// Convert Buffer to float.
					// values are received in mg


					buff[n]='\0';
					accelVec[i] = atof(buff);
					// printf("%s, AccelVec: %f\r\n",buff,accelVec[i]);

					

					i++;

				}
			}
		}
		i = 0;


    return 0;
}

int SetCamLightOnArduino(int valuePercent) {      
    // value in range of 0...100 is equal to percent
    // Arduino receives 0...255 where a minimum voltage of 3V is included
    printf("Set light to %d percent... \r\n", valuePercent);

    if (fd_ArduinoI2c < 0) {
        printf("ERROR: File descriptor is wrong \r\n");
        return -1;
    }

    // Write Bytes to Arduino 
    if (ioctl(fd_ArduinoI2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {
            printf("Failed to acquire bus access and/or talk to  slave.\n");
            //ERROR HANDLING; you can check errno to see what went wrong
            return -1;
    }

    int valueToArduino = 0;  // Equal to 0% lightning -> minimum Voltage of LED needed 230
    valueToArduino = 255 * valuePercent/100;
    int actual = write(fd_ArduinoI2c, &valueToArduino, sizeof(valueToArduino));
    if (actual != sizeof(valueToArduino)) {
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

    return 0;
} 

int StartCamRecording() {
    printf("Start Video Recording...\r\n");

    // Prepare Filename
    char fileName[] = "Data/yyyy-mm-dd_hh-mm-ss.avi";
    sprintf(fileName, "Data/%d-%02d-%02d_%02d-%02d-%02d.avi", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);


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
    // Use system call because library has problems with root -> Maybe use sudo
    // https://raspberrypi.stackexchange.com/questions/4906/control-hardware-pwm-frequency
    printf("Initialize PWM... \r\n");

    // Set pin to PWM
    char command[] = "gpio mode XX pwm";
    sprintf(command, "gpio mode %2d pwm", PWM_PIN_MEAS);
    system(command);
    sprintf(command, "gpio mode %2d pwm", PWM_PIN_IR);
    system(command);

    // Pin mode to PWM
    system("gpio pwm-ms");

    // Set PWM carrier frequency with resolution
    #ifdef RASPY_4
        // f_pwm = 54000000 / Clock / Range
        // Range=2000, Clock=27, f_pwm=1kHz
        // Range=3000, Clock=9, f_pwm=2kHz
        pwmRange = 3000;
        system("gpio pwmc 9");        // Clock
        system("gpio pwmr 3000");      // Range
    #else
        // f_pwm = 19200000 / Clock / Range
        // Range=1920, Clock=10, f_pwm=1kHz
        // Range=1920, Clock=5, f_pwm=2kHz
        // Range=1920, Clock=40, f_pwm=250Hz
        pwmRange = 1920;
        system("gpio pwmc 5"); 
        system("gpio pwmr 1920");
    #endif

    // Prepare Time vector with theoretical values
    for (int i = 0; i < FORCE_SIZE; i++) {
        timeVecAcc[i] = (float) 1.0/ SAMPLING_FREQUENCY *i;
        timeVecForce[i] = (float) 1.0/ (SAMPLING_FREQUENCY*2) *i;
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

    char command[] = "gpio pwm xx xxxx";            // Maybe sudo needed
    sprintf(command, "gpio pwm %2d %4d", pin, pwmRange/100*dutyCycle);
    
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

    fprintf(filePtr, "Time [s],Acceleration [mg],Time [s],Force [N]  \r\n");
    for(int i = 0; i < FORCE_SIZE; i++) { 
        fprintf(filePtr, "%.6f,%.3f,%.6f,%.3f \r\n", timeVecAcc[i], accelVec[i], timeVecForce[i], forceVec[i]);
    } 

    fclose(filePtr);
    return 0;
} 

int InitArduinoIMU() {
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


void DebugPrintState() {
    if (oldState != state) {
        oldState = state;

        printf("\r\n");
        printf("----- State Changed to: ");
        switch (state) {
            case probeInit: 
                printf("probeInit -------- \r\n");   break;
            case probeMoving: 
                printf("probeMoving ------- \r\n");   break;
            case freeFall: 
                printf("freeFall --------- \r\n");   break;
            case deceleration: 
                printf("deceleration ------- \r\n");   break;
            case stop: 
                printf("stop ----------- \r\n");   break;
            case probeRecovery: 
                printf("probeRecovery ------ \r\n");   break;
            default: break;
        }    
        printf("\r\n");
    }
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
    if (InitIMU() < 0)              printf("ERROR: InitIMU failed \r\n");
    if (InitArduinoIMU() <0)        printf("ERROR: Arduino IMU failed\r\n");

    if (SetDutyCyclePWM(PWM_PIN_MEAS, 0) < 0)           printf("ERROR: Failed to set Meas PWM to 0 \r\n");
    if (SetCamLightOnArduino(100) < 0)                    printf("ERROR: Failed to set cam light 0 \r\n");
    printf("Raspy Initialization is done \r\n ");
    
    printf("Start PWM for Measurement \r\n");
    if (PrepareDataFileName() < 0)                      printf("ERROR: Failed to prepare File name \r\n");
    if (SetDutyCyclePWM(PWM_PIN_MEAS, 50) < 0)          printf("ERROR: Failed to set Meas PWM to 50 \r\n");

    delay(200);
 //   sleep(3);       // Measurements
    
    printf("Measurement Done \r\n");
    if (SetDutyCyclePWM(PWM_PIN_MEAS, 0) < 0)           printf("ERROR: Failed to set Meas PWM to 0 \r\n");
    if (ReadForceVecFromArduino() < 0)                  printf("ERROR: Failed to read force vec from arduino \r\n");
    if (ReadAccelVectorFromIMU() < 0)                   printf("ERROR: Failed to read accel vec from IMU \r\n");
    if (SaveDataToCSV() < 0)                            printf("ERROR: Failed to save data to CSV \r\n");
    
/*
     printf("Start Cam Recording \r\n");
     if (SetCamLightOnArduino(50) < 0)                    printf("ERROR: Failed to set cam light 50 \r\n");
     if (StartCamRecording() < 0)                        printf("ERROR: Failed to start cam recording \r\n");

     sleep(recordingLength);
    if (SetCamLightOnArduino(0) < 0)                    printf("ERROR: Failed to set cam light 0 \r\n");
    printf("Finished cam Recording \r\n");   
*/

// -----------------------------------------------------------------------------------------


 #else
int freefallDelay = 100;        // in ms
float accel;

while(1) {
    DebugPrintState();

    switch (state) {
        case probeInit:  
            printf("Raspy Firmware is starting up... \r\n");  
            printf("Initialize Functions... \r\n");
            if (wiringPiSetup() < 0)        printf("ERROR: wiringPiSetup Failed! \r\n");
            if (OpenI2C() < 0)              printf("ERROR: OpenI2C failed \r\n");
            if (InitPWM() < 0)              printf("ERROR: InitPWM failed \r\n");
            if (InitIMU() < 0)              printf("ERROR: InitIMU failed \r\n");
	        if (InitArduinoIMU() <0)        printf("ERROR: Arduino IMU failed\r\n");

            printf("Set PWM signals... \r\n");
            if (SetDutyCyclePWM(PWM_PIN_IR, 50) < 0)            printf("ERROR: Failed to set IR PWM to 50 \r\n");    // PWM for IR-LED
            if (SetDutyCyclePWM(PWM_PIN_MEAS, 0) < 0)           printf("ERROR: Failed to set measurement PWM to 0 \r\n");
            state = probeMoving;
            break;

        case probeMoving:   
            state = probeMoving;
            accel = ReadAccelFromArduino();
            if (accel >= -0.2 && accel <= 0.2){
                printf("First freefall event detected... \r\n");
                delay(freefallDelay);
                accel = ReadAccelFromArduino();
                if (accel >= -0.2 && accel <= 0.2) {
                    printf("Second freefall event detected... \r\n");
                    delay(freefallDelay);
                    accel = ReadAccelFromArduino();
                    if (accel >= -0.2 && accel <= 0.2) {
                        // Freefall detected
                        printf("FREEFALL confirmed... \r\n");
                        state = freeFall;
                    } 
                } 
            }            
            break;

        case freeFall:      
            printf("Start Measurement PWM... \r\n");
            if (PrepareDataFileName() < 0)                      printf("ERROR: Failed to prepare data file name \r\n");
            if (SetDutyCyclePWM(PWM_PIN_MEAS, 50) < 0)          printf("ERROR: Failed to set measurement PWM to 50 \r\n");      // PWM for Measurement
delay(200);
            state = deceleration;
            break;

        case deceleration:  
            state = deceleration;
            accel = ReadAccelFromArduino();
            if (accel >= 0.95 && accel <= 1.05){
                printf("First deceleration event detected... \r\n");
                delay(freefallDelay);
                accel = ReadAccelFromArduino();
                if (accel >= 0.95 && accel <= 1.05){
                    printf("Second deceleration event detected... \r\n");
                    delay(freefallDelay);
                    accel = ReadAccelFromArduino();
                    if (accel >= 0.95 && accel <= 1.05){
                        // Stop detected
                        printf("STOP confirmed... \r\n");
                        state = stop;
                    }
                }  
            } 
            break;

        case stop:          
            printf("Disable Measurement PWM and waiting to recovery... \r\n");
            if (SetDutyCyclePWM(PWM_PIN_MEAS, 0) < 0)       printf("ERROR: Failed to set measurement PWM to 0 \r\n");

            state = stop;
            accel = ReadAccelFromArduino();
            if (accel <= 0.95|| accel >= 1.05){
                // Started moving upwards
                state = probeRecovery;
            } 
            break;

        case probeRecovery: 
            printf("Start Cam recording... \r\n");
            if (StartCamRecording() < 0)                        printf("ERRRO: Failed to start Cam recording \r\n");
            printf("Read measurement data... \r\n");
            if (ReadForceVecFromArduino() < 0)                  printf("ERROR: Failed to read force vec from arduino \r\n");
            if (ReadAccelVectorFromIMU() < 0)                   printf("ERROR: Failed to read accel vec from IMU \r\n");
            if (SaveDataToCSV() < 0)                            printf("ERROR: Failed to save data to CSV \r\n");
            sleep(recordingLength);      // Wait until recording is over
            printf("Finished gathering data and recording \r\n");
            state = probeMoving;
            break;
        default:       state = probeInit;     break;
    }
}
#endif



    printf("My batteries are low and it's getting dark... \r\n");
    return 0;
}



