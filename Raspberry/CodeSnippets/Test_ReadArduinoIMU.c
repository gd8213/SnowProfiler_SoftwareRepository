#include <stdio.h>
#include <stdlib.h>     // USB cam system call
#include <time.h>       // Get date and time to save recording

#include <stdbool.h>    // use bool
#include <string.h>     // Modify File name

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>     // for read/write of I2C


#include <wiringPi.h>   // GPIO -> see GPIO_Raspy.c for needed Setup
#include <wiringPiI2C.h>        // Just add this one



#define IMU_ADDRESS 0x6A

#define IMU_WHO_AM_I_REG       0X0F
#define IMU_CTRL1_XL           0X10
#define IMU_CTRL2_G            0X11

#define IMU_STATUS_REG         0X1E

#define IMU_CTRL6_C            0X15
#define IMU_CTRL7_G            0X16
#define IMU_CTRL8_XL           0X17

#define IMU_OUTX_L_G           0X22
#define IMU_OUTX_H_G           0X23
#define IMU_OUTY_L_G           0X24
#define IMU_OUTY_H_G           0X25
#define IMU_OUTZ_L_G           0X26
#define IMU_OUTZ_H_G           0X27

#define IMU_OUTX_L_XL          0X28
#define IMU_OUTX_H_XL          0X29
#define IMU_OUTY_L_XL          0X2A
#define IMU_OUTY_H_XL          0X2B
#define IMU_OUTZ_L_XL          0X2C
#define IMU_OUTZ_H_XL          0X2D


// http://wiringpi.com/reference/i2c-library/
// http://www.netzmafia.de/skripten/hardware/RasPi/RasPi_I2C.html



int main(){
    printf("Hello From the otter side \r\n");
    int retVal = 0;
    int fd_wPi = wiringPiI2CSetup(IMU_ADDRESS);
    int reg = wiringPiI2CReadReg8(fd_wPi, IMU_WHO_AM_I_REG);

    //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
    retVal = wiringPiI2CWriteReg8(fd_wPi, IMU_CTRL2_G, 0x4C);

    // Set the Accelerometer control register to work at 104 Hz, 4G,and in bypass mode and enable ODR/4
    // low pass filter(check figure9 of LSM6DS3's datasheet)
    retVal = wiringPiI2CWriteReg8(fd_wPi, IMU_CTRL1_XL, 0x4A);

    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    retVal = wiringPiI2CWriteReg8(fd_wPi, IMU_CTRL7_G, 0x00);

    // Set the ODR config register to ODR/4
    retVal = wiringPiI2CWriteReg8(fd_wPi, IMU_CTRL8_XL, 0x09);



    int16_t data[3];
    for (int i = 0; i < 3; i++) {
        data[0] = wiringPiI2CReadReg16(fd_wPi, IMU_OUTX_L_XL);
        data[1] = wiringPiI2CReadReg16(fd_wPi, IMU_OUTY_L_XL);
        data[2] = wiringPiI2CReadReg16(fd_wPi, IMU_OUTZ_L_XL);


        float x = data[0] * 4.0 / 32768.0;
        float y = data[1] * 4.0 / 32768.0;
        float z = data[2] * 4.0 / 32768.0;
        printf("x: %f, y: %f, z: %f \r\n", x,y,z);
    } 




    return 0;
} 