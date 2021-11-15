#include <stdio.h>
#include <wiringPi.h>

// add "-lwiringPi" to settings.json
// Compiler should look like:
// /usr/bin/gcc -g /home/pi/SnowProfiler_SoftwareRepository/Raspberry/GPIO_Raspy.c -o /home/pi/SnowProfiler_SoftwareRepository/Raspberry/GPIO_Raspy -lwiringPi

// Maybe update gpio
// http://wiringpi.com/wiringpi-updated-to-2-52-for-the-raspberry-pi-4b/

// Get pin number with "gpio readall"

int main (void) {
    printf("Raspberry Pi blink\n\r ");

    if (wiringPiSetup() == -1) { 
        return 1;
    } 

    pinMode(1, OUTPUT) ;         // aka BCM_GPIO pin 17
    digitalWrite(1, 1) ;       // On

    printf("Should be writing \n\r ");

  return 0 ;
}