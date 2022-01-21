# Raspberry Part

Hello User,

in this folder the  [Raspberry code](Raspy_Firmware.c) and [measurement results](Data) are placed. 

## Raspy_Firmware.c
This is the main file of the Raspberry Code. It includes the `LibArduino.c` file, where some registers of the Arduino IMU are defined.

## buildFirmware.sh
This bash-script is use to build the firmware on the Snowprofiler. In order to build it use:
```
bash buildFirmware.sh
```
and to run it:
```
./Raspby_Firmware
```


