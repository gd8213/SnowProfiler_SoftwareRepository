# SnowProfiler SoftwareRepository

Hello User, 

this is the code repository of the snow profiler. It consists of three parts: 
- the [Arduino](Arduino) (Force), 
- the [Steval Sensor Tile](Raspberry) (IMU) and 
- the [Raspberry](StevalSensorTile) (Main Unit).
 
 

## How to use the software
### Prerequirements:
- Firmware is uploaded on Steval Sensor Tile and Arduino -> Should be done
- Laptop or PC in the same Network as the Snow Profiler (Attention: No WLAN is available. Use a good, old patch cable)
- SSH Client on your machine
- SSH connection established. User: `pi` and password: `raspberry`

### Run the software

Navigate to the code folder 
```
cd SnowProfiler_SoftwareRepository/Raspberry/
```

build firmware if needed
```
bash buildFirmware.sh
```

run executable
```
./Raspby_Firmware
```

if everything is done correct, the output should look like the following example. Now the measurements can be done without further interventions in the software.

```
-----------------------------
Raspy Firmware is starting...
-----------------------------



----- State Changed to: probeInit --------

Raspy Firmware is starting up...
Initialize Functions...
Open I2C bus to arduino...
Initialize PWM...
Init of Arduino's IMU...
Set PWM signals...
Set Duty Cycle for pin 12 to 50 percent.
Set Duty Cycle for pin 13 to 0 percent.

----- State Changed to: probeMoving -------

First freefall event detected...
Second freefall event detected...
FREEFALL confirmed...

----- State Changed to: freeFall ---------

Start Measurement PWM...
Prepare File name with current time...
Set Duty Cycle for pin 13 to 50 percent.

----- State Changed to: deceleration -------

First deceleration event detected...
Second deceleration event detected...
First deceleration event detected...
Second deceleration event detected...
STOP confirmed...

----- State Changed to: stop -----------

Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.
Disable Measurement PWM and waiting to recovery...
Set Duty Cycle for pin 13 to 0 percent.

----- State Changed to: probeRecovery ------

Start Cam recording...
Set light to 100 percent...
Start Video Recording...
Read measurement data...
Read force data from Arduion...
Received Data from Arduino 16384/16384.
TX Bytes send: 7
Save data to file...
Finished gathering data and recording
Set light to 0 percent...

----- State Changed to: probeMoving -------
```
