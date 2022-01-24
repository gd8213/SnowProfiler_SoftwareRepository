# Sensortile IMU part
 
Hello User,

in this folder the  [Steval Sensor Tile](Raspberry) (IMU) code are placed. 

-Used IDE STM32CubeIDE 1.7.0

-open the project file in folder StmCubeIDE/Sensortile_IMU/ and build all

-current startup of IMU-code
	-startup sequence (appr. 1s) red LED is blinking
	-when while loop is reached red LED is switched on
	-in while loop MCU is waiting for rising edge of arduino PWM
	-when rising edge occurs data is saved to accel_data_z vector
	-whenever the master (arduino) requires data via UART the MCU(Sensortile) is sending
		the accel_data_z vector to the master

-how to flash the code to Sensortile
	1) connect debugger to Sensortile like it is shown in connectDebugger.jpg
	2) build code & debug on MCU
	3) make sure download is approved successfully


