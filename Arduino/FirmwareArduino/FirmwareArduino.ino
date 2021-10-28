// Firmware of the Arduino Nano 33 Iot.
// Version 0.1

// Communication with Raspberry using SPI. MOSI: D11/29, MISO D12/30, SCL: D10/PWM/28, SS: D13/1
// https://forum.arduino.cc/t/arduino-as-spi-slave/52206/2

// Or I2C
// Raspi Code: https://www.mikrocontroller.net/topic/415271
// Arduino Code: https://www.arduino.cc/en/Tutorial/LibraryExamples/MasterReader


// Force Measurement as analog signal: A0
// Synchronization with IR-Signal: 5/A1

#include <Arduino_LSM6DS3.h> // Accelerometer

// Constants
#define FORCE_SIZE 4096
float maxForceRange = 300.0f;   // in Newton -> ToDo Check with Sensor

enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };


// Pin Setup
// https://docs.arduino.cc/static/a84e4a994129669058db835f17f8ca9d/ABX00032%20(with%20headers)-datasheet.pdf
int analogForcePin = A0;   // A0 - Use whole name for analog pins
int syncSignalPin = 3;   // D3 - Just use number for digital pins


// Global variables
float forceVector[FORCE_SIZE];
ProbeState state = probeInit;
int syncIndex = 0;                  // Stores the index of synchronization -> IR Sensor
int currentForceIndex = 0;          // Stores the index where the measurement should be stored

// Function Prototypes
float ReadForceSensor();
float GetForceFromMeasurement( int rawValue);


void setup() {
  // Communication Debug
  Serial.begin(9600);     // For Debugging -> use output window or h-term

  // Setup Communication Raspberry
  // ToDo

  // Setup Force
  pinMode(analogForcePin, INPUT);
  analogReadResolution(10);   // Set Resolution of Force sensor to max 10 Bit

  pinMode(syncSignalPin, INPUT_PULLDOWN);   // Default 0


  // Setup Gyro and Accerelometer
  IMU.begin();
 
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  // The state machine of the device
  switch (state) {
    case probeInit:    
      Serial.print("--- State Init. \r\n");
      state = probeMoving;
      break;
    case probeMoving:   
      Serial.print("--- State Probe Moving to Location. \r\n");
      state = freeFall;
      break;
    case freeFall:      
      Serial.print("--- State Free Fall. \r\n");
      state = deceleration;
      break;
    case deceleration:  
      Serial.print("--- State Hit Surface. Deceleration of Probe. \r\n");
      ReadForceSensor();
      state = stop;
      break;
    case stop:          
      Serial.print("--- State Probe stoped moving. Prepare for Recovery. \r\n");
      state = probeRecovery;
      break;
    case probeRecovery: 
      Serial.print("--- State recovering Probe. \r\n");
      state = probeMoving;
      break;
    default:            break;
  }


  // Accelerometer and Gyro
  float accX, accY, accZ;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
  }

  float gyroX, gyroY, gyroZ;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }

delay(250);

 /* digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(250);   
  */ 
}


// --------------------------------------------------------------------------------
// ---------------------FORCE SENSOR-----------------------------------------------
// --------------------------------------------------------------------------------


float ReadForceSensor() {
  // Get new Value
  float analogValue = GetForceFromMeasurement(analogRead(analogForcePin));

  // Store in Array
  forceVector[currentForceIndex] = analogValue;

  // Check if Sync Signal was set
  static PinStatus oldSync = LOW;
  PinStatus syncSignal = digitalRead(syncSignalPin);
  if (oldSync != syncSignal && syncSignal == HIGH) {
    syncIndex = currentForceIndex;
  }
  oldSync = syncSignal;

  // Prepare Index for next measurement
  currentForceIndex++;                        
  if (currentForceIndex > FORCE_SIZE) {
    // Overflow
    currentForceIndex = 0; 
  }

  return analogValue;
}


float GetForceFromMeasurement(int rawValue) {
  // ToDo 
  // Voltage from 0-5V will map to 0-1023 (10Bit resolution). ATTENTION connect only 3.3V

  float voltage = (float) rawValue * 5 / 1023;
  float force = maxForceRange * voltage / 3.3;
  return force;
}


// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
