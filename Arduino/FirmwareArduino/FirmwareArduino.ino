// Firmware of the Arduino Nano 33 Iot.
// Version 0.1

// Communication with Raspberry using I2C

// Force Measurement as analog signal: A0
// Synchronization with IR-Signal: D3

// ATTENTION only on Domes Laptop !!!!!!!!!!!!!!!!!!!!
// ADC Speed increase from 1.2kHz to 2.38kHz
// https://forum.arduino.cc/t/nano-iot-33-analogread-is-slower-than-expected/635209


#include <Wire.h>               // I2C Communication with Raspberry


// Constants
#define FORCE_SIZE 4096
#define I2C_ADDR 0x05         // Address in I2C bus
float maxForceRange = 300.0f;   // in Newton -> ToDo Check with Sensor

enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };


// Pin Setup
int analogForcePin = A0;   // A0 - Use whole name for analog pins
int syncSignalPin = 3;   // D3 - Just use number for digital pins


// Global variables
float forceVector[FORCE_SIZE];
ProbeState state = probeInit;
int syncIndex = 0;                  // Stores the index of synchronization -> IR Sensor
int currentForceIndex = 0;          // Stores the index where the measurement should be stored

// Function Prototypes
float ReadForceSensor();                          // Get current Force
float GetForceFromMeasurement( int rawValue);     // Convert Raw data

void SendToRaspyEvent();                // I2C send to Raspy
void GetFromRaspyEvent(int howMany);     // I2C get from Raspy


// ----------------------------------------------------------------------
// ----------------------------------------------------------------------

void setup() {
  // Communication Debug
  Serial.begin(9600);     // For Debugging -> use output window or h-term

  // Setup Communication Raspberry via I2C
  Wire.begin(I2C_ADDR);    // Join Bus
  Wire.onRequest(SendToRaspyEvent);     // register Send to Raspy event
  Wire.onReceive(GetFromRaspyEvent);     // register get from Raspy event

  // Setup Force
  pinMode(analogForcePin, INPUT);
  analogReadResolution(10);   // Set Resolution of Force sensor to max 10 Bit
  // ATTENTION only on Domes Laptop the ADC speed is increased (see above). Default 1.2kHz

  pinMode(syncSignalPin, INPUT_PULLDOWN);   // Default 0
 
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {  


/*
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

*/

delay(250);
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
  // Voltage from 0-3.3V will map to 0-1023 (10Bit resolution). ATTENTION connect only 3.3V
  float maxVoltage = 3.3;

  float voltage = (float) rawValue * maxVoltage / 1023;
  float force = maxForceRange * voltage / 3.3;
  return force;
}


// --------------------------------------------------------------------------------
// -----------------------------------I2C------------------------------------------
// --------------------------------------------------------------------------------

void SendToRaspyEvent() {
  // Write Force vector to Raspy
  Serial.println("Write Force Vector to Raspy...");   // Debug
  Wire.write ( (byte *) &forceVector[0], sizeof(forceVector[0])*FORCE_SIZE);
}

void GetFromRaspyEvent(int howMany) {
  // Not used at the moment -> Maybe for setting light of Endoscope
  while(Wire.available()) { // loop through all but the last
    int c = Wire.read(); // receive byte as a character
    Serial.print("Got something ");
    Serial.print(c);
    Serial.print("\r\n");
  }
}

