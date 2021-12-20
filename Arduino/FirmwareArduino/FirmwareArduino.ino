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
float maxForceRange = 1000.0f;   // in Newton -> ToDo Check with Sensor

enum ProbeState { probeInit, probeMoving, freeFall, deceleration, stop, probeRecovery };


// Pin Setup
int analogForcePin = A0;    // A0 - Use whole name for analog pins
int syncSignalPin = 3;      // D3 - Just use number for digital pins
int pwmInterruptPin = 9;   // D9 - https://www.arduino.cc/reference/de/language/functions/external-interrupts/attachinterrupt/
int analogCamLightPin = A2; // A2 - Analog Value to set lightning of camera

// Global variables
float forceVector[FORCE_SIZE];
ProbeState state = probeInit;
int syncIndex = 0;                  // Stores the index of synchronization -> IR Sensor
int currentForceIndex = 0;          // Stores the index where the measurement should be stored

unsigned long prevMillis = 0;        // will store last time LED was updated
unsigned long interMillis = 1000;

// Function Prototypes
float ReadForceSensor();                          // Get current Force
float GetForceFromMeasurement( int rawValue);     // Convert Raw data

void SendToRaspyEvent();                // I2C send to Raspy
void GetLightFromRaspy(int howMany);     // I2C get from Raspy

void TogglePwmInterrupt(bool enable);
void ISR_PwmInterrupt();


// ----------------------------------------------------------------------
// ----------------------------------------------------------------------

void setup() {
  // Communication Debug
  Serial.begin(9600);     // For Debugging -> use output window or h-term
  Serial.print("---- Start Initialization \r\n");

  // Setup Communication Raspberry via I2C
  Serial.print("Initialise I2C... \r\n");
  Wire.begin(I2C_ADDR);    // Join Bus
  Wire.onRequest(SendToRaspyEvent);     // register Send to Raspy event
  Wire.onReceive(GetLightFromRaspy);     // register get from Raspy event

  // Setup Force
  Serial.print("Initialise Pins... \r\n");
  pinMode(analogForcePin, INPUT);
  analogReadResolution(10);   // Set Resolution of Force sensor to max 10 Bit
  // ATTENTION only on Domes Laptop the ADC speed is increased (see above). Default 1.2kHz

  // Sync Pins
  pinMode(syncSignalPin, INPUT_PULLDOWN);   // Default 0
  pinMode(pwmInterruptPin, INPUT_PULLDOWN);
  TogglePwmInterrupt(true);                   // Enable PWM Interrupt

  // Camera light
  pinMode(analogCamLightPin, OUTPUT);
  analogWrite(analogCamLightPin, 0);

  // Show running arduino
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("Initialization finished... \r\n");
  Serial.print("---- Start Running mode \r\n");
}



void loop() {  
  // Just show a runnning Raspy
  unsigned long currMillis = millis();
  static int ledState = LOW;             // ledState used to set the LED

  if (currMillis - prevMillis >= interMillis) {
    // save the last time you blinked the LED
    prevMillis = currMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(LED_BUILTIN, ledState);
  }  
}


// --------------------------------------------------------------------------------
// ---------------------FORCE SENSOR-----------------------------------------------
// --------------------------------------------------------------------------------


float ReadForceSensor() {
  // Get new Value
  float analogValue = GetForceFromMeasurement(analogRead(analogForcePin));

  // Store in Array
  forceVector[currentForceIndex] = analogValue;

/*
  // Check if Sync Signal was set
  static PinStatus oldSync = LOW;
  PinStatus syncSignal = digitalRead(syncSignalPin);
  if (oldSync != syncSignal && syncSignal == HIGH) {
    syncIndex = currentForceIndex;
  }
  oldSync = syncSignal;
*/

  // Prepare Index for next measurement
  currentForceIndex++;                        
  if (currentForceIndex >= FORCE_SIZE) {
    // Overflow
    currentForceIndex = 0; 
  }

  return analogValue;
}


float GetForceFromMeasurement(int rawValue) {
  // Voltage from 0-3.3V will map to 0-1023 (10Bit resolution). ATTENTION connect only 3.3V
  float maxVoltage = 3.3;

  float voltage = (float) rawValue * maxVoltage / 1023;
  float force = maxForceRange * voltage / maxVoltage;
  //float force = maxForceRange * (float) rawValue / 1023.0;
  return force;
}

void TogglePwmInterrupt(bool enable) {
  // Enable or Disable PWM Interrupts
  if (enable == true) {
    attachInterrupt(digitalPinToInterrupt(pwmInterruptPin), ISR_PwmInterrupt, CHANGE);    // RISING, CHANGE
  } else {
     detachInterrupt(digitalPinToInterrupt(pwmInterruptPin));
  }
}

void ISR_PwmInterrupt() {
  ReadForceSensor();
}

// --------------------------------------------------------------------------------
// -----------------------------------I2C------------------------------------------
// --------------------------------------------------------------------------------

void SendToRaspyEvent() {
  // Send 32 values each time what is equal to 128 Bytes
  static int packetIndex = -1;
  int valuesPerPacket = 32;

  if (packetIndex < 0) {
    // First packet is the start index (oldest value)
    Serial.print("Send force vector beginning: ");   // Debug
    Serial.print(currentForceIndex);
    Serial.print(" to Raspy... \r\n");

    Wire.write((byte *) &currentForceIndex, sizeof(currentForceIndex));
  } else {
    // Afterwards only data is sent
    Serial.print("Send force vector packet ");   // Debug
    Serial.print(packetIndex+1);
    Serial.print("/");
    Serial.print(FORCE_SIZE/valuesPerPacket);
    Serial.print(" to Raspy... \r\n");
    Wire.write( (byte *) &forceVector[packetIndex*valuesPerPacket], sizeof(forceVector[0])*valuesPerPacket);
  }


  packetIndex++;
  if (packetIndex >= FORCE_SIZE/valuesPerPacket) {
    Serial.print("Finished Transmission to Raspy. \r\n");  
    packetIndex = -1;
  }

  return;
}

void GetLightFromRaspy(int howMany) {
    unsigned int value = Wire.read(); // receive byte as a character
    Serial.print("Set Camera light to ");
    Serial.print(value);
    Serial.print("/255 \r\n");

    analogWrite(analogCamLightPin, value);    // analogWrite: 0.255
}
