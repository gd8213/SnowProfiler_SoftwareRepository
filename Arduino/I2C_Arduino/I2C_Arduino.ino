// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>


void setup() {
  Serial.begin(9600);
  Wire.begin(0x05);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  Serial.print("I life \r\n");
  delay(10000);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {

  Serial.print("I write \r\n");
  Wire.write("Hello ");
  // as expected by master
}

void receiveEvent(int howMany) {
  digitalWrite(LED_BUILTIN, HIGH);
  while(Wire.available()) { // loop through all but the last
    int c = Wire.read(); // receive byte as a character
    Serial.print("Got something ");
    Serial.print(c);
    Serial.print("\r\n");
  }
}
