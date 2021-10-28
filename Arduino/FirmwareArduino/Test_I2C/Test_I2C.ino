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
  Wire.begin(0x5);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
}

void loop() {
  Serial.print("I life \r\n");
  delay(2500);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write("hello "); // respond with message of 6 bytes
  Serial.print("I write \r\n");
  // as expected by master
}
