#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h>                // The task scheduling library of choice
#include "CytronMotorDriver.h"          // Library for the Cytron MDD10 motor driver

/*
Configure the motor driver.
*/
CytronMD throttleBodyMotor(PWM_DIR, 3, 4);          // PWM = Pin 3, DIR = Pin 4.

/*
Perform setup actions
*/
void setup() {
  SERIAL_PORT_MONITOR.begin(115200);    // Hardware serial port for debugging
  while (!Serial) { };                  // Wait for serial port to open for debug
  SERIAL_PORT_HARDWARE1.begin(500000);  // Hardware serial port for comms to 'master'
}

/*
Main execution loop
*/
void loop() {
  throttleBodyMotor.setSpeed(128);                  // Run forward at 50% speed.
  throttleBodyMotor.setSpeed(-128);                 // Run backward at 50% speed.
}
