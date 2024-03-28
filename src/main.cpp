#include "CytronMotorDriver.h" // Library for the Cytron MDD10 motor driver
#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h> // The task scheduling library of choice

#include "boostValveControl.h"

/*
Define variables
*/
float currentBoostValvePosition;
float boostValvePositionTarget;

/*
Configure the motor driver.
*/
CytronMD boostValveMotor(PWM_DIR, 3, 4);          // PWM = Pin 3, DIR = Pin 4

/*
Define our pretty tiny scheduler objects / tasks
*/
ptScheduler ptGetBoostValvePosition     = ptScheduler(PT_TIME_1S);
ptScheduler ptReadManifoldBoostPressure = ptScheduler(PT_TIME_50MS);

/*
Perform setup actions
*/
void setup() {
  SERIAL_PORT_MONITOR.begin(115200);    // Hardware serial port for debugging
  while (!Serial) { };                  // Wait for serial port to open for debug
  SERIAL_PORT_HARDWARE1.begin(500000);  // Hardware serial port for comms to 'master'
  // Calibrate travel limits of boost valve
  setBoostValveTravelLimits();
}

/*
Main execution loop
*/
void loop() {
  // boostValveMotor.setSpeed(128);                  // Run forward at 50% speed
  // boostValveMotor.setSpeed(-128);                 // Run backward at 50% speed
  if (ptGetBoostValvePosition.call()) {
    SERIAL_PORT_MONITOR.print("Open percentage is: ");
    SERIAL_PORT_MONITOR.println(getBoostValveOpenPercentage());
  }
  
}
