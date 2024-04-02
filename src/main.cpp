#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h>       // The task scheduling library of choice

#include "boostValveControl.h"

/*
Define pin constants
*/
const byte motorPotentiometerSignalPin = A14;
const byte inputPotentiometerSignalPin = A15;

/*
Define variables
*/
float currentBoostValvePosition;
float boostValvePositionTarget;
float inputPotentiometerVoltage;
float motorPotentiometerVoltage;

/*
Define our pretty tiny scheduler objects / tasks
*/
ptScheduler ptGetBoostValvePosition     = ptScheduler(PT_TIME_1S);
ptScheduler ptReadManifoldBoostPressure = ptScheduler(PT_TIME_50MS);

ptScheduler ptGetInputPotentiometerVoltage   = ptScheduler(PT_TIME_1S);
ptScheduler ptGetMotorPotentiometerVoltage   = ptScheduler(PT_TIME_1S);

/*
Define function - Get input potentiometer voltage reading
*/
void getInputPotentiometerVoltage() {
  inputPotentiometerVoltage = analogRead(inputPotentiometerSignalPin) * (5.0 / 1023.0);
  SERIAL_PORT_MONITOR.print("Input potentiometer voltage is: ");
  SERIAL_PORT_MONITOR.println(inputPotentiometerVoltage);
}

/*
Define function - Get motor potentiometer voltage reading
*/
void getMotorPotentiometerVoltage() {
  // Voltage range seems to be from 3.25V to 1.70V
  motorPotentiometerVoltage = analogRead(motorPotentiometerSignalPin) * (5.0 / 1023.0);
  SERIAL_PORT_MONITOR.print("Motor potentiometer voltage is: ");
  SERIAL_PORT_MONITOR.println(motorPotentiometerVoltage);
}

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
  if (ptGetBoostValvePosition.call() && 1 == 2) {
    SERIAL_PORT_MONITOR.print("Open percentage is: ");
    SERIAL_PORT_MONITOR.println(getBoostValveOpenPercentage());
  }

  if (ptGetInputPotentiometerVoltage.call()) {
    getInputPotentiometerVoltage();
  }

  if (ptGetMotorPotentiometerVoltage.call()) {
    getMotorPotentiometerVoltage();
  }

  // Get and set current manifold pressure

  // Get desired manifold / boost pressure

  // Drive towards target manifold / boost pressure
  
}

/*
TODO:
- Create a new file for dealing with IO
  - Getting of pressure readings in a high frequency way. Updates the array used for determining the 'real' value
  - Maybe the above can hand back the rolling average value ... or we want to do this on it's own less frequent schedule ?
- Create a new file for setting of the requested boost. For now is this just a fixed value ?
*/