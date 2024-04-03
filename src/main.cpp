#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h>       // The task scheduling library of choice

#include "boostValveControl.h"
#include "boostValveSetup.h"
#include "sensorsSendReceive.h"

/*
Define pin constants
*/
const byte motorPotentiometerSignalPin = A14;

/*
Define variables
*/
float currentBoostValvePosition;
float boostValvePositionTarget;
float inputPotentiometerVoltage;
float motorPotentiometerVoltage;
int desiredManifoldPressure = 5;  // Boost level in PSI

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

  // Perform checks of travel limits which were determined and don't hold any boost if out of range
}

/*
Main execution loop
*/
void loop() {
  if (ptGetBoostValvePosition.call()) {
    SERIAL_PORT_MONITOR.print("Open percentage is: ");
    SERIAL_PORT_MONITOR.println(getBoostValveOpenPercentage());
  }

  // Get and set current manifold pressure

  // Drive towards target manifold / boost pressure
  
}

/*
TODO:
- Create a new file for dealing with IO
  - Getting of pressure readings in a high frequency way. Updates the array used for determining the 'real' value
  - Maybe the above can hand back the rolling average value ... or we want to do this on it's own less frequent schedule ?
- Create a new file for setting of the requested boost. For now is this just a fixed value ?
*/