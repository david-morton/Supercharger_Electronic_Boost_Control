#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h> // The task scheduling library of choice

#include "boostValveControl.h"
#include "boostValveSetup.h"
#include "calculateDesiredBoost.h"
#include "sensorsSendReceive.h"

/*
Define pin constants
*/
const byte boostValvePositionSignalPin = A14;
const byte manifoldPressureSensorSignalPin = A15;

/*
Define variables
*/
float currentBoostValveOpenPercentage;
float currentManifoldPressureRaw;

int boostValvePositionReadingMinimum; // Throttle blade closed, hold all of the boost
int boostValvePositionReadingMaximum; // Throttle blade open, release all of the boost

float currentDesiredBoostPsi = 0;

int currentVehicleGear = 29;  // Will be updated via serial comms from master
int currentVehicleSpeed = 54; // Will be updated via serial comms from master
int currentVehicleRpm = 6555; // Will be updated via serial comms from master
bool clutchPressed = false;   // Will be updated via serial comms from master

/*
Define our pretty tiny scheduler objects / tasks
*/
ptScheduler ptGetBoostValveOpenPercentage = ptScheduler(PT_TIME_1S);
ptScheduler ptGetManifoldPressure = ptScheduler(PT_TIME_100MS);
ptScheduler ptCalculateDesiredBoostPsi = ptScheduler(PT_TIME_100MS);

/*
Perform setup actions
*/
void setup() {
  SERIAL_PORT_MONITOR.begin(115200); // Hardware serial port for debugging
  while (!Serial) {
  };                                   // Wait for serial port to open for debug
  SERIAL_PORT_HARDWARE1.begin(500000); // Hardware serial port for comms to 'master'

  // Calibrate travel limits of boost valve
  setBoostValveTravelLimits(&boostValvePositionReadingMinimum, &boostValvePositionReadingMaximum);

  // Perform checks of travel limits which were determined and don't hold any boost if out of range
}

/*
Main execution loop
*/
void loop() {
  // Get the current blade position open percentage
  if (ptGetBoostValveOpenPercentage.call()) {
    currentBoostValveOpenPercentage = getBoostValveOpenPercentage(boostValvePositionSignalPin, &boostValvePositionReadingMinimum, &boostValvePositionReadingMaximum);
  }

  // Get the current manifold pressure (raw sensor reading)
  if (ptGetManifoldPressure.call()) {
    currentManifoldPressureRaw = getManifoldPressure(manifoldPressureSensorSignalPin);
  }

  // Calculate the desired boost we should be running
  if (ptCalculateDesiredBoostPsi.call()) {
    currentDesiredBoostPsi = calculateDesiredBoostPsi(currentVehicleGear, currentVehicleSpeed, currentVehicleRpm, clutchPressed);
    SERIAL_PORT_MONITOR.println(currentDesiredBoostPsi);
  }
}

/*
TODO:
- Create a new file for setting of the requested boost. For now is this just a fixed value ?
- Implement periodic check for error conditions, set a flag which can be passed back to main controller and alarm sounded
  - Also on no master comms for a specified period, open the valve
- Build in robust fail safes in case we get corrupt data on the serial comms from master
*/