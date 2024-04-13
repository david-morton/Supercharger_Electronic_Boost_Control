#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h> // The task scheduling library of choice

#include "boostValveControl.h"
#include "boostValveSetup.h"
#include "calculateDesiredBoost.h"
#include "globalHelpers.h"
#include "sensorsSendReceive.h"
#include "serialCommunications.h"
#include "serialMessageProcessing.h"

/* ======================================================================
   VARIABLES: Debug and stat output
   ====================================================================== */
bool debugMode = false;
bool reportSerialMessageStats = true;

/* ======================================================================
   VARIABLES: Pin constants
   ====================================================================== */
const byte boostValvePositionSignalPin = A14;
const byte manifoldPressureSensorSignalPin = A15;

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
float currentBoostValveOpenPercentage;
float currentManifoldPressureRaw;
float currentTargetBoostPsi;
int currentManifoldTempRaw;

int boostValvePositionReadingMinimum; // Throttle blade closed, hold all of the boost
int boostValvePositionReadingMaximum; // Throttle blade open, release all of the boost

int currentVehicleGear = 0;    // Will be updated via serial comms from master
float currentVehicleSpeed = 0; // Will be updated via serial comms from master
int currentVehicleRpm = 0;     // Will be updated via serial comms from master
bool clutchPressed = true;     // Will be updated via serial comms from master

/* ======================================================================
   OBJECTS: Pretty tiny scheduler objects / tasks
   ====================================================================== */
ptScheduler ptGetBoostValveOpenPercentage = ptScheduler(PT_TIME_1S);
ptScheduler ptGetManifoldPressure = ptScheduler(PT_TIME_100MS);
ptScheduler ptCalculateDesiredBoostPsi = ptScheduler(PT_TIME_100MS);
ptScheduler ptUpdateBoostValveTarget = ptScheduler(PT_TIME_100MS);
ptScheduler ptSerialReadAndProcessMessage = ptScheduler(PT_TIME_5MS);
ptScheduler ptSerialCalculateMessageQualityStats = ptScheduler(PT_TIME_1S);
ptScheduler ptSerialReportMessageQualityStats = ptScheduler(PT_TIME_5S);
ptScheduler ptCheckFaultConditions = ptScheduler(PT_TIME_200MS);

/* ======================================================================
   SETUP
   ====================================================================== */
void setup() {
  SERIAL_PORT_MONITOR.begin(115200); // Hardware serial port for debugging
  while (!Serial) {
  }; // Wait for serial port to open for debug
  SERIAL_PORT_HARDWARE1.begin(500000); // Hardware serial port for comms to 'master'

  // Calibrate travel limits of boost valve
  setBoostValveTravelLimits(&boostValvePositionReadingMinimum, &boostValvePositionReadingMaximum);

  // TODO: Perform checks of travel limits which were determined and don't hold any boost if out of range
  // ie: There is not enough voltage separation between them. Write to some error buffer to output ?
}

/* ======================================================================
   MAIN LOOP
   ====================================================================== */
void loop() {
  // Get the current blade position open percentage
  if (ptGetBoostValveOpenPercentage.call()) {
    currentBoostValveOpenPercentage = getBoostValveOpenPercentage(boostValvePositionSignalPin, &boostValvePositionReadingMinimum, &boostValvePositionReadingMaximum);
  }

  // Get the current manifold pressure (raw sensor reading 0-1023)
  if (ptGetManifoldPressure.call()) {
    currentManifoldPressureRaw = getManifoldPressure(manifoldPressureSensorSignalPin);
  }

  // Calculate serial message quality stats, and set alarm condition if they are bad
  if (ptSerialCalculateMessageQualityStats.call()) {
    serialCalculateMessageQualityStats();
  }

  // Output serial debug stats
  if (ptSerialReportMessageQualityStats.call() && reportSerialMessageStats) {
    serialReportMessageQualityStats();
  }

  // Check to see if we have any serial messages waiting and process if so
  if (ptSerialReadAndProcessMessage.call()) {
    const char *serialMessage = serialGetIncomingMessage();
    int commandIdProcessed = -1;

    // Process the serial message if something was received (detected by start marker <)
    if (serialMessage[0] == '<') {
      commandIdProcessed = serialProcessMessage(serialMessage, &currentVehicleSpeed, &currentVehicleRpm, &currentVehicleGear, &clutchPressed);
    }

    if (commandIdProcessed == 0) { // Master has requested latest info from us
      DEBUG_PRINT("Received request from master for params upadte (command ID 0 message)");
      serialSendCommandId0Response(globalAlarmCritical, currentTargetBoostPsi, currentManifoldPressureRaw, currentManifoldTempRaw);
    }

    if (commandIdProcessed == 1) { // Updated parameters from master
      lastSuccessfulCommandId1Processed = millis();
      DEBUG_PRINT("Successfully processed command ID 1 message (update pushed params from master)");
    }
  }

  // Perform any checks specifically around critical alarm conditions and set flag if needed
  if (ptCheckFaultConditions.call()) {
    checkAndSetFaultConditions();
  }

  // Check if we are in a critical alarm state and need to fail safe. There are a number of conditions we look for to set the alarm:
  //   - Over boosting, unable to maintain the desired target
  //   - Not getting valid data from the master for x time (also accounts for bad quality comms)
  if (globalAlarmCritical == true) {
    // Stop valve motor immediately and allow spring to open it naturally
    // TODO: Actually stop driving the valve
  } else {
    // Calculate the desired boost we should be running and update PID valve control to drive to that target
    if (ptCalculateDesiredBoostPsi.call()) {
      currentTargetBoostPsi = calculateDesiredBoostPsi(currentVehicleSpeed, currentVehicleRpm, currentVehicleGear, clutchPressed);
      DEBUG_PRINT("Setting target boost level to " + String(currentTargetBoostPsi) + "psi");
    }
  }
}

/*
TODO:
- Implement periodic check for error conditions, set a flag which can be passed back to main controller and alarm sounded
- On device boot, if the engine is running (detected without comms from master ideally), fail to wide open valve ... do not even perform calibration etc.
  This is in case watchdog fires and we reboot the boost controller. We don't want to perform calibration at WOT say.
*/