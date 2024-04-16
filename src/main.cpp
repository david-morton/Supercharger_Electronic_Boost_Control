#include "CytronMotorDriver.h"
#include "PID_v1.h"
#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h>

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
bool debugSerialReceive = false;
bool debugSerialSend = false;
bool debugValveControl = true;
bool debugBoost = true;
bool debugGeneral = false;

bool reportSerialMessageStats = false;

/* ======================================================================
   VARIABLES: Pin constants
   ====================================================================== */
const byte boostValvePositionSignalPin = A14;
const byte manifoldPressureSensorSignalPin = A15;

/* ======================================================================
   VARIABLES: PID Tuning parameters for valve motor control
   ====================================================================== */
double Kp = 30.0; // Proportional term      30
double Ki = 0.0;  // Integral term          7
double Kd = 0.0;  // Derivative term        2

const int maximumReverseMotorSpeed = -250; // This is also hard coded in the setBoostValveTravelLimits function
const int maximumForwardMotorSpeed = 250;  // This is also hard coded in the setBoostValveTravelLimits function

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
double currentBoostValveMotorSpeed = 0;
double currentManifoldPressureAboveAtmosphericPsi;
double currentTargetBoostPsi;
float currentBoostValveOpenPercentage;
float currentManifoldPressureRaw;
int currentManifoldTempRaw;

float manifoldPressureAtmosphericOffsetRaw;
float manifoldPressureAtmosphericOffsetPsi;

int currentBoostValvePositionReadingRaw;
int boostValvePositionReadingMinimumRaw; // Throttle blade closed, hold all of the boost
int boostValvePositionReadingMaximumRaw; // Throttle blade open, release all of the boost

int currentVehicleGear = 0;    // Will be updated via serial comms from master
float currentVehicleSpeed = 0; // Will be updated via serial comms from master
int currentVehicleRpm = 0;     // Will be updated via serial comms from master
bool clutchPressed = true;     // Will be updated via serial comms from master

/* ======================================================================
   OBJECTS: Configure the motor driver board and PID object
   ====================================================================== */
CytronMD boostValveMotorDriver(PWM_DIR, 9, 8);
PID boostValvePID(&currentManifoldPressureAboveAtmosphericPsi, &currentBoostValveMotorSpeed, &currentTargetBoostPsi, Kp, Ki, Kd, REVERSE);

/* ======================================================================
   OBJECTS: Pretty tiny scheduler objects / tasks
   ====================================================================== */
ptScheduler ptGetBoostValveOpenPercentage = ptScheduler(PT_TIME_100MS);
ptScheduler ptGetManifoldPressure = ptScheduler(PT_TIME_50MS);
ptScheduler ptCalculateDesiredBoostPsi = ptScheduler(PT_TIME_200MS);
ptScheduler ptCalculatePidAndDriveValve = ptScheduler(PT_TIME_20MS);
ptScheduler ptSerialReadAndProcessMessage = ptScheduler(PT_TIME_5MS);
ptScheduler ptSerialCalculateMessageQualityStats = ptScheduler(PT_TIME_1S);
ptScheduler ptSerialReportMessageQualityStats = ptScheduler(PT_TIME_5S);
ptScheduler ptCheckFaultConditions = ptScheduler(PT_TIME_200MS);
ptScheduler ptGetBoostValvePositionReadingRaw = ptScheduler(PT_TIME_20MS);

ptScheduler ptOutputTargetAndCurrentBoostDebug = ptScheduler(PT_TIME_1S);

/* ======================================================================
   SETUP
   ====================================================================== */
void setup() {
  SERIAL_PORT_MONITOR.begin(115200); // Hardware serial port for debugging
  while (!Serial) {
  };                                   // Wait for serial port to open for debug
  SERIAL_PORT_HARDWARE1.begin(500000); // Hardware serial port for comms to 'master'

  // Get atmospheric reading from manifold pressure sensor before engine starts
  manifoldPressureAtmosphericOffsetRaw = getManifoldPressureRaw(manifoldPressureSensorSignalPin);
  manifoldPressureAtmosphericOffsetPsi = calculatePsiFromRaw(manifoldPressureAtmosphericOffsetRaw);

  // Output atmospheric readings
  SERIAL_PORT_MONITOR.println("\nINFO: Setting current atospheric pressure offset ... ");
  SERIAL_PORT_MONITOR.print("  Set at ");
  SERIAL_PORT_MONITOR.print(manifoldPressureAtmosphericOffsetPsi);
  SERIAL_PORT_MONITOR.println("psi\n");

  // Calibrate travel limits of boost valve
  setBoostValveTravelLimits(&boostValveMotorDriver, &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw);

  // TODO: Perform checks of travel limits which were determined and don't hold any boost if out of range
  // ie: There is not enough voltage separation between them. Write to some error buffer to output ?

  // Initialize the PID controller and set the motor speed limits to something sensible
  boostValvePID.SetMode(AUTOMATIC);
  boostValvePID.SetOutputLimits(maximumReverseMotorSpeed, maximumForwardMotorSpeed);
}

/* ======================================================================
   MAIN LOOP
   ====================================================================== */
void loop() {
  // Get the current boost valve blade position as a raw reading
  if (ptGetBoostValvePositionReadingRaw.call()) {
    currentBoostValvePositionReadingRaw = getBoostValvePositionReadingRaw(&boostValvePositionSignalPin);
  }

  // Get the current blade position open percentage
  if (ptGetBoostValveOpenPercentage.call()) {
    currentBoostValveOpenPercentage = getBoostValveOpenPercentage(&currentBoostValvePositionReadingRaw, &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw);
  }

  // Get the current manifold pressure as raw sensor reading (0-1023) and convert to psi above atmospheric
  if (ptGetManifoldPressure.call()) {
    currentManifoldPressureRaw = getManifoldPressureRaw(manifoldPressureSensorSignalPin);
    currentManifoldPressureAboveAtmosphericPsi = calculatePsiFromRaw(currentManifoldPressureRaw) - manifoldPressureAtmosphericOffsetPsi;
  }

  // Calculate serial message quality stats, and set alarm condition if they are bad
  if (ptSerialCalculateMessageQualityStats.call()) {
    serialCalculateMessageQualityStats();
  }

  // Output serial message quality stats
  if (ptSerialReportMessageQualityStats.call() && reportSerialMessageStats) {
    serialReportMessageQualityStats();
  }

  // Check to see if we have any serial messages waiting and if so, process them
  if (ptSerialReadAndProcessMessage.call()) {
    const char *serialMessage = serialGetIncomingMessage();
    int commandIdProcessed = -1;

    // Process the serial message if something was received (detected by start marker <)
    if (serialMessage[0] == '<') {
      commandIdProcessed = serialProcessMessage(serialMessage, &currentVehicleSpeed, &currentVehicleRpm, &currentVehicleGear, &clutchPressed);
    }

    if (commandIdProcessed == 0) { // Master has requested latest info from us
      DEBUG_SERIAL_SEND("Received request from master for params upadte (command ID 0 message)");
      serialSendCommandId0Response(globalAlarmCritical, currentTargetBoostPsi, currentManifoldPressureAboveAtmosphericPsi, currentManifoldTempRaw);
    }

    if (commandIdProcessed == 1) { // Updated parameters from master
      lastSuccessfulCommandId1Processed = millis();
      DEBUG_SERIAL_RECEIVE("Successfully processed command ID 1 message (update pushed params from master)");
    }
  }

  // Perform any checks specifically around critical alarm conditions and set flag if needed
  if (ptCheckFaultConditions.call()) {
    checkAndSetFaultConditions(&currentManifoldPressureAboveAtmosphericPsi, &currentTargetBoostPsi);
  }

  // Calculate the desired boost we should be running unless critical alarm is set
  // Critical alarm state may be set in a number of ways else where in the code:
  //   - Over boosting, unable to maintain the desired target
  //   - Not getting valid data from the master for x time (also accounts for bad quality comms)
  if (ptCalculateDesiredBoostPsi.call()) {
    if (globalAlarmCritical == true) {
      currentTargetBoostPsi = 0.0;
    } else {
      currentTargetBoostPsi = calculateDesiredBoostPsi(currentVehicleSpeed, currentVehicleRpm, currentVehicleGear, clutchPressed);
    }
  }

  // Update PID valve control to drive to that target unless critical alarm is set in which case, stop the motor and let the sprint open the valve
  if (ptCalculatePidAndDriveValve.call()) {
    if (globalAlarmCritical) {
      boostValveMotorDriver.setSpeed(0);
    } else {
      driveBoostValveToTargetByPressurePid(&boostValveMotorDriver, &boostValvePID, &currentBoostValveMotorSpeed, &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw, &currentBoostValvePositionReadingRaw);
    }
  }

  // Some temporary debug that may remain in place
  if (ptOutputTargetAndCurrentBoostDebug.call()) {
    DEBUG_BOOST("Target boost is " + String(currentTargetBoostPsi) + "psi and current is " + String(currentManifoldPressureAboveAtmosphericPsi) + "psi");
  }
}

/*
TODO:
- Implement periodic check for error conditions, set a flag which can be passed back to main controller and alarm sounded
- On device boot, if the engine is running (detected without comms from master ideally), fail to wide open valve ... do not even perform calibration etc.
  This is in case watchdog fires and we reboot the boost controller. We don't want to perform calibration at WOT say.
- Ensure that the atmospheric check is within sensible bounds, ideally engine confirmed off !! Can setup wait until getting serial data ??
*/
