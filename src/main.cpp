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
   VARIABLES: Temporary PID debug items, used with 3 potentiometers
   ====================================================================== */
const byte pidPinProportional = A3;
const byte pidPinIntegral = A4;
const byte pidPinDerivative = A5;

int pidRangeMaxProportional = 50;
int pidRangeMaxIntegral = 10;
int pidRangeMaxDerivative = 5;

ptScheduler ptReadPidPotsAndUpdateTuning = ptScheduler(PT_TIME_100MS);

void readPidPotsAndUpdateTuning(PID *pidMotor, double *pidLiveValueProportional, double *pidLiveValueIntegral, double *pidLiveValueDerivative) {
  int pidPotProportionalRaw = getAveragedAnaloguePinReading(pidPinProportional, 10, 0);
  int pidPotIntegralRaw = getAveragedAnaloguePinReading(pidPinIntegral, 10, 0);
  int pidPotDerivativeRaw = getAveragedAnaloguePinReading(pidPinDerivative, 10, 0);

  // Adjust the mapping for higher precision
  double factor = 100.0;

  *pidLiveValueProportional = map(pidPotProportionalRaw, 0, 1023, pidRangeMaxProportional, 0);
  *pidLiveValueIntegral = map(pidPotIntegralRaw, 0, 1023, pidRangeMaxIntegral * factor, 0) / factor;
  *pidLiveValueDerivative = map(pidPotDerivativeRaw, 0, 1023, pidRangeMaxDerivative * factor, 0) / factor;

  pidMotor->SetTunings(*pidLiveValueProportional, *pidLiveValueIntegral, *pidLiveValueDerivative);
}

/* ======================================================================
   VARIABLES: Debug and stat output
   ====================================================================== */
bool debugSerialReceive = false;
bool debugSerialSend = false;
bool debugValveControl = false;
bool debugBoost = false;
bool debugGeneral = false;
bool debugPid = false;

bool reportSerialMessageStats = false;

/* ======================================================================
   VARIABLES: Pin constants
   ====================================================================== */
const byte boostValvePositionSignalPin = A14;
const byte manifoldPressureSensorSignalPin = A15;

/* ======================================================================
   VARIABLES: PID Tuning parameters for valve motor control
   ====================================================================== */
double PressureKp = 30.0; // Proportional term      30
double PressureKi = 0.0;  // Integral term          7
double PressureKd = 0.0;  // Derivative term        2

double PositionKp = 2.5; // Proportional term     2.5
double PositionKi = 5.0; // Integral term        5.0
double PositionKd = 0.0; // Derivative term      0.0

const int maximumReverseMotorSpeed = -100; // This is also hard coded in the setBoostValveTravelLimits function
const int maximumForwardMotorSpeed = 100;  // This is also hard coded in the setBoostValveTravelLimits function

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
double currentBoostValveMotorSpeed = 0;
double currentManifoldPressureAboveAtmosphericPsi;
double currentTargetBoostPsi;
double currentBoostValveOpenPercentage;
float currentManifoldPressureRaw;
int currentManifoldTempRaw = 0;
double currentTargetBoostValveOpenPercentage = 100.0;

float valveControlPositionToPidTransitionFactor = 0.7;

float manifoldPressureAtmosphericOffsetRaw;
float manifoldPressureAtmosphericOffsetPsi;

bool usingPressureControl;
bool usingPositionControl;

int currentBoostValvePositionReadingRaw;
int boostValvePositionReadingMinimumRaw; // Throttle blade closed, hold all of the boost
int boostValvePositionReadingMaximumRaw; // Throttle blade open, release all of the boost

int currentVehicleGear = 0;    // Will be updated via serial comms from master
float currentVehicleSpeed = 0; // Will be updated via serial comms from master
int currentVehicleRpm = 0;     // Will be updated via serial comms from master
bool clutchPressed = true;     // Will be updated via serial comms from master

unsigned long arduinoLoopExecutionCount = 0;
unsigned long arduinoLoopExecutionPreviousExecutionMillis;

/* ======================================================================
   OBJECTS: Configure the motor driver board and PID objects
   ====================================================================== */
CytronMD boostValveMotorDriver(PWM_DIR, 9, 8);
PID boostValvePressurePID(&currentManifoldPressureAboveAtmosphericPsi, &currentBoostValveMotorSpeed, &currentTargetBoostPsi, PressureKp, PressureKi, PressureKd, REVERSE);
PID boostValvePositionPID(&currentBoostValveOpenPercentage, &currentBoostValveMotorSpeed, &currentTargetBoostValveOpenPercentage, PositionKp, PositionKi, PositionKd, DIRECT);

/* ======================================================================
   OBJECTS: Pretty tiny scheduler objects / tasks
   ====================================================================== */
// High frequency tasks
ptScheduler ptCalculatePidAndDriveValve = ptScheduler(PT_TIME_10MS);
ptScheduler ptGetBoostValveFeedbackPosition = ptScheduler(PT_TIME_10MS);
ptScheduler ptGetManifoldPressure = ptScheduler(PT_TIME_50MS);
ptScheduler ptSerialReadAndProcessMessage = ptScheduler(PT_TIME_10MS);

// Medium frequency tasks
ptScheduler ptCalculateDesiredBoostPsi = ptScheduler(PT_TIME_200MS);
ptScheduler ptCheckFaultConditions = ptScheduler(PT_TIME_200MS);
ptScheduler ptOutputTargetAndCurrentBoostDebug = ptScheduler(PT_TIME_500MS);

// Low frequency tasks
ptScheduler ptSerialCalculateMessageQualityStats = ptScheduler(PT_TIME_5S);
ptScheduler ptSerialReportMessageQualityStats = ptScheduler(PT_TIME_5S);
ptScheduler ptMonitorArduinoExecutionTime = ptScheduler(PT_TIME_5S);

/* ======================================================================
   SETUP
   ====================================================================== */
void setup() {
  SERIAL_PORT_MONITOR.begin(115200); // Hardware serial port for debugging
  while (!Serial) {
  }; // Wait for serial port to open for debug
  SERIAL_PORT_HARDWARE1.begin(500000); // Hardware serial port for comms to 'master'

  // Get atmospheric reading from manifold pressure sensor before engine starts
  manifoldPressureAtmosphericOffsetRaw = getAveragedAnaloguePinReading(manifoldPressureSensorSignalPin, 20, 0);
  manifoldPressureAtmosphericOffsetPsi = calculatePsiFromRaw(manifoldPressureAtmosphericOffsetRaw);

  // Output atmospheric readings
  SERIAL_PORT_MONITOR.println("\nINFO: Setting current atospheric pressure offset ... ");
  SERIAL_PORT_MONITOR.print("  Set at ");
  SERIAL_PORT_MONITOR.print(manifoldPressureAtmosphericOffsetPsi);
  SERIAL_PORT_MONITOR.println("psi\n");

  // Calibrate travel limits of boost valve (drive against full open / closed and record readings)
  setBoostValveTravelLimits(&boostValveMotorDriver, &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw);

  // Initialize the PID controller and set the motor speed limits
  boostValvePressurePID.SetMode(AUTOMATIC);
  boostValvePressurePID.SetOutputLimits(maximumReverseMotorSpeed, maximumForwardMotorSpeed);

  boostValvePositionPID.SetMode(AUTOMATIC);
  boostValvePositionPID.SetOutputLimits(maximumReverseMotorSpeed, maximumForwardMotorSpeed);
}

/* ======================================================================
   MAIN LOOP
   ====================================================================== */
void loop() {
  // Get the current boost valve blade position as a raw reading and update percentage
  if (ptGetBoostValveFeedbackPosition.call()) {
    currentBoostValvePositionReadingRaw = getAveragedAnaloguePinReading(boostValvePositionSignalPin, 20, 0);
    currentBoostValveOpenPercentage = getBoostValveOpenPercentage(&currentBoostValvePositionReadingRaw, &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw);
  }

  // Get the current manifold pressure as raw sensor reading (0-1023) and convert to psi above atmospheric
  if (ptGetManifoldPressure.call()) {
    currentManifoldPressureRaw = getAveragedAnaloguePinReading(manifoldPressureSensorSignalPin, 20, 0);
    currentManifoldPressureAboveAtmosphericPsi = calculatePsiFromRaw(currentManifoldPressureRaw - manifoldPressureAtmosphericOffsetRaw);
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

  // Update PID valve control to drive to target boost or position as needed
  // If critical alarm is set, stop the motor and let the return spring open the valve to 'fail safe'
  if (ptCalculatePidAndDriveValve.call()) {
    if (globalAlarmCritical) {
      boostValveMotorDriver.setSpeed(0);
    } else if (currentTargetBoostPsi == 0) {
      currentTargetBoostValveOpenPercentage = 100; // Fully open, release the boost
      // Some debug for when switching control modes
      if (usingPressureControl == true) {
        usingPositionControl = true;
        usingPressureControl = false;
        DEBUG_PID("Switching control mode to POSITIONAL");
      }
      driveBoostValveToTargetByOpenPercentagePid(&boostValveMotorDriver, &boostValvePositionPID, &currentBoostValveOpenPercentage,
                                                 &currentBoostValveMotorSpeed, &currentTargetBoostValveOpenPercentage);
    } else if (currentTargetBoostPsi > 0) {
      if (currentManifoldPressureAboveAtmosphericPsi < (currentTargetBoostPsi * valveControlPositionToPidTransitionFactor)) {
        currentTargetBoostValveOpenPercentage = 0; // Fully closed, hold the boost until we are closer to target
        // Some debug for when switching control modes
        if (usingPressureControl == true) {
          usingPositionControl = true;
          usingPressureControl = false;
          DEBUG_PID("Switching control mode to POSITIONAL");
        }
        driveBoostValveToTargetByOpenPercentagePid(&boostValveMotorDriver, &boostValvePositionPID, &currentBoostValveOpenPercentage,
                                                   &currentBoostValveMotorSpeed, &currentTargetBoostValveOpenPercentage);
      } else {
        // Some debug for when switching control modes
        if (usingPositionControl == true) {
          usingPositionControl = false;
          usingPressureControl = true;
          DEBUG_PID("Switching control mode to PRESSURE");
        }
        driveBoostValveToTargetByPressurePid(&boostValveMotorDriver, &boostValvePressurePID, &currentBoostValveMotorSpeed,
                                             &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw,
                                             &currentBoostValvePositionReadingRaw);
      }
    }
  }

  // Some temporary debug that may remain in place
  if (ptOutputTargetAndCurrentBoostDebug.call()) {
    DEBUG_PID("Target boost is " + String(currentTargetBoostPsi) + "psi and current is " + String(currentManifoldPressureAboveAtmosphericPsi) + "psi");
    // THIS NEEDS TO BE CHANGED BACK TO DEBUG_BOOST
    DEBUG_PID("Proportional value: " + String(PressureKp, 2) + " Integral value: " + String(PressureKi, 2) + " Derivative value: " + String(PressureKd, 2) + "\n");
    DEBUG_PID("Current open percentage: " + String(currentBoostValveOpenPercentage, 2) + " Target open percentage: " + String(currentTargetBoostValveOpenPercentage, 2) + "\n");
  }

  // Used for tuning PID values using potentiometers to adjust P, I and D values
  if (ptReadPidPotsAndUpdateTuning.call()) {
    readPidPotsAndUpdateTuning(&boostValvePressurePID, &PressureKp, &PressureKi, &PressureKd);
  }

  // Update counter for execution frequency metrics
  if (millis() > 10000) {
    arduinoLoopExecutionCount++;
  }

  if (ptMonitorArduinoExecutionTime.call() && millis() > 9000) {
    float loopFrequencyHz = (arduinoLoopExecutionCount / ((millis() - arduinoLoopExecutionPreviousExecutionMillis) / 1000));
    float loopExecutionMs = (millis() - arduinoLoopExecutionPreviousExecutionMillis) / arduinoLoopExecutionCount;
    SERIAL_PORT_MONITOR.print("Loop execution frequency (Hz): ");
    SERIAL_PORT_MONITOR.print(loopFrequencyHz);
    SERIAL_PORT_MONITOR.print(" or every ");
    SERIAL_PORT_MONITOR.print(loopExecutionMs);
    SERIAL_PORT_MONITOR.println("ms");
    arduinoLoopExecutionCount = 1;
    arduinoLoopExecutionPreviousExecutionMillis = millis();
  }
}
