#include "CytronMotorDriver.h"
#include "PID_v1.h"
#include "pwm.h"
#include <Arduino.h>
#include <Wire.h>
#include <ptScheduler.h>

#include "arduinoSecrets.h"
#include "boostValveControl.h"
#include "boostValveSetup.h"
#include "calculateDesiredBoost.h"
#include "globalHelpers.h"
#include "mqttPublish.h"
#include "sensorsSendReceive.h"
#include "serialCommunications.h"
#include "serialMessageProcessing.h"
#include "wifiHelpers.h"

/* ======================================================================
   VARIABLES: Temporary PID debug items, used with 3 potentiometers
   ====================================================================== */
const byte pidPinProportional = A3;
const byte pidPinIntegral = A4;
const byte pidPinDerivative = A5;

int pidRangeMaxProportional = 75;
int pidRangeMaxIntegral = 20;
int pidRangeMaxDerivative = 20;

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
   VARIABLES: Major functional area toggles
   ====================================================================== */
bool enableWifi = true;
bool enablePotPidTuning = true;
bool enableMqttPublish = true;

/* ======================================================================
   VARIABLES: Debug and stat output
   ====================================================================== */
bool debugSerialReceive = false;
bool debugSerialSend = false;
bool debugValveControl = false;
bool debugBoost = false;
bool debugGeneral = false;
bool debugPid = false;
bool debugPidPlotterOutput = false;

bool reportSerialMessageStats = true;
bool reportArduinoLoopStats = true;

/* ======================================================================
   VARIABLES: Pin constants
   ====================================================================== */
const byte boostValvePositionSignalPin = A0;
const byte manifoldTmapSensorPressureSignalPin = A1;
const byte intakeTmapSensorPressureSignalPin = A2;

/* ======================================================================
   VARIABLES: PID Tuning parameters for valve motor control
   ====================================================================== */
double PressureKp = 25.0; // Proportional term      30    25    19      9
double PressureKi = 2.0;  // Integral term          7     2     4.4     4
double PressureKd = 1.0;  // Derivative term        2     1     1.4     1.4

double PositionKp = 2.5; // Proportional term     2.5
double PositionKi = 5.0; // Integral term        5.0
double PositionKd = 0.0; // Derivative term      0.0

const int maximumReverseMotorSpeed = -150; // This is also hard coded in the setBoostValveTravelLimits function. This is closing the valve against the spring.
const int maximumForwardMotorSpeed = 100;  // This is also hard coded in the setBoostValveTravelLimits function This is opening the valve with the spring.

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
// Related to Bosch TMAP sensor readings
float manifoldPressureAtmosphericOffsetRaw, intakePressureAtmosphericOffsetRaw; // Absolute vs gauge pressures raw
float manifoldPressureAtmosphericOffsetKpa, intakePressureAtmosphericOffsetKpa; // Absolute vs gauge pressures KpA
float currentManifoldPressureAbsoluteRaw, currentIntakePressureAbsoluteRaw;     // Current absolute pressures raw
double currentManifoldPressureGaugeKpa, currentIntakePressureGaugeKpa;          // Current gauge pressures kPa
int currentManifoldTempRaw, currentIntakeTempRaw;                               // Current temperatures raw
int currentManifoldTempCelcius, currentIntakeTempCelcius;                       // Current temperatures ccelcius

// Related to boost control valve
int currentBoostValvePositionReadingRaw, boostValvePositionReadingMinimumRaw, boostValvePositionReadingMaximumRaw;
double currentBoostValveMotorSpeed = 0;
double currentBoostValveOpenPercentage;
double currentTargetBoostValveOpenPercentage = 100.0;
const float valvePositionToPressureControlTransitionFactor = 0.9;
bool usingPressureControl, usingPositionControl;

// Other variables
double currentTargetBoostKpa;

int currentVehicleGear = 0;    // Will be updated via serial comms from master
float currentVehicleSpeed = 0; // Will be updated via serial comms from master
int currentVehicleRpm = 0;     // Will be updated via serial comms from master
bool clutchPressed = true;     // Will be updated via serial comms from master

unsigned long arduinoLoopExecutionCount = 0;
bool mqttIsConnected = false; // Used to avoid trying to send when there is no connection to the broker

/* ======================================================================
   OBJECTS: Configure the motor driver board and PID objects
   ====================================================================== */
CytronMD boostValveMotorDriver(PWM_DIR, 9, 8); // Pin 9 is PWM, on Arduino Mega this is 'timer 2' or OC2B. We update this timer in the setup block
PwmOut cytronPwmPin(D9);                       // Configure a variable which we use to tweak the PWM output frequency later on

PID boostValvePressurePID(&currentManifoldPressureGaugeKpa, &currentBoostValveMotorSpeed, &currentTargetBoostKpa, PressureKp, PressureKi, PressureKd, REVERSE);
PID boostValvePositionPID(&currentBoostValveOpenPercentage, &currentBoostValveMotorSpeed, &currentTargetBoostValveOpenPercentage, PositionKp, PositionKi, PositionKd, DIRECT);

/* ======================================================================
   OBJECTS: Pretty tiny scheduler objects / tasks
   ====================================================================== */
// High frequency tasks
ptScheduler ptCalculatePidAndDriveValve = ptScheduler(PT_TIME_2MS);
ptScheduler ptGetBoostValveFeedbackPosition = ptScheduler(PT_TIME_2MS);
ptScheduler ptGetManifoldPressure = ptScheduler(PT_TIME_10MS);
ptScheduler ptSerialReadAndProcessMessage = ptScheduler(PT_TIME_10MS);

// Medium frequency tasks
ptScheduler ptMqttPublishMetricsToServer = ptScheduler(PT_TIME_50MS);
ptScheduler ptOutputPidDataForLivePlotter = ptScheduler(PT_TIME_50MS);
ptScheduler ptCalculateDesiredBoostKpa = ptScheduler(PT_TIME_200MS);
ptScheduler ptCheckFaultConditions = ptScheduler(PT_TIME_200MS);
ptScheduler ptOutputTargetAndCurrentBoostDebug = ptScheduler(PT_TIME_500MS);

// Low frequency tasks
ptScheduler ptSerialCalculateMessageQualityStats = ptScheduler(PT_TIME_5S);
ptScheduler ptSerialReportMessageQualityStats = ptScheduler(PT_TIME_5S);
ptScheduler ptReportArduinoLoopStats = ptScheduler(PT_TIME_5S);

/* ======================================================================
   SETUP
   ====================================================================== */
void setup() {
  Serial.begin(115200); // Hardware serial port for debugging
  while (!Serial) {
  }; // Wait for serial port to open for debug
  Serial1.begin(500000); // Hardware serial port for comms to 'master'

  // Update PWM frequency via timer for Arduino Mega 2560 and probably Uno R3. Done to prevent motor noise / whine.
  // TCCR2B = (TCCR2B & 0xF8) | 0x01; // 32kHz

  // Update PWM frequency for Arduino Uno R4. Done to prevent motor noise / whine.
  cytronPwmPin.begin(20000.0f, 0.0f);

  // Get atmospheric reading from manifold and intake pressure sensors before engine starts
  manifoldPressureAtmosphericOffsetRaw = getAveragedAnaloguePinReading(manifoldTmapSensorPressureSignalPin, 20, 0);
  intakePressureAtmosphericOffsetRaw = getAveragedAnaloguePinReading(intakeTmapSensorPressureSignalPin, 20, 0);

  manifoldPressureAtmosphericOffsetKpa = calculateBosch3BarKpaFromRaw(manifoldPressureAtmosphericOffsetRaw);
  intakePressureAtmosphericOffsetKpa = calculateBosch3BarKpaFromRaw(intakePressureAtmosphericOffsetRaw);

  // Output atmospheric readings
  Serial.println("\nINFO: Setting current atospheric pressure offsets ... ");
  Serial.print("  Manifold set at ");
  Serial.print(manifoldPressureAtmosphericOffsetKpa);
  Serial.println("kPa");
  Serial.print("  Intake set at ");
  Serial.print(intakePressureAtmosphericOffsetKpa);
  Serial.println("kPa");

  // Calibrate travel limits of boost valve (drive against full open / closed and record readings)
  setBoostValveTravelLimits(&boostValveMotorDriver, &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw);

  // Initialize the PID controller and set the motor speed limits
  boostValvePressurePID.SetMode(AUTOMATIC);
  boostValvePressurePID.SetOutputLimits(maximumReverseMotorSpeed, maximumForwardMotorSpeed);

  boostValvePositionPID.SetMode(AUTOMATIC);
  boostValvePositionPID.SetOutputLimits(maximumReverseMotorSpeed, maximumForwardMotorSpeed);

  // Setup our WiFi connectivity if needed
  if (enableWifi) {
    setupWiFi();
  }

  // Setup our MQTT connectivity if needed
  if (enableWifi && enableMqttPublish) {
    mqttIsConnected = connectMqttClientToBroker();
  }
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

  // Get the current manifold pressure as raw sensor reading (0-1023) and convert to kPa gauge
  if (ptGetManifoldPressure.call()) {
    currentManifoldPressureAbsoluteRaw = getAveragedAnaloguePinReading(manifoldTmapSensorPressureSignalPin, 20, 0);
    currentManifoldPressureGaugeKpa = calculateBosch3BarKpaFromRaw(currentManifoldPressureAbsoluteRaw - manifoldPressureAtmosphericOffsetRaw);
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
      DEBUG_SERIAL_SEND("Received request from master for params upadte (command ID 0 request, ID 2 response)");
      serialSendCommandId0Response(globalAlarmCritical, currentTargetBoostKpa, currentManifoldPressureGaugeKpa, currentManifoldTempCelcius,
                                   currentIntakePressureGaugeKpa, currentIntakeTempCelcius, currentBoostValveOpenPercentage);
    }

    if (commandIdProcessed == 1) { // Updated parameters from master
      lastSuccessfulCommandId1Processed = millis();
      DEBUG_SERIAL_RECEIVE("Successfully processed command ID 1 message (update pushed params from master)");
    }
  }

  // Perform any checks specifically around critical alarm conditions and set flag if needed
  if (ptCheckFaultConditions.call()) {
    checkAndSetFaultConditions(&currentManifoldPressureGaugeKpa, &currentTargetBoostKpa);
  }

  // Calculate the desired boost we should be running unless critical alarm is set
  // Critical alarm state may be set in a number of ways else where in the code:
  //   - Over boosting, unable to maintain the desired target
  //   - Not getting valid data from the master for x time (also accounts for bad quality comms)
  if (ptCalculateDesiredBoostKpa.call()) {
    if (globalAlarmCritical == true) {
      currentTargetBoostKpa = 0.0;
    } else {
      currentTargetBoostKpa = calculateDesiredBoostKpa(currentVehicleSpeed, currentVehicleRpm, currentVehicleGear, clutchPressed);
    }
  }

  // Update PID valve control to drive to target boost or position as needed
  // If critical alarm is set, stop the motor and let the return spring open the valve to 'fail safe'
  if (ptCalculatePidAndDriveValve.call()) {
    if (globalAlarmCritical) {
      boostValveMotorDriver.setSpeed(0);
      return;
    }

    if (currentTargetBoostKpa == 0) {
      currentTargetBoostValveOpenPercentage = 100;
      if (usingPressureControl) {
        usingPositionControl = true;
        usingPressureControl = false;
        DEBUG_PID("Switching control mode to POSITIONAL");
      }
      driveBoostValveToTargetByOpenPercentagePid(&boostValveMotorDriver, &boostValvePositionPID, &currentBoostValveOpenPercentage, &currentBoostValveMotorSpeed, &currentTargetBoostValveOpenPercentage);
      return;
    }

    if (currentTargetBoostKpa > 0) {
      if (currentManifoldPressureGaugeKpa < (currentTargetBoostKpa * valvePositionToPressureControlTransitionFactor)) {
        currentTargetBoostValveOpenPercentage = 0;
        if (usingPressureControl) {
          usingPositionControl = true;
          usingPressureControl = false;
          DEBUG_PID("Switching control mode to POSITIONAL");
        }
        driveBoostValveToTargetByOpenPercentagePid(&boostValveMotorDriver, &boostValvePositionPID, &currentBoostValveOpenPercentage, &currentBoostValveMotorSpeed, &currentTargetBoostValveOpenPercentage);
        return;
      } else {
        if (usingPositionControl) {
          usingPositionControl = false;
          usingPressureControl = true;
          DEBUG_PID("Switching control mode to PRESSURE");
        }
        driveBoostValveToTargetByPressurePid(&boostValveMotorDriver, &boostValvePressurePID, &currentBoostValveMotorSpeed, &boostValvePositionReadingMinimumRaw, &boostValvePositionReadingMaximumRaw, &currentBoostValvePositionReadingRaw);
      }
    }
  }

  // Output plotter friendly data for the Arduino IDE plotter
  if (ptOutputPidDataForLivePlotter.call() && debugPidPlotterOutput) {
    Serial.print("_zero:");
    Serial.print(0);
    Serial.print(",");
    Serial.print("Target:");
    Serial.print(currentTargetBoostKpa);
    Serial.print(",");
    Serial.print("Actual:");
    Serial.print(currentManifoldPressureGaugeKpa);
    Serial.print(",");
    Serial.print("Proportional:");
    Serial.println(PressureKp);
    Serial.print(",");
    Serial.print("Integral:");
    Serial.print(PressureKi);
    Serial.print(",");
    Serial.print("Derivative:");
    Serial.println(PressureKd);
  }

  // Some temporary debug that may remain in place
  if (ptOutputTargetAndCurrentBoostDebug.call()) {
    DEBUG_PID("Target boost is " + String(currentTargetBoostKpa) + "kPa and current is " + String(currentManifoldPressureGaugeKpa) + "kPa");
    // THIS NEEDS TO BE CHANGED BACK TO DEBUG_BOOST
    DEBUG_PID("Proportional value: " + String(PressureKp, 2) + " Integral value: " + String(PressureKi, 2) + " Derivative value: " + String(PressureKd, 2));
    // DEBUG_PID("Current open percentage: " + String(currentBoostValveOpenPercentage, 2) + " Target open percentage: " + String(currentTargetBoostValveOpenPercentage, 2) + "\n");
  }

  // Used for tuning PID values using potentiometers to adjust P, I and D values
  if (ptReadPidPotsAndUpdateTuning.call()) {
    readPidPotsAndUpdateTuning(&boostValvePressurePID, &PressureKp, &PressureKi, &PressureKd);
  }

  // Publish metrics via MQTT to server if needed
  if (ptMqttPublishMetricsToServer.call() && mqttIsConnected) {
    // Publish pressure metrics
    std::map<String, double> metricsPressures;
    metricsPressures["Target"] = currentTargetBoostKpa;
    metricsPressures["Actual"] = currentManifoldPressureGaugeKpa;
    publishMqttMetrics("pressures", metricsPressures);

    // Publish PID control metrics if needed
    std::map<String, double> metricsPids;
    metricsPids["kP"] = PressureKp;
    metricsPids["kI"] = PressureKi;
    metricsPids["kD"] = PressureKd;
    publishMqttMetrics("pids", metricsPids);

    // Publish valve open metric if needed
    std::map<String, double> metricsValveOpen;
    metricsValveOpen["Percentage"] = currentBoostValveOpenPercentage;
    publishMqttMetrics("valveopen", metricsValveOpen);
  }

  // Increment loop counter if needed so we can report on stats
  if (millis() > 10000 && reportArduinoLoopStats) {
    arduinoLoopExecutionCount++;
    if (ptReportArduinoLoopStats.call()) {
      reportArduinoLoopRate(&arduinoLoopExecutionCount);
    }
  }
}
