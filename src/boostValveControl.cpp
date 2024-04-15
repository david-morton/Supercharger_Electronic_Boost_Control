#include "boostValveControl.h"
#include "CytronMotorDriver.h"
#include "globalHelpers.h"
#include <PID_v1.h>

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
float boostValveOpenPercentage;
int boostValvePositionReading;

/* ======================================================================
   FUNCTION: Get current boost valve blade position
   ====================================================================== */
int getBoostValvePositionReadingRaw(const byte *signalPin) {
  const int numReadings = 20; // Number of readings to average
  int totalReadings = 0;

  // Take multiple readings and sum them up as fast as we can
  for (int i = 0; i < numReadings; i++) {
    // Read sensor position from analog pin
    boostValvePositionReading = analogRead(*signalPin);
    totalReadings += boostValvePositionReading;
  }

  // Calculate average of the readings
  int averageReading = totalReadings / numReadings;

  return averageReading;
}

/* ======================================================================
   FUNCTION: Determine current boost valve position percentage
   ====================================================================== */
float getBoostValveOpenPercentage(int *positionReadingCurrent, int *positionReadingMinimum, int *positionReadingMaximum) {
  if (*positionReadingCurrent >= *positionReadingMaximum) {
    DEBUG_VALVE("Valve open percentage hard set to 100\% as " + String(*positionReadingCurrent) + " >= " + String(*positionReadingMaximum));
    return 100.0;
  } else if (*positionReadingCurrent <= *positionReadingMinimum) {
    DEBUG_VALVE("Valve open percentage hard set to 0\% as " + String(*positionReadingCurrent) + " <= " + String(*positionReadingMinimum));
    return 0.0;
  } else {
    float boostValveOpenPercentage = (static_cast<float>(*positionReadingCurrent - *positionReadingMinimum) / (*positionReadingMaximum - *positionReadingMinimum)) * 100.0;
    DEBUG_VALVE("Valve open percentage calculated as " + String(boostValveOpenPercentage) + "%");
    return boostValveOpenPercentage;
  }
}

/* ======================================================================
   FUNCTION: Drive valve fully open
   ====================================================================== */

/* ======================================================================
   FUNCTION: Drive valve to target boost
   ====================================================================== */
void driveBoostValveToTarget(CytronMD *boostValveMotorDriver, PID *boostValvePid, double *boostValveMotorSpeed,
                             int *boostValveMinimumRaw, int *boostValveMaximumRaw, int *currentBoostValvePositionReadingRaw) {
  // Compute the latest output value for our PID control object
  boostValvePid->Compute();

  // Update the motor speed based on PID output calculation and current position feedback
  if (*currentBoostValvePositionReadingRaw >= *boostValveMaximumRaw || *currentBoostValvePositionReadingRaw <= *boostValveMinimumRaw) {
    // DEBUG_VALVE("Travel limit reached, stopping motor. Current position " + String(*currentBoostValvePositionReadingRaw) +
                // " vs min of " + String(*boostValveMinimumRaw) + " and max of " + String(*boostValveMaximumRaw));
    boostValveMotorDriver->setSpeed(0);
  } else {
    // DEBUG_VALVE("Updating PID output (motor speed) to be " + String(*boostValveMotorSpeed));
    boostValveMotorDriver->setSpeed(*boostValveMotorSpeed);
  }
}
