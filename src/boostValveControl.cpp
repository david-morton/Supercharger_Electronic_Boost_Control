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
   FUNCTION: Determine current boost valve position percentage
   ====================================================================== */
float getBoostValveOpenPercentage(const byte signalPin, int *positionReadingMinimum, int *positionReadingMaximum) {
  const int numReadings = 20; // Number of readings to average
  int totalReadings = 0;

  // Take multiple readings and sum them up
  for (int i = 0; i < numReadings; i++) {
    // Read sensor position from analog pin
    boostValvePositionReading = analogRead(signalPin);
    totalReadings += boostValvePositionReading;
  }

  // Calculate average of the readings
  float averageReading = totalReadings / static_cast<float>(numReadings); // Prevent returning int result

  // Calculate position percentage
  if (averageReading >= *positionReadingMaximum) {
    DEBUG_VALVE("Valve open percentage hard set to 100\% as " + String(averageReading) + " >= " + String(*positionReadingMaximum));
    return 100.0;
  } else if (averageReading <= *positionReadingMinimum) {
    DEBUG_VALVE("Valve open percentage hard set to 0\% as " + String(averageReading) + " <= " + String(*positionReadingMinimum));
    return 0.0;
  } else {
    float boostValveOpenPercentage = ((averageReading - *positionReadingMinimum) / (*positionReadingMaximum - *positionReadingMinimum)) * 100.0;
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
void driveBoostValveToTarget(CytronMD *boostValveMotorDriver, double *targetBoostPsi, double *manifoldPressurePsi, int *boostValveMinimumRaw, int *boostValveMaximumRaw) {
  DEBUG_VALVE("Updating valve motor drive to target " + String(*targetBoostPsi) + "psi. Manifold currently reading " + String(*manifoldPressurePsi) + "psi");
}
