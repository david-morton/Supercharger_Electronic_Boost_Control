#include "boostValveControl.h"
#include "cytronMotorDriver.h"
#include "globalHelpers.h"
#include <PID_v1.h>

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
float boostValveOpenPercentage;

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
   FUNCTION: Drive valve to target boost by PID pressure feedback
   ====================================================================== */
void driveBoostValveToTargetByPressurePid(PID *boostValvePressurePid, double *boostValveMotorSpeed,
                                          int *boostValveMinimumRaw, int *boostValveMaximumRaw, int *currentBoostValvePositionReadingRaw) {
  // Update the motor speed based on current position feedback
  if (*currentBoostValvePositionReadingRaw >= *boostValveMaximumRaw || *currentBoostValvePositionReadingRaw <= *boostValveMinimumRaw) {
    setCytronSpeedAndDirection(0.0);    // We are at the detected travel limit of the valve, no need to drive it into the stop
  } else {
    boostValvePressurePid->Compute();
    setCytronSpeedAndDirection(*boostValveMotorSpeed);
  }
}

/* ======================================================================
   FUNCTION: Drive valve to target open percentage by PID position feedback
   ====================================================================== */
void driveBoostValveToTargetByOpenPercentagePid(PID *boostValvePositionPid, double *currentBoostValveOpenPercentage,
                                                double *boostValveMotorSpeed, double *currentTargetBoostValveOpenPercentage) {
  boostValvePositionPid->Compute();
  setCytronSpeedAndDirection(*boostValveMotorSpeed);
}
