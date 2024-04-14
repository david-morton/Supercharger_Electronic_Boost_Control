#include "globalHelpers.h"

/* ======================================================================
   GLOBAL VARIABLES: Use throughout code
   ====================================================================== */
bool globalAlarmCritical = false;
unsigned long lastSuccessfulCommandId1Processed = millis();

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
const int millisWithoutSerialCommsBeforeFault = 1000; // How long is ms without serial comms from the master before we declare a critical alarm

/* ======================================================================
   FUNCTION: Check various fault conditions and set alarms if needed
   ====================================================================== */
const float overboostAllowancePressure = 1.1; // Allow some amount of wiggle to absorp transient readings
const int overboostAllowanceTime = 1000;      // Time we must be in overboost before setting alarm condition
unsigned long overboostStartMillis;
bool inOverboost = false;

void checkAndSetFaultConditions(double *currentManifoldPressurePsi, double *currentTargetBoostPsi) {
  // Messages from the master not received recently
  if (millis() > 10000 && (millis() > lastSuccessfulCommandId1Processed + millisWithoutSerialCommsBeforeFault)) {
    DEBUG_SERIAL_SEND("Setting critical alarm due to serial comms outage !!");
    globalAlarmCritical = true;
  }

  // Overboosting above allowance (in amount and in time) is detected
  if (*currentManifoldPressurePsi > (*currentTargetBoostPsi * overboostAllowancePressure)) {
    if (inOverboost == false) {
      inOverboost = true;
      overboostStartMillis = millis();
    }
    if ((millis() - overboostStartMillis) > overboostAllowanceTime && inOverboost == true && millis() > 10000) {
      DEBUG_BOOST("Setting critical alarm due to over boosting !!");
      globalAlarmCritical = true;
    }
  } else {
    inOverboost = false;
  }
}

/* ======================================================================
   FUNCTION: Convert Bosch 3 bar TMAP reading to psi
   ====================================================================== */
// Define some magic numbers which the Bosch TMAP sensor uses in it's formula from its data sheet
// https://www.bosch-motorsport-shop.com.au/t-map-sensor-3-bar-130-deg-c
const float boschMagicNumber1 = 5.4 / 280;  // 0.0192857142857143
const float boschMagicNumber2 = 0.85 / 280; // 0.0030357142857143
const float sensorSupplyVoltage = 5.0;      // Included for clarity and in case we can't supply exactly 5V

float calculatePsiFromRaw(float currentManifoldPressureRaw) {
  float sensorCurrentVoltage = (currentManifoldPressureRaw / 1023) * 5;
  float pressureKpa = (sensorCurrentVoltage - boschMagicNumber1 * sensorSupplyVoltage) / (boschMagicNumber2 * sensorSupplyVoltage);
  return pressureKpa * 0.145038; // Convert kPa to PSI
}
