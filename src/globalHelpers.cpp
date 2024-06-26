#include "globalHelpers.h"
#include <light_CD74HC4067.h>

/* ======================================================================
   VARIABLES: Pin constants
   ====================================================================== */
// Needed for multiplexer board setup
const int muxS0Pin = 4;
const int muxS1Pin = 5;
const int muxS2Pin = 6;
const int muxS3Pin = 7;
const byte muxSignalPin = A3;

/* ======================================================================
   GLOBAL VARIABLES: Use throughout code
   ====================================================================== */
bool globalAlarmCritical = false;
unsigned long lastSuccessfulCommandId1Processed = millis();

/* ======================================================================
   OBJECT DECLARATIOS
   ====================================================================== */
CD74HC4067 mux(muxS0Pin, muxS1Pin, muxS2Pin, muxS3Pin);

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
const int millisWithoutSerialCommsBeforeFault = 1000; // How long is ms without serial comms from the master before we declare a critical alarm

/* ======================================================================
   FUNCTION: Check various fault conditions and set alarms if needed
   ====================================================================== */
const float overboostAllowancePressure = 1.1; // Allow some amount of wiggle to absorp transient readings
const int overboostAllowanceTime = 8000;      // Time we must be in overboost before setting alarm condition TODO: Set back when not debugging !!!
unsigned long overboostStartMillis;
bool inOverboost = false;

void checkAndSetFaultConditions(double *currentManifoldPressurekPa, double *currentTargetBoostkPa) {
  // Messages from the master not received recently
  if (millis() > 10000 && (millis() > lastSuccessfulCommandId1Processed + millisWithoutSerialCommsBeforeFault)) {
    DEBUG_SERIAL_SEND("Setting critical alarm due to serial comms outage !!");
    globalAlarmCritical = true;
  }

  // Overboosting above allowance (in amount and in time) is detected
  if (*currentManifoldPressurekPa > (*currentTargetBoostkPa * overboostAllowancePressure)) {
    if (inOverboost == false) {
      inOverboost = true;
      overboostStartMillis = millis();
    }
    if ((millis() - overboostStartMillis) > overboostAllowanceTime && inOverboost == true && millis() > 10000) {
      DEBUG_BOOST("Setting critical alarm due to over boosting !! " + String(*currentManifoldPressurekPa) + "kPa vs " + String(*currentTargetBoostkPa * overboostAllowancePressure) + "kPa");
      globalAlarmCritical = true;
    }
  } else {
    inOverboost = false;
  }
}

/* ======================================================================
   FUNCTION: Convert Bosch 3 bar TMAP reading to kPa
   ====================================================================== */
// Define some magic numbers which the Bosch TMAP sensor uses in it's formula from its data sheet
// https://www.bosch-motorsport-shop.com.au/t-map-sensor-3-bar-130-deg-c
const float boschMagicNumber1 = 5.4 / 280;  // 0.0192857142857143
const float boschMagicNumber2 = 0.85 / 280; // 0.0030357142857143
const float sensorSupplyVoltage = 5.0;      // Included for clarity and in case we can't supply exactly 5V

float calculateBosch3BarKpaFromRaw(float sensorReadingRaw) {
  // There is an oddity here in that when passing in 0 as a raw reading we do NOT get 0 as output
  float sensorVoltage = (sensorReadingRaw / 1023) * 5;
  float pressureKpa = (sensorVoltage - boschMagicNumber1 * sensorSupplyVoltage) / (boschMagicNumber2 * sensorSupplyVoltage);
  return pressureKpa;
}

/* ======================================================================
   FUNCTION: Setup the multiplexer
   ====================================================================== */
void setupMux() {
  DEBUG_GENERAL("Configuring mux board ...");
  pinMode(muxSignalPin, INPUT);
}

/* ======================================================================
   FUNCTION: Get average readings from analogue pin
   ====================================================================== */
int getAveragedAnaloguePinReading(byte pin, int samples, int delayUs) {
  int totalReadings = 0;

  for (int i = 0; i < samples; i++) {
    if (delayUs != 0) {
      delayMicroseconds(delayUs);
    }
    totalReadings += analogRead(pin);
  }

  // Calculate average of the readings
  int averageReading = totalReadings / static_cast<float>(samples);
  return averageReading;
}

/* ======================================================================
   FUNCTION: Get average readings from multiplexed channel via CD74HC4067
   ====================================================================== */
int getAveragedMuxAnalogueChannelReading(byte channel, int samples, int delayUs) {
  mux.channel(channel);
  int averageReading = getAveragedAnaloguePinReading(muxSignalPin, samples, delayUs);
  return averageReading;
}

/* ======================================================================
   FUNCTION: Report Arduino loop rate information
   ====================================================================== */
unsigned long arduinoLoopExecutionPreviousExecutionMillis;

void reportArduinoLoopRate(unsigned long *loopCount) {
  float loopFrequencyHz = (*loopCount / ((millis() - arduinoLoopExecutionPreviousExecutionMillis) / 1000));
  float loopExecutionMs = (millis() - arduinoLoopExecutionPreviousExecutionMillis) / *loopCount;
  Serial.print("Loop execution frequency (Hz): ");
  Serial.print(loopFrequencyHz);
  Serial.print(" or every ");
  Serial.print(loopExecutionMs);
  Serial.println("ms");
  *loopCount = 1;
  arduinoLoopExecutionPreviousExecutionMillis = millis();
}

/* ======================================================================
   FUNCTION: Print debug info for Arduino IDE serial plotter
   ====================================================================== */
void outputArduinoIdePlotterData(double *currentTargetBoostKpa, double *currentManifoldPressureGaugeKpa,
                                 double *PressureKp, double *PressureKi, double *PressureKd) {
  Serial.print("_zeroline:");
  Serial.print(0);
  Serial.print(",");
  Serial.print("Target:");
  Serial.print(*currentTargetBoostKpa);
  Serial.print(",");
  Serial.print("Actual:");
  Serial.print(*currentManifoldPressureGaugeKpa);
  Serial.print(",");
  Serial.print("Proportional:");
  Serial.println(*PressureKp);
  Serial.print(",");
  Serial.print("Integral:");
  Serial.print(*PressureKi);
  Serial.print(",");
  Serial.print("Derivative:");
  Serial.println(*PressureKd);
}