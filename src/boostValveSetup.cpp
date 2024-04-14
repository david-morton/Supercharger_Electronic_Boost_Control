#include "boostValveSetup.h"
#include <ptScheduler.h>

/* ======================================================================
   VARIABLES: Pin constants
   ====================================================================== */
const byte boostValvePositionSignal1Pin = A14; // Words here

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
bool boostValveClosedPositionSet = false;
bool boostValveOpenPositionSet = false;

/* ======================================================================
   FUNCTION: Set travel limits of boost valve
   ====================================================================== */
void setBoostValveTravelLimits(CytronMD *boostValveMotorDriver, int *positionReadingMinimum, int *positionReadingMaximum) {
  ptScheduler ptIncrementValvePosition = ptScheduler(PT_TIME_50MS);
  const int windowSize = 10;      // Number of samples to consider for moving average
  int readingsClosed[windowSize]; // Create arrays for moving average samples
  int readingsOpen[windowSize];
  int currentIndexClosed = 0; // Set indexes for moving average data
  int currentIndexOpen = 0;
  float stabilityThreshold = 2.0; // How close our values need to be to be considered stable

  SERIAL_PORT_MONITOR.print("INFO: Setting boost valve travel limits ... ");
  // Set motor speed then immediately iterate on potentiometer / position feedback to determine movement limit
  boostValveMotorDriver->setSpeed(40); // Speed range is from -255 to 255
  while (boostValveClosedPositionSet == false) {
    if (ptIncrementValvePosition.call()) {
      int potentiometerValue = analogRead(boostValvePositionSignal1Pin);
      // Add current reading to the array
      readingsClosed[currentIndexClosed] = potentiometerValue;
      currentIndexClosed = (currentIndexClosed + 1) % windowSize;
      // Calculate moving average
      float sum = 0.0;
      for (int i = 0; i < windowSize; i++) {
        sum += readingsClosed[i];
      }
      int movingAverage = sum / windowSize;
      // Check for stability in moving average
      if (abs(potentiometerValue - movingAverage) < stabilityThreshold) {
        // Arm has hit a stop, stop the motor
        boostValveMotorDriver->setSpeed(0);
        *positionReadingMinimum = movingAverage;
        boostValveClosedPositionSet = true;
      }
    }
  }

  // Set motor speed then immediately iterate on potentiometer / position
  // feedback to determine movement limit
  boostValveMotorDriver->setSpeed(-40); // Speed range is from -255 to 255
  while (boostValveOpenPositionSet == false) {
    if (ptIncrementValvePosition.call()) {
      int potentiometerValue = analogRead(boostValvePositionSignal1Pin);
      // Add current reading to the array
      readingsOpen[currentIndexOpen] = potentiometerValue;
      currentIndexOpen = (currentIndexOpen + 1) % windowSize;
      // Calculate moving average
      float sum = 0.0;
      for (int i = 0; i < windowSize; i++) {
        sum += readingsOpen[i];
      }
      int movingAverage = sum / windowSize;
      // Check for stability in moving average
      if (abs(potentiometerValue - movingAverage) < stabilityThreshold) {
        // Arm has hit a stop, stop the motor
        boostValveMotorDriver->setSpeed(0);
        *positionReadingMaximum = movingAverage;
        boostValveOpenPositionSet = true;
      }
    }
  }

  // Set voltage at open position (spring is assisting)
  SERIAL_PORT_MONITOR.print("\n  Set fully closed (max boost) at ");
  SERIAL_PORT_MONITOR.print(*positionReadingMinimum * (5.0 / 1023.0));
  SERIAL_PORT_MONITOR.print("V (");
  SERIAL_PORT_MONITOR.print(*positionReadingMinimum);
  SERIAL_PORT_MONITOR.println(")");
  // Set voltage at closed position (working against the spring)
  SERIAL_PORT_MONITOR.print("  Set fully open (no boost) at ");
  SERIAL_PORT_MONITOR.print(*positionReadingMaximum * (5.0 / 1023.0));
  SERIAL_PORT_MONITOR.print("V (");
  SERIAL_PORT_MONITOR.print(*positionReadingMaximum);
  SERIAL_PORT_MONITOR.println(")\n");
}