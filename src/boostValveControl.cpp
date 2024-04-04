#include "boostValveControl.h"
#include "CytronMotorDriver.h" // Library for the Cytron MDD10 motor driver

/*
Define variables
*/
float boostValveOpenPercentage;
int boostValvePositionReading;

/*
Define function - Determine current boost valve position percentage
*/
float getBoostValveOpenPercentage(const byte signalPin, int* positionReadingMinimum, int* positionReadingMaximum) {
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
    return 100.0;
  } else if (averageReading <= *positionReadingMinimum) {
    return 0.0;
  } else {
    float boostValveOpenPercentage = ((averageReading - *positionReadingMinimum) /
                                      (*positionReadingMaximum - *positionReadingMinimum)) * 100.0;
    return boostValveOpenPercentage;
  }
}
