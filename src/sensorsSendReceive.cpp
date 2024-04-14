#include "sensorsSendReceive.h"
#include <PID_v1.h>

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
int pressureSensorReading;

/* ======================================================================
   FUNCTION: Get manifold pressure sensor reading
   ====================================================================== */
float getManifoldPressureRaw(const byte signalPin) {
  const int numReadings = 20; // Number of readings to average
  int totalReadings = 0;

  // Take multiple readings and sum them up
  for (int i = 0; i < numReadings; i++) {
    // Read sensor position from analog pin
    pressureSensorReading = analogRead(signalPin);
    totalReadings += pressureSensorReading;
  }

  // Calculate average of the readings
  float averageReading = totalReadings / static_cast<float>(numReadings); // Prevent returning int result
  return averageReading;
}
