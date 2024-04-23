#include "pidPotentiometers.h"
#include "PID_v1.h"
#include "globalHelpers.h"
#include <ptScheduler.h>

/* ======================================================================
   VARIABLES: Temporary PID debug items, used with 3 potentiometers
   ====================================================================== */
const byte pidPinProportional = A3;
const byte pidPinIntegral = A4;
const byte pidPinDerivative = A5;

int pidRangeMaxProportional = 75;
int pidRangeMaxIntegral = 20;
int pidRangeMaxDerivative = 20;

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
