#include "pidPotentiometers.h"
#include "PID_v1.h"
#include "globalHelpers.h"
#include <ptScheduler.h>

/* ======================================================================
   VARIABLES: Temporary PID debug items, used with 3 potentiometers
   ====================================================================== */
const byte pidChannelProportional = 0;
const byte pidChannelIntegral = 1;
const byte pidChannelDerivative = 2;

int pidRangeMaxProportional = 75;
int pidRangeMaxIntegral = 20;
int pidRangeMaxDerivative = 20;

void readPidPotsAndUpdateTuning(PID *pidMotor, double *pidLiveValueProportional, double *pidLiveValueIntegral, double *pidLiveValueDerivative) {
  int pidPotProportionalRaw = getAveragedMuxAnalogueChannelReading(pidChannelProportional, 10, 0);
  int pidPotIntegralRaw = getAveragedMuxAnalogueChannelReading(pidChannelIntegral, 10, 0);
  int pidPotDerivativeRaw = getAveragedMuxAnalogueChannelReading(pidChannelDerivative, 10, 0);

  // Adjust the mapping for higher precision
  double factor = 100.0;

  *pidLiveValueProportional = map(pidPotProportionalRaw, 0, 1023, pidRangeMaxProportional, 0);
  *pidLiveValueIntegral = map(pidPotIntegralRaw, 0, 1023, pidRangeMaxIntegral * factor, 0) / factor;
  *pidLiveValueDerivative = map(pidPotDerivativeRaw, 0, 1023, pidRangeMaxDerivative * factor, 0) / factor;

  pidMotor->SetTunings(*pidLiveValueProportional, *pidLiveValueIntegral, *pidLiveValueDerivative);
}
