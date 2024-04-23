#ifndef PIDPOTENTIOMETERS_H
#define PIDPOTENTIOMETERS_H

#include "PID_v1.h"
#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
void readPidPotsAndUpdateTuning(PID *, double *, double *, double *);

#endif
