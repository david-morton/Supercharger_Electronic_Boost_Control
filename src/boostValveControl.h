#ifndef BOOSTVALVECONTROL_H
#define BOOSTVALVECONTROL_H

#include "PID_v1.h"
#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
int getBoostValvePositionReadingRaw(const byte *);
float getBoostValveOpenPercentage(int *, int *, int *);
void driveBoostValveToTargetByPressurePid(PID *, double *, int *, int *, int *);
void driveBoostValveToTargetByOpenPercentagePid(PID *, double *, double *, double *);

#endif
