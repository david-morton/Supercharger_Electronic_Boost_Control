#ifndef BOOSTVALVECONTROL_H
#define BOOSTVALVECONTROL_H

#include "CytronMotorDriver.h"
#include "PID_v1.h"
#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
int getBoostValvePositionReadingRaw(const byte *);
float getBoostValveOpenPercentage(int *, int *, int *);
void driveBoostValveToTarget(CytronMD *, PID *, double *, int *, int *, int *);

#endif
