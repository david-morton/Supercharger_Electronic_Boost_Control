#ifndef BOOSTVALVECONTROL_H
#define BOOSTVALVECONTROL_H

#include <Arduino.h>
#include "CytronMotorDriver.h"

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
float getBoostValveOpenPercentage(const byte, int *, int *);
void driveBoostValveToTarget(CytronMD *, float *, float *, float *, int *, int *);

#endif
