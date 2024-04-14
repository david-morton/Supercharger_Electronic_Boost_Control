#ifndef BOOSTVALVECONTROL_H
#define BOOSTVALVECONTROL_H

#include "CytronMotorDriver.h"
#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
float getBoostValveOpenPercentage(const byte, int *, int *);
void driveBoostValveToTarget(CytronMD *, double *, double *, int *, int *);

#endif
