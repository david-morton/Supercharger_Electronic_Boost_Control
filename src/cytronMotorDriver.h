#ifndef CYTRONMOTORDRIVER_H
#define CYTRONMOTORDRIVER_H

#include <Arduino.h>

/* ======================================================================
   VARIABLES: Pin constants
   ====================================================================== */
const int MOTOR_PWM_PIN = 9; // PWM pin for motor speed control
const int MOTOR_DIR_PIN = 8; // Direction pin for motor control

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
void initCytronMotorDriver();
void setCytronSpeedAndDirection(float *);
void setCytronSpeedAndDirection(double);

#endif
