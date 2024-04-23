#include "cytronMotorDriver.h"
#include "pwm.h"

/* ======================================================================
   VARIABLES: General use
   ====================================================================== */
PwmOut cytronPwm(MOTOR_PWM_PIN);

/* ======================================================================
   FUNCTION: Initialise the motor driver
   ====================================================================== */
void initCytronMotorDriver() {
  Serial.println("\nINFO: Initialising Cytron motor driver board ...\n");
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  cytronPwm.begin(25000.0f, 0.0f); // 25kHz PWM frequency and 0% duty
}

/* ======================================================================
   FUNCTION: Set the motor speed and direction (pointer)
   ====================================================================== */
void setCytronSpeedAndDirection(float *speedAndDirection) {
  // Sanity check the values
  if (*speedAndDirection >= 100.0) {
    *speedAndDirection = 100.0;
  } else if (*speedAndDirection <= -100.0) {
    *speedAndDirection = -100.0;
  }

  // Set the output as needed
  if (*speedAndDirection > 0) {
    digitalWrite(MOTOR_DIR_PIN, HIGH); // Forward
    cytronPwm.pulse_perc(abs(*speedAndDirection));
    return;
  }
  if (*speedAndDirection < 0) {
    digitalWrite(MOTOR_DIR_PIN, LOW); // Backwards
    cytronPwm.pulse_perc(abs(*speedAndDirection));
    return;
  } else {
    cytronPwm.pulse_perc(0.0f);
    return;
  }
}

/* ======================================================================
   FUNCTION: Set the motor speed and direction (raw value)
   ====================================================================== */
void setCytronSpeedAndDirection(double speedAndDirection) {
  // Sanity check the values
  if (speedAndDirection >= 100.0) {
    speedAndDirection = 100.0;
  } else if (speedAndDirection <= -100.0) {
    speedAndDirection = -100.0;
  }

  // Set the output as needed
  if (speedAndDirection > 0) {
    digitalWrite(MOTOR_DIR_PIN, HIGH); // Forward
    cytronPwm.pulse_perc(abs(speedAndDirection));
    return;
  }
  if (speedAndDirection < 0) {
    digitalWrite(MOTOR_DIR_PIN, LOW); // Backwards
    cytronPwm.pulse_perc(abs(speedAndDirection));
    return;
  } else {
    cytronPwm.pulse_perc(0.0f);
    return;
  }
}
