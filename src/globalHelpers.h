#ifndef GLOBALHELPERS_H
#define GLOBALHELPERS_H

#include <Arduino.h>

/* ======================================================================
   HELPER: Global flags and functions for debugging
   ====================================================================== */
extern bool debugSerialReceive;
extern bool debugSerialSend;
extern bool debugValveControl;
extern bool debugBoost;
extern bool debugGeneral;
extern bool debugPid;

/* ======================================================================
   HELPERS: Variables to determine alarm status
   ====================================================================== */
extern bool globalAlarmCritical;
extern unsigned long lastSuccessfulCommandId1Processed;

/* ======================================================================
   HELPERS: Debug output definitions
   ====================================================================== */
// Define the DEBUG_SERIAL_RECEIVE macro
#define DEBUG_SERIAL_RECEIVE(message)          \
  do {                                         \
    if (debugSerialReceive) {                  \
      Serial.print("[DEBUG SERIAL RECEIVE] "); \
      Serial.println(message);                 \
    }                                          \
  } while (0)

// Define the DEBUG_SERIAL_SEND macro
#define DEBUG_SERIAL_SEND(message)          \
  do {                                      \
    if (debugSerialSend) {                  \
      Serial.print("[DEBUG SERIAL SEND] "); \
      Serial.println(message);              \
    }                                       \
  } while (0)

// Define the DEBUG_VALVE macro
#define DEBUG_VALVE(message)          \
  do {                                \
    if (debugValveControl) {          \
      Serial.print("[DEBUG VALVE] "); \
      Serial.println(message);        \
    }                                 \
  } while (0)

// Define the DEBUG_BOOST macro
#define DEBUG_BOOST(message)          \
  do {                                \
    if (debugBoost) {                 \
      Serial.print("[DEBUG BOOST] "); \
      Serial.println(message);        \
    }                                 \
  } while (0)

// Define the DEBUG_PID macro
#define DEBUG_PID(message)          \
  do {                              \
    if (debugPid) {                 \
      Serial.print("[DEBUG PID] "); \
      Serial.println(message);      \
    }                               \
  } while (0)

// Define the DEBUG_GENERAL macro
#define DEBUG_GENERAL(message)          \
  do {                                  \
    if (debugGeneral) {                 \
      Serial.print("[DEBUG GENERAL] "); \
      Serial.println(message);          \
    }                                   \
  } while (0)

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
void checkAndSetFaultConditions(double *, double *);
float calculateBosch3BarKpaFromRaw(float);
int getAveragedAnaloguePinReading(byte pin, int samples, int delayMs);
void reportArduinoLoopRate(unsigned long *);

#endif
