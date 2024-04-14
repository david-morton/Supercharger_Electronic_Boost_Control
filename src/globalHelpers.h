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

// Define the DEBUG_SERIAL_RECEIVE macro
#define DEBUG_SERIAL_RECEIVE(message)                       \
  do {                                                      \
    if (debugSerialReceive) {                               \
      SERIAL_PORT_MONITOR.print("[DEBUG SERIAL RECEIVE] "); \
      SERIAL_PORT_MONITOR.println(message);                 \
    }                                                       \
  } while (0)

// Define the DEBUG_SERIAL_SEND macro
#define DEBUG_SERIAL_SEND(message)                       \
  do {                                                   \
    if (debugSerialSend) {                               \
      SERIAL_PORT_MONITOR.print("[DEBUG SERIAL SEND] "); \
      SERIAL_PORT_MONITOR.println(message);              \
    }                                                    \
  } while (0)

// Define the DEBUG_VALVE macro
#define DEBUG_VALVE(message)                       \
  do {                                             \
    if (debugValveControl) {                       \
      SERIAL_PORT_MONITOR.print("[DEBUG VALVE] "); \
      SERIAL_PORT_MONITOR.println(message);        \
    }                                              \
  } while (0)

// Define the DEBUG_BOOST macro
#define DEBUG_BOOST(message)                       \
  do {                                             \
    if (debugBoost) {                              \
      SERIAL_PORT_MONITOR.print("[DEBUG BOOST] "); \
      SERIAL_PORT_MONITOR.println(message);        \
    }                                              \
  } while (0)

// Define the DEBUG_GENERAL macro
#define DEBUG_GENERAL(message)                       \
  do {                                               \
    if (debugGeneral) {                              \
      SERIAL_PORT_MONITOR.print("[DEBUG GENERAL] "); \
      SERIAL_PORT_MONITOR.println(message);          \
    }                                                \
  } while (0)

/* ======================================================================
   HELPERS: Variables to determine alarm status
   ====================================================================== */
extern bool globalAlarmCritical;
extern unsigned long lastSuccessfulCommandId1Processed;

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
void checkAndSetFaultConditions(double *, double *);
float calculatePsiFromRaw(float);

#endif
