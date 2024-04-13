#ifndef GLOBALHELPERS_H
#define GLOBALHELPERS_H

#include <Arduino.h>

/* ======================================================================
   HELPER: Global flag for debugging
   ====================================================================== */
extern bool debugMode;

// Define the DEBUG_PRINT macro
#define DEBUG_PRINT(message)                 \
  do {                                       \
    if (debugMode) {                         \
      SERIAL_PORT_MONITOR.print("[DEBUG] "); \
      SERIAL_PORT_MONITOR.println(message);  \
    }                                        \
  } while (0)

/* ======================================================================
   HELPERS: Variables to determine alarm status
   ====================================================================== */
extern bool globalAlarmCritical;
extern unsigned long lastSuccessfulCommandId1Processed;

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
void checkAndSetFaultConditions();

#endif
