#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#include <Arduino.h>

// Global flag for debugging
extern bool debugMode;

// Define the DEBUG_PRINT macro
#define DEBUG_PRINT(message)                 \
  do {                                       \
    if (debugMode) {                         \
      SERIAL_PORT_MONITOR.print("[DEBUG] "); \
      SERIAL_PORT_MONITOR.println(message);  \
    }                                        \
  } while (0)

#endif
