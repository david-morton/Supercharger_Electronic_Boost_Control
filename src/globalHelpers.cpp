#include "globalHelpers.h"

/* ======================================================================
   GLOBAL VARIABLES: Use throughout code
   ====================================================================== */
bool globalAlarmCritical = false;
unsigned long lastSuccessfulCommandId1Processed = millis();

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
int millisWithoutSerialCommsBeforeFault = 1000; // How long is ms without serial comms from the master before we declare a critical alarm

/* ======================================================================
   FUNCTION: Check various fault conditions and set alarms if needed
   ====================================================================== */
void checkAndSetFaultConditions() {
  // Messages from the master not received recently
  if (millis() > 10000 && (millis() > lastSuccessfulCommandId1Processed + millisWithoutSerialCommsBeforeFault)) {
    DEBUG_SERIAL_SEND("Setting critical alarm due to serial comms outage !!");
    globalAlarmCritical = true;
  }
}
