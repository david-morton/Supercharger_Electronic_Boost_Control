#ifndef SERIALCOMMUNICATIONS_H
#define SERIALCOMMUNICATIONS_H

#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
const char *serialGetIncomingMessage();
void serialReportMessageQualityStats();
void serialCalculateMessageQualityStats();
void serialSendCommandId0Response(bool, float, float, int, float, int, double);

#endif
