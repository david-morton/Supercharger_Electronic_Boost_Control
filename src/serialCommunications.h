#ifndef SERIALCOMMUNICATIONS_H
#define SERIALCOMMUNICATIONS_H

#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
const char *serialGetIncomingMessage();
void serialReportMessageQualityStats();
bool serialIsChecksumValid();
void serialProcessMessage(const char *, float *, int *, int *, bool *);
void serialCalculateMessageQualityStats();

#endif
