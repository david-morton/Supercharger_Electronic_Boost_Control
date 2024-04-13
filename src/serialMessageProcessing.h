#ifndef SERIALMESSAGEPROCESSING_H
#define SERIALMESSAGEPROCESSING_H

#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
void serialProcessCommandId0(const char *serialMessage);
void serialProcessCommandId1(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed);
void serialProcessMessage(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed);

#endif
