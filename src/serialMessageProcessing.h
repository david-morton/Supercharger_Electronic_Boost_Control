#ifndef SERIALMESSAGEPROCESSING_H
#define SERIALMESSAGEPROCESSING_H

#include <Arduino.h>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
void serialProcessCommandId1(const char *, float *, int *, int *, bool *);
int serialProcessMessage(const char *, float *, int *, int *, bool *);

#endif
