#ifndef SERIALCOMMUNICATIONS_H
#define SERIALCOMMUNICATIONS_H

#include <Arduino.h>

/*
Define function prototypes
*/
const char *serialGetIncomingMessage();
void serialReportPerformanceStats();
bool serialIsChecksumValid();
void serialProcessMessage(const char *, float *, int *, int *, bool *);

#endif
