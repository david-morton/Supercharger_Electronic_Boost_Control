#ifndef SERIALMESSAGEPROCESSING_H
#define SERIALMESSAGEPROCESSING_H

#include <Arduino.h>

/*
Define function prototypes
*/
void processMessageType0(const char *serialMessage);
void processMessageType1(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed);
void serialProcessMessage(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed);

#endif
