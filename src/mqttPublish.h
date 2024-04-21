#ifndef MQTTPUBLISH_H
#define MQTTPUBLISH_H

#include <Arduino.h>
#include <map>

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */
bool connectMqttClientToBroker();
void publishMqttMetrics(String topic, std::map<String, double> metrics);

#endif
