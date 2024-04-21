#ifndef WIFI_HELPER_H
#define WIFI_HELPER_H

#include <WiFiS3.h>

void setupWiFi();
void printWifiData();
void printCurrentNet();
void printMacAddress(byte mac[]);

#endif
