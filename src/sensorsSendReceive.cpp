#include "sensorsSendReceive.h"
#include <PID_v1.h>

/*
Define pin constants
*/
const byte inputPotentiometerSignalPin = A15;

/*
Define function - Get motor potentiometer voltage reading
*/
// void getMotorPotentiometerVoltage() {
//   // Voltage range seems to be from 3.25V to 1.70V
//   motorPotentiometerVoltage = analogRead(motorPotentiometerSignalPin) * (5.0 / 1023.0);
//   SERIAL_PORT_MONITOR.print("Motor potentiometer voltage is: ");
//   SERIAL_PORT_MONITOR.println(motorPotentiometerVoltage);
// }
