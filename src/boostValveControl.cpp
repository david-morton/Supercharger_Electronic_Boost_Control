#include "boostValveControl.h"

/*
Define pin constants
*/
const byte boostValvePositionSignal1Pin = A14;          // Words here
const byte boostValvePositionSignal2Pin = A15;          // Words here

/*
Define variables
*/
float boostValvePosition;
float boostValvePositionVoltage1;
float boostValvePositionVoltage2;

/*
Define function - Determine current boost valve position percentage
*/
float getBoostValvePosition(){
    boostValvePositionVoltage1 = analogRead(boostValvePositionSignal1Pin) * (5.0 / 1023.0);
    boostValvePositionVoltage2 = analogRead(boostValvePositionSignal2Pin) * (5.0 / 1023.0);
    SERIAL_PORT_MONITOR.print("Sensor 1 on pin A14 is: ");
    SERIAL_PORT_MONITOR.println(boostValvePositionVoltage1);
    SERIAL_PORT_MONITOR.print("Sensor 2 on pin A15 is: ");
    SERIAL_PORT_MONITOR.println(boostValvePositionVoltage2);
    return 0.0;
}