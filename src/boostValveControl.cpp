#include "boostValveControl.h"
#include <PID_v1.h>

/*
Define pin constants
*/
const byte boostValvePositionSignal1Pin = A14;   // Words here

/*
Define variables
*/
float boostValveOpenPercentage;
float boostValvePositionVoltage1;

float boostValvePositionVoltageMinimum = 0.45;          // Throttle blade closed, hold all of the boost
float boostValvePositionVoltageMaximum = 4.00;          // Throttle blade open, release all of the boost

/*
Define function - Set travel limits of boost valve
*/
void setBoostValveTravelLimits(){
    SERIAL_PORT_MONITOR.print("INFO: Setting boost valve travel limits ... ");
    // We subtract 0.25V as the throttle body used opens past 90 degrees and we don't want it to target the physical stop
    boostValvePositionVoltageMaximum = (analogRead(boostValvePositionSignal1Pin) * (5.0 / 1023.0)) - 0.25;
    SERIAL_PORT_MONITOR.println("DONE");
    SERIAL_PORT_MONITOR.print("  Set fully closed (max boost) at ");
    SERIAL_PORT_MONITOR.print(boostValvePositionVoltageMinimum);
    SERIAL_PORT_MONITOR.println("V");
    SERIAL_PORT_MONITOR.print("  Set fully open (no boost) at ");
    SERIAL_PORT_MONITOR.print(boostValvePositionVoltageMaximum);
    SERIAL_PORT_MONITOR.println("V");
}

/*
Define function - Determine current boost valve position percentage
*/
float getBoostValveOpenPercentage(){
    // Read sensor voltages
    boostValvePositionVoltage1 = analogRead(boostValvePositionSignal1Pin) * (5.0 / 1023.0);
    // Calculate position percentage
    if (boostValvePositionVoltage1 >= boostValvePositionVoltageMaximum) {
        return 100.0;
    } else if (boostValvePositionVoltage1 <= boostValvePositionVoltageMinimum) {
        return 0.00;
    } else {
        boostValveOpenPercentage = ((boostValvePositionVoltage1 - boostValvePositionVoltageMinimum) /
                                    (boostValvePositionVoltageMaximum - boostValvePositionVoltageMinimum)) * 100.0;
        return boostValveOpenPercentage;
    }
}

// 0.037 volts per degree roughly

// Rapid move to 10 degrees from closed = 0.45V + (10 * 0.037) = 0.82V sensor voltage
// Move more slowly looking at commanded vs achieved until we can see 