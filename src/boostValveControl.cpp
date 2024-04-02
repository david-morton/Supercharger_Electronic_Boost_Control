#include "boostValveControl.h"
#include <PID_v1.h>
#include <ptScheduler.h>
#include "CytronMotorDriver.h" // Library for the Cytron MDD10 motor driver

/*
Define pin constants
*/
const byte boostValvePositionSignal1Pin = A14;   // Words here

/*
Configure the motor driver board
*/
CytronMD boostValveMotor(PWM_DIR, 9, 8);          // PWM = Pin 3, DIR = Pin 4

/*
Define variables
*/
float boostValveOpenPercentage;
float boostValvePositionVoltage1;

float boostValvePositionVoltageMinimum;          // Throttle blade closed, hold all of the boost
float boostValvePositionVoltageMaximum;          // Throttle blade open, release all of the boost

/*
Define function - Set travel limits of boost valve
*/
void setBoostValveTravelLimits(){
    ptScheduler ptIncrementValvePosition = ptScheduler(PT_TIME_50MS);
    bool boostValveClosedPositionSet = false;
    bool boostValveOpenPositionSet = false;
    const int windowSize = 5;           // Number of samples to consider for moving average
    int readings[windowSize];           // Create array for moving average samples
    int currentIndex = 0;               // Set index for moving average data
    float stabilityThreshold = 20.0;

    SERIAL_PORT_MONITOR.print("INFO: Setting boost valve travel limits ... ");

    // Set motor speed then immediately iterate on potentiometer / position feedback to determine movement limit
    boostValveMotor.setSpeed(32);   // Speed range is from -255 to 255
    // delay(200); // Ensure the mechanism has time to start moving
    while (boostValveClosedPositionSet == false) {
        if (ptIncrementValvePosition.call()) {
            int potentiometerValue = analogRead(boostValvePositionSignal1Pin);

            // Add current reading to the array
            SERIAL_PORT_MONITOR.println("ADDING");
            readings[currentIndex] = potentiometerValue;
            currentIndex = (currentIndex + 1) % windowSize;
            // Calculate moving average
            float sum = 0.0;
            for (int i = 0; i < windowSize; i++) {
                sum += readings[i];
            }
            float movingAverage = sum / windowSize;

            // Check for stability in moving average
            if (abs(potentiometerValue - movingAverage) < stabilityThreshold) {
                // Arm has hit a stop, stop the motor
                boostValveMotor.setSpeed(0);
                boostValvePositionVoltageMinimum = movingAverage * (5.0 / 1023.0);
                boostValveClosedPositionSet = true;
                SERIAL_PORT_MONITOR.print("\n\nWe have hit stop as ");
                SERIAL_PORT_MONITOR.print(abs(potentiometerValue - movingAverage));
                SERIAL_PORT_MONITOR.print(" < ");
                SERIAL_PORT_MONITOR.println(stabilityThreshold);
                for (int i = 0; i < windowSize; i++) {
                    Serial.print(readings[i]);
                    Serial.print(" ");
                }
                Serial.println();
            }
        }
    }
    
    // Set voltage at open position (spring is assisting)
    SERIAL_PORT_MONITOR.print("  Set fully closed (max boost) at ");
    SERIAL_PORT_MONITOR.print(boostValvePositionVoltageMinimum);
    SERIAL_PORT_MONITOR.println("V");

    // Set voltage at closed position (working against the spring)

    SERIAL_PORT_MONITOR.print("  Set fully open (no boost) at ");
    SERIAL_PORT_MONITOR.print(boostValvePositionVoltageMaximum);
    SERIAL_PORT_MONITOR.println("V\n");
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