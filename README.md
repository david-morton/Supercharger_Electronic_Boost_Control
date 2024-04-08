# Project Purpose & Description
- This Arduino is kept separate from the 'master' so that it can do a single thing, and do it well. It should not create any dangerous situations in the absense of the master and must always fail safe (valve open, spilling all the boost).

# Hardware Used
The hardware used for my particular appliation is:
- Arduino Mega
  - Where this code is run

# Implementation Detail & Architecture
- Words here later

# Serial Protocol
This section defines how the serial comms between master and slave work; and defines the message types and structures.

## Message Structures
- Message boundaries are indicated by < and > symbols for start and end respectively
- We cater for multiple fields by being comma delimited
- A basic XOR checksum is used so we can discard (most) corrupt messages

## Message Types
Currently we only have a few message types supported (described from master perspective)
- Push current readings for speed, RPM, gear and clutch pedal status
- Request current readings for boost pressure and valve open percentage
- Request error status

## Example Messages
The standard message format is as below. The number and type of the data fields is unique to each commandId.
<commandId,data1,data2,data3,...,checksum>

Command ID's in use are as below. The master is the 'main' car module which drives the dashboard via CAN and the slave is the boost controller who's code is in this repo.
| Command ID  | Direction       | Data Fields                                                     | Description                                       |
| ----------- | --------------- | --------------------------------------------------------------- | ------------------------------------------------- |
| 0           | Master to slave | -                                                               | Request current data from boost controller        |
| 1           | Master to slave | currentRpm,currentSpeed,currentGear,clutchPressed               | Push current data from master to boost controller |
| 2           | Slave to master | errorStatus,currentBoost,currentTemp,currentValveOpenPercentage | Response to command ID 0                          |

# Technical Notes
- Words here later

# Todo
- Words here later