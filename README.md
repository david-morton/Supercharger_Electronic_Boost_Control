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

### Message Structures
- Message boundaries are indicated by < and > symbols for start and end respectively
- We cater for multiple fields by being comma delimited
- A basic XOR checksum is used so we can discard (most) corrupt messages

### Message Types
Currently we only have a few message types supported (described from master perspective)
- Push current readings for speed, RPM, gear and clutch pedal status
- Request current readings for boost pressure and valve open percentage
- Request error status

### Example Messages
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

### Boost Control Rules / Behaviour
The following conditions cause the valve to immediately drive to 100% open (not relying on the return spring)
- Clutch pressed (sent over serial from master)
- Neutral gear detected (sent over serial from master)
- RPM is less than 1000
- Speed is less than 2 (essentially stationary)

The following conditions cause the valve to return to 100% open using the return spring only
- Critical fault detected
  - No serial comms from master
  - Overboost for more than allowed duration

### Todo List
- Perform checks of valve travel limits which were determined and fire critical failure if no good (not enough spread of readings)
- Ensure we can reliably detect if car is on when performing setup. Could cause calibration to be way off. Critical fail if not confident.
- Check calibration of atmospheric pressure value when performing setup. Critical fail if not happy.
- Think about driving conditions we will encounter, like coasting down a hill at say 3000rpm with throttle closed. Do we look at MAP sensor measuring vacuum in the manifold and compare against pressure in plumbing ?
- We will really need to account for manifold vacuum so we don't cause compressor to work against a dead head ... do we have any / much vacuum on VQ37 ?