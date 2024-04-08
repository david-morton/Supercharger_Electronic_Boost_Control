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
- We cater for multi-byte data by indicating field length
- A basic XOR checksum is used so we can discard (most) corrupt messages

## Message Types
Currently we only have a few message types supported (described from master perspective)
- Push current readings for speed, RPM, gear and clutch pedal status
- Request current readings for boost pressure and valve open percentage
- Request error status

## Example Messages
<CommandID, Length1, Data1, Length2, Data2, ..., Checksum>

# Technical Notes
- Words here later

# Todo
- Words here later