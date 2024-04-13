#include "serialMessageProcessing.h"
#include "globalHelpers.h"

/* ======================================================================
   FUNCTION: Parse received message and take action based on command ID
   ====================================================================== */
int serialProcessMessage(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed) {
  // Determine message type
  int CommandId = -1;
  sscanf(serialMessage, "<%d", &CommandId);

  switch (CommandId) {
    case 0:
      // Master is requesting our current information to be sent over serial
      DEBUG_PRINT("PROCESSING command ID " + String(CommandId) + " message " + String(serialMessage));
      serialProcessCommandId0(serialMessage);
      return 0;

    case 1:
      // Master is pushing us the current state so we can update our local variables and make good decisions
      DEBUG_PRINT("PROCESSING command ID " + String(CommandId) + " message " + String(serialMessage));
      serialProcessCommandId1(serialMessage, speed, rpm, gear, clutchPressed);
      return 1;

    default:
      // Unknown message type
      DEBUG_PRINT("Command ID " + String(CommandId) + " not supported, unable to process " + String(serialMessage));
      return 255;
  }
}

/* ======================================================================
   FUNCTION: Process command ID 0 (request current data of boost controller from master)
   ====================================================================== */
void serialProcessCommandId0(const char *serialMessage) {
  // Print or use the parsed values for demonstration
}

/* ======================================================================
   FUNCTION: Process command ID 1 (update pushed status data from master)
   ====================================================================== */
void serialProcessCommandId1(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed) {
  // Create tokens from comma delimited message
  char *token = strtok(const_cast<char *>(serialMessage), ",");
  int positionCounter = 0;

  // Extract the values we are interested in based on position
  while (token != NULL) {
    switch (positionCounter) {
      case 1:
        *speed = atof(token);
        break;
      case 2:
        *rpm = atoi(token);
        break;
      case 3:
        *gear = atoi(token);
        break;
      case 4:
        *clutchPressed = (strcmp(token, "1") == 0);
        break;
    }
    // Get the next token
    token = strtok(NULL, ",");
    positionCounter++;
  }
}
