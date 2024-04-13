#include "serialMessageProcessing.h"
#include "debugUtils.h"

/* ======================================================================
   FUNCTION: Parse received message and take action based on command ID
   ====================================================================== */
void serialProcessMessage(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed) {
  // Determine message type
  int messageType = -1;
  sscanf(serialMessage, "<%d", &messageType);
  DEBUG_PRINT("Message type detected as " + String(messageType));

  switch (messageType) {
    case 0:
      processMessageType0(serialMessage);
      break;

    case 1:
      processMessageType1(serialMessage, speed, rpm, gear, clutchPressed);
      break;

    default:
      // Unknown message type
      DEBUG_PRINT("Unknown message type, unable to process");
      break;
  }
}

/* ======================================================================
   FUNCTION: Process command ID 0 (request current data of boost controller from master)
   ====================================================================== */
void processMessageType0(const char *serialMessage) {
  // Print or use the parsed values for demonstration
  DEBUG_PRINT("Got request for message type 0 and message is " + String(serialMessage));
}

/* ======================================================================
   FUNCTION: Process command ID 1 (update pushed status data from master)
   ====================================================================== */
void processMessageType1(const char *serialMessage, float *speed, int *rpm, int *gear, bool *clutchPressed) {
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
