#include "serialCommunications.h"
#include "debugUtils.h"

/*
Define variables
*/
const int maxMessageSize = 120; // Maximum size of the message
bool partialMessagePresent = false;
char partialMessage[maxMessageSize] = {'\0'};

int messagesReceived = 0;
int partialMessagesReceived = 0;
int messagesWithBadChecksum = 0;
int corruptMessages = 0;

/*
Function - Output serial debug stats
*/
void serialReportPerformanceStats() {
  SERIAL_PORT_MONITOR.print("\nTotal messages received: ");
  SERIAL_PORT_MONITOR.println(messagesReceived);
  SERIAL_PORT_MONITOR.print("Partial messages received: ");
  SERIAL_PORT_MONITOR.println(partialMessagesReceived);
  SERIAL_PORT_MONITOR.print("Messages with bad checksum: ");
  SERIAL_PORT_MONITOR.println(messagesWithBadChecksum);
  SERIAL_PORT_MONITOR.print("Corrupt messages: ");
  SERIAL_PORT_MONITOR.println(corruptMessages);
  SERIAL_PORT_MONITOR.println();
}

/*
Function - Check if checksum is valid
*/
bool serialIsChecksumValid(const char *message) {
  int len = strlen(message);
  int lastCommaIndex = -1;

  for (int i = len - 1; i >= 0; i--) {
    if (message[i] == ',') {
      lastCommaIndex = i;
      break;
    }
  }

  if (lastCommaIndex == -1 || len - lastCommaIndex - 1 > 3) {
    return false;
  }

  char receivedChecksum[4];
  strncpy(receivedChecksum, &message[lastCommaIndex + 1], 3);
  receivedChecksum[3] = '\0';

  char calculatedChecksum = 0;
  for (int i = 1; i < lastCommaIndex; i++) {
    calculatedChecksum ^= message[i];
  }

  return atoi(receivedChecksum) == calculatedChecksum;
}

/*
Function - Parse received message and take action based on command ID
*/
void serialProcessMessage(const char *message, float *speed, int *rpm, int *gear, bool *clutchPressed) {

}

/*
Function - Read new serial message
*/
const char *serialGetIncomingMessage() {
  static char message[maxMessageSize] = {'\0'}; // Array to store the message
  static char returnMessage[15] = {'\0'};       // A simply array to hold our return strings
  int messageSize = 0;                          // Current size of the message
  bool validMessage = false;

  // Throw away any characters until we get a message start or there is no more data, but only if we are not apending to a previous partial message
  if (partialMessagePresent == false) {
    while (SERIAL_PORT_HARDWARE1.peek() != '<' && SERIAL_PORT_HARDWARE1.available() > 0) {
      SERIAL_PORT_HARDWARE1.read();
    }
  } else if (partialMessagePresent == true) { // If we are appending to a previous partial message, load it in before we start reading new characters
    strcpy(message, partialMessage);
    messageSize = strlen(message);
    DEBUG_PRINT("Retrieving partial message: " + String(partialMessage));
    DEBUG_PRINT("Main message now contains: " + String(message));
  }

  // Read characters from Serial until end marker '>' is received
  while (SERIAL_PORT_HARDWARE1.available() > 0) {
    char incomingChar = SERIAL_PORT_HARDWARE1.read();

    if (partialMessagePresent == true) {
      DEBUG_PRINT("Read in character: " + String(incomingChar));
    }

    // Add the character to the message and guard against buffer overflow
    if (messageSize < maxMessageSize - 1) { // Ensure there is space for the null terminator
      message[messageSize++] = incomingChar;
      message[messageSize] = '\0'; // Null terminate after adding each character
    } else {
      // Handle buffer full condition, optionally print an error message
      DEBUG_PRINT("Buffer full, message truncated");
      break; // Exit the loop to avoid further processing
    }

    if (incomingChar == '>') { // End of message, break out and anything received before next function execution will be discarded
      partialMessagePresent = false;
      messagesReceived++;

      if (messageSize < maxMessageSize) {
        message[messageSize] = '\0';
      } else {
        message[maxMessageSize - 1] = '\0'; // Ensure null termination
      }

      // Check if the message is valid
      int countOpenBracket = 0;
      int countCloseBracket = 0;

      // Count the occurrences of '<' and '>'
      for (int i = 0; i < messageSize; i++) {
        if (message[i] == '<') {
          countOpenBracket++;
        } else if (message[i] == '>') {
          countCloseBracket++;
        }
      }

      // Validate the message format
      bool validStart = (message[0] == '<');
      bool validEnd = (message[messageSize - 1] == '>');
      bool validCounts = (countOpenBracket == 1) && (countCloseBracket == 1);

      if (validStart && validEnd && validCounts) {
        validMessage = true;
      } else {
        corruptMessages++;
        validMessage = false;
        DEBUG_PRINT("CORRUPT message: " + String(message));
        strcpy(returnMessage, "corrupt");
        partialMessagePresent = false; // Clear partial message after processing
        partialMessage[0] = '\0';
        return returnMessage;
      }

      // Process the valid message
      if (validMessage) {
        DEBUG_PRINT("VALID message format: " + String(message));
        // Ensure the checksum is valid, discard if it is not
        if (serialIsChecksumValid(message)) {
          return message;
        } else {
          DEBUG_PRINT("CHECKSUM failure: " + String(message));
          messagesWithBadChecksum++;
          strcpy(returnMessage, "badChecksum");
          return returnMessage;
        }
      }
    } else if (SERIAL_PORT_HARDWARE1.available() == 0) { // There is no more content in the buffer, and end of message not received. Store message content for appending to later
      partialMessagePresent = true;
      partialMessagesReceived++;
      strcpy(partialMessage, message);
      DEBUG_PRINT("Storing partial message: " + String(message) + " and last character is " + String(incomingChar));
      strcpy(returnMessage, "partial"); // We have stored a partially received message and will pick it up next function call
      return returnMessage;
    }
  }
  strcpy(returnMessage, "empty"); // The buffer was empty and we must return from the function call
  return returnMessage;
}

// TODO:
// Calculate checksum from string if possible and compare, discard if computation no good
//   If checksum is good, extract all fields and hand back to main from function
//   This should cater for various command ID's
// Create function for checksum calculation
//
// Look at better checksum as value is the same for multiple values
