#include "serialCommunications.h"

/*
Define variables
*/
bool partialMessagePresent = false;
char partialMessage[60] = {'\0'};

int messagesReceived = 0;
int partialMessagesReceived = 0;
int messagesWithBadChecksum = 0;
int corruptMessages = 0;

/*
Function - Output serial debug stats
*/
void serialReportDebugStats() {
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
Function - Check for new serial messages
*/
void serialCheckForMessage() {
  const int maxMessageSize = 60;         // Maximum size of the message
  char message[maxMessageSize] = {'\0'}; // Array to store the message
  int messageSize = 0;                   // Current size of the message
  bool validMessage = false;

  // Throw away any characters until we get a message start or there is no more data, but only if we are not apending to a previous partial message
  if (partialMessagePresent == false) {
    while (SERIAL_PORT_HARDWARE1.peek() != '<' && SERIAL_PORT_HARDWARE1.available() > 0) {
      SERIAL_PORT_HARDWARE1.read();
    }
  } else if (partialMessagePresent == true) { // If we are appending to a previous partial message, load it in before we start reading new characters
    strcpy(message, partialMessage);
    messageSize = strlen(message);
    SERIAL_PORT_MONITOR.println("Retrieving partial message");
  }

  // Read characters from Serial until '>' is received
  while (SERIAL_PORT_HARDWARE1.available() > 0) {
    char incomingChar = SERIAL_PORT_HARDWARE1.read();

    // Add the character to the message
    message[messageSize++] = incomingChar;

    if (incomingChar == '>') { // End of message, break out and anything received before next function execution will be discarded
      partialMessagePresent = false;
      messagesReceived++;
      message[messageSize] = '\0';

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
        Serial.print("BAD message: ");
        Serial.println(message);
      }

      // Print the final message
      if (validMessage) {
        SERIAL_PORT_MONITOR.print("GOOD message: ");
        SERIAL_PORT_MONITOR.println(message);
        SERIAL_PORT_MONITOR.println();
      }
      break;
    } else if (SERIAL_PORT_HARDWARE1.available() == 0) { // There is no more content in the buffer, and end of message not received
      // Store message content for appending to later
      partialMessagePresent = true;
      partialMessagesReceived++;
      strcpy(partialMessage, message);
      SERIAL_PORT_MONITOR.print("Storing partial message: ");
      SERIAL_PORT_MONITOR.println(message);
    }
  }
}

// TODO:
// Calculate checksum from string if possible and compare, discard if computation no good
//   If checksum is good, extract all fields and hand back to main from function
//   This should cater for various command ID's
// Create function for checksum calculation
// Take care of case where we overwhelm the receiver with the likes of:
//   Received message: <1,344,344.00,4,<1,353,353.00,4,1,26>
//
// Look at better checksum as value is the same for multiple values
