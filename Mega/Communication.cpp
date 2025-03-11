#include "Communication.h"

///////////
// INPUT //
///////////
// Buffer to hold the incoming NMEA sentence
char buffer[BUFFERSIZE];
int index = 0;
void readNMEAData() {
  // Read the data from Serial
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '$') { // Start of a new sentence
      clearBuffer();
      index = 0;
    } 
    else if (c == '*') {
      buffer[index] = '\0'; // End of the sentence
      lastContactTime = currentTime; //Consider any complete sentence a valid contact
      parseNMEA(buffer); // Call function to parse the sentence
      clearBuffer();
      break;
    } 
    else {
      if (index < BUFFERSIZE - 1) { // Ensure space for null terminator
        buffer[index] = c;
        index += 1;
      } 
      else {
        clearQueue();
        clearBuffer();

        #if DEBUG
        prepareDebugData("Arduino: Data input was too long and has been discarded.");
        #endif

        break;
      }
    }
  }
}

void clearQueue() {
  while (Serial.available()) {
    char c = Serial.read();
  }
}

void clearBuffer() {
  memset(buffer, 0, sizeof(buffer));
  index = 0;
}

// Function to parse/process the received NMEA sentence
void parseNMEA(char* sentence) {
  #if DEBUG
    //prepareDebugData("Arduino: Trying to read Orin data."");
  #endif

  if (strncmp(sentence, "WHEEL", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    int speeds[NUM_WHEELS - 1];
    int dirs[NUM_WHEELS - 1];
    for (int i = 0; i < NUM_WHEELS-1; i++) {
      int speed = atoi(token);
      token = strtok(NULL, ",");
      int dir = atoi(token);
      token = strtok(NULL, ",");
      speeds[i] = speed;
      dirs[i] = dir;
    }
    handleDrive(speeds, dirs);
  }
  else if (strncmp(sentence, "LASER", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    int laser = atoi(token);
    handleLaser(laser);
  }
  else if (strncmp(sentence, "ELEVA", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    int speed = atoi(token);
    token = strtok(NULL, ",");
    int dir = atoi(token);
    token = strtok(NULL, ",");
    handleElevator(speed, dir);
  }
  else if (strncmp(sentence, "CLICK", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    bool click = atoi(token);
    handleClickerCommand(click);
  }
  else if (strncmp(sentence, "FPVSV", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    float yaw = atof(token);
    token = strtok(NULL, ",");
    float pitch = atof(token);
    handleFPV(yaw, pitch);
  }
  else if (strncmp(sentence, "HEART", 5) == 0) {
    char* token = strtok(sentence + 6, ","); 
    float elapsedConnection = atof(token);
    handleHeartbeat(elapsedConnection);
  }
}

////////////
// OUTPUT //
////////////
void sendIRData() {
  // Construct NMEA sentence for IR data
  String sentence = "$IRLIG,";

  for (int i = 0; i < 2; i++) {
    sentence += String(sensorArray[i]) + ",";
  }

  sentence += "*"; // End sentence

  // Send data
  if (Serial.availableForWrite() > 0)
    Serial.println(sentence);
}

#if DEBUG
void prepareDebugData(int newMessageAsInt) {
  char newMessageAsString[20] = {0};
  sprintf(newMessageAsString, "%d", newMessageAsInt);
  prepareDebugData(newMessageAsString);
}

void prepareDebugData(const char* newMessage) {
  // Ensure we don't overwrite the buffer
  int len = strlen(debugMessage);  // Find current length of debugMessage
  if (len + strlen(newMessage) < DEBUG_MSG_SIZE - 9) { // -9 accounts for '$DEBUG,' and ',*' that will be appended
    snprintf(debugMessage + len, DEBUG_MSG_SIZE - len, "%s", newMessage);
  } 
  else {
    memset(debugMessage, 0, sizeof(debugMessage));
    snprintf(debugMessage, DEBUG_MSG_SIZE, "Arduino: Debug message overflow.");
    sendDebugData();
  }
}

void sendDebugData() {
  // Construct NMEA sentence for Debug data
  // Insert the "$DEBUG," prefix at the beginning
  memmove(debugMessage + 7, debugMessage, strlen(debugMessage));
  memcpy(debugMessage, "$DEBUG,", 7);  // 7 characters to place "$DEBUG,"
  // Insert the ",*" suffix to the end
  strcat(debugMessage, ",*");
  
  // Send data
  if (Serial.availableForWrite() > 0)
    Serial.println(debugMessage);
  else
    prepareDebugData("$DEBUG,Arduino: Not ready to write at time of write call.,*");
  
  // Clear debug message buffer after sending
  memset(debugMessage, 0, sizeof(debugMessage));
}
#endif
