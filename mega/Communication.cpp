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
    bool force = atoi(token);
    handleElevator(speed, dir, force);
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
    float elapsedConnectionSec = atof(token);
    handleHeartbeat(elapsedConnectionSec * 1000);
  }
}

////////////
// OUTPUT //
////////////
void sendIRData() {
  // Construct NMEA sentence for IR data
  char sentence[50] = {0};
  strcat(sentence, "$IRLIG,");
  for (int i = 0; i < 2; i++) {
    char sensorData[10] = {0};
    sprintf(sensorData, "%d", sensorArray[i]);
    strcat(sentence, sensorData);
    strcat(sentence, ",");
  }
  strcat(sentence, "*\n"); // End sentence

  // Send data
  queuePrint(sentence);
}

unsigned long lastSuccessfulWrite = ULONG_MAX;
void writeNMEAData()
{
  // Chunk data to avoid buffer overflow
  //  Chunking also lets us do async writing,
  //  because we return to the main loop during the writing
  //  of each chunk (and between chunks).
  if (writeStartIdx != writeEndIdx) {
    if (Serial.availableForWrite() >= 16)
    {
      char chunk[16] = {0};
      dequeuePrint(chunk, 16);
      Serial.print(chunk);
      lastSuccessfulWrite = currentTime;
    }
    
    #if DEBUG
    if (currentTime >= lastSuccessfulWrite && (currentTime - lastSuccessfulWrite) > 500) {
      prepareDebugData("Arduino: Not ready to write at time of write call.");
      lastSuccessfulWrite = ULONG_MAX; // Dont want to queue the same hang
    }
    #endif
  }
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
  if (len + strlen(newMessage) < DEBUG_MSG_SIZE - 10) { // -10 accounts for '$DEBUG,' and ',*\n' that will be appended
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
  memcpy(debugMessage, "$DEBUG,", 7);
  strcat(debugMessage, ",*\n");
  
  queuePrint(debugMessage);
  
  // Clear debug message buffer after sending
  memset(debugMessage, 0, sizeof(debugMessage));
}
#endif
