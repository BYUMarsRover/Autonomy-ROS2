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
  else if (strncmp(sentence, "ELEVA", 5) == 0 && !cardiacArrest) {
    char* token = strtok(sentence + 6, ","); 
    int speed = atoi(token);
    token = strtok(NULL, ",");
    int dir = atoi(token);
    token = strtok(NULL, ",");
    handleElevator(speed, dir);
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
