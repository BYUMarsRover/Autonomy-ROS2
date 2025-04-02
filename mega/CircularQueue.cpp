
#include "CircularQueue.h"

// Function to calculate the available space in the buffer
size_t availableSpace() {
  if (writeEndIdx >= writeStartIdx)
    return (WRITE_QUEUE_SIZE - writeEndIdx + writeStartIdx - 1);
  else
    return (writeStartIdx - writeEndIdx - 1);
}

// Function to enqueue (write) c-string data to the buffer
void queuePrint(const char* msg) {
  size_t avail = availableSpace();
  size_t len = strlen(msg);
  
  // If there is not enough space, return false
  if (len > avail) {
    memset(writeQueue, 0, sizeof(writeQueue));
    #if DEBUG
    prepareDebugData("Arduino: Write buffer full; wiping all old data.");
    #endif
    return;
  }

  for (size_t i = 0; i < len; ++i) {
    writeQueue[writeEndIdx] = msg[i];
    writeEndIdx = (writeEndIdx + 1) % WRITE_QUEUE_SIZE;
  }
}

// Function to dequeue data from the buffer until newline '\n' is found
void dequeuePrint(char* output, int maxLength) {
  int outputIndex = 0;

  while (writeStartIdx != writeEndIdx) {
    char currentByte = writeQueue[writeStartIdx];

    // Stop if newline is encountered
    if (currentByte == '\n') {
      output[outputIndex] = '\0';
      writeStartIdx = (writeStartIdx + 1) % WRITE_QUEUE_SIZE;
      return;
    }
      
    output[outputIndex++] = currentByte;
    writeStartIdx = (writeStartIdx + 1) % WRITE_QUEUE_SIZE;

    if (outputIndex == maxLength - 1) {
      output[outputIndex] = '\0';
      return; // Reached chunk limit
    }
  }
}
