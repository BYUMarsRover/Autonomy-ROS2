#include "print_func.h"
#include <Arduino.h>

void printByteAsHex(uint8_t b) {
    Serial.print("0x");
    Serial.print(b < 16 ? "0" : "");
    Serial.print(b, HEX);
    Serial.print(" ");
}

// Print bytes in hexidecimal notation
void printBytesInHex(uint8_t* byte_ptr, uint16_t byte_cnt) {
    for (uint8_t i = 0; i < byte_cnt; i++) {
        // Print it in 0xAA hexadecimal style
        printByteAsHex(byte_ptr[i]);
    }
}