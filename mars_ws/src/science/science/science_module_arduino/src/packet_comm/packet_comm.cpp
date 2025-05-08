#include "packet_comm.h"
#include "../definitions/definitions.h"
#include "../actuator_manager/actuator_manager.h"
#include "../print_func/print_func.h"
#include "../error/error.h"

//Writes to the message buffer the value contained in memory at the address readAddress
void writeToMessageBuffer(uint8_t error_code, void* readAddress, uint8_t sizeOfData) {
    response_buffer.error_code = error_code;
    memcpy(response_message_buffer, readAddress, sizeOfData); // Copy Data into message
    response_buffer.message_byte_len = sizeOfData;
}

// read serial bytes until the next possible start of serial command packet
// - true if the next bytes in the buffer are a possible command
// - false if there is no possible serial command packet in the serial buffer
bool findCommand() {

    // Loop through the buffer until a possible command is identified
    while (Serial.available()) {
        if (Serial.peek() == COMMAND_PACKET_HEADER) {
            // Found the command packet, returning true
            return true;
        }
        else {
            // Whats in the buffer is not the header byte
            // consume and proceed to next byte
            Serial.read();
        }
    }
    // There is nothing in the buffer, return false, no command here
    return false;

}

// Ensure the operands are of the expected length
// Sends an error message and returns false if not
bool verifyCommandOperandLength(uint8_t expected_length) {
    bool correct = command_buffer.operand_byte_len == expected_length;
    if (!correct) error::badOperandLengthFixed(expected_length);
    return correct;
}

// Ensure the operands are of the expected length for a curve config
// Sends an error message and returns false if not
bool verifyValidCurveConfig(uint8_t start_index) {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        // Ensure there is a complete list of floats
        uint8_t size = sizeof(float);
    #else
        // Ensure there is a complete list of calibration_point_t
        uint8_t size = sizeof(calibration_point_t);
    #endif

    bool valid = (command_buffer.operand_byte_len - start_index) % size == 0;
    if (!valid) error::badOperandLengthModulus(size);
    return valid;
}

// Ensure the actuator is question is not reserved
// Sends an error message and returns false if not
bool ensureActuatorIsFree(uint8_t actuator_index) {
    if (actuator_manager::is_reserved(actuator_index)) {
        error::actuatorReserved(actuator_index);
        return false;
    }
    return true;
}

bool verifyCommand() {
    return (command_buffer.read_header == COMMAND_PACKET_HEADER)
        && (command_buffer.read_footer == COMMAND_PACKET_FOOTER);
}

// Read from the serial buffer assuming it is a command
// and identify data fields
// returns true if the packet is normal
// returns false if the packet is malformed
bool loadCommand() {
    #ifdef DEBUG
    Serial.println(F("Loading command..."));
    #endif

    // Serial.readBytes() will timeout after 1000ms
    // Serial.read() will fail immediatnely

    // Read header, command word, and operand length
    Serial.readBytes((char*)(&command_buffer), 3);

    // Read in the operands
    Serial.readBytes((char*)(&command_operand_buffer), command_buffer.operand_byte_len);
    
    // Read final footer field
    Serial.readBytes(&(command_buffer.read_footer), 1);

    // Return the command if it looks valid
    // Else discard the command
    if (verifyCommand()) {
        #ifdef DEBUG
        Serial.println(F("Received a good packet:"));
        #endif
        return true;
    } else {
        #ifdef DEBUG
        Serial.println(F("Received a malformed packet:"));
        #endif
        return false;
    }
}

// Echo the command back and send the response
void sendResponsePacket() {
    Serial.write(RESPONSE_PACKET_HEADER);
    Serial.write(4 + command_buffer.operand_byte_len);
    Serial.write(COMMAND_PACKET_HEADER);
    Serial.write(command_buffer.read_command_word);
    Serial.write(command_buffer.operand_byte_len);
    Serial.write(command_operand_buffer, command_buffer.operand_byte_len);
    Serial.write(COMMAND_PACKET_FOOTER);
    Serial.write(response_buffer.error_code);
    Serial.write(response_buffer.message_byte_len);
    Serial.write(response_message_buffer, response_buffer.message_byte_len);
    Serial.write(RESPONSE_PACKET_FOOTER);
    response_buffer.error_code = ERROR_CODE_SUCCESS; // Clear the error code if there is one
    response_buffer.message_byte_len = 0; // Clear the message
    command_buffer.read_command_word = 0; // Clear the command_word
}

bool shouldSendResponse() {
    if (response_buffer.message_byte_len > 0) {
        return true;
    } else if ((command_buffer.read_command_word & 0b00100000) != 0) {
        // if acknowledge bit is set
        return true;
    } else if (response_buffer.error_code != ERROR_CODE_SUCCESS) {
        // if unique error
        return true;
    } else return false;
    // if ((command_buffer.read_command_word & 0b10000000) == 0) {
    //     // is Query
    //     return true;
    // }
    // else if (response_buffer.error_code != 0) {
    //     // is Error
    //     return true;
    // }
    // else if ((command_buffer.read_command_word & 0b00100000) != 0) {
    //     // is not a error, is a command, and acknowledge bit is set
    //     return true;
    // }
    // else return false;
}