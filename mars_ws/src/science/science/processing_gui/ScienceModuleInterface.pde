import java.nio.ByteBuffer;

class ScienceModuleInterface
{
    // Definitions
    byte COMMAND_HEADER = 0x53; //'S'
    byte COMMAND_FOOTER = 0x4D; //'M'
    byte RESPONSE_HEADER = 0x52; //'R'
    byte RESPONSE_FOOTER = 0x46; //'F'

    int MAX_OPERAND_ARRAY_SIZE = 255; // 2^8
    int MAX_FUNCTION_ADDR = 31; // 2^5

    byte[] author_packet(ScienceModuleCommand command)
    {
        return author_packet(command, false, false);
    }

    byte[] author_packet(ScienceModuleCommand command, boolean override, boolean ack)
    {

        byte command_word = command.command_word;
        byte[] operands = command.operands;
        
        //// Check for valid configuration
        //if (operands.length > MAX_OPERAND_ARRAY_SIZE);
        //    throw new Exception("Cannot send command, operands array exceeds maximum size of " + str(MAX_OPERAND_ARRAY_SIZE));

        // Set the override and ack commands
        if (override)   command_word |= 0b01000000;
        if (ack)        command_word |= 0b00100000;

        // Build byte array to hold command packet
        // Structure:
        // - Command Header
        // - Control Word
        // - Number of Operands
        // - Operands
        // - Command Footer
        byte[] packet = new byte[4 + operands.length];
        packet[0] = COMMAND_HEADER;
        packet[1] = command_word;
        packet[2] = byte(operands.length);
        for (int i = 0; i < operands.length; i++) packet[3 + i] = operands[i];
        packet[3 + operands.length] = COMMAND_FOOTER;

        // Send the bytes to the science module
        return packet;
    }
}
