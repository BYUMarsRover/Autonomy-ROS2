# Contains the payload of a science serial protocol packet to be sent to the
# Science Arduino over it's serial interface

uint8 command_word
uint8[] operands

# if packet contains data the command_word and operands fields are ignored
uint8[] packet