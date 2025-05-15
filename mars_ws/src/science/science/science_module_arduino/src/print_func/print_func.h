#ifndef PRINT_FUNC_H
#define PRINT_FUNC_H

#include <stdint.h>

void printByteAsHex(uint8_t b);
void printBytesInHex(uint8_t* byte_ptr, uint16_t byte_cnt);

#endif /* PRINT_FUNC_H */