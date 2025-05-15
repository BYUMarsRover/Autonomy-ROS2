#ifndef HARDWARE_FUNCTIONS_H
#define HARDWARE_FUNCTIONS_H

#include <stdint.h>

void setPinModes();
bool isDigitalWrittenHigh(uint8_t pin);
void stopActuator(uint8_t actuator_index);
void stopToolActuator(uint8_t tool_index);
void stopAllToolActuators();
void stopCacheActuator(uint8_t cache_index);
void stopAllCacheActuators();
void stopAllActuators();
void writeActuator(uint8_t actuator_index, int8_t speed);
void stopDrill();
void writeDrill(int8_t drill_speed);


#endif /* HARDWARE_FUNCTIONS_H */
