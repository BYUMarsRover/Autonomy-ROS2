#ifndef POLY_CURVE_H
#define POLY_CURVE_H

#include <EEPROM.h>

float calculatePolynomial(float* coeff_array_addr_eeprom, uint8_t n_coeff, float x_norm);
uint8_t readTempCoeffCnt();
uint8_t readHumCoeffCnt();
void setTempCoeffCnt(uint8_t cnt);
void setHumCoeffCnt(uint8_t cnt);
void clearAllCoeffs();
void clearTempCoeffs();
void clearHumCoeffs();
float readCoefficentAt(float* coeff_array_addr_eeprom, uint8_t index);
void writeTempCoeffs(float* buffer, uint8_t length);
void writeHumCoeffs(float* buffer, uint8_t length);
void echoTempCoeffs();
void echoHumCoeffs();

#endif /* POLY_CURVE_H */