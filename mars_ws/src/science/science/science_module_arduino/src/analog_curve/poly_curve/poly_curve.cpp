#include "poly_curve.h"
#include "../../error/error.h"
#include "../../definitions/definitions.h"
#include "../../mem_manager/mem_manager.h"
#include "../analog_curve.h"
#include <Arduino.h>

// Coefficients are stored in ascending value
// Example for Cubic Temperature Equation:
// 0x000: 4
// 0x002: 5.3
// 0x003: 2.6
// 0x004: 9.4
// 0x005: 0.01
// y = 5.3 + 2.6*x^1 + 9.4*x^2 + 0.01x^3

// Calculates a polynomial using norm_x as the input
// x_norm is uint16_t [0-1023] mapped to float [0.0-1.0]
float calculatePolynomial(float* coeff_array_addr_eeprom, uint8_t n_coeff, float x_norm) {
    float result = 0.0;
    for (int i = 0; i < n_coeff; i++) {
        float term = readCoefficentAt(coeff_array_addr_eeprom, i) * pow(x_norm, i);
        result += term;
    }
    return result;
}

// Returns number of active coefficents in the temperature equation
uint8_t readTempCoeffCnt() {
    return EEPROM.read((size_t)EEPROM_PTR_TEMP_COEFF_COUNT);
}

// Returns number of active coefficents in the humidity equation
uint8_t readHumCoeffCnt() {
    return EEPROM.read((size_t)EEPROM_PTR_HUM_COEFF_COUNT);
}

void setTempCoeffCnt(uint8_t cnt) {
    EEPROM.write((size_t)EEPROM_PTR_TEMP_COEFF_COUNT, cnt);
}

void setHumCoeffCnt(uint8_t cnt) {
    EEPROM.write((size_t)EEPROM_PTR_HUM_COEFF_COUNT, cnt);
}

// Set Temp Coeff Cnt and Hum Coeff Cnt to 0
void clearAllCoeffs() {
    clearTempCoeffs();
    clearHumCoeffs();
}

// Set Temp Coeff Cnt to 0
void clearTempCoeffs() {
    setTempCoeffCnt(0);
}

// Set Hum Coeff Cnt to 0
void clearHumCoeffs() {
    setHumCoeffCnt(0);
}

// Return one coefficent at a certain index
float readCoefficentAt(float* coeff_array_addr_eeprom, uint8_t index) {
    float coeff;
    return EEPROM.get((size_t)(coeff_array_addr_eeprom + index), coeff);
}

void writeTempCoeffs(float* buffer, uint8_t length) {
    EEPROM_writeArray<float>((float*)EEPROM_PTR_TEMP_COEFF_ARRAY, (float*)buffer, length);
}

void writeHumCoeffs(float* buffer, uint8_t length) {
    EEPROM_writeArray<float>((float*)EEPROM_PTR_HUM_COEFF_ARRAY, (float*)buffer, length);
}

// Sends all the configured temperature coefficents to the response buffer
void echoTempCoeffs() {
    #ifdef DEBUG
    Serial.println(F("Echoing Temp array..."));
    #endif
    EEPROM_echoArray((float*)EEPROM_PTR_TEMP_COEFF_COUNT, (float*)EEPROM_PTR_TEMP_COEFF_ARRAY);
}

// Sends all the configured humidity coefficents to the response buffer
void echoHumCoeffs() {
    #ifdef DEBUG
    Serial.println(F("Echoing Humidity array..."));
    #endif DEBUG
    EEPROM_echoArray((float*)EEPROM_PTR_HUM_COEFF_COUNT, (float*)EEPROM_PTR_HUM_COEFF_ARRAY);
}
