#ifndef ANALOG_CURVE_H
#define ANALOG_CURVE_H

#include <stdint.h>

// Two Supported Curve Structures
// Interpolated Linear
//      - Interpolate between calibated Analog-Float Points
//      - Pass in as many points as you'd like
// Polynomial Coefficients
//      - Perform a regression outside of the Nano and import the coefficents

#define LINEAR_INTERPOLATE 0
#define POLY_COEFFS 1

// Change this value to switch styles
#define SELECTED_CURVE_STYLE POLY_COEFFS
// #define SELECTED_CURVE_STYLE LINEAR_INTERPOLATE

#include "../mem_manager/mem_map.h"

float calcTemperatureCelsius();
float calcRelativeHumidity();
void saveTemperatureCurveData(uint8_t* operand_buffer, uint16_t buffer_length);
void saveHumidityCurveData(uint8_t* operand_buffer, uint8_t buffer_length);
void echoTemperatureCurveData();
void echoHumidityCurveData();
void clearTemperatureCurveData();
void clearHumidityCurveData();

#endif /* ANALOG_CURVE_H */
