#include "analog_curve.h"
#include "../definitions/definitions.h"
#include "../mem_manager/mem_map.h"
#include "../mem_manager/mem_manager.h"

#include "poly_curve/poly_curve.h"
#include "interpolate_curve/interpolate_curve.h"

// Returns the temperature in celsius
// converted from the analog values according to saved curve
float calcTemperatureCelsius() {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        #ifdef DEBUG
        Serial.print(F("Coeffs Configured: "));
        Serial.println(readTempCoeffCnt());
        #endif
        return calculatePolynomial(
            (float*)EEPROM_PTR_TEMP_COEFF_ARRAY,
            readTempCoeffCnt(),
            analogRead(TEMPERATURE_PIN) / 1023.0
        );
    #else
        return interpolateAnalogValue(
            (calibration_point_t*)EEPROM_TEMP_CAL_PTS_ARRAY,
            readTempPointCnt(),
            analogRead(TEMPERATURE_PIN)
        );
    #endif
}

// Returns the relative humidity 0% - 100%
// converted from the analog values according to saved curve
float calcRelativeHumidity() {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        return calculatePolynomial(
            (float*)EEPROM_PTR_HUM_COEFF_ARRAY,
            readHumCoeffCnt(),
            analogRead(MOISTURE_PIN) / 1023.0
        );
    #else
        return interpolateAnalogValue(
            (calibration_point_t*)EEPROM_HUM_CAL_PTS_ARRAY,
            readHumPointCnt(),
            analogRead(MOISTURE_PIN)
        );
    #endif
}

// Take the command operand buffer and save it to the configured
// analog curve structure for temperature
// Reports to message buffer
void saveTemperatureCurveData(uint8_t* operand_buffer, uint16_t buffer_length) {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        // Overwrite the previous coefficients
        int coeff_n = buffer_length / sizeof(float);
        writeTempCoeffs((float*)operand_buffer, coeff_n);
        setTempCoeffCnt(coeff_n);
    #else
        parseOperandsForInterpolate(operand_buffer, buffer_length, (calibration_point_t*)EEPROM_TEMP_CAL_PTS_ARRAY);
    #endif
}

// Take the command operand buffer and save it to the configured
// analog curve structure for temperature
// Reports to message buffer
void saveHumidityCurveData(uint8_t* operand_buffer, uint8_t buffer_length) {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        // Overwrite the previous coefficients
        int coeff_n = buffer_length / sizeof(float);
        writeHumCoeffs((float*)operand_buffer, coeff_n);
        setHumCoeffCnt(coeff_n);
    #else
        parseOperandsForInterpolate(operand_buffer, buffer_length, (calibration_point_t*)EEPROM_HUM_CAL_PTS_ARRAY);
    #endif
}

void echoTemperatureCurveData() {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        echoTempCoeffs();
    #else
        echoTempCurve();
    #endif
}

void echoHumidityCurveData() {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        echoHumCoeffs();
    #else
        echoHumCurve();
    #endif
}

void clearTemperatureCurveData() {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        clearTempCoeffs();
    #else
        clearTempCurve();
    #endif
}

void clearHumidityCurveData() {
    #if SELECTED_CURVE_STYLE == POLY_COEFFS
        clearHumCoeffs();
    #else
        clearHumCurve();
    #endif
}
