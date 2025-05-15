#ifndef INTERPOLATE_CURVE_H
#define INTERPOLATE_CURVE_H

#include <stdint.h>
#include <EEPROM.h>

// (X,Y) Pair for calibration curves
struct calibration_point_t {
    uint16_t analog_value; // 2 Bytes
    float calibrated_value; // 4 Bytes
};

void parseOperandsForInterpolate(uint8_t* operand_buffer, uint16_t buffer_length, bool isTemperature);
float interpolateAnalogValue(calibration_point_t* eprom_ptr, uint8_t point_cnt, uint16_t analog_value);
void writeCalibrationPoint(calibration_point_t* eprom_ptr, calibration_point_t calibration_pt);
void writeCalibrationPoint(calibration_point_t* array_ptr, calibration_point_t calibration_pt);
void pushTempCalibrationPoint(calibration_point_t calibration_pt);
void pushHumCalibrationPoint(calibration_point_t calibration_pt);
void clearCalibrationCurves();
void clearTempCurve();
void clearHumCurve();
uint8_t readTempPointCnt();
uint8_t readHumPointCnt();
calibration_point_t readCalibrationPoint(calibration_point_t* array_ptr, uint8_t index);
calibration_point_t readCalibrationPoint(calibration_point_t* array_ptr);
void swapCalibrationPoints(calibration_point_t* pt_1, calibration_point_t* pt_2);
void echoTempCurve();
void echoHumCurve();
#endif /* INTERPOLATE_CURVE_H */