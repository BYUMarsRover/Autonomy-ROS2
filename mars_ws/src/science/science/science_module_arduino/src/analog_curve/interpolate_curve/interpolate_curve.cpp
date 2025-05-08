#include "interpolate_curve.h"
#include "../../error/error.h"
#include "../../definitions/definitions.h"
#include "../../mem_manager/mem_manager.h"
#include "../analog_curve.h"

// Determine the appropriate action based on the number of opreands
void parseOperandsForInterpolate(uint8_t* operand_buffer, uint16_t buffer_length, bool isTemperature) {

    // If the buffer only has one float, assume its a calibration request
    if (buffer_length == sizeof(float)) {

        // Check to see if there is room for another point
        if ((isTemperature ? readTempPointCnt() : readHumPointCnt()) == NUM_CALIBRATION_PTS) {
            // TODO send an error, the buffer is full
            return;
        }

        // This is a calibration request, pair with the current sensor value;
        calibration_point_t cal_pt;
        cal_pt.analog_value = analogRead(isTemperature ? TEMPERATURE_PIN : MOISTURE_PIN);
        cal_pt.calibrated_value = *((float*)operand_buffer);
        if (isTemperature) pushTempCalibrationPoint(cal_pt);
        else pushHumCalibrationPoint(cal_pt);

    } else {

        // Check for possible buffer overflow
        if (buffer_length > CURVE_ARRAY_SIZE) {
            // TODO send an error, too many points 
            return;
        }

        // These are complete calibration points to push, overwrite all previous
        // Then copy memeory into the buffer
        EEPROM_pushSizedArray(
            (calibration_point_t*)(isTemperature ? EEPROM_TEMP_CAL_PTS_COUNT : EEPROM_TEMP_CAL_PTS_ARRAY),
            (calibration_point_t*)(isTemperature ? EEPROM_HUM_CAL_PTS_COUNT : EEPROM_HUM_CAL_PTS_ARRAY),
            (calibration_point_t*)operand_buffer,
            (buffer_length / sizeof(calibration_point_t))
        );
    }
}

// Interpolate between two calibration_points
float interpolate(calibration_point_t a, calibration_point_t b, uint16_t analog_value) {
    // a.y + ((b.y-a.y)/(b.x-a.x))*(x - a.x)
    return a.calibrated_value
        + ((b.calibrated_value - a.calibrated_value) / (b.analog_value - a.analog_value))
        * (analog_value - a.analog_value);
}

// Calculates the predicted value based on linear interpolation between saved calibrated points
float interpolateAnalogValue(calibration_point_t* eprom_ptr, uint8_t point_cnt, uint16_t analog_value) {
    calibration_point_t higher_point;
    calibration_point_t lower_point = readCalibrationPoint(eprom_ptr, 0);

    // Analog Value is less than lowest calibration point
    if (analog_value < lower_point.analog_value) {
        // Value is not within the range, extending lines between pt0 and pt1 backwards
        higher_point = readCalibrationPoint(eprom_ptr, 1);
        return interpolate(lower_point, higher_point, analog_value);
    }

    // Analog Value is between two calibration point
    for (int i = 1; i < point_cnt; i++) {
        higher_point = readCalibrationPoint(eprom_ptr, i);
        if (analog_value > lower_point.analog_value && analog_value < higher_point.analog_value) {
            return interpolate(lower_point, higher_point, analog_value);
        } else {
            // Try the next two point pairs
            lower_point = higher_point;
        }
    }

    // Analog Value is greater than highest calibration point
    return interpolate(lower_point, higher_point, analog_value);
}

// Write a calibration point to any location in EEPROM
void writeCalibrationPoint(calibration_point_t* eeprom_ptr, uint8_t index, calibration_point_t calibration_pt) {
    EEPROM.put((size_t)(eeprom_ptr + index), calibration_pt);
}

// Add a point to the temp of the humidity curve
void pushTempCalibrationPoint(calibration_point_t calibration_pt) {
    writeCalibrationPoint((calibration_point_t*)EEPROM_TEMP_CAL_PTS_ARRAY, readTempPointCnt(), calibration_pt);
}

// Add a point to the end of the humidity curve
void pushHumCalibrationPoint(calibration_point_t calibration_pt) {
    writeCalibrationPoint((calibration_point_t*)EEPROM_HUM_CAL_PTS_ARRAY, readHumPointCnt(), calibration_pt);
}

// Manage Calibration Curve Memory
// Doesn't actually delete the data, just tells the system
// they are none saved (old binary isn't overwritten)
void clearCalibrationCurves() {
    clearTempCurve();
    clearHumCurve();
}

void clearTempCurve() {
    EEPROM.write((size_t)EEPROM_TEMP_CAL_PTS_COUNT, 0);
}

void clearHumCurve() {
    EEPROM.write((size_t)EEPROM_HUM_CAL_PTS_COUNT, 0);
}

// Returns number of active temperature calibration points
uint8_t readTempPointCnt() {
    return EEPROM.read((size_t)EEPROM_TEMP_CAL_PTS_COUNT);
}

// Returns number of active humidity calibration points
uint8_t readHumPointCnt() {
    return EEPROM.read((size_t)EEPROM_HUM_CAL_PTS_COUNT);
}

// Returns a calibration point by a pointer
// e.g. readCalibrationPoint(cali_point_ptr);
calibration_point_t readCalibrationPoint(calibration_point_t* array_ptr, uint8_t index) {
    return readCalibrationPoint(array_ptr + index);
}

calibration_point_t readCalibrationPoint(calibration_point_t* array_ptr) {
    calibration_point_t point;
    EEPROM.get((size_t)array_ptr, point);
    return point;
}

// Swap two calibration points
void swapCalibrationPoints(calibration_point_t* pt_1, calibration_point_t* pt_2) {
    calibration_point_t pt_cache = readCalibrationPoint(pt_2);
    writeCalibrationPoint(pt_2, readCalibrationPoint(pt_1));
    writeCalibrationPoint(pt_1, pt_cache);
}

// Echo the temperature csv points to the response buffer
void echoTempCurve() {
    EEPROM_echoArray((calibration_point_t*)EEPROM_TEMP_CAL_PTS_COUNT,
        (calibration_point_t*)EEPROM_TEMP_CAL_PTS_ARRAY
    );
}

// Echo the humidity csv points to the response buffer
void echoHumCurve() {
    EEPROM_echoArray((calibration_point_t*)EEPROM_HUM_CAL_PTS_COUNT,
        (calibration_point_t*)EEPROM_HUM_CAL_PTS_ARRAY
    );
}
