#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "config.h"

// Sensor validation results
enum class SensorStatus {
    OK_ALL,
    PENDING_FAULT,
    ONE_ILLOGICAL,
    TWO_ILLOGICAL
#ifdef USE_3_PTS
    , THREE_ILLOGICAL
#endif
};

class PressureSensor {
public:
    PressureSensor();
    
    // Validate pressure sensor readings and return status and chosen pressure
#ifdef USE_3_PTS
    SensorStatus validateThreeSensors(float P1, float P2, float P3, float& chosenPressure);
#else
    SensorStatus validateTwoSensors(float P1, float P2, float& chosenPressure);
#endif
    SensorStatus processTwoValidSensors(float P1, float P2, float& chosenPressure);

    // Check if a single pressure reading is valid
    bool isPressureValid(float pressure);
    void updateConsecInvalidity(bool* valid);
    
    void resetConsecutiveFaults();
    
private:
    int m_consecutiveDifferenceFaults, m_consecutiveInvalid[SensorConfig::NUM_PTS];
};

#endif // PRESSURE_SENSOR_H