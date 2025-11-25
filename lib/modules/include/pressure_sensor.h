#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "config.h"

// Sensor validation results
enum class SensorStatus {
    OK_BOTH,
    ONE_ILLOGICAL,
    TWO_ILLOGICAL,
    PENDING_FAULT
};

class PressureSensor {
public:
    PressureSensor();
    
    // Validate two pressure sensor readings and return status and chosen pressure
    SensorStatus validateTwoSensors(float P1, float P2, float& chosenPressure);
    
    bool isPressureValid(float pressure);
    void updateConsecInvalidity(bool* valid);
    
    void resetConsecutiveFaults();
    
private:
    int m_consecutiveDifferenceFaults, m_consecutiveInvalid[SensorConfig::NUM_PTS];
};

#endif // PRESSURE_SENSOR_H