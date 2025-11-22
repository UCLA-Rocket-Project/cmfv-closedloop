#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

// Sensor validation results
enum class SensorStatus {
    OK_ALL,
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
    SensorStatus validateSensors(float P1, float P2, float P3, float& chosenPressure);
#endif
    SensorStatus validateSensors(float P1, float P2, float& chosenPressure); // In the 3-PT scenario, we use this if only 2 PTs are valid

    // Check if a single pressure reading is valid
    bool isPressureValid(float pressure);
    
    // Reset the consecutive fault counter
    void resetConsecutiveFaults();
    
private:
    int m_consecutiveDifferenceFaults;
    float m_lastGoodPressure;
};

#endif // PRESSURE_SENSOR_H