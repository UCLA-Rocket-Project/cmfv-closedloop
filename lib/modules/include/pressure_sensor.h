#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

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
    
    // Check if a single pressure reading is valid
    bool isPressureValid(float pressure);
    
    // Check if pressure jump is within acceptable limits
    bool isJumpAcceptable(float currentPressure, float previousPressure);
    
    // Update previous readings for jump detection
    void updatePreviousReadings(float P1, float P2);
    
    // Reset the consecutive fault counter
    void resetConsecutiveFaults();
    
private:
    float m_prevP1;
    float m_prevP2;
    bool m_firstReading;
    int m_consecutiveDifferenceFaults;
    int m_consecutiveInvalidP1;
    int m_consecutiveInvalidP2;
};

#endif // PRESSURE_SENSOR_H