#include "pressure_sensor.h"
#include "config.h"
#include <math.h>

PressureSensor::PressureSensor() 
    : m_prevP1(0.0f), m_prevP2(0.0f), m_firstReading(true) {}

SensorStatus PressureSensor::validateTwoSensors(float P1, float P2, float& chosenPressure) {
    bool ok1 = isPressureValid(P1) && (m_firstReading || isJumpAcceptable(P1, m_prevP1));
    bool ok2 = isPressureValid(P2) && (m_firstReading || isJumpAcceptable(P2, m_prevP2));
    
    if (!ok1 && !ok2) {
        chosenPressure = 0.0f; // Invalid value
        // Serial.print("@@@@@@");
        // Serial.print(P1);
        // Serial.print(",");
        // Serial.print(P2);
        return SensorStatus::TWO_ILLOGICAL;
    }
    
    if (ok1 && ok2) {
        // Both sensors are valid, check if they agree
        float difference = fabs(P1 - P2);
        if (difference <= SensorConfig::PAIR_DIFFERENCE_THRESHOLD) {
            // Sensors agree, average them
            chosenPressure = (P1 + P2) * 0.5f;
            return SensorStatus::OK_BOTH;
        } else {
            // Sensors disagree significantly
            chosenPressure = 0.0f;
            // Serial.print("!!!!!!");
            // Serial.print(P1);
            // Serial.print(",");
            // Serial.print(P2);
            // Serial.print("~~~~~~");
            return SensorStatus::TWO_ILLOGICAL;
        }
    }
    
    // Exactly one sensor is valid
    chosenPressure = ok1 ? P1 : P2;
    return SensorStatus::ONE_ILLOGICAL;
}

bool PressureSensor::isPressureValid(float pressure) {
    return (pressure >= SensorConfig::P_MIN && pressure <= SensorConfig::P_MAX);
}

bool PressureSensor::isJumpAcceptable(float currentPressure, float previousPressure) {
    // return fabs(currentPressure - previousPressure) <= SensorConfig::MAX_PRESSURE_JUMP;
    return true;
}

void PressureSensor::updatePreviousReadings(float P1, float P2) {
    m_prevP1 = P1;
    m_prevP2 = P2;
    m_firstReading = false;
}