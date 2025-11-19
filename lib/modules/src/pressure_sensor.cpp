#include "pressure_sensor.h"
#include "config.h"
#include <math.h>

PressureSensor::PressureSensor() 
    : m_prevP1(0.0f), m_prevP2(0.0f), m_firstReading(true), m_consecutiveDifferenceFaults(0), m_lastGoodPressure(0.0f) {}

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
            m_lastGoodPressure = chosenPressure; 
            m_consecutiveDifferenceFaults = 0;  
            return SensorStatus::OK_BOTH;
        } else {
            // Sensors disagree significantly
            m_consecutiveDifferenceFaults++;
            
            // Only trigger fault after consecutive threshold is reached
            if (m_consecutiveDifferenceFaults >= SensorConfig::PAIR_DIFFERENCE_CONSECUTIVE_COUNT) {
                chosenPressure = 0.0f;
                return SensorStatus::TWO_ILLOGICAL;
            } else {
                chosenPressure = m_lastGoodPressure;
                return SensorStatus::OK_BOTH;
            }
        }
    }
    
    // Exactly one sensor is valid
    chosenPressure = ok1 ? P1 : P2;
    m_lastGoodPressure = chosenPressure; 
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

void PressureSensor::resetConsecutiveFaults() {
    m_consecutiveDifferenceFaults = 0;
}