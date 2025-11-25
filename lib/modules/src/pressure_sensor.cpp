#include "pressure_sensor.h"
#include "config.h"
#include <math.h>

PressureSensor::PressureSensor() 
    : m_consecutiveDifferenceFaults(0), m_consecutiveInvalid{0} {}

SensorStatus PressureSensor::validateTwoSensors(float P1, float P2, float& chosenPressure) {
    bool valid[SensorConfig::NUM_PTS] = { isPressureValid(P1), isPressureValid(P2) };
    updateConsecInvalidity(valid);
    
    if (m_consecutiveInvalid[0] > 0 && m_consecutiveInvalid[1] > 0 && 
            (m_consecutiveInvalid[0] < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD ||
             m_consecutiveInvalid[1] < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD))
        return SensorStatus::PENDING_FAULT;

    bool ok1 = m_consecutiveInvalid[0] == 0, ok2 = m_consecutiveInvalid[1] == 0;
    
    if (!ok1 && !ok2) return SensorStatus::TWO_ILLOGICAL;
    
    if (ok1 && ok2) {
        float difference = fabs(P1 - P2);

        if (difference <= SensorConfig::PAIR_DIFFERENCE_THRESHOLD) {
            chosenPressure = (P1 + P2) * 0.5f;
            m_consecutiveDifferenceFaults = 0;  
            return SensorStatus::OK_BOTH;

        } else {
            m_consecutiveDifferenceFaults++;
            
            // Only trigger fault after consecutive threshold is reached
            if (m_consecutiveDifferenceFaults >= SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD)
                return SensorStatus::TWO_ILLOGICAL;
            else
                return SensorStatus::PENDING_FAULT;
        }
    }
    
    // Exactly one sensor is valid
    chosenPressure = ok1 ? P1 : P2;
    m_consecutiveDifferenceFaults = 0;
    return SensorStatus::ONE_ILLOGICAL;
}

bool PressureSensor::isPressureValid(float pressure) {
    return (pressure >= SensorConfig::P_MIN && pressure <= SensorConfig::P_MAX);
}

void PressureSensor::updateConsecInvalidity(bool* valid) {
    for (int i = 0; i < SensorConfig::NUM_PTS; i++) {
        if (!valid[i]) m_consecutiveInvalid[i]++;
        else m_consecutiveInvalid[i] = 0;
    }
}

void PressureSensor::resetConsecutiveFaults() {
    m_consecutiveDifferenceFaults = 0;
    for (int i = 0; i < SensorConfig::NUM_PTS; i++) m_consecutiveInvalid[i] = 0;
}