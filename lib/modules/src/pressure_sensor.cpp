#include "assert_own.h"
#include "pressure_sensor.h"
#include "config.h"
#include <math.h>

PressureSensor::PressureSensor() 
    : m_consecutiveDifferenceFaults(0), m_consecutiveInvalid{0} {}

SensorStatus PressureSensor::validateThreeSensors(float P1, float P2, float P3, float& chosenPressure) {
    uint8_t ok = isPressureValid(P1) + (isPressureValid(P2) << 1) + (isPressureValid(P3) << 2);    
    
    // All PT readings are invalid
    if (ok == 0) {
        chosenPressure = 0.0f;
        return SensorStatus::THREE_ILLOGICAL;
    }

    // If only one PT reading is valid, then only 1 bit will be turned on 
    if (ok == (ok & -ok)) {
        switch (ok) {
            case 4: 
                chosenPressure = P3; break;
            case 2: 
                chosenPressure = P2; break;
            case 1:
                chosenPressure = P1; break;
            default:
                assert("Pressure Sensor 3 PTs, One valid PT error");
        }
        return SensorStatus::TWO_ILLOGICAL;
    }

    // All PT readings are valid
    if (ok == 7) {
        float P[3] = {P1, P2, P3};

        // Check if any pairwise differences are above the threshold
        for (int i = 1; i < 3; i++) {
            for (int j= 0; j < i; j++) {
                if (fabs(P[i] - P[j]) > SensorConfig::PAIR_DIFFERENCE_THRESHOLD) {
                    m_consecutiveDifferenceFaults++;

                    // Only trigger fault after consecutive threshold is reached
                    if (m_consecutiveDifferenceFaults >= SensorConfig::PAIR_DIFFERENCE_CONSECUTIVE_COUNT) {
                        chosenPressure = 0.0f;
                        return SensorStatus::TWO_ILLOGICAL;
                    } else {
                        chosenPressure = m_lastGoodPressure;
                        return SensorStatus::OK_ALL;
                    }
                }
            }
        }

        // Return the median
        chosenPressure = ((P1-P2)*(P2-P3) > 0 ? P2 : ((P1-P2)*(P1-P3) < 0 ? P1 : P3));
        m_consecutiveDifferenceFaults = 0;  
        m_lastGoodPressure = chosenPressure;
        return SensorStatus::OK_ALL;
    }
    
    // If two sensors are valid, then we use validateTwoSensors on it
    switch (ok) {
        case 3:
            return validateSensors(P1, P2, chosenPressure);
        case 5:
            return validateSensors(P1, P3, chosenPressure);
        case 6:
            return validateSensors(P2, P3, chosenPressure);
        default:
            return SensorStatus::THREE_ILLOGICAL; // Should never get here
    }
}

SensorStatus PressureSensor::validateTwoSensors(float P1, float P2, float& chosenPressure) {
    bool valid[SensorConfig::NUM_PTS] = { isPressureValid(P1), isPressureValid(P2) };
    updateConsecInvalidity(valid);


SensorStatus PressureSensor::validateSensors(float P1, float P2, float& chosenPressure) {
    bool ok1 = isPressureValid(P1);
    bool ok2 = isPressureValid(P2);
    
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
            return SensorStatus::OK_ALL;
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