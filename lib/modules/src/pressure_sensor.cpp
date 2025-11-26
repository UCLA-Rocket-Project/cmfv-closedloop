#include "assert_own.h"
#include "pressure_sensor.h"
#include "config.h"
#include <math.h>

PressureSensor::PressureSensor() 
    : m_consecutiveDifferenceFaults(0), m_consecutiveInvalid{0} {}

#if USE_3_PTS

SensorStatus PressureSensor::validateThreeSensors(float P1, float P2, float P3, float& chosenPressure) {
    bool valid[SensorConfig::NUM_PTS] = { isPressureValid(P1), isPressureValid(P2), isPressureValid(P3) };
    updateConsecInvalidity(valid);

    if (m_consecutiveInvalid[0] > 0 && m_consecutiveInvalid[1] > 0 && m_consecutiveInvalid[2] > 0 &&
            (m_consecutiveInvalid[0] < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD ||
             m_consecutiveInvalid[1] < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD || 
             m_consecutiveInvalid[2] < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD))
        return SensorStatus::PENDING_FAULT;

    uint8_t ok = valid[0] + (valid[1] << 1) + (valid[2] << 2);    
    
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

                    // int k = 3 - i - j;
                    // bool close_to_i = fabs(P[k] - P[i]) <= SensorConfig::PAIR_DIFFERENCE_THRESHOLD;
                    // bool close_to_j = fabs(P[k] - P[j]) <= SensorConfig::PAIR_DIFFERENCE_THRESHOLD;

                    // if (close_to_i || close_to_j) {
                    //     chosenPressure = close_to_i ? (P[k] + P[i])/2 : (P[k] + P[j])/2;
                    //     return SensorStatus::TWO_ILLOGICAL;
                    // }

                    // We return THREE_ILLOGICAL even though it is ambiguous, so that we go into an error
                    // state. 
                    return m_consecutiveDifferenceFaults >= SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD ? 
                        SensorStatus::THREE_ILLOGICAL : SensorStatus::PENDING_FAULT;
                }
            }
        }

        // Return the median
        chosenPressure = ((P1-P2)*(P2-P3) > 0 ? P2 : ((P1-P2)*(P1-P3) < 0 ? P1 : P3));
        m_consecutiveDifferenceFaults = 0;
        return SensorStatus::OK_ALL;
    }
    
    // If two sensors are valid, then we use validateTwoSensors on it
    switch (ok) {
        case 3:
            return processTwoValidSensors(P1, P2, chosenPressure);
        case 5:
            return processTwoValidSensors(P1, P3, chosenPressure);
        case 6:
            return processTwoValidSensors(P2, P3, chosenPressure);
        default:
            return SensorStatus::THREE_ILLOGICAL; // Should never get here
    }
}

#else

SensorStatus PressureSensor::validateTwoSensors(float P1, float P2, float& chosenPressure) {
    bool valid[SensorConfig::NUM_PTS] = { isPressureValid(P1), isPressureValid(P2) };
    updateConsecInvalidity(valid);
    
    if (m_consecutiveInvalid[0] > 0 && m_consecutiveInvalid[1] > 0 && 
            (m_consecutiveInvalid[0] < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD ||
             m_consecutiveInvalid[1] < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD))
        return SensorStatus::PENDING_FAULT;

    bool ok1 = m_consecutiveInvalid[0] == 0, ok2 = m_consecutiveInvalid[1] == 0;
    
    if (!ok1 && !ok2) return SensorStatus::TWO_ILLOGICAL;
    
    // In the 3-PT case, only this block is called
    if (ok1 && ok2) return processTwoValidSensors(P1, P2, chosenPressure);
    
    // Exactly one sensor is valid
    chosenPressure = ok1 ? P1 : P2;
    m_consecutiveDifferenceFaults = 0;
    return SensorStatus::ONE_ILLOGICAL;
}

#endif

SensorStatus PressureSensor::processTwoValidSensors(float P1, float P2, float& chosenPressure) {
    float difference = fabs(P1 - P2);

    if (difference <= SensorConfig::PAIR_DIFFERENCE_THRESHOLD) {
        chosenPressure = (P1 + P2) * 0.5f;
        m_consecutiveDifferenceFaults = 0;  
        return SensorStatus::OK_ALL;
    } else {
        m_consecutiveDifferenceFaults++;
        
        // Only trigger fault after consecutive threshold is reached
        if (m_consecutiveDifferenceFaults >= SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD)
#if USE_3_PTS
            return SensorStatus::THREE_ILLOGICAL;
#else
            return SensorStatus::TWO_ILLOGICAL;
#endif
        else
            return SensorStatus::PENDING_FAULT;
    }
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