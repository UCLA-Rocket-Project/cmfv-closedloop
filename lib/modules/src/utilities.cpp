#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "utilities.h"

// Encoder calibration constant (system-specific)
const float FULLY_CLOSED_OFFSET = fmod((HardwareConfig::FULLY_OPEN_OFFSET - 90.0f + 360.0f), 360.0f); 

/* IMPORTANT: 
 * - MPV_CONTROL informs the bunker whether we should close MPV manually
 * - MPV_STATE is the state of MPV told to the motor controller by the Pi
 */
bool MPV_CONTROL = false;
bool MPV_STATE = false;

// Encoder utility functions
float getEncoderAngle() {
    uint16_t encoderVal;
    uint8_t attempts = 0;

    do {
        encoderVal = encoder->getPositionSPI();
        delayMicroseconds(40); // Ensure time between reads
    } while (encoderVal == 0xFFFF && ++attempts < 3);

    if (encoderVal == 0xFFFF) {
        Serial.println("Encoder error: Invalid data after 3 attempts.");
        systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
        return -1; // Fails isAngleValid check
        // while (1); // Halt execution here for debugging purposes
    }

    // Convert raw encoder value to degrees
    float rawAngle = ((float)encoderVal / HardwareConfig::MAX_12_BIT_VAL) * 360.0f;
    float adjustedAngle = rawAngle - FULLY_CLOSED_OFFSET;

    if (adjustedAngle < 0.0f) {
        adjustedAngle += 360.0f;
    }

    return adjustedAngle;
}

bool isAngleValid(float angle) {
    // Angle must be in [0, 360) and not error value
    return (angle >= 0.0f && angle < 360.0f);
}

// System utility functions
bool readManifoldPressures(float& pressure) {
    PressureData pressureData = commHandler->getPressureData();
    
    if (!pressureData.valid) {
        return false;
    }
    
    // Validate manifold pressures using the two sensors for this system
    SensorStatus status = pressureSensor->validateTwoSensors(
        pressureData.sensor1, pressureData.sensor2, pressure);
    
    // Update previous readings for jump detection
    pressureSensor->updatePreviousReadings(pressureData.sensor1, pressureData.sensor2);
    
    // Check if manifold has two illogical sensors
    if (status == SensorStatus::TWO_ILLOGICAL) {
        faults.sensorFault = true;
        return false;
    }
    
    // Check if sensors are pending fault
    if (status == SensorStatus::PENDING_FAULT) {
        return false; 
    }
    
    return true;
}

bool isManualAbortPressed() {
    return digitalRead(HardwareConfig::MANUAL_ABORT_PIN) == LOW;
}

void applyMoveFilter(float& deltaAngle) {
    // Apply move filter scale and angle cap
    deltaAngle *= ValveConfig::MOVE_FILTER_SCALE;
    
    if (deltaAngle > ValveConfig::MAX_ANGLE_CHANGE_PER_CYCLE) {
        deltaAngle = ValveConfig::MAX_ANGLE_CHANGE_PER_CYCLE;
    } else if (deltaAngle < -ValveConfig::MAX_ANGLE_CHANGE_PER_CYCLE) {
        deltaAngle = -ValveConfig::MAX_ANGLE_CHANGE_PER_CYCLE;
    }
}

float constrainAngle(float angle) {
    return constrain(angle, ValveConfig::MIN_VALVE_ANGLE, ValveConfig::MAX_VALVE_ANGLE);
}

/* 
 * NOTE: The motor controller does not have direct control over the MPV switch. 
 * This function is simply to allow for the sending of a telemetry signal back
 * to the Pi to signal if MPV should be closed on the Controls Box side. 
 */
void setMPV(bool open) {
    MPV_CONTROL = open;
}

void setMPVState(bool state) {
    MPV_STATE = state;
}

bool getMPVState() {
    return MPV_STATE;
}

void publishTelemetry() {
    static unsigned long lastTelemetryTime = 0;
    
    if (millis() - lastTelemetryTime > (1000.0f / CommConfig::TELEMETRY_RATE_HZ)) {
        // if closeMPV is true, close MPV
        bool closeMPV = !MPV_CONTROL;

        // Send telemetry for single system: "T,state,angle,pid_error,system_type,close_mpv"
        // Note: We modify the telemetry format to indicate which system this is
        commHandler->sendTelemetry(systemState.currentState,
                                  channel.currentAngle,
                                  controller->getError(),
                                  controller->getIntegral(),
                                  HardwareConfig::SYSTEM_NAME,
                                  closeMPV);

        lastTelemetryTime = millis();
    }
}

void resetSystemOnMpvCycle() {
    // Reset controller (integral error, previous error, first call flag)
    controller->reset();
    
    // Reset oscillation detector
#ifdef USE_OSCILLATION_DETECTOR
    controller->getOscillationDetector().reset();
#endif
    
    // Reset pressure sensor consecutive faults
    pressureSensor->resetConsecutiveFaults();
    
    // Reset timing variables
    systemState.initTimers();
    
    // Reset MPV state tracking
    systemState.mpvWasOpen = false;
    
    // Keep persistent hardware faults while resetting the rest
    bool preserveEncoderFault = faults.encoderMismatch;
    bool preserveMotionFault = faults.noMotion;
    bool preserveManualAbort = faults.manualAbort;
    
    faults.clear();
    
    faults.encoderMismatch = preserveEncoderFault;
    faults.noMotion = preserveMotionFault;
    faults.manualAbort = preserveManualAbort;
    
    // Transition to OPEN_LOOP_INIT state to restart
    systemState.changeStateTo(SystemStateEnum::OPEN_LOOP_INIT);
}

bool isValidState(SystemStateEnum s) {
    if (s == SystemStateEnum::BOOT_INIT ||
        s == SystemStateEnum::OPEN_LOOP_INIT ||
        s == SystemStateEnum::CLOSED_LOOP ||
        s == SystemStateEnum::FORCED_OPEN_LOOP ||
        s == SystemStateEnum::EMERGENCY_STOP) {
        return true;
    }
    return false;
}

bool isAtStartAngle(float currentAngle) {
    return fabs(currentAngle - ValveConfig::START_ANGLE) <= ValveConfig::ANGLE_TOLERANCE;
}

bool checkRedBand(float pressure) {    
    if (pressure > ControllerConfig::TARGET_PRESSURE_PSI + MotorControlConfig::REDBAND_PRESSURE_UPPER) {
        return false;
    }
    
    if (pressure < ControllerConfig::TARGET_PRESSURE_PSI - MotorControlConfig::REDBAND_PRESSURE_LOWER) {
        return false;
    }
    
    return true;
}

SystemStateEnum getSyncedState(SystemStateEnum currentState, SystemStateEnum otherControllerState, float currentAngle) {
    // Priority order: STOP > OPENF > CLOSED > OPENI
    
    // EMERGENCY_STOP is a terminal state - cannot transition out automatically
    if (currentState == SystemStateEnum::EMERGENCY_STOP) {
        return currentState;
    }
    
    // Rule 1: If other controller is in STOP, stop this valve
    if (otherControllerState == SystemStateEnum::EMERGENCY_STOP) {
        return SystemStateEnum::EMERGENCY_STOP;
    }
    
    // Rule 2: If other controller is in OPENF and current is CLOSED or OPENI, move to OPENF
    // But for OPENI, only if we're at start angle
    if (otherControllerState == SystemStateEnum::FORCED_OPEN_LOOP) {
        if (currentState == SystemStateEnum::CLOSED_LOOP) {
            return SystemStateEnum::FORCED_OPEN_LOOP;
        }
        if (currentState == SystemStateEnum::OPEN_LOOP_INIT && isAtStartAngle(currentAngle) && (millis() - systemState.preClosedLoopTimer >= (TimingConfig::SAFE_TIMER_S * 1000UL))) {
            return SystemStateEnum::FORCED_OPEN_LOOP;
        }
    }
    
    // Rule 3: If other controller is in CLOSED and current is in OPENI, move to CLOSED
    // But only if OPENI is at start angle
    if (otherControllerState == SystemStateEnum::CLOSED_LOOP) {
        if (currentState == SystemStateEnum::OPEN_LOOP_INIT && isAtStartAngle(currentAngle) && (millis() - systemState.preClosedLoopTimer >= (TimingConfig::SAFE_TIMER_S * 1000UL))) {
            return SystemStateEnum::CLOSED_LOOP;
        }
    }
    
    // Note: Don't sync with BOOT or OPENI states from other controller
    
    // If no sync rules apply, keep current state
    return currentState;
}