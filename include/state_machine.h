#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>

#include "config.h"

// System states according to the flow diagram
enum class SystemStateEnum : uint8_t {
    BOOT_INIT,
    OPEN_LOOP_INIT,
    CLOSED_LOOP,
    FORCED_OPEN_LOOP,
    EMERGENCY_STOP
};

// Fault flags
struct FaultFlags {
    bool manualAbort;
    bool commTimeout;
    bool oscillationDetected;
    bool encoderMismatch;
    bool noMotion;
    bool sensorFault;
    bool redBandFault;
    
    FaultFlags() : manualAbort(false), commTimeout(false), oscillationDetected(false),
                   encoderMismatch(false), noMotion(false), sensorFault(false), redBandFault(false) {}
                   
    void clear() {
        manualAbort = false;
        commTimeout = false;
        oscillationDetected = false;
        encoderMismatch = false;
        noMotion = false;
        sensorFault = false;
        redBandFault = false;
    }
};

// System state (most importantly of the state machine)
struct SystemState {
    SystemStateEnum currentState = SystemStateEnum::BOOT_INIT;

    // System status
    bool systemInitialized = false;
    bool mpvWasOpen = false;

    // Time variables
    unsigned long preClosedLoopTimer = 0;
    unsigned long stateEntryTime = 0;
    unsigned long lastControlTime = 0;
    unsigned long enterClosedLoopTime = 0;
    
    SystemState() : currentState(SystemStateEnum::BOOT_INIT), systemInitialized(false), 
                    mpvWasOpen(false), preClosedLoopTimer(0), stateEntryTime(0), lastControlTime(0) {}
                   
    void changeStateTo(SystemStateEnum stateTo) {
        stateEntryTime = millis();
        currentState = stateTo;
    }
    void initTimers() {
        preClosedLoopTimer = millis();
        stateEntryTime = millis();
        lastControlTime = millis();
    }
};

// Channel controller state (single manifold)
struct ChannelState {
    float prevError;
    float targetAngle;
    float currentAngle;
    
    ChannelState() : prevError(0.0f), targetAngle(ValveConfig::START_ANGLE), currentAngle(ValveConfig::START_ANGLE) {}
};

#endif // STATE_MACHINE_H