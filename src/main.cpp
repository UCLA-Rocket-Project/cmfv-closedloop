// Ares EGSE Closed Loop CMFV Code - Single System Version
// Purpose: Complete closed-loop single valve control system with state machine
// Controls either fuel or oxidizer valve based on IS_FUEL_SYSTEM configuration

#include <SPI.h>
#include <AMT22_lib.h>
#include <HighPowerStepperDriver.h>
#include <math.h>

#include "config.h"
#include "state_machine.h"
#include <controller.h>
#include <pressure_sensor.h>
#include <comm_handler.h>
#include <utilities_motor.h>
#include <utilities.h>

// Hardware objects for single valve system
HighPowerStepperDriver stepperDriver;
AMT22* encoder;
Controller* controller;
PressureSensor* pressureSensor;
CommHandler* commHandler;

// System state variables
SystemState systemState;
ChannelState channel;
FaultFlags faults;

extern bool MPV_CONTROL;

bool redBandCheck = false;

// Function declarations
void stateMachineUpdate();
void globalMonitors();
void bootInit();
void openLoopInit();
void closedLoop();
void forcedOpenLoop();
void emergencyStop();
void resetSystemOnMpvCycle();

void setup() {
    Serial.begin(CommConfig::BAUD_RATE);
    
    // Initialize SPI and pins
    SPI.begin();
    pinMode(HardwareConfig::CB_ONOFF_PIN, INPUT);
#ifndef NO_MANUAL_ABORT
    pinMode(HardwareConfig::MANUAL_ABORT_PIN, INPUT_PULLUP);
#endif

    // Configure stepper driver
    stepperDriver.setChipSelectPin(HardwareConfig::MOTOR_CS_PIN);
    stepperDriver.resetSettings();
    stepperDriver.clearStatus();
    stepperDriver.setDecayMode(HPSDDecayMode::AutoMixed);
    stepperDriver.setCurrentMilliamps36v4(HardwareConfig::MOTOR_CURRENT_MA);
    stepperDriver.setStepMode(HPSDStepMode::MicroStep1);
    stepperDriver.enableDriver();
    delay(20);
    
    // Configure encoder (Arduino-defined constants)
    setUpSPI(PIN_SPI_MOSI, PIN_SPI_MISO, 
             PIN_SPI_SCK, HardwareConfig::ENCODER_SPI_MAX_DATA_RATE);
    
    // Initialize encoder
    pinMode(HardwareConfig::ENCODER_CS_PIN, OUTPUT);
    digitalWrite(HardwareConfig::ENCODER_CS_PIN, HIGH);
    encoder = new AMT22(HardwareConfig::ENCODER_CS_PIN, 12);
    delay(20);

    // Initialize global objects
    controller = new Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);
    pressureSensor = new PressureSensor();
    commHandler = new CommHandler();
    
    // Initialize timers
    systemState.initTimers();
    
    // Initialize MPV to closed state
    setMPV(false);
}

void loop() {
    // Process incoming communications
    commHandler->processIncomingNonBlocking();
    
    // Run global monitors
    globalMonitors();
    
    // Run state machine
    stateMachineUpdate();
    
    // Periodic telemetry (rate limited internally)
    publishTelemetry();
}

void globalMonitors() {

#ifndef NO_MANUAL_ABORT
    // Manual abort check
    if (isManualAbortPressed()) {
        faults.manualAbort = true;
        setMPV(false);
        systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
        return;
    }
#endif
    
    // If system was running and MPV gets turned off, transition to open loop init and reset
    if (!getMPVState() && (systemState.currentState == SystemStateEnum::CLOSED_LOOP || systemState.currentState == SystemStateEnum::FORCED_OPEN_LOOP)) {
        setMPV(false);
        resetSystemOnMpvCycle(); 
        return;
    }

    // Communication watchdog
    static unsigned long commLostTime = 0;
    if (!commHandler->isCommHealthy()) {
        faults.commTimeout = true;
        if (commLostTime == 0) commLostTime = millis();
        // If comm not reestablished in COMM_TIMEOUT_S, go to open loop
        if (millis() - commLostTime > (TimingConfig::COMM_TIMEOUT_S / 2 * 1000)) {
            if (systemState.currentState == SystemStateEnum::CLOSED_LOOP) {
                systemState.changeStateTo(SystemStateEnum::FORCED_OPEN_LOOP);
            }
        }
    } else {
        // Reset commLostTime if comm is healthy
        faults.commTimeout = false;
        commLostTime = 0;
    }

    // State Sync Check with other controller
    if (commHandler->isCommHealthy()) {        
        // Get current angle for position-based sync rules
        float currentAngle = getEncoderAngle();
        if (!isAngleValid(currentAngle)) {
            faults.encoderMismatch = true;
            systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
            return;
        }
        
        // Apply state synchronization rules
        SystemStateEnum syncedState = getSyncedState(systemState.currentState, commHandler->getOtherCtrlerState(), currentAngle);
        
        // Only change state if sync rules determine a different state is needed
        if (syncedState != systemState.currentState) {
            systemState.changeStateTo(syncedState);
        }
    }
}

void stateMachineUpdate() {
    switch (systemState.currentState) {
        case SystemStateEnum::BOOT_INIT:
            bootInit();
            break;
            
        case SystemStateEnum::OPEN_LOOP_INIT:
            openLoopInit();
            break;
            
        case SystemStateEnum::CLOSED_LOOP:
            closedLoop();
            break;
            
        case SystemStateEnum::FORCED_OPEN_LOOP:
            forcedOpenLoop();
            break;
            
        case SystemStateEnum::EMERGENCY_STOP:
            emergencyStop();
            break;
            
        default:
            // Invalid state, go to emergency stop
            systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
            break;
    }
}

void bootInit() {
    if (!systemState.systemInitialized) {
        faults.clear();
        channel.targetAngle = ValveConfig::START_ANGLE;
        float encAngle = getEncoderAngle();
        if (!isAngleValid(encAngle)) {
            faults.encoderMismatch = true;
            systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
            return;
        }
        channel.currentAngle = encAngle;
        controller->reset();
        
        systemState.systemInitialized = true;
        systemState.preClosedLoopTimer = millis();
    }

    systemState.changeStateTo(SystemStateEnum::OPEN_LOOP_INIT);
}

// Drive valve to starting position for closed-loop operation
void openLoopInit() {
    // Reset redBandCheck
    redBandCheck = false;

    float currentAngle = getEncoderAngle();
    if (!isAngleValid(currentAngle)) {
        faults.encoderMismatch = true;
        systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
        return;
    }
    // Update current angle for telemetry
    channel.currentAngle = currentAngle;
    
    bool atTarget = isAtStartAngle(currentAngle);
    // Move valve if not at target
    if (!atTarget) {
        float deltaAngle = ValveConfig::START_ANGLE - currentAngle;
        applyMoveFilter(deltaAngle);

        channel.targetAngle = currentAngle + deltaAngle;

        serviceMotor();
        
        // Check for encoder issues
        float newAngle = getEncoderAngle();
        if (!isAngleValid(newAngle)) {
            faults.encoderMismatch = true;
            systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
            return;
        }
        // Update current angle after movement
        channel.currentAngle = newAngle;
    }
    
    // If valve at starting position, check if MPV is open, then after the pre closed loop timer elapses go into closed loop
    if (atTarget) {
        if (getMPVState()) {
            if (!systemState.mpvWasOpen) {
                systemState.mpvWasOpen = true;
                setMPV(true);
                systemState.preClosedLoopTimer = millis();
            }
            if (millis() - systemState.preClosedLoopTimer >= (TimingConfig::SAFE_TIMER_S * 1000UL)) {
                systemState.changeStateTo(SystemStateEnum::CLOSED_LOOP);
                systemState.enterClosedLoopTime = millis();
                systemState.lastControlTime = millis(); // Reset timing for dt calculation
            }
        } else {
            systemState.mpvWasOpen = false;
            setMPV(false);
        }
    }
}

void closedLoop() {
    // Calculate actual time step for this control loop iteration
    unsigned long now = millis();
    float dt = (now - systemState.lastControlTime) / 1000.0f; 
    systemState.lastControlTime = now;

    // Minimum dt to avoid division by zero or unrealistic values
    if (dt <= 0.0f || dt > 10.0f) {
        dt = TimingConfig::CONTROL_PERIOD_S; // Fallback
    }
    
    // Read pressure sensors for this manifold
    float pressure;
    if (!readManifoldPressures(pressure)) {
        // Sensor fault detected (two illogical PTs)
        faults.sensorFault = true;
        systemState.changeStateTo(SystemStateEnum::FORCED_OPEN_LOOP);
        return;
    }

    
    if (redBandCheck || (millis() - systemState.enterClosedLoopTime) > TimingConfig::REDBAND_TIMEOUT * 1000UL){
        redBandCheck = true;
        if (!checkRedBand(pressure)){
            faults.redBandFault = true;
            systemState.changeStateTo(SystemStateEnum::FORCED_OPEN_LOOP);
            return;
        }
    }

    
    // Calculate error for this manifold
    float error = ControllerConfig::TARGET_PRESSURE_PSI - pressure;
    bool inTolerance = fabs(error) <= SensorConfig::PRESSURE_TOLERANCE;

    // VALVE CONTROL
    if (!inTolerance) {
        // Update controller
        controller->update(error, dt);
        float deltaAngle = controller->getError();
        
        // Apply move filtering (5-degree cap)
        applyMoveFilter(deltaAngle);
        
        // Calculate new target angle
        float newTargetAngle = channel.targetAngle + deltaAngle;
        float oldTargetAngle = channel.targetAngle;
        newTargetAngle = constrainAngle(newTargetAngle);
        
        // Command stepper motor
        float angleBeforeMove = getEncoderAngle();
        if (!isAngleValid(angleBeforeMove)) {
            faults.encoderMismatch = true;
            setMPV(false);
            systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
            return;
        }
        channel.targetAngle = newTargetAngle;
        serviceMotor();
        // Verify encoder response
        float angleAfterMove = getEncoderAngle();
        if (!isAngleValid(angleAfterMove)) {
            faults.encoderMismatch = true;
            setMPV(false);
            systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
            return;
        }
        channel.currentAngle = angleAfterMove;

#ifdef USE_OSCILLATION_DETECTOR
        // The check for whether the move is within tolerance is removed for now, placeholder in case we want to add back.
        // Check for oscillations
        if (controller->getOscillationDetector().checkOscillation(error, millis())) {
            faults.oscillationDetected = true;
            systemState.changeStateTo(SystemStateEnum::FORCED_OPEN_LOOP);
            return;
        }
#endif
    }

}

// Safe fallback mode - move valve to open loop target angle
void forcedOpenLoop() {
    float currentAngle = getEncoderAngle();
    if (!isAngleValid(currentAngle)) {
        faults.encoderMismatch = true;
        systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
        return;
    }
    // Update current angle for telemetry
    channel.currentAngle = currentAngle;
    
    // Move valve to open loop target angle if not already there
    if (fabs(currentAngle - ValveConfig::OPENLOOP_TARGET_ANGLE) > ValveConfig::ANGLE_TOLERANCE) {
        float deltaAngle = ValveConfig::OPENLOOP_TARGET_ANGLE - currentAngle;
        applyMoveFilter(deltaAngle);
        channel.targetAngle = currentAngle + deltaAngle;
        serviceMotor();

        float angleAfterMove = getEncoderAngle();
        if (!isAngleValid(angleAfterMove)) {
            faults.encoderMismatch = true;
            setMPV(false);
            systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
            return;
        }
        // Update current angle after movement
        channel.currentAngle = angleAfterMove;
    }

    // // Check for recovery conditions
    // if (commHandler->isCommHealthy() && !faults.manualAbort) {
    //     float pressure;
    //     if (readManifoldPressures(pressure)) {
    //         // Clear transient faults and attempt recovery
    //         if (millis() - stateEntryTime > (TimingConfig::RECOVERY_DWELL_S * 1000)) {
    //             faults.clear();
    //             controller->reset();
    //             currentState = SystemStateEnum::CLOSED_LOOP;
    //             stateEntryTime = millis();
    //             lastControlTime = millis();
    //         }
    //     }
    // }
}

void emergencyStop() {
    // Terminal safe state
    setMPV(false);
    stepperDriver.disableDriver();
    
    // System remains in emergency stop until power cycle or manual reset
}