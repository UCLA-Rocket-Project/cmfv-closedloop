// CMFV repeated small-error motor reproduction (isolated, unpressurized bench only).
// Replace ONLY arduino/src/main.cpp with this file.
// Does not run the normal state machine, pressure/HITL communications, or binary telemetry.
// Requires explicit ARM over serial. Opening a serial monitor may reset an Arduino Uno.
// Once armed, each trial targets the most recent encoder angle minus 0.68 deg.
// Never commands a return stroke. A non-converging trial remains in its loop.
// Stop using an independent motor-power cutoff: serial Ctrl+C cannot stop it.

#include <Arduino.h>
#include <SPI.h>
#include <AMT22_lib.h>
#include <HighPowerStepperDriver.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "state_machine.h"
#include <controller.h>
#include <pressure_sensor.h>
#include <comm_handler.h>
#include <utilities_motor.h>
#include <utilities.h>

// Global objects required by the existing library modules.
HighPowerStepperDriver stepperDriver;
AMT22* encoder = nullptr;
Controller* controller = nullptr;
PressureSensor* pressureSensor = nullptr;
CommHandler* commHandler = nullptr;
SystemState systemState;
ChannelState channel;
FaultFlags faults;
bool redBandCheck = false;

static constexpr float TEST_DELTA_DEG = 0.68f;
// Conservative software travel boundary for the isolated valve test.
// Not a substitute for mechanical stops or an independent power cutoff.
static constexpr float MIN_TEST_ANGLE_DEG = 32.0f;
static constexpr float MAX_TEST_ANGLE_DEG = 88.0f;
static char command[16] = {0};
static uint8_t commandLength = 0;
static bool running = false;

static long cd(float degrees) { return lroundf(degrees * 100.0f); }

static void runCorrection(unsigned long trial, float target, float initial) {
    float error = target - initial;
    int noMotionCount = 0;
    unsigned long iter = 0;
    unsigned long startMs = millis();
    unsigned long totalSteps = 0;

    Serial.print(F("[TRIAL_START] trial=")); Serial.print(trial);
    Serial.print(F(" t_ms=")); Serial.print(startMs);
    Serial.print(F(" initial_cdeg=")); Serial.print(cd(initial));
    Serial.print(F(" target_cdeg=")); Serial.print(cd(target));
    Serial.print(F(" error_cdeg=")); Serial.println(cd(error));

    // This is intentionally the original unbounded target-error condition.
    while (fabs(error) > MotorControlConfig::ANGLE_DEADBAND_DEG) {
        ++iter;
        const float desiredSteps = MotorControlConfig::KP_STEPS_PER_DEG * error;
        const long steps = lroundf(desiredSteps);
        const float before = getEncoderAngle();  // original encoder read #1

        Serial.print(F("[MOVE] trial=")); Serial.print(trial);
        Serial.print(F(" iter=")); Serial.print(iter);
        Serial.print(F(" t_ms=")); Serial.print(millis());
        Serial.print(F(" before_cdeg=")); Serial.print(cd(before));
        Serial.print(F(" target_cdeg=")); Serial.print(cd(target));
        Serial.print(F(" error_cdeg=")); Serial.print(cd(error));
        Serial.print(F(" steps=")); Serial.println(steps);

        // Cannot safely use invalid encoder values to command further movement.
        if (!isAngleValid(before)) {
            Serial.println(F("[STOP] INVALID ENCODER BEFORE MOTOR STEPS"));
            stepperDriver.disableDriver();
            while (true) delay(1000);
        }
        // Do not request a correction that would knowingly cross the bench
        // travel boundary. Note: a misbehaving motor can still overshoot.
        // if (before < MIN_TEST_ANGLE_DEG ||
        //     target < MIN_TEST_ANGLE_DEG ||
        //     (steps < 0 && before + steps * HardwareConfig::DEGREES_PER_STEP < MIN_TEST_ANGLE_DEG)) {
        //     Serial.println(F("[STOP] LOWER TRAVEL BOUNDARY (no more steps)"));
        //     stepperDriver.disableDriver();
        //     while (true) delay(1000);
        // }
        if (steps == 0) {
            Serial.println(F("[TRIAL_EXIT] steps rounded to zero (original early return)"));
            return;
        }

        // Identical direction mapping, step() calls and delay to original motor loop.
        stepperDriver.setDirection(steps > 0 ? 0 : 1);
        const long n = labs(steps);
        const unsigned long stepStart = micros();
        for (long i = 0; i < n; ++i) {
            stepperDriver.step();
            delayMicroseconds(HardwareConfig::STEP_PERIOD_US);
        }
        totalSteps += (unsigned long)n;
        const unsigned long stepDuration = micros() - stepStart;

        const float motionRead = getEncoderAngle(); // original encoder read #2
        if (!isAngleValid(motionRead)) {
            Serial.println(F("[STOP] INVALID ENCODER AFTER MOTOR STEPS"));
            stepperDriver.disableDriver();
            while (true) delay(1000);
        }
        const bool detected = fabs(motionRead - before) >
            FDIRConfig::MOTION_DETECTED_ANGLE_THRESHOLD;
        if (!detected) {
            ++noMotionCount;
            if (noMotionCount >= 3) faults.noMotion = true; // same flag behavior
        } else {
            noMotionCount = 0;
        }

        const float errorRead = getEncoderAngle(); // original encoder read #3
        if (!isAngleValid(errorRead)) {
            Serial.println(F("[STOP] INVALID ENCODER AT ERROR UPDATE"));
            stepperDriver.disableDriver();
            while (true) delay(1000);
        }
        const float priorError = error;
        error = target - errorRead;

        Serial.print(F("[RESULT] trial=")); Serial.print(trial);
        Serial.print(F(" iter=")); Serial.print(iter);
        Serial.print(F(" motion_read_cdeg=")); Serial.print(cd(motionRead));
        Serial.print(F(" error_read_cdeg=")); Serial.print(cd(errorRead));
        Serial.print(F(" delta_cdeg=")); Serial.print(cd(motionRead - before));
        Serial.print(F(" expected_delta_cdeg=")); Serial.print(cd(steps * HardwareConfig::DEGREES_PER_STEP));
        Serial.print(F(" error_after_cdeg=")); Serial.print(cd(error));
        Serial.print(F(" detected=")); Serial.print(detected ? 1 : 0);
        Serial.print(F(" no_motion_count=")); Serial.print(noMotionCount);
        Serial.print(F(" step_us=")); Serial.print(stepDuration);
        Serial.print(F(" elapsed_ms=")); Serial.println(millis() - startMs);
        if (priorError * error < 0.0f) Serial.println(F("[OSC] ERROR SIGN CHANGED"));
        if (fabs(error) > fabs(priorError)) Serial.println(F("[PROGRESS] ERROR INCREASED"));
        else if (fabs(error) < fabs(priorError)) Serial.println(F("[PROGRESS] ERROR DECREASED"));
        else Serial.println(F("[PROGRESS] ERROR UNCHANGED"));
    }
    Serial.print(F("[TRIAL_DONE] trial=")); Serial.print(trial);
    Serial.print(F(" iters=")); Serial.print(iter);
    Serial.print(F(" steps=")); Serial.print(totalSteps);
    Serial.print(F(" elapsed_ms=")); Serial.print(millis() - startMs);
    Serial.print(F(" final_error_cdeg=")); Serial.println(cd(error));
}

static void runRepeatedTests() {
    running = true;
    const float firstAngle = getEncoderAngle();
    if (!isAngleValid(firstAngle)) {
        Serial.println(F("[REFUSED] Invalid angle or too close to lower travel boundary"));
        running = false;
        return;
    }

    Serial.print(F("[ARMED] first_angle_cdeg=")); Serial.println(cd(firstAngle));
    Serial.println(F("[ARMED] Each target = current encoder angle - 0.68 degrees"));
    Serial.println(F("[ARMED] No return strokes; no iteration/step/time caps."));
    Serial.print(F("[ARMED] lower_boundary_cdeg=")); Serial.println(cd(MIN_TEST_ANGLE_DEG));
    Serial.println(F("[ARMED] Independent motor-power cutoff required."));
    stepperDriver.enableDriver();
    delay(20);

    unsigned long trial = 0;
    while (true) {
        // NEW: compute *both* initial angle and target on every trial.
        // No fixed center, no alternate return target. A trial that fails
        // to converge remains in runCorrection and prevents new trials.
        const float current = getEncoderAngle();
        if (!isAngleValid(current)) {
            Serial.println(F("[STOP] INVALID ENCODER BETWEEN TRIALS"));
            break;
        }
        const float target = current + TEST_DELTA_DEG;
        // if (target > MAX_TEST_ANGLE_DEG) {
        //     Serial.print(F("[STOP] Reached lower test boundary. current_cdeg="));
        //     Serial.println(cd(current));
        //     break;
        // }
        ++trial;
        runCorrection(trial, target, current);
    }
    stepperDriver.disableDriver();
    running = false;
}

void setup() {
    Serial.begin(CommConfig::BAUD_RATE);
    SPI.begin();
    stepperDriver.setChipSelectPin(HardwareConfig::MOTOR_CS_PIN);
    stepperDriver.resetSettings();
    stepperDriver.clearStatus();
    stepperDriver.setDecayMode(HPSDDecayMode::AutoMixed);
    stepperDriver.setCurrentMilliamps36v4(HardwareConfig::MOTOR_CURRENT_MA);
    stepperDriver.setStepMode(HPSDStepMode::MicroStep1);
    delay(20);

    setUpSPI(PIN_SPI_MOSI, PIN_SPI_MISO,
             PIN_SPI_SCK, HardwareConfig::ENCODER_SPI_MAX_DATA_RATE);
    pinMode(HardwareConfig::ENCODER_CS_PIN, OUTPUT);
    digitalWrite(HardwareConfig::ENCODER_CS_PIN, HIGH);
    encoder = new AMT22(HardwareConfig::ENCODER_CS_PIN, 12);
    delay(20);
    systemState.initTimers();
    faults.clear();
    setMPV(false);

    Serial.println(F("[BOOT] ONE-DIRECTION RELATIVE SMALL-ERROR REPRO; NO AUTO MOTION"));
    Serial.println(F("[SETUP] Isolated/unpressurized bench ONLY. Type ARM and Enter."));
    Serial.print(F("[CONFIG] test_delta_cdeg=68 deadband_cdeg="));
    Serial.print(cd(MotorControlConfig::ANGLE_DEADBAND_DEG));
    Serial.print(F(" step_period_us="));
    Serial.println(HardwareConfig::STEP_PERIOD_US);
    Serial.print(F("[CONFIG] min_test_angle_cdeg="));
    Serial.println(cd(MIN_TEST_ANGLE_DEG));
}

void loop() {
    while (Serial.available() > 0) {
        const char ch = (char)Serial.read();
        if (ch == '\r' || ch == '\n') {
            if (commandLength) {
                command[commandLength] = '\0';
                if (strcmp(command, "ARM") == 0 && !running) {
                    commandLength = 0;
                    runRepeatedTests(); // deliberately doesn't return on normal operation
                    return;
                }
                Serial.println(F("[INFO] Type ARM and Enter to begin"));
                commandLength = 0;
            }
        } else if (commandLength < sizeof(command) - 1) {
            command[commandLength++] = ch;
        }
    }
}
