#include <math.h>

#include "config.h"
#include "utilities_motor.h"
#include "utilities.h"
#include "motor_debug.h"

extern CommHandler* commHandler;

// Motor control functions
void serviceSingleMotor(HighPowerStepperDriver& drv,
                               float currentAngleDeg,
                               float targetAngleDeg) {
    float errDeg = targetAngleDeg - currentAngleDeg;
    static int noMotionCount = 0;
#ifdef MOTOR_DEBUG_TEXT
    unsigned long debugIteration = 0;
    int previousErrorSign = 0;
    MOTOR_LOG("[MOTOR] ENTER t=%lu initial_cdeg=%ld target_cdeg=%ld deadband_cdeg=%ld\n",
              millis(), debugCentiDeg(currentAngleDeg), debugCentiDeg(targetAngleDeg),
              debugCentiDeg(MotorControlConfig::ANGLE_DEADBAND_DEG));
#endif

    while (fabs(errDeg) > MotorControlConfig::ANGLE_DEADBAND_DEG){
#ifdef MOTOR_DEBUG_TEXT
        ++debugIteration;
#endif
        // Proportional steps
        float desiredSteps = MotorControlConfig::KP_STEPS_PER_DEG * errDeg;
        long steps = lroundf(desiredSteps);
        float angleBeforeMove = getEncoderAngle();
#ifdef MOTOR_DEBUG_TEXT
        MOTOR_LOG("[MOVE] iter=%lu t=%lu before_cdeg=%ld target_cdeg=%ld error_cdeg=%ld steps=%ld\n",
                  debugIteration, millis(), debugCentiDeg(angleBeforeMove),
                  debugCentiDeg(targetAngleDeg), debugCentiDeg(errDeg), steps);
        int currentErrorSign = errDeg > 0.0f ? 1 : -1;
        if (previousErrorSign != 0 && currentErrorSign != previousErrorSign) {
            MOTOR_LOG("[OSC] error sign reversal iter=%lu\n", debugIteration);
        }
        previousErrorSign = currentErrorSign;
#endif

        // Direction and stepping
        if (steps == 0) {
            MOTOR_LOG("[MOTOR] zero steps; original early return\n");
            return;
        }
        drv.setDirection(steps > 0 ? 0 : 1);
        long n = labs(steps);

        // Move motor
        MOTOR_LOG("[STEP] BEGIN iter=%lu n=%ld t=%lu\n", debugIteration, n, millis());
        for (long i = 0; i < n; ++i) {
            drv.step();
            delayMicroseconds(HardwareConfig::STEP_PERIOD_US);
        }
        MOTOR_LOG("[STEP] END iter=%lu t=%lu\n", debugIteration, millis());

        // No motion check
        bool motionDetected = fabs(getEncoderAngle() - angleBeforeMove) > FDIRConfig::MOTION_DETECTED_ANGLE_THRESHOLD;
        MOTOR_LOG("[MOTION] iter=%lu detected=%d noMotionCount_before=%d\n",
                  debugIteration, (int)motionDetected, noMotionCount);

        if (!motionDetected) {
            noMotionCount++;
            if (noMotionCount >= 3){
                faults.noMotion = true;
                MOTOR_LOG("[FAULT] no-motion flag set; original code continues\n");
                // setMPV(false);
                // systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
                // return;
            }
        } else {
            noMotionCount = 0;
        }

        errDeg = targetAngleDeg - getEncoderAngle();
        MOTOR_LOG("[RESULT] iter=%lu error_after_cdeg=%ld inferred_after_cdeg=%ld t=%lu\n",
                  debugIteration, debugCentiDeg(errDeg),
                  debugCentiDeg(targetAngleDeg - errDeg), millis());
    }
    MOTOR_LOG("[MOTOR] TARGET WITHIN DEADBAND t=%lu remaining_error_cdeg=%ld\n",
              millis(), debugCentiDeg(errDeg));

#ifndef PIO_UNIT_TESTING
    // Discard half-received PT update packet (if any)
    // commHandler->flushInputBuffer();
#endif
}

void serviceMotor() {
    float cur = getEncoderAngle();

    float tgt = channel.targetAngle;
    serviceSingleMotor(stepperDriver, cur, tgt);
}