#include <math.h>

#include "config.h"
#include "utilities_motor.h"
#include "utilities.h"

extern CommHandler* commHandler;

// Motor control functions
void serviceSingleMotor(HighPowerStepperDriver& drv,
                               float currentAngleDeg,
                               float targetAngleDeg) {
    float errDeg = targetAngleDeg - currentAngleDeg;
    static int noMotionCount = 0;

    while (fabs(errDeg) > MotorControlConfig::ANGLE_DEADBAND_DEG){
        // Proportional steps
        float desiredSteps = MotorControlConfig::KP_STEPS_PER_DEG * errDeg;
        long steps = lroundf(desiredSteps);
        float angleBeforeMove = getEncoderAngle();

        // Direction and stepping
        if (steps == 0) return;
        drv.setDirection(steps > 0 ? 0 : 1);
        long n = labs(steps);

        // Move motor
        for (long i = 0; i < n; ++i) {
            drv.step();
            delayMicroseconds(HardwareConfig::STEP_PERIOD_US);
        }

        // No motion check
        bool motionDetected = fabs(getEncoderAngle() - angleBeforeMove) > FDIRConfig::MOTION_DETECTED_ANGLE_THRESHOLD;

        if (!motionDetected) {
            noMotionCount++;
            if (noMotionCount >= 3){
                faults.noMotion = true;
                setMPV(false);
                systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
                return;
            }
        } else {
            noMotionCount = 0;
        }

        errDeg = targetAngleDeg - getEncoderAngle();
    }

#ifndef PIO_UNIT_TESTING
    // Discard half-received PT update packet (if any)
    commHandler->flushInputBuffer();
#endif
}

void serviceMotor() {
    float cur = getEncoderAngle();

    float tgt = channel.targetAngle;
    serviceSingleMotor(stepperDriver, cur, tgt);
}