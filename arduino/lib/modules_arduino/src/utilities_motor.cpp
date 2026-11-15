#include <math.h>

#include "config.h"
#include "utilities_motor.h"
#include "utilities.h"

extern CommHandler* commHandler;
extern bool MPV_CONTROL;

void _publishTelemetry() {
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
            _publishTelemetry();
        }

        // No motion check
        bool motionDetected = fabs(getEncoderAngle() - angleBeforeMove) > FDIRConfig::MOTION_DETECTED_ANGLE_THRESHOLD;

        if (!motionDetected) {
            noMotionCount++;
            if (noMotionCount >= 3){
                faults.noMotion = true;
                // setMPV(false);
                // systemState.changeStateTo(SystemStateEnum::EMERGENCY_STOP);
                // return;
            }
        } else {
            noMotionCount = 0;
        }
        
        //theoretically, the motor can get stuck without this being called, so let's make sure that it is
        _publishTelemetry();

        errDeg = targetAngleDeg - getEncoderAngle();
    }

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