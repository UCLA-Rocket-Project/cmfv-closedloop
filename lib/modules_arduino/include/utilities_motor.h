#ifndef UTILITIES_MOTOR_H
#define UTILITIES_MOTOR_H

#include <HighPowerStepperDriver.h>

extern HighPowerStepperDriver stepperDriver;

// Motor control functions
void serviceMotor();
void serviceSingleMotor(HighPowerStepperDriver& drv, float currentAngleDeg, float targetAngleDeg);

#endif // UTILITIES_MOTOR_H