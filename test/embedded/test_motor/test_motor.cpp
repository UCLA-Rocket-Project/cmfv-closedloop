#include <Arduino.h>
#include <unity.h>

#include <SPI.h>
#include <AMT22_lib.h>
#include <HighPowerStepperDriver.h>
#include "config.h"
#include <utilities.h>
#include <utilities_motor.h>

HighPowerStepperDriver stepperDriver;
AMT22* encoder;

// Because we extern some symbols which are accessible to utilities.h and utilities_motor.h
#include <comm_handler.h>
#include "state_machine.h"
CommHandler* commHandler;
SystemState systemState;
FaultFlags faults;

void setUp(void) {}
void tearDown(void) {}

void test_encoder_valid_angle() {
    float curAngle = getEncoderAngle();
    TEST_ASSERT_TRUE(isAngleValid(curAngle));
}

void test_motor_move_fwd_ninety() {
    float oldAngle = getEncoderAngle();
    TEST_ASSERT_TRUE(isAngleValid(oldAngle));

    float targetAngle = oldAngle > 270.0f ? oldAngle - 270.0f : oldAngle + 90.0f;
    serviceSingleMotor(stepperDriver, oldAngle, targetAngle);
    serviceSingleMotor(stepperDriver, getEncoderAngle(), targetAngle);
    float errDeg = fabs(getEncoderAngle() - targetAngle);
    TEST_ASSERT_LESS_THAN_FLOAT(MotorControlConfig::ANGLE_DEADBAND_DEG, errDeg);
}

void test_motor_move_bck_ninety() {
    float oldAngle = getEncoderAngle();
    TEST_ASSERT_TRUE(isAngleValid(oldAngle));

    float targetAngle = oldAngle < 90.0f ? oldAngle + 270.0f : oldAngle - 90.0f;
    serviceSingleMotor(stepperDriver, oldAngle, targetAngle);
    serviceSingleMotor(stepperDriver, getEncoderAngle(), targetAngle);
    float errDeg = fabs(getEncoderAngle() - targetAngle);
    TEST_ASSERT_LESS_THAN_FLOAT(MotorControlConfig::ANGLE_DEADBAND_DEG, errDeg);
}

void test_motor_move_fwd_ninety_with_5_degree_cap() {
    
}

void setup() {
    SPI.begin(); 

    // Configure stepper driver
    stepperDriver.setChipSelectPin(HardwareConfig::MOTOR_CS_PIN);
    stepperDriver.resetSettings();
    stepperDriver.clearStatus();
    stepperDriver.setDecayMode(HPSDDecayMode::AutoMixed);
    stepperDriver.setCurrentMilliamps36v4(HardwareConfig::MOTOR_CURRENT_MA);
    stepperDriver.setStepMode(HPSDStepMode::MicroStep1);
    stepperDriver.enableDriver();

    // Initialize encoder
    pinMode(HardwareConfig::ENCODER_CS_PIN, OUTPUT);
    digitalWrite(HardwareConfig::ENCODER_CS_PIN, HIGH);
    encoder = new AMT22(HardwareConfig::ENCODER_CS_PIN, 12);
    delay(20);

    UNITY_BEGIN();
    RUN_TEST(test_encoder_valid_angle);
    RUN_TEST(test_motor_move_fwd_ninety);
    RUN_TEST(test_motor_move_bck_ninety);
    UNITY_END();
}

void loop() {}