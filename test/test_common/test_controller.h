#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <math.h>
#include <unity.h>

#include "assert_own.h"
#include "config.h"
#include <controller.h>

void test_controller() {
    float expectedIntegralError = 0.0f;
    Controller controller = Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);

    // First reading - no differential term involved
    float e1 = -1.0f, dt1 = 0.01f;
    controller.update(e1, dt1);
    expectedIntegralError += e1 * dt1;
    // We are testing with the expectation that the integral error will NOT be clamped
    assert(expectedIntegralError >= ControllerConfig::I_MIN && expectedIntegralError <= ControllerConfig::I_MAX);
    float r1Expected = ControllerConfig::KP * e1 + ControllerConfig::KI * expectedIntegralError; 
    TEST_ASSERT_EQUAL_FLOAT(r1Expected, controller.getError());

    // Second reading - differential term should be involved
    float e2 = 5.0f, dt2 = 0.02f;
    controller.update(e2, dt2);
    expectedIntegralError += e2 * dt2;
    // We are testing with the expectation that the integral error will NOT be clamped
    assert(expectedIntegralError >= ControllerConfig::I_MIN && expectedIntegralError <= ControllerConfig::I_MAX);
    float r2Expected = ControllerConfig::KP * e2 + ControllerConfig::KI * expectedIntegralError + ControllerConfig::KD * (5.0f - (-1.0f) / 0.02f);
    TEST_ASSERT_EQUAL_FLOAT(r2Expected, controller.getError());
}

void test_controller_integral_error_clamping() {
    Controller controller = Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);

    controller.update(abs(ControllerConfig::I_MAX), 1.5f);
    TEST_ASSERT_EQUAL_FLOAT(ControllerConfig::I_MAX, controller.getIntegral());

    controller.update(-abs(ControllerConfig::I_MIN), 3.0f);
    TEST_ASSERT_EQUAL_FLOAT(ControllerConfig::I_MIN, controller.getIntegral());
}

void run_all_controller_tests() {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_controller);
    RUN_TEST(test_controller_integral_error_clamping);
}

#endif // TEST_CONTROLLER_H