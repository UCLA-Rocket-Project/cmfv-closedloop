#ifndef TEST_PRESSURE_SENSOR_H
#define TEST_PRESSURE_SENSOR_H

#include <unity.h>

#include "config.h"
#include "pressure_sensor.h"
#include "state_machine.h"

/* Test 1: Both pressure sensors read illogical values. */
void test_pressure_validation_1() {
    float pressureReturned;
    PressureSensor ps;
    SensorStatus ss;

    // Register N - 1 invalid PT readings for both PTs
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateTwoSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }

    // Register a valid reading for both PTs
    ss = ps.validateTwoSensors(SensorConfig::P_MIN, SensorConfig::P_MIN, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::OK_BOTH, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MIN, pressureReturned);

    // We should need to register N invalid PT readings for both PTs to get TWO_ILLOGICAL
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateTwoSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }
    ss = ps.validateTwoSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::TWO_ILLOGICAL, ss);
}

/* Test 2: Both pressure sensors read logical values, but differ by too much. */
void test_pressure_validation_2() {
    float pressureReturned;
    PressureSensor ps;
    SensorStatus ss;
    if (SensorConfig::P_MAX - SensorConfig::P_MIN < SensorConfig::PAIR_DIFFERENCE_THRESHOLD)
        return;

    // Register N - 1 sensor differences which exceed the threshold
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateTwoSensors(SensorConfig::P_MIN, SensorConfig::P_MAX, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }

    // Register a sensor difference which does not exceed the threshold
    ss = ps.validateTwoSensors(SensorConfig::P_MIN, SensorConfig::P_MIN, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::OK_BOTH, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MIN, pressureReturned);

    // We should need to register N sensor differences which exceed the threshold to get TWO_ILLOGICAL
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateTwoSensors(SensorConfig::P_MIN, SensorConfig::P_MAX, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }
    ss = ps.validateTwoSensors(SensorConfig::P_MIN, SensorConfig::P_MAX, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::TWO_ILLOGICAL, ss);
}

/* Test 3: Only one pressure sensor reads a logical value; the other, illogical. */
void test_pressure_validation_3() {
    float pressureReturned;
    PressureSensor ps;
    SensorStatus ss1 = ps.validateTwoSensors(SensorConfig::P_MIN, SensorConfig::P_MAX + 1, pressureReturned);

    TEST_ASSERT_EQUAL(SensorStatus::ONE_ILLOGICAL, ss1);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MIN, pressureReturned);

    SensorStatus ss2 = ps.validateTwoSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX, pressureReturned);
    
    TEST_ASSERT_EQUAL(SensorStatus::ONE_ILLOGICAL, ss2);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MAX, pressureReturned);
}

/* Test 4: Both pressure sensors read a logical value; the average should be returned. */
void test_pressure_validation_4() {
    float pressureReturned, 
        pressureHigh = 
            SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD > SensorConfig::P_MAX ? 
                SensorConfig::P_MAX : 
                SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD, 
        pressureExpected = 
            SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD > SensorConfig::P_MAX ?
                (SensorConfig::P_MIN + SensorConfig::P_MAX) * 0.5f : 
                SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD * 0.5f; 

    PressureSensor ps;
    SensorStatus ss = ps.validateTwoSensors(SensorConfig::P_MIN, pressureHigh, pressureReturned);

    TEST_ASSERT_EQUAL(SensorStatus::OK_BOTH, ss);
    TEST_ASSERT_EQUAL_FLOAT(pressureExpected, pressureReturned);
}

void run_all_pressure_sensor_tests() {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_pressure_validation_1);
    RUN_TEST(test_pressure_validation_2);
    RUN_TEST(test_pressure_validation_3);
    RUN_TEST(test_pressure_validation_4);
}

#endif // TEST_PRESSURE_SENSOR_H
