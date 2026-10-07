#ifndef TEST_PRESSURE_SENSOR_H
#define TEST_PRESSURE_SENSOR_H

#include <unity.h>

#include "config.h"
#include "pressure_sensor.h"
#include "state_machine.h"

/*** TESTS FOR 2 PT SETUP ***/

#if !USE_3_PTS

/* Test 1: Both pressure sensors read illogical values. */
void test_pressure_validation_two_sensors_1() {
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
    TEST_ASSERT_EQUAL(SensorStatus::OK_ALL, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MIN, pressureReturned);

    // We should need to register N invalid PT readings for both PTs to get TWO_ILLOGICAL
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateTwoSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }
    ss = ps.validateTwoSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::TWO_ILLOGICAL, ss);
}

/* Test 2: Both pressure sensors read logical values, but differ by too much. This should occur for a certain number of times before the error is registered. */
void test_pressure_validation_two_sensors_2() {
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
    TEST_ASSERT_EQUAL(SensorStatus::OK_ALL, ss);
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
void test_pressure_validation_two_sensors_3() {
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
void test_pressure_validation_two_sensors_4() {
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

    TEST_ASSERT_EQUAL(SensorStatus::OK_ALL, ss);
    TEST_ASSERT_EQUAL_FLOAT(pressureExpected, pressureReturned);
}

#endif

/*** TESTS FOR 3 PT SETUP ***/

#if USE_3_PTS

/* Test 1: All 3 sensors read illogical values. */
void test_pressure_validation_three_sensors_1() {
    float pressureReturned;
    PressureSensor ps;
    SensorStatus ss;

    // Register N - 1 invalid PT readings for all 3 PTs
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateThreeSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, SensorConfig::P_MAX + 1, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }

    // Register a valid reading for all 3 PTs
    ss = ps.validateThreeSensors(SensorConfig::P_MIN, SensorConfig::P_MIN, SensorConfig::P_MIN, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::OK_ALL, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MIN, pressureReturned);

    // We should need to register N invalid PT readings for all 3 PTs to get THREE_ILLOGICAL
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateThreeSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }
    ss = ps.validateThreeSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, SensorConfig::P_MAX + 1, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::THREE_ILLOGICAL, ss);
}

/* Test 2: All 3 pressure sensors read logical values, but consecutively, a pair always differs by too much. This should occur for a certain number of times before the error is registered. */
void test_pressure_validation_three_sensors_2() {
    float pressureReturned;
    PressureSensor ps;
    if (SensorConfig::P_MAX - SensorConfig::P_MIN < SensorConfig::PAIR_DIFFERENCE_THRESHOLD)
        return;

    // Register N - 1 sensor differences which exceed the threshold
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        SensorStatus ss = i % 2 ?
            ps.validateThreeSensors(SensorConfig::P_MIN, SensorConfig::P_MAX, SensorConfig::P_MAX, pressureReturned) :
            ps.validateThreeSensors(SensorConfig::P_MAX, SensorConfig::P_MIN, SensorConfig::P_MAX, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }

    // Register a sensor difference which does not exceed the threshold
    float p_middle = (SensorConfig::P_MIN + SensorConfig::P_MAX) / 2;
    SensorStatus ss = ps.validateThreeSensors(p_middle, p_middle, p_middle, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::OK_ALL, ss);
    TEST_ASSERT_EQUAL_FLOAT(p_middle, pressureReturned);

    // We should need to register N sensor differences which exceed the threshold before it errors
    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        SensorStatus ss = i % 2 ?
            ps.validateThreeSensors(SensorConfig::P_MIN, SensorConfig::P_MAX, SensorConfig::P_MAX, pressureReturned) :
            ps.validateThreeSensors(SensorConfig::P_MAX, SensorConfig::P_MIN, SensorConfig::P_MAX, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }

    ss = ps.validateThreeSensors(SensorConfig::P_MAX, SensorConfig::P_MIN, SensorConfig::P_MAX, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::THREE_ILLOGICAL, ss);
}

/* Test 3: Only one pressure sensor reads a logical value; the others, illogical. */
void test_pressure_validation_three_sensors_3() {
    float pressureReturned;
    PressureSensor ps;
    SensorStatus ss;
    
    ss = ps.validateThreeSensors(SensorConfig::P_MIN, SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::TWO_ILLOGICAL, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MIN, pressureReturned);

    ss = ps.validateThreeSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX, SensorConfig::P_MAX + 1, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::TWO_ILLOGICAL, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MAX, pressureReturned);

    ss = ps.validateThreeSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX + 1, SensorConfig::P_MAX, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::TWO_ILLOGICAL, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MAX, pressureReturned);
}

/* Test 4: Two pressure sensors read a logical value; one reads an illogical value. */
void test_pressure_validation_three_sensors_4() {
    float pressureReturned;
    PressureSensor ps;
    SensorStatus ss;
    
    ss = ps.validateThreeSensors(SensorConfig::P_MIN, SensorConfig::P_MAX, SensorConfig::P_MAX + 1, pressureReturned);
    if (SensorConfig::P_MAX - SensorConfig::P_MIN < SensorConfig::PAIR_DIFFERENCE_THRESHOLD) {
        TEST_ASSERT_EQUAL(SensorStatus::ONE_ILLOGICAL, ss);
        TEST_ASSERT_EQUAL_FLOAT((SensorConfig::P_MAX + SensorConfig::P_MIN)/2, pressureReturned);
    }
    else
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss); // Because P_MIN and P_MAX are too far apart

    ss = ps.validateThreeSensors(SensorConfig::P_MIN - 1, SensorConfig::P_MAX - SensorConfig::PAIR_DIFFERENCE_THRESHOLD, SensorConfig::P_MAX, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::ONE_ILLOGICAL, ss);
    TEST_ASSERT_EQUAL_FLOAT(SensorConfig::P_MAX - SensorConfig::PAIR_DIFFERENCE_THRESHOLD/2, pressureReturned);

    for (int i = 1; i < SensorConfig::CONSEC_BEFORE_ERR_THRESHOLD; i++) {
        ss = ps.validateThreeSensors(SensorConfig::P_MIN, SensorConfig::P_MIN - 1, SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD + 1, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::PENDING_FAULT, ss);
    }
    ss = ps.validateThreeSensors(SensorConfig::P_MIN, SensorConfig::P_MIN - 1, SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD + 1, pressureReturned);
    TEST_ASSERT_EQUAL(SensorStatus::THREE_ILLOGICAL, ss);
}

/* Test 5: All 3 pressure sensors read a logical value; the median should be returned. */
void test_pressure_validation_three_sensors_5() {
    float pressureReturned;
    PressureSensor ps;
    SensorStatus ss;

    if (SensorConfig::P_MAX - SensorConfig::P_MIN < SensorConfig::PAIR_DIFFERENCE_THRESHOLD) {
        float p_middle = (SensorConfig::P_MIN + SensorConfig::P_MAX) / 2;
        ss = ps.validateThreeSensors(SensorConfig::P_MIN, p_middle, SensorConfig::P_MAX, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::OK_ALL, ss);
        TEST_ASSERT_EQUAL(p_middle, pressureReturned);
    } else {
        ss = ps.validateThreeSensors(SensorConfig::P_MIN,
                                     SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD/2,
                                     SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD, pressureReturned);
        TEST_ASSERT_EQUAL(SensorStatus::OK_ALL, ss);
        TEST_ASSERT_EQUAL(SensorConfig::P_MIN + SensorConfig::PAIR_DIFFERENCE_THRESHOLD/2, pressureReturned);
    }
}

#endif

void run_all_pressure_sensor_tests() {
    UnitySetTestFile(__FILE__);
#if USE_3_PTS
    RUN_TEST(test_pressure_validation_three_sensors_1);
    RUN_TEST(test_pressure_validation_three_sensors_2);
    RUN_TEST(test_pressure_validation_three_sensors_3);
    RUN_TEST(test_pressure_validation_three_sensors_4);
    RUN_TEST(test_pressure_validation_three_sensors_5);
#else
    RUN_TEST(test_pressure_validation_two_sensors_1);
    RUN_TEST(test_pressure_validation_two_sensors_2);
    RUN_TEST(test_pressure_validation_two_sensors_3);
    RUN_TEST(test_pressure_validation_two_sensors_4);
#endif
}

#endif // TEST_PRESSURE_SENSOR_H
