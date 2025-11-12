#ifndef TEST_OSCILLATION_DETECTION_H
#define TEST_OSCILLATION_DETECTION_H

#include <unity.h>

#include "config.h"
#include <controller.h>

/* API tests */

void test_oscillation_detector_osc_detected() {
    Controller controller = Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);

    // Force oscillation detection
    controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f, 1000);
    for (int i=0; i<FDIRConfig::MAX_SIGN_CHANGES; i++) {
        for (int j=0; j<FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD; j++) {
            controller.getOscillationDetector().checkOscillation(
                FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f * (i % 2 ? 1.0f : -1.0f), 1000 + i);
        }
    }

    // At this point, OscillationDetector should detect oscillations, even if the error is below the magnitude threshold
    TEST_ASSERT_TRUE(controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f, 1000 + FDIRConfig::MAX_SIGN_CHANGES - 1));
    TEST_ASSERT_TRUE(controller.getOscillationDetector().checkOscillation(-FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 0.5f, 1000 + FDIRConfig::MAX_SIGN_CHANGES - 1));
}

void test_oscillation_detector_osc_not_detected_1() {
    Controller controller = Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);

    // No detection should occur since the magnitudes are all insufficient
    controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f, 1000);
    for (int i=0; i<FDIRConfig::MAX_SIGN_CHANGES; i++) {
        for (int j=0; j<FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD; j++) {
            controller.getOscillationDetector().checkOscillation(
                FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 0.5f * (i % 2 ? 1.0f : -1.0f), 1000 + i);
        }
    }

    TEST_ASSERT_FALSE(controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f, 1000 + FDIRConfig::MAX_SIGN_CHANGES - 1));
}

void test_oscillation_detector_osc_not_detected_2() {
    Controller controller = Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);

    // One of the errors being of insufficient magnitude should almost always result in
    // non-detection of oscillations since there should be insufficient sign changes recorded
    controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f, 1000);
    for (int i=0; i<FDIRConfig::MAX_SIGN_CHANGES; i++) {
        for (int j=0; j<FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD; j++) {
            if (i==0 && j==FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD - 1)
                controller.getOscillationDetector().checkOscillation(
                    FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 0.5f * (i % 2 ? 1.0f : -1.0f), 1000 + i);
            else
                controller.getOscillationDetector().checkOscillation(
                    FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f * (i % 2 ? 1.0f : -1.0f), 1000 + i);
        }
    }

    TEST_ASSERT_FALSE(controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 0.5f, 1000 + FDIRConfig::MAX_SIGN_CHANGES - 1));
    TEST_ASSERT_FALSE(controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f, 1000 + FDIRConfig::MAX_SIGN_CHANGES - 1));
}

/* Internals tests */

void test_oscillation_detector_internals_currentSign() {
    Controller controller = Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);
    TEST_ASSERT_EQUAL(0, controller.getOscillationDetector().getCurrentSign()); // m_currentSign = UNINITIALIZED

    // Error of insufficient magnitude - currentSign should still be UNINITIALIZED
    controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 0.5f, 1000);
    TEST_ASSERT_EQUAL(0, controller.getOscillationDetector().getCurrentSign());

    // Error of sufficient magnitude - currentSign should now be POSITIVE
    controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD, 1001);
    TEST_ASSERT_EQUAL(1, controller.getOscillationDetector().getCurrentSign()); // m_currentSign = POSITIVE

    // Error of insufficient magnitude in the opposite direction
    controller.getOscillationDetector().checkOscillation(-FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 0.5f, 1002);
    TEST_ASSERT_EQUAL(1, controller.getOscillationDetector().getCurrentSign());
    TEST_ASSERT_EQUAL(0, controller.getOscillationDetector().getConsecErrorCnt());

    // Errors of sufficient magnitude in the opposite direction should increment the consec error cnt
    for (int i=0; i<FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD - 1; i++)
        controller.getOscillationDetector().checkOscillation(-FDIRConfig::ERROR_MAGNITUDE_THRESHOLD, 1003 + i);
    TEST_ASSERT_EQUAL(1, controller.getOscillationDetector().getCurrentSign());
    TEST_ASSERT_EQUAL(4, controller.getOscillationDetector().getConsecErrorCnt());

    // Error of sufficient magnitude in the original direction should zero the consec error cnt
    controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD, 1003 + FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD - 1);
    TEST_ASSERT_EQUAL(1, controller.getOscillationDetector().getCurrentSign());
    TEST_ASSERT_EQUAL(0, controller.getOscillationDetector().getConsecErrorCnt());

    // Sufficient no. of consecutive errors of sufficient magnitude in the opposite direction to cause a sign change
    for (int i=0; i<FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD; i++)
        controller.getOscillationDetector().checkOscillation(-FDIRConfig::ERROR_MAGNITUDE_THRESHOLD, 1003 + FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD + i);
    TEST_ASSERT_EQUAL(2, controller.getOscillationDetector().getCurrentSign()); // m_currentSign = NEGATIVE
    TEST_ASSERT_EQUAL(0, controller.getOscillationDetector().getConsecErrorCnt());
    TEST_ASSERT_EQUAL(1, controller.getOscillationDetector().getSignChgCnt());
    TEST_ASSERT_EQUAL(1, controller.getOscillationDetector().getRightPtr());
}

void test_oscillation_detector_internals_signChange() {
    Controller controller = Controller(ControllerConfig::KP, ControllerConfig::KI, ControllerConfig::KD);

    // Register a bunch of sign changes at earlier times
    // - This test tests the ability to register up to (FDIRConfig::MAX_SIGN_CHANGES + 1) sign changes
    controller.getOscillationDetector().checkOscillation(FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f, 1000);
    for (int i=0; i<FDIRConfig::MAX_SIGN_CHANGES + 1; i++) {
        for (int j=0; j<FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD; j++) {
            controller.getOscillationDetector().checkOscillation(
                FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f * (i % 2 ? 1.0f : -1.0f), 1000 + i);
        }

        // Sanity checks on OscillationDetector internals
        TEST_ASSERT_EQUAL(i % 2 ? 1 : 2, controller.getOscillationDetector().getCurrentSign());
        TEST_ASSERT_EQUAL(0, controller.getOscillationDetector().getConsecErrorCnt());
        TEST_ASSERT_EQUAL(i + 1, controller.getOscillationDetector().getSignChgCnt());
        TEST_ASSERT_EQUAL(i + 1, controller.getOscillationDetector().getRightPtr());
    }

    // Test that our buffer of stored times of sign changes is as expected
    unsigned long expectedArr[FDIRConfig::MAX_SIGN_CHANGES + 1];
    for (int i=0; i<FDIRConfig::MAX_SIGN_CHANGES + 1; i++)
        expectedArr[i] = 1000 + i;
    TEST_ASSERT_UINT_ARRAY_WITHIN(0, expectedArr, controller.getOscillationDetector().getOscTimes(), FDIRConfig::MAX_SIGN_CHANGES + 1);

    // Register errors at times which should result in the earlier sign changes being invalidated
    for (int i=0; i<FDIRConfig::MAX_SIGN_CHANGES + 1; i++) {
        controller.getOscillationDetector().checkOscillation(
                FDIRConfig::ERROR_MAGNITUDE_THRESHOLD * 2.0f * (i % 2 ? 1.0f : -1.0f), 1000 + FDIRConfig::SIGN_CHANGE_WINDOW_S * 1000 + i + 1);

        // Sanity checks on OscillationDetector internals
        TEST_ASSERT_EQUAL(FDIRConfig::MAX_SIGN_CHANGES - i, controller.getOscillationDetector().getSignChgCnt());
        TEST_ASSERT_EQUAL(i + 1, controller.getOscillationDetector().getLeftPtr());
    }
}

void run_all_oscillation_detection_tests() {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_oscillation_detector_osc_detected);
    RUN_TEST(test_oscillation_detector_osc_not_detected_1);
    RUN_TEST(test_oscillation_detector_osc_not_detected_2);

    // Testing of internals
    // - These tests don't necessarily have to pass for the API to work since the
    //   internals may simply be different
    RUN_TEST(test_oscillation_detector_internals_currentSign);
    RUN_TEST(test_oscillation_detector_internals_signChange);
}

#endif // TEST_OSCILLATION_DETECTION_H