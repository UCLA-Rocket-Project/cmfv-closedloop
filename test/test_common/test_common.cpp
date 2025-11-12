#include "test_comm_handler.h"
#include "test_controller.h"
#include "test_pressure_sensor.h"
#ifdef USE_OSCILLATION_DETECTOR
    #include "test_oscillation_detection.h"
#endif

#include <unity.h>

#ifdef BUILD_NATIVE
    #include <ArduinoFake.h>
    using namespace fakeit;
#endif

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
#ifdef BUILD_NATIVE
    When(Method(ArduinoFake(), millis)).AlwaysReturn();
#endif

    UNITY_BEGIN();
    run_all_pressure_sensor_tests();
    run_all_controller_tests();
    run_all_comm_handler_tests();
#ifdef USE_OSCILLATION_DETECTOR
    run_all_oscillation_detection_tests();
#endif
    return UNITY_END();
}