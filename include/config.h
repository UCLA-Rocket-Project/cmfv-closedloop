#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// This file contains all tunable parameters for the CMFV system
// Modify these values to adapt the system to your specific hardware and requirements

// ================================
// SYSTEM CONFIGURATION
// ================================

// Toggle between fuel and oxidizer system
// Set to true for FUEL system, false for OXIDIZER system
#define IS_FUEL_SYSTEM true

// Open Loop toggle
#define OPEN_LOOP_MODE false

// ================================
// UTILITY FUNCTIONS
// ================================

template <typename T>
constexpr T sqr(T t) {
    return t * t;
}

// ================================
// CONTROL SYSTEM PARAMETERS
// ================================

// PID Controller gains (system-specific)
struct ControllerConfig {
#if IS_FUEL_SYSTEM
    // Fuel Controller gains
    static constexpr float KP = 0.1f;
    static constexpr float KI = 0.05f;
    static constexpr float KD = 0.0f;
    static constexpr float TARGET_PRESSURE_PSI = 300.0f;  // Fuel manifold target pressure
#else
    // Oxidizer Controller gains
    static constexpr float KP = 0.08f;
    static constexpr float KI = 0.05f;
    static constexpr float KD = 0.0f;
    static constexpr float TARGET_PRESSURE_PSI = 300.0f; // Ox manifold target pressure
#endif
    
    // Integral limits (common for both systems)
    static constexpr float I_MIN = -10.0f;
    static constexpr float I_MAX = 10.0f;
};

// ================================
// TIMING AND SAFETY
// ================================

struct TimingConfig {
    static constexpr float CONTROL_PERIOD_HZ = 20.0f;  // Control loop frequency in Hz
    static constexpr float CONTROL_PERIOD_S = 1.0f / CONTROL_PERIOD_HZ; // Expected period (used as fallback)
    static constexpr float COMM_TIMEOUT_S = 3.0f;      // Communication check
    static constexpr float SAFE_TIMER_S = 0.5f;        // Safety delay before MPV
    static constexpr float RECOVERY_DWELL_S = 3.0f;    // Time in forced open loop before recovery
    static constexpr float REDBAND_TIMEOUT = 2.0f;      // Time until redband activation
};

// ================================
// VALVE CONTROL
// ================================

struct ValveConfig {
    // Movement filtering
    static constexpr float MOVE_FILTER_SCALE = 1.0f;           // 100% move filter
    static constexpr float MAX_ANGLE_CHANGE_PER_CYCLE = 5.0f;  // Degrees per cycle limit
    
    // Angle limits
    static constexpr float MIN_VALVE_ANGLE = 30.0f;    // Fully closed
    static constexpr float MAX_VALVE_ANGLE = 90.0f;   // Fully open
    static constexpr float SAFE_ANGLE = 30.0f;         // Safe position for emergencies

    #if IS_FUEL_SYSTEM
        static constexpr float OPENLOOP_TARGET_ANGLE = 45.0f; // Target angle for forced open loop
        static constexpr float START_ANGLE = 45.0f;       // Starting position for closed loop
    #else
        static constexpr float OPENLOOP_TARGET_ANGLE = 45.0f; // Target angle for forced open loop
        static constexpr float START_ANGLE = 45.0f;       // Starting position for closed loop
    #endif

    // Position tolerance
    static constexpr float ANGLE_TOLERANCE = 0.5f;    // Acceptable angle error (degrees)
};

// ================================
// HARDWARE CONFIGURATION  
// ================================

struct HardwareConfig {
#if IS_FUEL_SYSTEM
    // Pin assignments for Fuel System
    static constexpr uint8_t MOTOR_CS_PIN = 10;
    static constexpr uint8_t ENCODER_CS_PIN = 9;
    static constexpr float FULLY_OPEN_OFFSET = 228.48f;
    static constexpr const char* SYSTEM_NAME = "FUEL";
#else
    // Pin assignments for Oxidizer System
    static constexpr uint8_t MOTOR_CS_PIN = 10;
    static constexpr uint8_t ENCODER_CS_PIN = 9;
    static constexpr float FULLY_OPEN_OFFSET = 56.97f;
    static constexpr const char* SYSTEM_NAME = "OX";
#endif
    
    // Shared pins
    static constexpr uint8_t CB_ONOFF_PIN = 2;     // MPV status (read only)
    static constexpr uint8_t MANUAL_ABORT_PIN = 0;
    
    // SPI pins. Already defined in the Arduino library
    // static constexpr uint8_t PIN_SPI_MOSI = 11;
    // static constexpr uint8_t PIN_SPI_MISO = 12;
    // static constexpr uint8_t PIN_SPI_SCK = 13;

    // Encoder SPI bus configuration
    static constexpr uint32_t ENCODER_SPI_MAX_DATA_RATE = 2000000;
    
    // Stepper motor configuration
    static constexpr uint16_t STEP_PERIOD_US = 750;
    static constexpr float DEGREES_PER_STEP = 0.036f;
    static constexpr uint16_t MOTOR_CURRENT_MA = 1500;
    
    // Encoder configuration & calibration
    static constexpr float MAX_12_BIT_VAL = 4095.0f; 
};

// ================================
// MOTOR INNER-LOOP (PROPORTIONAL)
// ================================

struct MotorControlConfig {
    // Proportional gain: steps to move per degree of error.
    static constexpr float KP_STEPS_PER_DEG = 1.0f / HardwareConfig::DEGREES_PER_STEP;

    // // Maximum number of steps allowed per update cycle.
    // static constexpr int   MAX_STEPS_PER_UPDATE = 10;

    // Ignore angle errors smaller than this threshold.
    static constexpr float ANGLE_DEADBAND_DEG = ValveConfig::ANGLE_TOLERANCE;

    // Redband pressure tolerances (PSI) - maximum allowable error from target pressure
    static constexpr float REDBAND_PRESSURE_UPPER = 25.0;
    static constexpr float REDBAND_PRESSURE_LOWER = 35.0;  
};

// ================================
// PRESSURE SENSORS
// ================================

struct SensorConfig {
#ifdef USE_3_PTS
    static constexpr int NUM_PTS = 3;
#else
    static constexpr int NUM_PTS = 2;
#endif

    // Sensor validation limits
    static constexpr float P_MIN = -20.0f;                      // Minimum valid pressure (PSI)
    static constexpr float P_MAX = 1000.0f;                     // Maximum valid pressure (PSI)
    static constexpr float PAIR_DIFFERENCE_THRESHOLD = 25.0f;   // Max difference between PT1/PT2
    static constexpr int CONSEC_BEFORE_ERR_THRESHOLD = 5;       // Number of consecutive faults before triggering
    
    // Control tolerance
    static constexpr float PRESSURE_TOLERANCE = 5.0f; // Acceptable pressure error (PSI)
};

// ================================
// FAULT DETECTION
// ================================

struct FDIRConfig {
    // Oscillation detection
    static constexpr int MAX_SIGN_CHANGES = 10;         // No. of registered sign changes within the set time window
                                                        // before concluding that oscillations are detected
    static constexpr float SIGN_CHANGE_WINDOW_S = 1.0f; // Sliding time window that accumulates and loses sign changes
    
    // Encoder fault detection
    static constexpr float MOTION_EXPECTED_ANGLE_THRESHOLD = 0.25f; // Minimum detectable motion (degrees)
    static constexpr float MOTION_DETECTED_ANGLE_THRESHOLD = 0.05f;

    // Threshold constants for sign change detection
    static constexpr int CONSEC_SAME_SIGN_THRESHOLD = 5;        // No. of consecutive readings of a particular magnitude of a
                                                                // different sign for us to register sign change
    static constexpr float ERROR_MAGNITUDE_THRESHOLD = 1.0f;    // Minimum error magnitude to count error
};

// ================================
// COMMUNICATION
// ================================

struct CommConfig {
    static constexpr unsigned long BAUD_RATE = 115200;
    static constexpr int INPUT_BUFFER_SIZE = 128;
    static constexpr float TELEMETRY_RATE_HZ = 5.0f;  

    static constexpr int MAX_NUM_CONSEC_INVALIDS = 5;
};

// ================================
// COMPILE-TIME VALIDATION
// ================================

// Static assertions to catch configuration errors at compile time
static_assert(ValveConfig::MIN_VALVE_ANGLE < ValveConfig::MAX_VALVE_ANGLE, 
              "Invalid valve angle range");
              
static_assert(ValveConfig::START_ANGLE >= ValveConfig::MIN_VALVE_ANGLE && 
              ValveConfig::START_ANGLE <= ValveConfig::MAX_VALVE_ANGLE,
              "Start angle outside valid range");
              
static_assert(ValveConfig::OPENLOOP_TARGET_ANGLE >= ValveConfig::MIN_VALVE_ANGLE && 
              ValveConfig::OPENLOOP_TARGET_ANGLE <= ValveConfig::MAX_VALVE_ANGLE,
              "Open loop target angle outside valid range");

static_assert(ValveConfig::MOVE_FILTER_SCALE > 0 && ValveConfig::MAX_ANGLE_CHANGE_PER_CYCLE > 0,
                "Constants used for applying the move filter on the motor must be positive");
              
static_assert(SensorConfig::P_MIN < SensorConfig::P_MAX, 
              "Invalid pressure sensor range");
              
static_assert(TimingConfig::CONTROL_PERIOD_HZ > 0, 
              "Invalid control period");

static_assert(ControllerConfig::I_MIN < ControllerConfig::I_MAX,
              "Invalid integral limits");

#endif // CONFIG_H