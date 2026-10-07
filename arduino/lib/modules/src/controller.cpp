#include "assert_own.h"
#include "controller.h"
#include "config.h"

#include <stdlib.h>
#include <math.h>

Controller::Controller(float kp, float ki, float kd)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_curError(0.0f), m_integralError(0.0f), m_previousError(0.0f), m_firstCall(true)
#ifdef USE_OSCILLATION_DETECTOR
    , m_oscDetector() 
#endif
    {}

#ifdef USE_OSCILLATION_DETECTOR
// OscillationDetector implementation
Controller::OscillationDetector::OscillationDetector()
    : m_currentSign(CurrentSign::UNINITIALIZED), m_lPtr(0), m_rPtr(0), m_signChgCnt(0), m_consecErrorCnt(0) {}

bool Controller::OscillationDetector::checkOscillation(float error, unsigned long now) {
    // Remove expired oscillations
    while(m_lPtr != m_rPtr && m_signChgCnt > 0 && m_oscTimes[m_lPtr] < now - FDIRConfig::SIGN_CHANGE_WINDOW_S * 1000)
        m_rmSignChg();

    // Only process further if the error is of sufficient magnitude
    if (abs(error) >= FDIRConfig::ERROR_MAGNITUDE_THRESHOLD) {
        // If the current sign is uninitialized, then initialize it
        // (meant to fall through to set m_consecErrorCnt = 0 and then return)
        if (m_currentSign == CurrentSign::UNINITIALIZED)
            m_currentSign = error > 0 ? CurrentSign::POSITIVE : CurrentSign::NEGATIVE;

        // If the error is of the same sign as the current state, then we reset the consec error cnt
        // (of diff sign) and move on
        if ((error > 0 && m_currentSign == CurrentSign::POSITIVE) || 
            (error < 0 && m_currentSign == CurrentSign::NEGATIVE)) {
            m_consecErrorCnt = 0;
        }  
        // Else if adding the error to the consec cnt fails (i.e. it hit the threshold), then register a sign change
        else if (!m_tryAddErrorToCnt(error))
            m_addSignChg(now);
    }

    return m_signChgCnt >= FDIRConfig::MAX_SIGN_CHANGES;
}

void Controller::OscillationDetector::reset() {
    m_currentSign = CurrentSign::UNINITIALIZED;
    m_lPtr = 0;
    m_rPtr = 0;
    m_signChgCnt = 0;
    m_consecErrorCnt = 0;
}

bool Controller::OscillationDetector::m_tryAddErrorToCnt(float error) {
    if (m_consecErrorCnt == FDIRConfig::CONSEC_SAME_SIGN_THRESHOLD - 1) {
        m_currentSign = error > 0 ? CurrentSign::POSITIVE : CurrentSign::NEGATIVE;
        m_consecErrorCnt = 0;
        return false;
    } else {
        m_consecErrorCnt++;
        return true;
    }
}

void Controller::OscillationDetector::m_addSignChg(unsigned long timeToAdd) {
    assert(m_signChgCnt <= FDIRConfig::MAX_SIGN_CHANGES * M_SIGN_CHG_BUF_SIZE_MULTIPLIER); // M_SIGN_CHG_BUF_SIZE_MULTIPLIER is an arbitrary number (of the buffer that stores times)
    m_oscTimes[m_rPtr] = timeToAdd;
    m_incrementCirPtr(m_rPtr);
    m_signChgCnt++;
}

void Controller::OscillationDetector::m_rmSignChg() {
    assert(m_signChgCnt > 0);
    if (m_signChgCnt <= 0) return; // When NDEBUG is defined, assert does nothing, and we want to avoid an array OOB
    m_incrementCirPtr(m_lPtr);
    m_signChgCnt--;
}

void Controller::OscillationDetector::m_incrementCirPtr(int& cirPtr) {
    cirPtr = (cirPtr + 1) % (FDIRConfig::MAX_SIGN_CHANGES * M_SIGN_CHG_BUF_SIZE_MULTIPLIER);
}
#endif

void Controller::update(float error, float dt_seconds) {
    // Integral term with anti-windup
    m_integralError += error * dt_seconds;
    m_integralError = clamp(m_integralError, ControllerConfig::I_MIN, ControllerConfig::I_MAX);
    
    // Derivative term
    float derivative = 0.0f;
    if (!m_firstCall && dt_seconds > 0.0f) {
        derivative = (error - m_previousError) / dt_seconds;
    }
    
    // Update m_curError so that the PID controller always tracks this internally
    m_curError = m_kp * error + m_ki * m_integralError + m_kd * derivative;
    
    // Update m_previousError for the derivative term of the next iteration
    m_previousError = error;
    m_firstCall = false;
}

void Controller::reset() {
    m_integralError = 0.0;
    m_previousError = 0.0;
    m_firstCall = true;
}

float Controller::clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}