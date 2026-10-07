#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "config.h"

class Controller {
public:
    Controller(float kp, float ki, float kd);
    void update(float error, float dt);
    void reset();
    float getError() const { return m_curError; }
    float getIntegral() const { return m_integralError; }

#ifdef USE_OSCILLATION_DETECTOR
private: 
    static constexpr uint8_t M_SIGN_CHG_BUF_SIZE_MULTIPLIER = 10;

public: 

    // Oscillation detection nested class
    class OscillationDetector {
    public:
        OscillationDetector();
        bool checkOscillation(float error, unsigned long now);
        void reset();

    #ifdef PIO_UNIT_TESTING
        int getCurrentSign() const { return m_currentSign; }
        int getLeftPtr() const { return m_lPtr; }
        int getRightPtr() const { return m_rPtr; }
        int getSignChgCnt() const { return m_signChgCnt; }
        int getConsecErrorCnt() const { return m_consecErrorCnt; }

        const unsigned long* getOscTimes() const { return m_oscTimes; }
    #endif

    private:
        enum CurrentSign { UNINITIALIZED, POSITIVE, NEGATIVE };
        CurrentSign m_currentSign; 

        int m_lPtr, m_rPtr, m_signChgCnt, m_consecErrorCnt;
        unsigned long m_oscTimes[FDIRConfig::MAX_SIGN_CHANGES * M_SIGN_CHG_BUF_SIZE_MULTIPLIER];
        
        bool m_tryAddErrorToCnt(float error);
        void m_addSignChg(unsigned long timeToAdd);
        void m_rmSignChg();
        void m_incrementCirPtr(int& cirPtr);
    };

    OscillationDetector& getOscillationDetector() { return m_oscDetector; }
#endif

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_curError;
    float m_integralError;
    float m_previousError;
    bool m_firstCall;

#ifdef USE_OSCILLATION_DETECTOR
    OscillationDetector m_oscDetector;
#endif
    float clamp(float value, float min, float max);
};

#endif // CONTROLLER_H