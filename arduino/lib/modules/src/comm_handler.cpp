#include <Arduino.h>
#include <CRC.h>
#include <string.h>

#include "assert_own.h"
#include "comm_handler.h"
#include "config.h"
#include <utilities.h>

extern bool MPV_STATE;

CommHandler::CommHandler() : m_bufLen(0), m_pressureUpdateSuccess(false),
                             m_crc16(CRC16_XMODEM_POLYNOME, CRC16_XMODEM_INITIAL, CRC16_XMODEM_XOR_OUT, CRC16_XMODEM_REV_IN, CRC16_XMODEM_REV_OUT),
                             m_otherCtrlerState(SystemStateEnum::BOOT_INIT), m_numConsecInvalidPUP(0) {
    m_lastCommTime = millis();
}

void CommHandler::processIncomingNonBlocking() {
    while (Serial.available()) {
        processIncomingSerialByte(Serial.read());
    }
}

void CommHandler::flushInputBuffer() {
    m_bufLen = 0;
}

void CommHandler::processIncomingSerialByte(uint8_t c) {
    // Magic byte detection
    if (m_bufLen < 2) { 
        if (m_bufLen == 0 && c == (MAGIC_START & 0xff)) {
            m_inputBuffer.bytes[m_bufLen++] = c;
        } else if (m_bufLen == 1) {
            if (c == MAGIC_START >> 8)
                m_inputBuffer.bytes[m_bufLen++] = c;
            else m_bufLen = 0;
        }
        return;
    }

    m_inputBuffer.bytes[m_bufLen++] = c;

    // Packet fully received
    if (m_bufLen == UPDTPKT_SIZE) {
        m_pressureUpdateSuccess = parsePressureUpdatePacket();
        updateNumConsecInvalidPUP(m_pressureUpdateSuccess);
        if (m_numConsecInvalidPUP > CommConfig::MAX_NUM_CONSEC_INVALIDS)
            m_pressureData.valid = false;
        else
            m_pressureData.valid = true;
        m_bufLen = 0;
    }
}

/* See comm_handler.h for the structure of a Pressure Update Packet */
bool CommHandler::parsePressureUpdatePacket() {
    // Verify CRC16 checksum
    if (calcChecksum(m_inputBuffer.bytes + 4, UPDTPKT_SIZE - 4) != m_inputBuffer.data._checksum)
        return false;

    // Update pressures
    m_pressureData.sensor1 = m_inputBuffer.data.pt1Reading;
    m_pressureData.sensor2 = m_inputBuffer.data.pt2Reading;
#if USE_3_PTS
    m_pressureData.sensor3 = m_inputBuffer.data.pt3Reading;
#endif

    // Update MPV Open/Close state
    MPV_STATE = (m_inputBuffer.data.flags & UPDTPKT_FLAGS_MPV_OPEN) != 0;
    
    // Update our record of the other controller's state
    if (!isValidState(m_inputBuffer.data.otherState))
        return false;
    m_otherCtrlerState = m_inputBuffer.data.otherState;
    
    m_lastCommTime = millis();
    return true;
}

bool CommHandler::isCommHealthy() const {
    return (millis() - m_lastCommTime) < (TimingConfig::COMM_TIMEOUT_S / 2 * 1000);
}

/* See comm_handler.h for the structure of a Telemetry Packet */
void CommHandler::sendTelemetry(SystemStateEnum state, float motorAngle, float deltaAngle, float pidIntegralError, const char* systemType, bool ifMpvShdBeClosed) {    
    m_outputBuffer.data._magic = MAGIC_START;
    m_outputBuffer.data.systemState = state;

    m_outputBuffer.data.flags = 0;
    if (MPV_STATE) m_outputBuffer.data.flags |= TELPKT_FLAGS_MPV_OPEN_DETECTED;
    if (ifMpvShdBeClosed) m_outputBuffer.data.flags |= TELPKT_FLAGS_MPV_SHD_BE_CLOSED;
    if (strcmp(systemType, "FUEL") == 0) m_outputBuffer.data.flags |= TELPKT_FLAGS_SYSTEM_TYPE;

    m_outputBuffer.data.faults = 0;
    if (faults.manualAbort) m_outputBuffer.data.faults |= TELPKT_FAULTS_MANUAL_ABORT;
    if (faults.commTimeout) m_outputBuffer.data.faults |= TELPKT_FAULTS_COMM_TIMEOUT;
    if (faults.encoderMismatch) m_outputBuffer.data.faults |= TELPKT_FAULTS_ENCODER_ERROR;
    if (faults.noMotion) m_outputBuffer.data.faults |= TELPKT_FAULTS_NO_MOTION;
    if (faults.sensorFault) m_outputBuffer.data.faults |= TELPKT_FAULTS_SENSOR_FAULT;
    if (faults.redBandFault) m_outputBuffer.data.faults |= TELPKT_FAULTS_REDBAND_FAULT;
#ifdef USE_OSCILLATION_DETECTOR
    if (faults.oscillationDetected) m_outputBuffer.data.faults |= TELPKT_FAULTS_OSCILLATION_DETECTED;
#endif

    m_outputBuffer.data._unused = 0;

    m_outputBuffer.data.curMotorAngle = motorAngle;
    m_outputBuffer.data.curDeltaAngle = deltaAngle;
    m_outputBuffer.data.curIntError = pidIntegralError;
    m_outputBuffer.data.pt1Reading = m_pressureData.sensor1;
    m_outputBuffer.data.pt2Reading = m_pressureData.sensor2;
#if USE_3_PTS
    m_outputBuffer.data.pt3Reading = m_pressureData.sensor3;
#endif

    // Calculate CRC16 checksum
    m_outputBuffer.data._checksum = calcChecksum(m_outputBuffer.bytes + 4, TELPKT_SIZE - 4);

    Serial.write(m_outputBuffer.bytes, TELPKT_SIZE);
}

void CommHandler::updateNumConsecInvalidPUP(bool valid) {
    if (valid)
        m_numConsecInvalidPUP = 0;
    else
        m_numConsecInvalidPUP++;
}

uint16_t CommHandler::calcChecksum(const uint8_t *array, unsigned int length) {
    m_crc16.add(array, length);
    uint16_t checksum = m_crc16.calc();
    m_crc16.restart();
    return checksum;
}