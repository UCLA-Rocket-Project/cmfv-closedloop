#ifndef COMM_HANDLER_H
#define COMM_HANDLER_H

#include <CRC.h>

#include "state_machine.h"

/* 
 * NOTE ABOUT ENDIANNESS: All fields in the packet structs, except for the magic bytes
 * at the start which is b'\xfb\xad', are little endian. 
 */

/*
 *  Telemetry Packet Layout
 *  0        8       16       24       32 (Bits)
 *  +--------+--------+--------+--------+
 *  |   Magic Bytes   |    Checksum     |
 *  +--------+--------+--------+--------+
 *  | State  | Flags  | Faults | Unused |
 *  +--------+--------+--------+--------+
 *  |        Current Motor Angle        |
 *  +--------+--------+--------+--------+
 *  |        Current Delta Angle        |
 *  +--------+--------+--------+--------+
 *  |       Current Integral Error      |
 *  +--------+--------+--------+--------+
 *  |      Registered PT1 Reading       |
 *  +--------+--------+--------+--------+
 *  |      Registered PT2 Reading       |
 *  +--------+--------+--------+--------+
 */

#define MAGIC_START 0xadfb // NOTE: THIS IS LITTLE ENDIAN - WE SHOULD BE RECEIVING 0xfbad
#define MAGIC_START_LEN 2

#define TELPKT_FLAGS_MPV_OPEN_DETECTED 0x1
#define TELPKT_FLAGS_MPV_SHD_BE_CLOSED 0x2
#define TELPKT_FLAGS_SYSTEM_TYPE 0x4 // FUEL is 1, OX is 0

#define TELPKT_FAULTS_MANUAL_ABORT 0x1
#define TELPKT_FAULTS_COMM_TIMEOUT 0x2
#define TELPKT_FAULTS_ENCODER_ERROR 0x4
#define TELPKT_FAULTS_NO_MOTION 0x8
#define TELPKT_FAULTS_SENSOR_FAULT 0x10
#define TELPKT_FAULTS_REDBAND_FAULT 0x20
#ifdef USE_OSCILLATION_DETECTOR
    #define TELPKT_FAULTS_OSCILLATION_DETECTED 0x80
#endif

#define TELPKT_SIZE sizeof(telemetryPacket_t)

typedef struct {
    uint16_t _magic;
    uint16_t _checksum;
    SystemStateEnum systemState;
    uint8_t flags;
    uint8_t faults;
    uint8_t _unused; // Should be set to 0 so checksumming works

    float curMotorAngle;
    float curDeltaAngle;
    float curIntError;
    float pt1Reading;
    float pt2Reading;
} telemetryPacket_t;

typedef union {
    telemetryPacket_t data;
    uint8_t bytes[TELPKT_SIZE];
} telemetryPacketU_t;

/*
 *  Layout of Incoming Pressure Update Packet Layout
 *  0        8       16       24       32 (Bits)
 *  +--------+--------+--------+--------+
 *  |   Magic Bytes   |    Checksum     |
 *  +--------+--------+--------+--------+
 *  | State  | Flags  |     Unused      |
 *  +--------+--------+--------+--------+
 *  |            PT1 Reading            |
 *  +--------+--------+--------+--------+
 *  |            PT2 Reading            |
 *  +--------+--------+--------+--------+
 */

#define UPDTPKT_FLAGS_MPV_OPEN 0x1

#define UPDTPKT_SIZE sizeof(pressureUpdatePacket_t)

typedef struct {
    uint16_t _magic;
    uint16_t _checksum;
    SystemStateEnum otherState;
    uint8_t flags;
    uint16_t _unused; // Should be set to 0 so checksumming works

    float pt1Reading;
    float pt2Reading;
} pressureUpdatePacket_t;

typedef union {
    pressureUpdatePacket_t data;
    uint8_t bytes[UPDTPKT_SIZE];
} pressureUpdatePacketU_t;


struct PressureData {
    float sensor1;
    float sensor2;
    bool valid;
    
    PressureData() : sensor1(0.0f), sensor2(0.0f), valid(false) {}
};


class CommHandler {
public:
    CommHandler();
    
    // Communication-related
    void processIncomingNonBlocking();
    void flushInputBuffer();
    
    // Getters
    PressureData getPressureData() const { return m_pressureData; }
    SystemStateEnum getOtherCtrlerState() const { return m_otherCtrlerState; }

    // Check if communication is healthy
    bool isCommHealthy() const;
    
    // Send telemetry data
    void sendTelemetry(SystemStateEnum state, float motorAngle, float deltaAngle, float pidIntegralError, const char* systemType, bool ifMpvOpen);

#ifdef PIO_UNIT_TESTING
    void processIncomingSerialByte(uint8_t c);
    bool parsePressureUpdatePacket();

    const pressureUpdatePacketU_t& getInputBuffer() const { return m_inputBuffer; };
    const bool getPressureUpdateSuccess() const { return m_pressureUpdateSuccess; };
#endif
    
private:
    pressureUpdatePacketU_t m_inputBuffer;
    unsigned int m_bufLen;
    bool m_pressureUpdateSuccess;

    telemetryPacketU_t m_outputBuffer;

    CRC16 m_crc16;

    SystemStateEnum m_otherCtrlerState;
    PressureData m_pressureData;
    unsigned long m_lastCommTime;
    int m_numConsecInvalidPUP;

#ifndef PIO_UNIT_TESTING
    void processIncomingSerialByte(uint8_t c);
    bool parsePressureUpdatePacket();
#endif

    // Utility functions
    void updateNumConsecInvalidPUP(bool valid);
    bool isValidPressure(float p);
    uint16_t calcChecksum(const uint8_t *array, unsigned int length);
};

#endif // COMM_HANDLER_H