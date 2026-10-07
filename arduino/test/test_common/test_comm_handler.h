#ifndef TEST_COMM_HANDLER_H
#define TEST_COMM_HANDLER_H

#include <CRC.h>
#include <unity.h>

// #include "debug.h"
#include "config.h"
#include <comm_handler.h>

extern bool MPV_STATE;

// Because we extern some symbols which are accessible to utilities.h
#include <AMT22_lib.h>
#include <controller.h>
#include <pressure_sensor.h>
#include <utilities.h>
#include "state_machine.h"
AMT22* encoder;
CommHandler* commHandler;
Controller* controller;
PressureSensor* pressureSensor;
SystemState systemState;
ChannelState channel;
FaultFlags faults;

pressureUpdatePacketU_t testPUP;
constexpr float defaultPt1Reading = (SensorConfig::P_MIN + SensorConfig::P_MAX) / 2;
constexpr float defaultPt2Reading = SensorConfig::P_MIN + (SensorConfig::P_MAX - SensorConfig::P_MIN) / 3;

CRC16 m_crc16(CRC16_XMODEM_POLYNOME, CRC16_XMODEM_INITIAL, CRC16_XMODEM_XOR_OUT, CRC16_XMODEM_REV_IN, CRC16_XMODEM_REV_OUT);

void populatePUP(uint16_t _magic, bool calcChecksum, uint16_t _checksum, SystemStateEnum otherState, bool ifMpvOpen,
                 float pt1Reading, float pt2Reading) {
    
    testPUP.data._magic = _magic;
    testPUP.data.otherState = otherState;

    testPUP.data.flags = 0;
    if (ifMpvOpen) testPUP.data.flags |= UPDTPKT_FLAGS_MPV_OPEN;

    testPUP.data.pt1Reading = pt1Reading;
    testPUP.data.pt2Reading = pt2Reading;

    // Allow for the use of an invalid checksum based on calChecksum
    if (calcChecksum) {
        m_crc16.add(testPUP.bytes + 4, UPDTPKT_SIZE - 4);
        testPUP.data._checksum = m_crc16.calc();
        m_crc16.restart();
    }
    else testPUP.data._checksum = _checksum;
}

void assembleDefaultValidPUP() {
    populatePUP(MAGIC_START, true, 0x0, SystemStateEnum::CLOSED_LOOP, true, 
                defaultPt1Reading, defaultPt2Reading);
}

void test_comm_handler_receive_valid_packet() {
    CommHandler commHandler;
    assembleDefaultValidPUP();

    // Send packet
    for (unsigned int i=0; i<UPDTPKT_SIZE; i++)
        commHandler.processIncomingSerialByte(testPUP.bytes[i]);
        
    TEST_ASSERT_EQUAL(SystemStateEnum::CLOSED_LOOP, commHandler.getOtherCtrlerState());
    TEST_ASSERT_TRUE(MPV_STATE);

    TEST_ASSERT_TRUE(commHandler.getPressureData().valid);
    TEST_ASSERT_EQUAL_FLOAT(defaultPt1Reading, commHandler.getPressureData().sensor1);
    TEST_ASSERT_EQUAL_FLOAT(defaultPt2Reading, commHandler.getPressureData().sensor2);
}

void test_comm_handler_receive_valid_packet_in_between_bytes() {
    CommHandler commHandler;
    assembleDefaultValidPUP();

    // Send some junk bytes
    for (int i=0; i<=0xff; i++) commHandler.processIncomingSerialByte(uint8_t(i));

    // Send packet
    for (unsigned int i=0; i<UPDTPKT_SIZE; i++)
        commHandler.processIncomingSerialByte(testPUP.bytes[i]);

    // Send some more junk bytes
    for (int i=0xff; i>=0; i--) commHandler.processIncomingSerialByte(uint8_t(i));
    
    TEST_ASSERT_EQUAL(SystemStateEnum::CLOSED_LOOP, commHandler.getOtherCtrlerState());
    TEST_ASSERT_TRUE(MPV_STATE);

    TEST_ASSERT_TRUE(commHandler.getPressureData().valid);
    TEST_ASSERT_EQUAL_FLOAT(defaultPt1Reading, commHandler.getPressureData().sensor1);
    TEST_ASSERT_EQUAL_FLOAT(defaultPt2Reading, commHandler.getPressureData().sensor2);
}

void test_comm_handler_invalid_checksum() {
    CommHandler commHandler;
    assembleDefaultValidPUP();
    testPUP.data._checksum = calcCRC16(testPUP.bytes + 4, UPDTPKT_SIZE - 4) - 1;

    for (unsigned int i=0; i<UPDTPKT_SIZE; i++)
        commHandler.processIncomingSerialByte(testPUP.bytes[i]);
    
    TEST_ASSERT_FALSE(commHandler.getPressureUpdateSuccess());
}

void run_all_comm_handler_tests() {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_comm_handler_receive_valid_packet);
    RUN_TEST(test_comm_handler_receive_valid_packet_in_between_bytes);
    RUN_TEST(test_comm_handler_invalid_checksum);
}

#endif // TEST_COMM_HANDLER_H