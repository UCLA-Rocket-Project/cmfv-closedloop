import serial
import time
import logging
import sys
import atexit
import os
import struct
import numpy as np
from crc import Calculator, Crc16
from enum import Enum

##########################
#### CONFIGURATIONS ######
##########################
IS_FUEL_SYSTEM = False

OCV_SERIAL_PORT = '/dev/cu.usbmodem11301' # Controller 1, OCV (Oxidizer)
FCV_SERIAL_PORT = '/dev/cu.usbmodem11201' # Controller 2, FCV (Fuel)

# Serial port config
if IS_FUEL_SYSTEM:
    SERIAL_PORT = FCV_SERIAL_PORT # Controller 2, FCV
else:
    SERIAL_PORT = OCV_SERIAL_PORT # Controller 1, OCV

BAUDRATE = 115200

LOG_TELEMETRY_FILE = './telemetry_from_arduino.csv'
LOG_SENT_DUMMY_DATA_FILE = './sent_dummy_data.csv'

PRESSURE_LOOKUP_TABLE_FILE = './pressures_lut_1_28_25.csv'
LUT_ANGLE_RANGE_MIN = 30
LUT_ANGLE_RANGE_MAX = 90
LUT_ANGLE_INTERVAL = 0.5
ASSUMED_ANGLE_OTHER_VAL = 60

DUMMY_PRESSURE_SET_POINT = 601.54

MAGIC_START = b'\xfb\xad'
TELPKT_FMTSTR_WO_MAGIC = '<HBBBBfffff'
TELPKT_SIZE_WO_MAGIC = struct.calcsize(TELPKT_FMTSTR_WO_MAGIC)
# TELPKT flags
TELPKT_MPV_OPEN_DETECTED = 0x1
TELPKT_MPV_SHD_BE_CLOSED = 0x2
TELPKT_SYSTEM_TYPE = 0x4
# TELPKT faults
TELPKT_FAULTS_MANUAL_ABORT = 0x1
TELPKT_FAULTS_COMM_TIMEOUT = 0x2
TELPKT_FAULTS_ENCODER_ERROR = 0x4
TELPKT_FAULTS_NO_MOTION = 0x8
TELPKT_FAULTS_SENSOR_FAULT = 0x10
TELPKT_FAULTS_REDBAND_FAULT = 0x20
TELPKT_FAULTS_OSCILLATION_DETECTED = 0x80

UPDTPKT_FMTSTR_WO_MAGIC = '<HBBHff'
UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM = '<BBHff'
UPDTPKT_SIZE_WO_MAGIC = struct.calcsize(UPDTPKT_FMTSTR_WO_MAGIC)
# UPDTPKT flags
UPDPKT_IF_MPV_OPEN = 0x1

class SystemState(Enum):
    BOOT_INIT = 0
    OPEN_LOOP_INIT = 1
    CLOSED_LOOP = 2
    FORCED_OPEN_LOOP = 3
    EMERGENCY_STOP = 4

CONTROLLER_TELEMETRY_RATE = 5   # Controller sends telemetry (Hz); used for status cadence only
PRESSURE_SEND_RATE = 200        # Pi sends pressure data to controller (Hz)

######################
#### LOGGER SETUP ####
######################
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    stream=sys.stdout
)

###########################
#### CREATE LOGS DIR ######
###########################
log_telemetry_dir = os.path.dirname(LOG_TELEMETRY_FILE)
log_sent_data_dir = os.path.dirname(LOG_SENT_DUMMY_DATA_FILE)

os.makedirs(log_telemetry_dir, exist_ok=True)
os.makedirs(log_sent_data_dir, exist_ok=True)

#######################
#### SERIAL SETUP #####
#######################
serial_port = None
open_delay_s = 1.0

while True:
    try:
        time.sleep(open_delay_s)
        serial_port = serial.Serial(
            SERIAL_PORT,
            BAUDRATE,
            timeout=0.0,         
            write_timeout=0.02   
        )
        break
    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt! Exiting...")
        sys.exit(1)
    except Exception as e:
        logging.exception("Failure to bind serial port; retrying...")
        open_delay_s = min(open_delay_s * 1.5, 5.0)

##############################
##### LOOKUP TABLE SETUP #####
##############################
lookup_table = np.genfromtxt(PRESSURE_LOOKUP_TABLE_FILE, dtype='float', delimiter=',', names=True)
num_angles_one_valve = int((LUT_ANGLE_RANGE_MAX - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL) + 1
num_angles_till_assumed_angle = int((ASSUMED_ANGLE_OTHER_VAL - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL)

if IS_FUEL_SYSTEM: 
    lookup_table_truncated = lookup_table[num_angles_one_valve * num_angles_till_assumed_angle : num_angles_one_valve * (num_angles_till_assumed_angle + 1)]
else:
    lookup_table_truncated = [lookup_table[num_angles_one_valve * i + num_angles_till_assumed_angle] for i in range(num_angles_one_valve)]

##########################
#### LOG FILE SETUP ######
##########################
# Default buffering; we'll flush periodically ourselves
log_telemetry = open(LOG_TELEMETRY_FILE, 'a')
log_sent_data = open(LOG_SENT_DUMMY_DATA_FILE, 'a')

###################################
#### CHECKSUM CALCULATOR SETUP ####
###################################
crcCalculator = Calculator(Crc16.XMODEM)

##########################
#### RESOURCE CLEANUP ####
##########################
def cleanup():
    global log_telemetry, log_sent_data, serial_port
    try:
        if serial_port and serial_port.is_open:
            logging.info("Closing serial port...")
            serial_port.close()
        if log_telemetry:
            logging.info("Closing telemetry log file...")
            log_telemetry.flush()
            log_telemetry.close()
        if log_sent_data:
            logging.info("Closing dummy sent data log file...")
            log_sent_data.flush()
            log_sent_data.close()
    except Exception as e:
        logging.exception("Cleanup encountered an error")

atexit.register(cleanup)

####################################################
#### TELEMETRY COLLECTION & DATA STREAMING LOOP ####
####################################################
def epoch_ms() -> int:
    return int(time.time_ns() // 1_000_000)  # for logs

from time import monotonic_ns
def now_ms() -> int:
    return int(monotonic_ns() // 1_000_000)  # for scheduling

def parse_telemetry(packet: bytes) -> dict:
    """Telemetry Packet Layout
    0        8       16       24       32 (Bits)
    +--------+--------+--------+--------+
    |   Magic Bytes   |    Checksum     |
    +--------+--------+--------+--------+
    | State  | Flags  |     Unused      |
    +--------+--------+--------+--------+
    |        Current Motor Angle        |
    +--------+--------+--------+--------+
    |        Current Delta Angle        |
    +--------+--------+--------+--------+
    |       Current Integral Error      |
    +--------+--------+--------+--------+
    |      Registered PT1 Reading       |
    +--------+--------+--------+--------+
    |      Registered PT2 Reading       |
    +--------+--------+--------+--------+"""
    if not packet or len(packet) != TELPKT_SIZE_WO_MAGIC:
        return None
    
    try:
        unpacked_data = struct.unpack(TELPKT_FMTSTR_WO_MAGIC, packet)
        if len(unpacked_data) == 10:
            # Validate checksum
            receivedChecksum = unpacked_data[0]
            if receivedChecksum != crcCalculator.checksum(packet[2:]): 
                logging.warning(f"Invalid checksum for packet: {MAGIC_START + packet}")
                return None
            
            # Ensure state is valid, otherwise fall back to logging the previos ctrler state
            try: 
                systemState = SystemState(unpacked_data[1])
            except ValueError as e: 
                logging.warning(f"Invalid controller state received, exception: '{e}'. Keeping current state: {controller_state}")

            return {
                '_checksum': receivedChecksum,
                'systemState': systemState,

                'ifMpvShdBeClosed': unpacked_data[2] & TELPKT_MPV_SHD_BE_CLOSED != 0,
                'ifMpvOpenDetected': unpacked_data[2] & TELPKT_MPV_OPEN_DETECTED != 0,
                'systemType': 'FUEL' if (unpacked_data[2] & TELPKT_SYSTEM_TYPE) != 0 else 'OX',

                'faults.manualAbort': unpacked_data[3] & TELPKT_FAULTS_MANUAL_ABORT != 0,
                'faults.commTimeout': unpacked_data[3] & TELPKT_FAULTS_COMM_TIMEOUT != 0,
                'faults.encoderError': unpacked_data[3] & TELPKT_FAULTS_ENCODER_ERROR != 0,
                'faults.noMotion': unpacked_data[3] & TELPKT_FAULTS_NO_MOTION != 0,
                'faults.sensorFault': unpacked_data[3] & TELPKT_FAULTS_SENSOR_FAULT != 0,
                'faults.redbandFault': unpacked_data[3] & TELPKT_FAULTS_REDBAND_FAULT != 0,
                'faults.oscillationDetected': unpacked_data[3] & TELPKT_FAULTS_OSCILLATION_DETECTED != 0,

                'motorAngle': unpacked_data[5],
                'deltaAngle': unpacked_data[6],
                'pidIntergralError': unpacked_data[7],
                'pt1Reading': unpacked_data[8],
                'pt2Reading': unpacked_data[9]
            }
    except (ValueError, IndexError) as e:
        logging.warning(f"Failed to parse telemetry line '{packet}': {e}")
    
    return None

def craft_pressure_update_packet(other_state: int, if_mpv_open: bool, pt1_reading: float, pt2_reading: float) -> bytes:
    flags = 0
    if (if_mpv_open): flags |= UPDPKT_IF_MPV_OPEN
    pup_wo_magic_and_checksum = struct.pack(UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM, other_state, flags, 0, pt1_reading, pt2_reading)
    checksum = crcCalculator.checksum(pup_wo_magic_and_checksum)
    return MAGIC_START + struct.pack('<H', checksum) + pup_wo_magic_and_checksum

# State management
controller_state = SystemState.BOOT_INIT
simulated_other_state = SystemState.BOOT_INIT

# Simulate that MPV is on
simulated_mpv = True

# Dictionary of telemtry data received from the controller
controller_telemetry = None

# Non-blocking read buffer for partial lines
_rx_buf = bytearray()
cur_packet_wo_magic = bytearray()
currently_receiving = False

def drain_telemetry():
    """Consume all currently buffered bytes; decode and process complete telemtry packets"""
    global _rx_buf, cur_packet_wo_magic, controller_telemetry, currently_receiving
    drained = 0
    try:
        waiting = serial_port.in_waiting
    except Exception:
        waiting = 0

    if waiting > 0:
        try:
            chunk = serial_port.read(waiting)
            if chunk:
                if currently_receiving:
                    to_receive = TELPKT_SIZE_WO_MAGIC - len(cur_packet_wo_magic)
                    if len(chunk) >= to_receive: 
                        cur_packet_wo_magic.extend(chunk[:to_receive])
                        chunk = chunk[to_receive:]
                        controller_telemetry = parse_telemetry(bytes(cur_packet_wo_magic))
                        cur_packet_wo_magic.clear()
                        currently_receiving = False
                    else:
                        cur_packet_wo_magic.extend(chunk)
                        chunk = b''
                _rx_buf.extend(chunk)
        except Exception as e:
            logging.debug(f"Serial read chunk failed: {e}")
            return 0

    while True:
        pos = _rx_buf.find(MAGIC_START)
        if pos == -1:
            break
        del _rx_buf[:pos+2]  # Also drop MAGIC_START
        if len(_rx_buf) >= TELPKT_SIZE_WO_MAGIC:
            cur_packet_wo_magic = _rx_buf[:TELPKT_SIZE_WO_MAGIC]
            del _rx_buf[:TELPKT_SIZE_WO_MAGIC]
            controller_telemetry = parse_telemetry(bytes(cur_packet_wo_magic))
            cur_packet_wo_magic.clear()
        else:
            cur_packet_wo_magic = _rx_buf[:]
            _rx_buf.clear()
            currently_receiving = True
        ts = epoch_ms()
        log_telemetry.write(f"{ts}: {str(controller_telemetry)}\n")
        drained += 1
    return drained

# Scheduling using monotonic time
send_period_ms = max(1, int(1000 / PRESSURE_SEND_RATE))
flush_period_ms = 1000
status_period_ms = max(1, int(1000 / max(1, min(PRESSURE_SEND_RATE, CONTROLLER_TELEMETRY_RATE))))

next_send = now_ms() + send_period_ms
next_flush = now_ms() + flush_period_ms
next_status = now_ms() + status_period_ms

start_mono_ms = now_ms()

try:
    serial_port.reset_input_buffer()
except Exception:
    pass

cur_pressure = 0
pressure_data_str = None

while True:
    try:
        curr_mono = now_ms()

        # Drain any available telemetry without blocking
        _ = drain_telemetry()

        # Simulate other controller state based on monotonic time
        elapsed = curr_mono - start_mono_ms
        prev_sim_state = simulated_other_state
        # if 15000 <= elapsed < 20000:
        #     simulated_other_state = SystemState.FORCED_OPEN_LOOP
        if elapsed < 3000:
            simulated_other_state = SystemState.OPEN_LOOP_INIT
        else:
            simulated_other_state = SystemState.CLOSED_LOOP

        if curr_mono >= next_send:

            # Use LUT to generate pressures
            if controller_telemetry:
                motor_angle_rounded_clamped = min(LUT_ANGLE_RANGE_MAX, max(LUT_ANGLE_RANGE_MIN, round(controller_telemetry['motorAngle'] * 2) / 2))
                cur_pressure = lookup_table_truncated[int((motor_angle_rounded_clamped - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL)][3 if IS_FUEL_SYSTEM else 2]

                controller_state = controller_telemetry['systemState']
            
            try:
                pressure_update_packet = craft_pressure_update_packet(simulated_other_state.value, 
                                                                      simulated_mpv, 
                                                                      cur_pressure, 
                                                                      cur_pressure)
                serial_port.write(pressure_update_packet)
                pressure_data_str = f"P,{cur_pressure:.2f},{cur_pressure:.2f},{simulated_other_state},{simulated_mpv}"
                log_sent_data.write(f"{epoch_ms()},{pressure_data_str}\n")
            except serial.SerialException:
                logging.exception("Serial write failed")
            except Exception:
                logging.exception("Unexpected error during serial write")

            # schedule next tick and catch up if we're behind
            next_send += send_period_ms
            if curr_mono - next_send > 5 * send_period_ms:
                next_send = curr_mono + send_period_ms

        # Flush logs periodically
        if curr_mono >= next_flush:
            try:
                log_telemetry.flush()
                log_sent_data.flush()
            except Exception:
                logging.exception("Log flush failed")
            next_flush += flush_period_ms

        # Periodic status print
        if curr_mono >= next_status:
            print(f"Received: {str(controller_telemetry) if controller_telemetry else 'No telemetry yet'}")
            print(f"Last Sent: {pressure_data_str if pressure_data_str else 'Not sent yet'}")
            print(f"Controller State: {controller_state}")
            print(f"Simulated Other State: {simulated_other_state}")
            next_status += status_period_ms

    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt! Exiting...")
        sys.exit(0)
    except Exception as e:
        logging.exception("Unexpected error in main loop")
        # If nothing was received recently, emit a lightweight marker
        if len(_rx_buf) == 0:
            logging.error("PING")
        # brief backoff to avoid tight error loop
        time.sleep(0.01)
