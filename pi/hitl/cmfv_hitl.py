"""CSV-driven two-controller HITL, based on cmfv2.py.

CSV layout: header followed by OCV angle, FCV angle, oxidizer pressure,
fuel pressure; FCV varies fastest, angles 30..90 degrees in 0.5 steps.
Requires pyserial, numpy, crc and synnax. No PT boards are opened.
"""
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
import synnax as sy

##########################
#### CONFIGURATIONS ######
##########################

##### MAY NEED TO CHANGE THESE #####
USE_3_PTS = False

# Serial port config for both controllers
# ; FCV
# upload_port = /dev/cu.usbmodem11301
# ; OCV
# ; upload_port = /dev/cu.usbmodem11201

OCV_SERIAL_PORT = '/dev/cu.usbmodem11201' # Controller 1, OCV (Oxidizer)
FCV_SERIAL_PORT = '/dev/cu.usbmodem11301' # Controller 2, FCV (Fuel)
ARDUINO_BOARD_BAUDRATE = 115200

FCV_SETPOINT = 620.0
OCV_SETPOINT = 580.0

LOG_TELEMETRY_FILE = './logs/cmfv_hitl/telemetry_from_both_controllers.csv'
LOG_SENT_DUMMY_DATA_FILE = './logs/cmfv_hitl/sent_pressure_data_both_controllers.csv'

PRESSURE_LOOKUP_TABLE_FILE = '/Users/Nik/cmfv-closedloop/pi/hitl/pressures_lut_1_28_25.csv'
LUT_ANGLE_RANGE_MIN = 30
LUT_ANGLE_RANGE_MAX = 90
LUT_ANGLE_INTERVAL = 0.5

# Default angles when no telemetry is available
DEFAULT_OCV_ANGLE = 58 
DEFAULT_FCV_ANGLE = 63.5

DUMMY_PRESSURE_SET_POINT = 601.54

MAGIC_START = b'\xfb\xad'
TELPKT_FMTSTR_WO_MAGIC = '<HBBBBfffff'
TELPKT_FMTSTR_WO_MAGIC += 'I'
TELPKT_FMTSTR_WO_MAGIC += 'I' #timestamp

# if USE_3_PTS:
#     TELPKT_FMTSTR_WO_MAGIC += 'I'
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

UPDTPKT_FMTSTR_WO_MAGIC += 'I'
UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM += 'I'

# if USE_3_PTS:
#     UPDTPKT_FMTSTR_WO_MAGIC += 'f'
#     UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM += 'f'

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

SOFTWARE_LPF_WEIGHTING_FACTOR = 0.3

MPV_CHANNEL_NAME = "controls_state_11"
isMPVOpen = False

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

lookup_table = np.atleast_1d(np.genfromtxt(PRESSURE_LOOKUP_TABLE_FILE, dtype='float', delimiter=',', names=True))
num_angles_one_valve = int((LUT_ANGLE_RANGE_MAX - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL) + 1

if len(lookup_table) != num_angles_one_valve ** 2 or len(lookup_table.dtype.names or ()) < 4:
    raise ValueError("CSV must contain the complete angle grid and at least four columns")
if not all(np.isfinite(lookup_table[name]).all() for name in lookup_table.dtype.names[:4]):
    raise ValueError("CSV angles and pressures must be finite")

def get_pressures_from_two_angles(ocv_angle, fcv_angle):
    # Round to nearest 0.5 and clamp within range
    ocv_angle_rounded = min(LUT_ANGLE_RANGE_MAX, max(LUT_ANGLE_RANGE_MIN, round(ocv_angle * 2) / 2))
    fcv_angle_rounded = min(LUT_ANGLE_RANGE_MAX, max(LUT_ANGLE_RANGE_MIN, round(fcv_angle * 2) / 2))
    
    # Calculate indices for both angles
    ocv_index = int((ocv_angle_rounded - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL)
    fcv_index = int((fcv_angle_rounded - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL)
    
    # Ensure indices are within bounds
    ocv_index = min(max(0, ocv_index), num_angles_one_valve - 1)
    fcv_index = min(max(0, fcv_index), num_angles_one_valve - 1)
    
    # Calculate flat index in the lookup table 
    # Based on the table structure: FCV angle varies faster (inner loop), OCV angle varies slower (outer loop)
    table_index = ocv_index * num_angles_one_valve + fcv_index
    
    # Get the row from lookup table and extract pressures
    if table_index < len(lookup_table):
        row = lookup_table[table_index]
        ox_pressure = row[2]  
        fuel_pressure = row[3]  
        return ox_pressure, fuel_pressure
    else:
        # Fallback if index is out of bounds
        logging.warning(f"Lookup table index {table_index} out of bounds for angles OCV={ocv_angle_rounded}, FCV={fcv_angle_rounded}")
        return DUMMY_PRESSURE_SET_POINT, DUMMY_PRESSURE_SET_POINT


#######################
#### SERIAL SETUP #####
#######################
ocv_serial_port = None  
fcv_serial_port = None  
open_delay_s = 1.0

def setup_serial_port(port_name, port_description, baudrate):
    serial_port = None
    local_delay = open_delay_s
    
    while True:
        try:
            time.sleep(local_delay)
            serial_port = serial.Serial(
                port_name,
                baudrate,
                timeout=0.0,         
                write_timeout=0.02   
            )
            logging.info(f"Successfully connected to {port_description}")
            break
        except KeyboardInterrupt:
            logging.info("Caught KeyboardInterrupt! Exiting...")
            sys.exit(1)
        except Exception as e:
            logging.exception(f"Failure to bind {port_description}; retrying...")
            local_delay = min(local_delay * 1.5, 5.0)
    
    return serial_port

# Setup all serial ports
logging.info("Setting up OCV (Oxidizer) controller connection...")
ocv_serial_port = setup_serial_port(OCV_SERIAL_PORT, "OCV Controller", ARDUINO_BOARD_BAUDRATE)

logging.info("Setting up FCV (Fuel) controller connection...")
fcv_serial_port = setup_serial_port(FCV_SERIAL_PORT, "FCV Controller", ARDUINO_BOARD_BAUDRATE)

##############################
#### SYNNAX CHANNEL SETUP ####
##############################
# Authenticate credentials for cluster
client = sy.Synnax(
    host="localhost",
    port=9091,
    username="synnax",
    password="seldon",
    secure=False
)

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
    global log_telemetry, log_sent_data, ocv_serial_port, fcv_serial_port, streamer
    try:
        if ocv_serial_port and ocv_serial_port.is_open:
            logging.info("Closing OCV serial port...")
            ocv_serial_port.close()
        if fcv_serial_port and fcv_serial_port.is_open:
            logging.info("Closing FCV serial port...")
            fcv_serial_port.close()
        if log_telemetry:
            logging.info("Closing telemetry log file...")
            log_telemetry.flush()
            log_telemetry.close()
        if log_sent_data:
            logging.info("Closing dummy sent data log file...")
            log_sent_data.flush()
            log_sent_data.close()
        if 'streamer' in globals():
            logging.info("Closing Synnax streamer...")
            try:
                if streamer is not None:
                    streamer.close()
            except Exception:
                pass
        try:
            client.close()
        except Exception:
            pass
    except Exception as e:
        logging.exception("Cleanup encountered an error")

atexit.register(cleanup)

####################################################
#### TELEMETRY COLLECTION & DATA STREAMING LOOP ####
####################################################
def epoch_ms() -> int:
    return int(time.time_ns() // 1_000_000)

from time import monotonic_ns
def now_ms() -> int:
    return int(monotonic_ns() // 1_000_000)

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
    +--------+--------+--------+--------+
    
    If we are using 3 PTs, then there will be a third registered PT
    reading (float, 4 bytes) appended to the packet. """
    if not packet or len(packet) != TELPKT_SIZE_WO_MAGIC:
        return None
    
    try:
        unpacked_data = struct.unpack(TELPKT_FMTSTR_WO_MAGIC, packet)
        if len(unpacked_data) == len(TELPKT_FMTSTR_WO_MAGIC) - 1:
            # Validate checksum
            receivedChecksum = unpacked_data[0]
            if receivedChecksum != crcCalculator.checksum(packet[2:]): 
                logging.warning(f"Invalid checksum for packet: {MAGIC_START + packet}")
                return None
            
            # Ensure state is valid, otherwise fall back to the previous controller state
            try: 
                systemState = SystemState(unpacked_data[1])
            except ValueError as e: 
                logging.warning(f"Invalid controller state received, exception: '{e}'. Discarding packet.")
                return None

            to_return = {
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
            # if USE_3_PTS: 
            #     to_return['pt3Reading'] = unpacked_data[10]

            to_return['packetNo'] = unpacked_data[10]
            to_return['timestamp'] = unpacked_data[11]
            return to_return
    except (ValueError, IndexError) as e:
        logging.warning(f"Failed to parse telemetry line '{packet}': {e}")
    
    return None

def craft_pressure_update_packet(other_state: int, if_mpv_open: bool, pt1_reading: float, pt2_reading: float, packetNo: int) -> bytes:
    flags = 0
    if (if_mpv_open): flags |= UPDPKT_IF_MPV_OPEN
    # if (pt3_reading != None):
    #     pup_wo_magic_and_checksum = struct.pack(UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM, other_state, flags, 0, pt1_reading, pt2_reading, pt3_reading)
    # else:
    pup_wo_magic_and_checksum = struct.pack(UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM, other_state, flags, 0, pt1_reading, pt2_reading, packetNo)
    checksum = crcCalculator.checksum(pup_wo_magic_and_checksum)
    return MAGIC_START + struct.pack('<H', checksum) + pup_wo_magic_and_checksum

# State management for both controllers
ocv_controller_state = SystemState.BOOT_INIT
fcv_controller_state = SystemState.BOOT_INIT

# Dictionary of telemetry data received from each controller
ocv_controller_telemetry = None
fcv_controller_telemetry = None

# Non-blocking read buffers for both controllers
ocv_rx_buf = bytearray()
ocv_cur_packet_wo_magic = bytearray()
ocv_currently_receiving = False

fcv_rx_buf = bytearray()
fcv_cur_packet_wo_magic = bytearray()
fcv_currently_receiving = False

def parse_service_motor_data(raw: bytes):
    start, packet_data, packet_len, end = struct.unpack(
        "<I24sII", raw
    )
    if start != 0xDEAD or end != 0xBEEF or packet_len > 24:
        return None, 0
    return packet_data[:packet_len], packet_len

def drain_telemetry_from_port(serial_port, rx_buf, cur_packet_wo_magic, currently_receiving, controller_name):
    """Consume all currently buffered bytes from a specific port; decode and process complete telemetry packets"""
    drained = 0
    latest_telemetry = None 
    
    try:
        waiting = serial_port.in_waiting
    except Exception as e:
        logging.debug(f"Failed to check in_waiting for {controller_name}: {e}")
        waiting = 0

    if waiting > 0:
        try:
            chunk = serial_port.read(waiting)
            if chunk:
                if currently_receiving:
                    to_receive = TELPKT_SIZE_WO_MAGIC - len(cur_packet_wo_magic)
                    if len(chunk) >= to_receive:
                        # complete the pending packet
                        cur_packet_wo_magic.extend(chunk[:to_receive])
                        chunk = chunk[to_receive:]
                        telemetry = parse_telemetry(bytes(cur_packet_wo_magic))
                        cur_packet_wo_magic.clear()
                        currently_receiving = False
                        if telemetry:
                            ts = epoch_ms()
                            log_telemetry.write(f"{ts},{controller_name}: {str(telemetry)}\n")
                            latest_telemetry = telemetry
                        # test if it is a debug packet and log it

                        byte_dump, packet_len = parse_service_motor_data(cur_packet_wo_magic)

                        if byte_dump is not None and packet_len > 0:
                            with open("out.txt", "a") as file:
                                file.write(f"packetLen: {packet_len} | ")
                                file.write(f"packetData: {bytes(byte_dump[:packet_len]).hex(' ')}\n\n")

                            # return some old values as garbage for now...
                            return latest_telemetry, drained, currently_receiving

                    else:
                        # still waiting for more bytes
                        cur_packet_wo_magic.extend(chunk)
                        chunk = b''
                if chunk:
                    rx_buf.extend(chunk)
        except Exception as e:
            logging.debug(f"Serial read chunk failed for {controller_name}: {e}")
            return latest_telemetry, drained, currently_receiving

    while True:
        pos = rx_buf.find(MAGIC_START)
        if pos == -1:
            break
        del rx_buf[:pos+2]  # Also drop MAGIC_START
        if len(rx_buf) >= TELPKT_SIZE_WO_MAGIC:
            cur_packet_wo_magic.clear()
            cur_packet_wo_magic.extend(rx_buf[:TELPKT_SIZE_WO_MAGIC])
            del rx_buf[:TELPKT_SIZE_WO_MAGIC]
            telemetry = parse_telemetry(bytes(cur_packet_wo_magic))
            cur_packet_wo_magic.clear()
            currently_receiving = False
            if telemetry:
                ts = epoch_ms()
                log_telemetry.write(f"{ts},{controller_name}: {str(telemetry)}\n")
                latest_telemetry = telemetry  
            drained += 1
        else:
            cur_packet_wo_magic.clear()
            cur_packet_wo_magic.extend(rx_buf[:])
            rx_buf.clear()
            currently_receiving = True
            break  
    
    return latest_telemetry, drained, currently_receiving

# def drain_telemetry_from_port(serial_port, rx_buf, cur_packet_wo_magic, currently_receiving, controller_name):
#     """Consume all currently buffered bytes from a specific port; decode and process complete telemetry packets"""
#     drained = 0
#     latest_telemetry = None 
    
#     try:
#         waiting = serial_port.in_waiting
#     except Exception as e:
#         logging.debug(f"Failed to check in_waiting for {controller_name}: {e}")
#         waiting = 0

#     if waiting > 0:
#         try:
#             chunk = serial_port.read(waiting)
#             if chunk:
#                 if currently_receiving:
#                     to_receive = TELPKT_SIZE_WO_MAGIC - len(cur_packet_wo_magic)
#                     if len(chunk) >= to_receive:
#                         # complete the pending packet
#                         cur_packet_wo_magic.extend(chunk[:to_receive])
#                         chunk = chunk[to_receive:]
#                         telemetry = parse_telemetry(bytes(cur_packet_wo_magic))
#                         cur_packet_wo_magic.clear()
#                         currently_receiving = False
#                         if telemetry:
#                             ts = epoch_ms()
#                             log_telemetry.write(f"{ts},{controller_name}: {str(telemetry)}\n")
#                             latest_telemetry = telemetry 
#                     else:
#                         # still waiting for more bytes
#                         cur_packet_wo_magic.extend(chunk)
#                         chunk = b''
#                 if chunk:
#                     rx_buf.extend(chunk)
#         except Exception as e:
#             logging.debug(f"Serial read chunk failed for {controller_name}: {e}")
#             return latest_telemetry, drained, currently_receiving

#     while True:
#         pos = rx_buf.find(MAGIC_START)
#         if pos == -1:
#             break
#         del rx_buf[:pos+2]  # Also drop MAGIC_START
#         if len(rx_buf) >= TELPKT_SIZE_WO_MAGIC:
#             cur_packet_wo_magic.clear()
#             cur_packet_wo_magic.extend(rx_buf[:TELPKT_SIZE_WO_MAGIC])
#             del rx_buf[:TELPKT_SIZE_WO_MAGIC]
#             telemetry = parse_telemetry(bytes(cur_packet_wo_magic))
#             cur_packet_wo_magic.clear()
#             currently_receiving = False
#             if telemetry:
#                 ts = epoch_ms()
#                 log_telemetry.write(f"{ts},{controller_name}: {str(telemetry)}\n")
#                 latest_telemetry = telemetry  
#             drained += 1
#         else:
#             cur_packet_wo_magic.clear()
#             cur_packet_wo_magic.extend(rx_buf[:])
#             rx_buf.clear()
#             currently_receiving = True
#             break  
    
#     return latest_telemetry, drained, currently_receiving

def drain_telemetry():
    """Drain telemetry from both controllers"""
    global ocv_controller_telemetry, fcv_controller_telemetry
    global ocv_currently_receiving, fcv_currently_receiving
    
    # Drain from OCV controller
    new_ocv_telemetry, ocv_drained, ocv_currently_receiving = drain_telemetry_from_port(
        ocv_serial_port, ocv_rx_buf, ocv_cur_packet_wo_magic, ocv_currently_receiving, "OCV"
    )
    if new_ocv_telemetry:
        ocv_controller_telemetry = new_ocv_telemetry
        # Update controller state from telemetry if valid
        if 'systemState' in new_ocv_telemetry:
            global ocv_controller_state
            ocv_controller_state = new_ocv_telemetry['systemState']
    
    # Drain from FCV controller
    new_fcv_telemetry, fcv_drained, fcv_currently_receiving = drain_telemetry_from_port(
        fcv_serial_port, fcv_rx_buf, fcv_cur_packet_wo_magic, fcv_currently_receiving, "FCV"
    )
    if new_fcv_telemetry:
        fcv_controller_telemetry = new_fcv_telemetry
        # Update controller state from telemetry if valid
        if 'systemState' in new_fcv_telemetry:
            global fcv_controller_state
            fcv_controller_state = new_fcv_telemetry['systemState']
    
    return ocv_drained + fcv_drained


# Scheduling using monotonic time
send_period_ms = max(1, int(1000 / PRESSURE_SEND_RATE))
flush_period_ms = 1000
status_period_ms = max(1, int(1000 / max(1, min(PRESSURE_SEND_RATE, CONTROLLER_TELEMETRY_RATE))))

next_send = now_ms() + send_period_ms
next_flush = now_ms() + flush_period_ms
next_status = now_ms() + status_period_ms

global_start_ms = epoch_ms()

try:
    ocv_serial_port.reset_input_buffer()
    fcv_serial_port.reset_input_buffer()
except Exception:
    pass

# Initialize pressure tracking
current_ox_pressure_pt1 = None
current_ox_pressure_pt2 = None
current_ox_pressure_pt3 = None
current_fuel_pressure_pt1 = None
current_fuel_pressure_pt2 = None
current_fuel_pressure_pt3 = None
pressure_data_str = None

# Open Synnax streamer once outside the loop
streamer = None
try:
    streamer = client.open_streamer([MPV_CHANNEL_NAME])
except Exception as e:
    logging.exception("Failed to open Synnax streamer; proceeding with MPV assumed CLOSED.")

start_time = int(time.time())

packetNo = 0

while True:
    try:
        curr_mono = now_ms()

        # Drain any available telemetry from controllers
        _ = drain_telemetry()

        # Read MPV signal from Synnax 
        try:
            if streamer is not None:
                frame = streamer.read(timeout=0)
                if frame is not None and len(frame[MPV_CHANNEL_NAME]) > 0:
                    isMPVOpen = np.uint8(frame[MPV_CHANNEL_NAME][-1]) == 1
        except Exception as e:
            logging.debug(f"Failed to read MPV signal: {e}")


        if ocv_controller_telemetry is None:
            ocv_controller_state = SystemState.BOOT_INIT
        if fcv_controller_telemetry is None:
            fcv_controller_state = SystemState.BOOT_INIT

        if curr_mono >= next_send and ocv_controller_telemetry and fcv_controller_telemetry:
            packetNo = (packetNo + 1) & 0xFFFFFFFF
            ocv_angle = ocv_controller_telemetry['motorAngle']
            fcv_angle = fcv_controller_telemetry['motorAngle']
            current_ox_pressure, current_fuel_pressure = get_pressures_from_two_angles(ocv_angle, fcv_angle)
            current_ox_pressure_pt1 = current_ox_pressure_pt2 = current_ox_pressure
            current_fuel_pressure_pt1 = current_fuel_pressure_pt2 = current_fuel_pressure

            # Determine what to send to each controller
            ocv_pt1 = current_ox_pressure_pt1
            ocv_pt2 = current_ox_pressure_pt2
            ocv_pt3 = current_ox_pressure_pt3 if USE_3_PTS else None
            fcv_pt1 = current_fuel_pressure_pt1
            fcv_pt2 = current_fuel_pressure_pt2
            fcv_pt3 = current_fuel_pressure_pt3 if USE_3_PTS else None
            
            # Get other controller's state (use BOOT_INIT if not available)
            fcv_state_for_ocv = fcv_controller_state.value if fcv_controller_telemetry else SystemState.BOOT_INIT.value
            ocv_state_for_fcv = ocv_controller_state.value if ocv_controller_telemetry else SystemState.BOOT_INIT.value

            # Get current angles for logging (with defaults if no telemetry)
            ocv_angle = ocv_controller_telemetry['motorAngle'] if ocv_controller_telemetry else DEFAULT_OCV_ANGLE
            fcv_angle = fcv_controller_telemetry['motorAngle'] if fcv_controller_telemetry else DEFAULT_FCV_ANGLE

            # Send pressure updates to both controllers
            try:
                ocv_pressure_packet = craft_pressure_update_packet(
                    fcv_state_for_ocv,  
                    isMPVOpen, 
                    ocv_pt1,  
                    ocv_pt2,
                    packetNo
                )
                ocv_serial_port.write(ocv_pressure_packet)
                
                fcv_pressure_packet = craft_pressure_update_packet(
                    ocv_state_for_fcv,  
                    isMPVOpen, 
                    fcv_pt1, 
                    fcv_pt2,
                    packetNo
                )
                fcv_serial_port.write(fcv_pressure_packet)
                
                if USE_3_PTS:
                    pressure_data_str = f"OCV_PT1,{ocv_pt1:.2f},OCV_PT2,{ocv_pt2:.2f},OCV_PT3,{ocv_pt3:.2f},FCV_PT1,{fcv_pt1:.2f},FCV_PT2,{fcv_pt2:.2f},FCV_PT3,{fcv_pt3:.2f},OCV_angle,{ocv_angle:.1f},FCV_angle,{fcv_angle:.1f},MPV_Open,{isMPVOpen},OCV_state,{ocv_state_for_fcv},FCV_state,{fcv_state_for_ocv}"
                else:
                    pressure_data_str = f"OCV_PT1,{ocv_pt1:.2f},OCV_PT2,{ocv_pt2:.2f},FCV_PT1,{fcv_pt1:.2f},FCV_PT2,{fcv_pt2:.2f},OCV_angle,{ocv_angle:.1f},FCV_angle,{fcv_angle:.1f},MPV_Open,{isMPVOpen},OCV_state,{ocv_state_for_fcv},FCV_state,{fcv_state_for_ocv}"
                log_sent_data.write(f"{epoch_ms()},{pressure_data_str},packetNo,{packetNo}\n")
                
            except serial.SerialException as e:
                logging.exception(f"Serial write failed: {e}")
            except Exception as e:
                logging.exception(f"Unexpected error during serial write: {e}")

        if curr_mono >= next_send:
            # schedule next tick and catch up if we're behind
            next_send += send_period_ms
            if curr_mono - next_send > 5 * send_period_ms:
                next_send = curr_mono + send_period_ms

        # Flush logs periodically
        if curr_mono >= next_flush:
            try:
                for f in (log_telemetry, log_sent_data):
                    try:
                        f.flush()
                    except Exception:
                        logging.exception("Log flush failed for one file")
            finally:
                next_flush += flush_period_ms


        # Periodic status print
        if curr_mono >= next_status:
            ocv_status = str(ocv_controller_telemetry) if ocv_controller_telemetry else 'No telemetry yet'
            fcv_status = str(fcv_controller_telemetry) if fcv_controller_telemetry else 'No telemetry yet'
            
            print(f"OCV Received: {ocv_status}")
            print(f"FCV Received: {fcv_status}")
            print(f"Last Sent: {pressure_data_str if pressure_data_str else 'Not sent yet'}")
            print(f"OCV Controller State: {ocv_controller_state}")
            print(f"FCV Controller State: {fcv_controller_state}")
            print(f"MPV State: {'OPEN' if isMPVOpen else 'CLOSED'}")
            print(f"Packet Number: {packetNo}")
            next_status += status_period_ms

    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt! Exiting...")
        sys.exit(0)
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
        time.sleep(0.01)
