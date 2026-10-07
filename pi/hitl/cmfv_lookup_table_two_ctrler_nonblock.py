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

# Serial port config for both controllers
OCV_SERIAL_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_24230303637351C09181-if00' # Controller 1, OCV (Oxidizer)
FCV_SERIAL_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_75135333636351C001C0-if00' # Controller 2, FCV (Fuel)

BAUDRATE = 115200

LOG_TELEMETRY_FILE = '/home/ares_gs/synnax_node/scripts/logs/cmfv_hitl/telemetry_from_both_controllers.csv'
LOG_SENT_DUMMY_DATA_FILE = '/home/ares_gs/synnax_node/scripts/logs/cmfv_hitl/sent_pressure_data_both_controllers.csv'

PRESSURE_LOOKUP_TABLE_FILE = '/home/ares_gs/synnax_node/scripts/hitl/cmfv/pressures_lut_1_28_25.csv'
LUT_ANGLE_RANGE_MIN = 30
LUT_ANGLE_RANGE_MAX = 90
LUT_ANGLE_INTERVAL = 0.5

# Default angles when no telemetry is available
DEFAULT_OCV_ANGLE = 45  
DEFAULT_FCV_ANGLE = 45

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
ocv_serial_port = None  
fcv_serial_port = None  
open_delay_s = 1.0

def setup_serial_port(port_name, port_description):
    serial_port = None
    local_delay = open_delay_s
    
    while True:
        try:
            time.sleep(local_delay)
            serial_port = serial.Serial(
                port_name,
                BAUDRATE,
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

# Setup both serial ports
logging.info("Setting up OCV (Oxidizer) controller connection...")
ocv_serial_port = setup_serial_port(OCV_SERIAL_PORT, "OCV Controller")

logging.info("Setting up FCV (Fuel) controller connection...")
fcv_serial_port = setup_serial_port(FCV_SERIAL_PORT, "FCV Controller")

##############################
##### LOOKUP TABLE SETUP #####
##############################
lookup_table = np.genfromtxt(PRESSURE_LOOKUP_TABLE_FILE, dtype='float', delimiter=',', names=True)
num_angles_one_valve = int((LUT_ANGLE_RANGE_MAX - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL) + 1

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
    global log_telemetry, log_sent_data, ocv_serial_port, fcv_serial_port
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
            
            # Ensure state is valid, otherwise fall back to the previous controller state
            try: 
                systemState = SystemState(unpacked_data[1])
            except ValueError as e: 
                logging.warning(f"Invalid controller state received, exception: '{e}'. Discarding packet.")
                return None

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

def drain_telemetry_from_port(serial_port, rx_buf, cur_packet_wo_magic, currently_receiving, controller_name):
    """Consume all currently buffered bytes from a specific port; decode and process complete telemetry packets"""
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
                        # complete the pending packet
                        cur_packet_wo_magic.extend(chunk[:to_receive])
                        chunk = chunk[to_receive:]
                        telemetry = parse_telemetry(bytes(cur_packet_wo_magic))
                        cur_packet_wo_magic.clear()
                        currently_receiving = False
                        if telemetry:
                            ts = epoch_ms()
                            log_telemetry.write(f"{ts},{controller_name}: {str(telemetry)}\n")
                        drained += 1
                    else:
                        # still waiting for more bytes
                        cur_packet_wo_magic.extend(chunk)
                        chunk = b''
                if chunk:
                    rx_buf.extend(chunk)
        except Exception as e:
            logging.debug(f"Serial read chunk failed for {controller_name}: {e}")
            return None, 0, currently_receiving

    telemetry = None
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
        else:
            cur_packet_wo_magic.clear()
            cur_packet_wo_magic.extend(rx_buf[:])
            rx_buf.clear()
            currently_receiving = True
        if telemetry:
            ts = epoch_ms()
            log_telemetry.write(f"{ts},{controller_name}: {str(telemetry)}\n")
        drained += 1
    
    return telemetry, drained, currently_receiving

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

start_mono_ms = now_ms()

try:
    ocv_serial_port.reset_input_buffer()
    fcv_serial_port.reset_input_buffer()
except Exception:
    pass

# Initialize pressure tracking
current_ox_pressure = 0.0
current_fuel_pressure = 0.0
pressure_data_str = None

while True:
    try:
        curr_mono = now_ms()

        # Drain any available telemetry without blocking
        _ = drain_telemetry()

        # Controller states are updated from telemetry in drain_telemetry()
        # Only set initial states if no telemetry has been received yet
        if ocv_controller_telemetry is None:
            ocv_controller_state = SystemState.BOOT_INIT
        if fcv_controller_telemetry is None:
            fcv_controller_state = SystemState.BOOT_INIT

        if curr_mono >= next_send:
            # Wait for first valid telemetry from both controllers before sending pressures
            if ocv_controller_telemetry is None or fcv_controller_telemetry is None:
                next_send += send_period_ms
                if curr_mono - next_send > 5 * send_period_ms:
                    next_send = curr_mono + send_period_ms
                # skip this tick until both are online
                continue


            # Get current angles from both controllers, with defaults if no telemetry
            ocv_angle = DEFAULT_OCV_ANGLE
            fcv_angle = DEFAULT_FCV_ANGLE
            
            if ocv_controller_telemetry:
                ocv_angle = ocv_controller_telemetry['motorAngle']
            if fcv_controller_telemetry:
                fcv_angle = fcv_controller_telemetry['motorAngle']

            # Use both angles as inputs to get both pressures (simple approach)
            current_ox_pressure, current_fuel_pressure = get_pressures_from_two_angles(ocv_angle, fcv_angle)

            # Send pressure updates to both controllers
            try:
                ocv_pressure_packet = craft_pressure_update_packet(
                    fcv_controller_state.value,  
                    True, 
                    current_ox_pressure,  
                    current_ox_pressure  
                )
                ocv_serial_port.write(ocv_pressure_packet)
                
                # Send to FCV controller (fuel system)
                fcv_pressure_packet = craft_pressure_update_packet(
                    ocv_controller_state.value,  
                    True, 
                    current_fuel_pressure, 
                    current_fuel_pressure 
                )
                fcv_serial_port.write(fcv_pressure_packet)
                
                pressure_data_str = f"OCV_P,{current_ox_pressure:.2f},FCV_P,{current_fuel_pressure:.2f},OCV_angle,{ocv_angle:.1f},FCV_angle,{fcv_angle:.1f}"
                log_sent_data.write(f"{epoch_ms()},{pressure_data_str}\n")
                
            except serial.SerialException as e:
                logging.exception(f"Serial write failed: {e}")
            except Exception as e:
                logging.exception(f"Unexpected error during serial write: {e}")

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
            print(f"OCV Received: {str(ocv_controller_telemetry) if ocv_controller_telemetry else 'No telemetry yet'}")
            print(f"FCV Received: {str(fcv_controller_telemetry) if fcv_controller_telemetry else 'No telemetry yet'}")
            print(f"Last Sent: {pressure_data_str if pressure_data_str else 'Not sent yet'}")
            print(f"OCV Controller State: {ocv_controller_state}")
            print(f"FCV Controller State: {fcv_controller_state}")
            next_status += status_period_ms

    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt! Exiting...")
        sys.exit(0)
    except Exception as e:
        logging.exception("Unexpected error in main loop")
        # If nothing was received recently, emit a lightweight marker
        if len(ocv_rx_buf) == 0 and len(fcv_rx_buf) == 0:
            logging.error("PING")
        # brief backoff to avoid tight error loop
        time.sleep(0.01)
