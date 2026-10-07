import serial
import time
import logging
import sys
import atexit
import os
import struct
from crc import Calculator, Crc16
from enum import Enum

##########################
#### CONFIGURATIONS ######
##########################
# Serial port config (choose one to uncomment)
# SERIAL_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_24230303637351C09181-if00' # Controller 1, OCV
SERIAL_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_75135333636351C001C0-if00' # Controller 2, FCV

BAUDRATE = 115200

LOG_TELEMETRY_FILE = '/home/ares_gs/synnax_node/scripts/logs/cmfv_hitl/telemetry_from_arduino.csv'
LOG_SENT_DUMMY_DATA_FILE = '/home/ares_gs/synnax_node/scripts/logs/cmfv_hitl/sent_dummy_data.csv'

DUMMY_PRESSURE_SET_POINT = 601.54

MAGIC_START = b'\xfb\xad'
TELPKT_FMTSTR_WO_MAGIC = '<HBBHfffff'
TELPKT_SIZE_WO_MAGIC = struct.calcsize(TELPKT_FMTSTR_WO_MAGIC)
# TELPKT flags
TELPKT_MPV_OPEN_DETECTED = 0x1
TELPKT_MPV_SHD_BE_CLOSED = 0x2
TELPKT_SYSTEM_TYPE = 0x4

UPDTPKT_FMTSTR_WO_MAGIC = '<HBBHff'
UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM = '<BBHff'
UPDTPKT_SIZE_WO_MAGIC = struct.calcsize(TELPKT_FMTSTR_WO_MAGIC)
# UPDTPKT flags
UPDPKT_IF_MPV_OPEN = 0x1

class SystemState(Enum):
    BOOT_INIT = 0
    OPEN_LOOP_INIT = 1
    CLOSED_LOOP = 2
    FORCED_OPEN_LOOP = 3
    EMERGENCY_STOP = 4

CONTROLLER_TELEMETRY_RATE = 5 # Rate at which the controller sends telemetry to the Pi (Hz)
PRESSURE_SEND_RATE = 200 # Rate at which the Pi should send pressure data to the controller (Hz)

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
# Extract directory paths from log file paths
log_telemetry_dir = os.path.dirname(LOG_TELEMETRY_FILE)
log_sent_data_dir = os.path.dirname(LOG_SENT_DUMMY_DATA_FILE)

# Create directories if they don't exist
os.makedirs(log_telemetry_dir, exist_ok=True)
os.makedirs(log_sent_data_dir, exist_ok=True)

#######################
#### SERIAL SETUP #####
#######################
serial_port = None
while True:
    try:
        time.sleep(1)
        serial_port = serial.Serial(SERIAL_PORT, BAUDRATE)
    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt! Exiting...")
        sys.exit(1)
    except:
        logging.info("Failure to bind serial ports.")
    else:
        break

########################
#### LOG FILE SETUP ####
########################
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
    if serial_port and serial_port.is_open:
        logging.info("Closing serial port...")
        serial_port.close()
    if log_telemetry:
        logging.info("Closing telemtry log file...")
        log_telemetry.flush()
        log_telemetry.close()
    if log_sent_data:
        logging.info("Closing dummy sent data log file...")
        log_sent_data.flush()
        log_sent_data.close()

atexit.register(cleanup)

####################################################
#### TELEMETRY COLLECTION & DATA STREAMING LOOP ####
####################################################
def getTime():
    return int(time.time_ns() / 1000000) # time in ms

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
        if len(unpacked_data) == 9:
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
                'motorAngle': unpacked_data[4],
                'deltaAngle': unpacked_data[5],
                'pidIntergralError': unpacked_data[6],
                'pt1Reading': unpacked_data[7],
                'pt2Reading': unpacked_data[8]
            }
    except (ValueError, IndexError) as e:
        logging.warning(f"Failed to parse telemetry line '{line}': {e}")
    
    return None

def craft_pressure_update_packet(other_state: int, if_mpv_open: bool, pt1_reading: float, pt2_reading: float) -> bytes:
    flags = 0
    if (if_mpv_open): flags |= UPDPKT_IF_MPV_OPEN
    pup_wo_magic_and_checksum = struct.pack(UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM, other_state, flags, 0, pt1_reading, pt2_reading)
    checksum = crcCalculator.checksum(pup_wo_magic_and_checksum)
    return MAGIC_START + struct.pack('<H', checksum) + pup_wo_magic_and_checksum

prev_telemetry_time = 0
prev_data_send_time = 0
prev_stdout_log_time = 0
prev_flush_time = 0
start_time = getTime()
serial_port.reset_input_buffer()

# State management
controller_state = SystemState.BOOT_INIT # State of the controller we're communicating with
simulated_other_state = SystemState.BOOT_INIT # Simulated state of the other controller
simulated_mpv = True
telemetry_line = ''

# Dummy data arrays
dummy_pressure_data_1 = [DUMMY_PRESSURE_SET_POINT] * 200 + [DUMMY_PRESSURE_SET_POINT - 10] * 200 + [DUMMY_PRESSURE_SET_POINT] * 200 + [DUMMY_PRESSURE_SET_POINT + 10] * 200 + [DUMMY_PRESSURE_SET_POINT] * 200
cur_data_1_index = 0
dummy_pressure_data_2 = [DUMMY_PRESSURE_SET_POINT] * 200 + [DUMMY_PRESSURE_SET_POINT - 10] * 200 + [DUMMY_PRESSURE_SET_POINT] * 200 + [DUMMY_PRESSURE_SET_POINT + 10] * 200 + [DUMMY_PRESSURE_SET_POINT] * 200
cur_data_2_index = 0

while True:
    try:
        curr_time = getTime()

        ######### TELEMETRY FROM CONTROLLER #########
        # Receive telemetry
        if (curr_time - prev_telemetry_time) > (1000 / CONTROLLER_TELEMETRY_RATE):
            try:
                serial_port.read_until(MAGIC_START, None)
                telemetry_packet = serial_port.read(TELPKT_SIZE_WO_MAGIC)
            except Exception as e:
                logging.warning(f"Failed to read telemetry: {e}")
                telemetry_packet = ""
            
            # Parse telemetry data
            controller_telemetry = parse_telemetry(telemetry_packet)

            # Log raw telemetry
            log_telemetry.write(str(curr_time) + ': ' + str(controller_telemetry) + "\n")

            prev_telemetry_time = curr_time

            # Extract state with fallback to previously-recorded state
            new_state = controller_telemetry['systemState'] if controller_telemetry else controller_state

            # Use logger to log state changes
            if new_state != controller_state:
                logging.info(f"Controller state change: {controller_state} -> {new_state}")
            controller_state = new_state

        ####### SIMULATE OTHER CONTROLLER STATE AND SEND DATA ########
        # Send data to the motor controller
        if (curr_time - prev_data_send_time) > (1000 / PRESSURE_SEND_RATE):

            # Simulate state changes in the other controller based on time
            prev_simulated_state = simulated_other_state
            if ((curr_time - start_time) > 15000 and (curr_time - start_time) < 20000):
                simulated_other_state = SystemState.FORCED_OPEN_LOOP
            elif ((curr_time - start_time) < 3000):
                simulated_other_state = SystemState.OPEN_LOOP_INIT
            else:
                simulated_other_state = SystemState.CLOSED_LOOP

            pressure_update_packet = craft_pressure_update_packet(simulated_other_state.value, simulated_mpv, dummy_pressure_data_1[cur_data_1_index], dummy_pressure_data_2[cur_data_2_index])
            serial_port.write(pressure_update_packet)

            # Log sent data
            pressure_data_str = f"P,{dummy_pressure_data_1[cur_data_1_index]:.2f},{dummy_pressure_data_2[cur_data_2_index]:.2f},{simulated_other_state},{simulated_mpv}"
            log_sent_data.write(str(curr_time) + ',' + pressure_data_str + "\n")

            prev_data_send_time = curr_time
            cur_data_1_index = (cur_data_1_index + 1) % len(dummy_pressure_data_1)
            cur_data_2_index = (cur_data_2_index + 1) % len(dummy_pressure_data_2)

        # Flush log files every 1 second
        if (curr_time - prev_flush_time) > 1000:
            log_telemetry.flush()
            log_sent_data.flush()
            prev_flush_time = curr_time

        # Log the communication status periodically
        if (curr_time - prev_stdout_log_time) > (1000 / min(PRESSURE_SEND_RATE, CONTROLLER_TELEMETRY_RATE)): 
            print(f"Received: {controller_telemetry if telemetry_packet else ''}")
            print(f"Sent: {pressure_data_str if 'pressure_data_str' in locals() else 'Not sent yet'}")
            print(f"Controller State: {controller_state}")
            print(f"Simulated Other State: {simulated_other_state}")
            prev_stdout_log_time = curr_time
        
    except ValueError as ve:
        logging.warning(f"Data validation error: {ve}")
    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt! Exiting...")
        sys.exit(1)
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
        if len(telemetry_line) == 0:
            logging.error("PING")