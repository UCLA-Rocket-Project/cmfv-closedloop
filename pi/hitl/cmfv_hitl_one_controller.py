"""Single-controller CSV HITL with cmfv2.py packets and Synnax MPV input.

Set IS_FUEL_SYSTEM to select FCV (True) or OCV (False). The other valve
stays at ASSUMED_ANGLE_OTHER_VAL in the CSV model. Its simulated state is
OPEN_LOOP_INIT for three seconds, then CLOSED_LOOP, as in the original.
CSV columns: OCV angle, FCV angle, oxidizer pressure, fuel pressure;
FCV angle varies fastest. Requires pyserial, numpy, crc and synnax.
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
IS_FUEL_SYSTEM = False

OCV_SERIAL_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_8513531363535160A0C1-if00' # Controller 1, OCV (Oxidizer)
FCV_SERIAL_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_75135333636351C001C0-if00' # Controller 2, FCV (Fuel)

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
TELPKT_FMTSTR_WO_MAGIC = '<HBBBBfffffI'
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

UPDTPKT_FMTSTR_WO_MAGIC = '<HBBHffI'
UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM = '<BBHffI'
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

os.makedirs(log_telemetry_dir or ".", exist_ok=True)
os.makedirs(log_sent_data_dir or ".", exist_ok=True)

##############################
##### LOOKUP TABLE SETUP #####
##############################
lookup_table = np.atleast_1d(np.genfromtxt(PRESSURE_LOOKUP_TABLE_FILE, dtype='float', delimiter=',', names=True))
num_angles_one_valve = int((LUT_ANGLE_RANGE_MAX - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL) + 1
num_angles_till_assumed_angle = int((ASSUMED_ANGLE_OTHER_VAL - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL)

if len(lookup_table) != num_angles_one_valve ** 2 or len(lookup_table.dtype.names or ()) < 4:
    raise ValueError("CSV must contain the full 30..90 degree grid and at least four columns")
if not all(np.isfinite(lookup_table[name]).all() for name in lookup_table.dtype.names[:4]):
    raise ValueError("CSV angles and pressures must be finite")
if not LUT_ANGLE_RANGE_MIN <= ASSUMED_ANGLE_OTHER_VAL <= LUT_ANGLE_RANGE_MAX:
    raise ValueError("Assumed other valve angle is outside the lookup range")
if not float((ASSUMED_ANGLE_OTHER_VAL - LUT_ANGLE_RANGE_MIN) / LUT_ANGLE_INTERVAL).is_integer():
    raise ValueError("Assumed other valve angle must lie on the lookup grid")

if IS_FUEL_SYSTEM: 
    lookup_table_truncated = lookup_table[num_angles_one_valve * num_angles_till_assumed_angle : num_angles_one_valve * (num_angles_till_assumed_angle + 1)]
else:
    lookup_table_truncated = [lookup_table[num_angles_one_valve * i + num_angles_till_assumed_angle] for i in range(num_angles_one_valve)]

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
        if streamer is not None:
            try:
                streamer.close()
            except Exception:
                logging.exception("Failed to close Synnax streamer")
        if client is not None:
            try:
                client.close()
            except Exception:
                logging.debug("Synnax client close unavailable or failed")
    except Exception as e:
        logging.exception("Cleanup encountered an error")

client = None
streamer = None
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
            return to_return
    except (struct.error, ValueError, IndexError) as e:
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

# State management
controller_state = SystemState.BOOT_INIT
simulated_other_state = SystemState.BOOT_INIT

# MPV comes from Synnax; default to CLOSED until a sample arrives.
MPV_CHANNEL_NAME = "controls_state_11"
isMPVOpen = False

# Dictionary of telemtry data received from the controller
controller_telemetry = None

# Non-blocking read buffer for partial lines
_rx_buf = bytearray()
cur_packet_wo_magic = bytearray()
currently_receiving = False

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

def drain_telemetry():
    global controller_telemetry, controller_state, currently_receiving
    latest, drained, currently_receiving = drain_telemetry_from_port(
        serial_port, _rx_buf, cur_packet_wo_magic, currently_receiving,
        "FCV" if IS_FUEL_SYSTEM else "OCV"
    )
    if latest is not None:
        controller_telemetry = latest
        controller_state = latest['systemState']
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

# Same Synnax connection and nonblocking MPV subscription as cmfv2.py.
client = sy.Synnax(host="localhost", port=9091, username="synnax",
                   password="seldon", secure=False)
try:
    streamer = client.open_streamer([MPV_CHANNEL_NAME])
except Exception:
    logging.exception("Failed to open Synnax streamer; proceeding with MPV assumed CLOSED.")

packetNo = 0
cur_pressure = 0
pressure_data_str = None

while True:
    try:
        curr_mono = now_ms()

        # Drain any available telemetry without blocking
        _ = drain_telemetry()

        try:
            if streamer is not None:
                frame = streamer.read(timeout=0)
                if frame is not None and len(frame[MPV_CHANNEL_NAME]) > 0:
                    isMPVOpen = np.uint8(frame[MPV_CHANNEL_NAME][-1]) == 1
        except Exception as e:
            logging.debug(f"Failed to read MPV signal: {e}")

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
            packetNo = (packetNo + 1) & 0xFFFFFFFF

            # Use LUT to generate pressures
            if controller_telemetry:
                motor_angle_rounded_clamped = min(LUT_ANGLE_RANGE_MAX, max(LUT_ANGLE_RANGE_MIN, round(controller_telemetry['motorAngle'] * 2) / 2))
                cur_pressure = lookup_table_truncated[int((motor_angle_rounded_clamped - LUT_ANGLE_RANGE_MIN) // LUT_ANGLE_INTERVAL)][3 if IS_FUEL_SYSTEM else 2]

                controller_state = controller_telemetry['systemState']
            
            try:
                pressure_update_packet = craft_pressure_update_packet(simulated_other_state.value, 
                                                                      isMPVOpen, 
                                                                      cur_pressure, 
                                                                      cur_pressure,
                                                                      packetNo)
                serial_port.write(pressure_update_packet)
                pressure_data_str = f"P,{cur_pressure:.2f},{cur_pressure:.2f},{simulated_other_state},{isMPVOpen},packetNo,{packetNo}"
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
            print(f"MPV State: {'OPEN' if isMPVOpen else 'CLOSED'}")
            print(f"Packet Number: {packetNo}")
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
