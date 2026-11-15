import serial
import time
from crc import Calculator, Crc16
import struct
from enum import Enum

# crcCalculator = Calculator(Crc16.XMODEM)

# MAGIC_START = b'\xfb\xad'
PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_75932313638351C0A0C1-if00"
BAUD = 115200

# UPDTPKT_FMTSTR_WO_MAGIC = '<HBBHff'
# UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM = '<BBHff'

# UPDTPKT_FMTSTR_WO_MAGIC += 'I'
# UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM += 'I'

# # if USE_3_PTS:
# #     UPDTPKT_FMTSTR_WO_MAGIC += 'f'
# #     UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM += 'f'

# UPDTPKT_SIZE_WO_MAGIC = struct.calcsize(UPDTPKT_FMTSTR_WO_MAGIC)
# # UPDTPKT flags
# UPDPKT_IF_MPV_OPEN = 0x1

# class SystemState(Enum):
#     BOOT_INIT = 0
#     OPEN_LOOP_INIT = 1
#     CLOSED_LOOP = 2
#     FORCED_OPEN_LOOP = 3
#     EMERGENCY_STOP = 4

# def craft_pressure_update_packet(other_state: int, if_mpv_open: bool, pt1_reading: float, pt2_reading: float, packetNo: int) -> bytes:
#     flags = 0
#     if (if_mpv_open): flags |= UPDPKT_IF_MPV_OPEN
#     # if (pt3_reading != None):
#     #     pup_wo_magic_and_checksum = struct.pack(UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM, other_state, flags, 0, pt1_reading, pt2_reading, pt3_reading)
#     # else:
#     pup_wo_magic_and_checksum = struct.pack(UPDTPKT_FMTSTR_WO_MAGIC_AND_CHECKSUM, other_state, flags, 0, pt1_reading, pt2_reading, packetNo)
#     checksum = crcCalculator.checksum(pup_wo_magic_and_checksum)
#     return MAGIC_START + struct.pack('<H', checksum) + pup_wo_magic_and_checksum

# executed = False
# i = 0
with serial.Serial(PORT, BAUD, timeout=0) as ser:
    print(f"Listening on {PORT} at 100 Hz...")

    try:
        while True:
            start = time.monotonic()

            # Check for any available bytes
            n = ser.in_waiting

            if n > 0:
                data = ser.read(n)
                print(f"Received {len(data)} bytes: {data!r}", flush=True)

            # Clear input buffer every timestep
            ser.reset_input_buffer()

            # Run at approximately 100 Hz
            time.sleep(max(0, 0.01 - (time.monotonic() - start)))
            # i += 1

            # if i > 10 and not executed:
            #     print("Sending PUP")
            #     ocv_pressure_packet = craft_pressure_update_packet(
            #         SystemState.CLOSED_LOOP.value,  
            #         True, 
            #         10,  
            #         10,
            #         6767
            #     )
            #     ser.write(ocv_pressure_packet)
            #     executed = True

    except KeyboardInterrupt:
        print("\nStopped.")