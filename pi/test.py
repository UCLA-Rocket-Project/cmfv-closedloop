"""Receive-only CMFV Arduino telemetry dumper (no serial writes)."""
import argparse
import binascii
import struct
import time
import serial

MAGIC = b"\xfb\xad"
NO_DATA_TIMEOUT_S = 30.0
STATES = ["BOOT_INIT", "OPEN_LOOP_INIT", "CLOSED_LOOP", "FORCED_OPEN_LOOP", "EMERGENCY_STOP"]
FAULTS = {
    "manualAbort": 0x01,
    "commTimeout": 0x02,
    "encoderError": 0x04,
    "noMotion": 0x08,
    "sensorFault": 0x10,
    "redbandFault": 0x20,
    "oscillationDetected": 0x80,
}


def decode(packet, fmt, num_pts):
    # Packet: 2 magic bytes, 2-byte CRC, 4 status bytes, floats, uint32 packet number.
    values = struct.unpack(fmt, packet[2:])
    checksum, state, flags, fault_bits, _unused = values[:5]
    computed = binascii.crc_hqx(packet[4:], 0)  # CRC-16/XMODEM
    result = {
        "crc_ok": checksum == computed,
        "_checksum": checksum,
        "computed_checksum": computed,
        "systemState": STATES[state] if state < len(STATES) else f"UNKNOWN({state})",
        "systemType": "FUEL" if flags & 0x04 else "OX",
        "ifMpvOpenDetected": bool(flags & 0x01),
        "ifMpvShdBeClosed": bool(flags & 0x02),
        "fault_bits": f"0x{fault_bits:02x}",
        "motorAngle": values[5],
        "deltaAngle": values[6],
        "pidIntegralError": values[7],
        "pt1Reading": values[8],
        "pt2Reading": values[9],
    }
    if num_pts == 3:
        result["pt3Reading"] = values[10]
    result["packetNo"] = values[-1]
    result.update({f"faults.{name}": bool(fault_bits & bit) for name, bit in FAULTS.items()})
    return result


def main():
    ap = argparse.ArgumentParser(description="Read-only CMFV Arduino telemetry")
    ap.add_argument("--port", required=True, help="Arduino serial device")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pts", type=int, choices=(2, 3), default=2,
                    help="Number of PTs in Arduino telemetry (default: 2)")
    args = ap.parse_args()

    fmt = "<HBBBB" + ("f" * (3 + args.pts)) + "I"
    size = 2 + struct.calcsize(fmt)
    print(f"Read-only: {args.port} at {args.baud} baud, {args.pts} PTs, {size}-byte packets")
    print("Ctrl+C to exit. Opening USB serial may reset the Arduino.")
    buf = bytearray()
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            last_received = time.monotonic()
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    buf.extend(chunk)
                    last_received = time.monotonic()
                elif time.monotonic() - last_received >= NO_DATA_TIMEOUT_S:
                    print(f"No serial data received for {NO_DATA_TIMEOUT_S:g} seconds. Exiting.", flush=True)
                    break

                while True:
                    start = buf.find(MAGIC)
                    if start < 0:
                        # Keep a trailing first magic byte in case packet is split.
                        buf[:] = buf[-1:] if buf.endswith(MAGIC[:1]) else b""
                        break
                    if start:
                        del buf[:start]
                    if len(buf) < size:
                        break
                    packet = bytes(buf[:size])
                    data = decode(packet, fmt, args.pts)
                    print(data)
                    print(f"RAW: {packet!r}", flush=True)
                    # Bad CRC: advance one byte to look for a real packet boundary.
                    del buf[:size if data["crc_ok"] else 1]
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
