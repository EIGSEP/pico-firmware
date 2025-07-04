#!/usr/bin/env python3
import argparse
import subprocess
import time
import sys
import json
from serial import Serial
import struct
import base64


def decode_uint16(b64: str) -> int:
    raw = base64.b64decode(b64)     # returns a bytes object
    # < = little-endian, H = uint16
    return struct.unpack('<H', raw)[0]

def decode_uint32(b64: str) -> int:
    raw = base64.b64decode(b64)
    return struct.unpack('<I', raw)[0]   # <I = little-endian uint32

def decode_float(b64: str) -> float:
    raw = base64.b64decode(b64)
    return struct.unpack('<f', raw)[0]   # <f = little-endian 32-bit float

def decode_bytes(b64: str) -> bytes:
    return base64.b64decode(b64)         # raw binary payload


def watch_json_from_serial(port, baud):
    """
    Open the serial port, read until a valid JSON line appears or timeout.
    """
    with Serial(port, baudrate=baud) as ser:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            try:
                data = json.loads(line)
                print(data)
            except json.JSONDecodeError:
                continue

def main():
    data = watch_json_from_serial(sys.argv[-1], 115200)


if __name__ == "__main__":
    main()
