import sys
import json
from serial import Serial


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
                print(json.dumps(data, indent=2, sort_keys=False))
            except json.JSONDecodeError:
                continue


def main():
    watch_json_from_serial(sys.argv[-1], 115200)


if __name__ == "__main__":
    main()
