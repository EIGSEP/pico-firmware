import sys
import json
from serial import Serial


def watch_json_from_serial(dev, baud):
    """
    Open the serial dev, read until a valid JSON line appears or timeout.
    """
    with Serial(dev, baudrate=baud) as ser:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            try:
                data = json.loads(line)
                sensor_name = data['sensor_name']
                print(sensor_name, '=' * (40 - len(sensor_name)))
                print(json.dumps(data, indent=2, sort_keys=False))
            except (KeyError, json.JSONDecodeError):
                print(line)


def main():
    watch_json_from_serial(sys.argv[-1], 115200)


if __name__ == "__main__":
    main()
