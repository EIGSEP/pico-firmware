from argparse import ArgumentParser
import json
import time
import queue
from picohost import PicoIMU

parser = ArgumentParser(
    description="Calibrate magnetometer and accelerometer from IMU device(s)."
)
parser.add_argument(
    "-p",
    "--port",
    type=str,
    default="/dev/ttyACM0",
    help="Serial port for Pico device (default: /dev/ttyACM0)",
)
parser.add_argument(
    "-v", action="store_true", help="Print calibration data to console"
)
parser.add_argument(
    "-ch",
    "-channel",
    type=int,
    default=0,
    help="IMU channel: 0 for both, 1 for alt IMU only, 2 for az IMU only",
)
args = parser.parse_args()

data_queue = queue.Queue()
calibration_data = []


def add_calibration_data(json_data):
    """Callback function to add temperature data."""
    data_queue.put(json_data)


def handle_commands(cmd, ch, imu):
    if not cmd:
        return
    parts = cmd.split()
    cmd = parts[0].lower()

    if cmd == "calibrate":
        if imu.calibrate(ch):
            print(f"Channel {ch} enabled.")
    else:
        print(f"Unknown command: {cmd}. Available commands: calibrate.")


c = PicoIMU(args.port)

c.set_response_handler(add_calibration_data)
print_time = 0
print_cadence = 2  # seconds
with c:
    print("Recording IMU calibration data data. Press Ctrl+C to stop.")
    while True:
        try:
            handle_commands("calibrate", args.ch, c)
            try:
                json_data = data_queue.get_nowait()
            except queue.Empty:
                json_data = None
            if json_data:
                calibration_data.append(json_data)
                if args.v:
                    now = time.time()
                    if now - print_time < print_cadence:
                        continue
                    print(json.dumps(json_data, indent=2))
                    print_time = now
                if (
                    json_data.get("accel_cal") == 3
                    and json_data.get("mag_cal") == 3
                ):
                    print("IMU calibration complete: accel_cal=3, mag_cal=3")
                    break
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("Recording stopped.")
            break
