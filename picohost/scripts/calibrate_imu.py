from argparse import ArgumentParser
import json
import time
import queue
from picohost import PicoIMU

parser = ArgumentParser(
    description="Calibrate magnetometer and accelerometer on 2 independent Pico IMUs."
)
parser.add_argument(
    "-p1",
    "--port1",
    type=str,
    default="/dev/ttyACM0",
    help="Serial port for IMU1 (default: /dev/ttyACM0)",
)
parser.add_argument(
    "-p2",
    "--port2",
    type=str,
    default="/dev/ttyACM1",
    help="Serial port for IMU2 (default: /dev/ttyACM1)",
)
parser.add_argument(
    "-v", "--verbose", action="store_true", help="Print calibration data to console",
)
args = parser.parse_args()

# Per‑IMU queues
imu_queues = {1: queue.Queue(), 2: queue.Queue()}
calibration_data = []

def make_callback(which):
    def callback(data):
        imu_queues[which].put(data)
    return callback

def request_calibration(imu, name):
    if imu.calibrate():
        print(f"Sent calibration request to {name}")
    else:
        print(f"Failed to send calibration request to {name}")

imu1 = PicoIMU(args.port1)
imu2 = PicoIMU(args.port2)

imu1.set_response_handler(make_callback(1))
imu2.set_response_handler(make_callback(2))

imu_done = {1: False, 2: False}
print_time = 0
print_cadence = 2  # seconds

with imu1, imu2:
    print("Recording IMU calibration data. Press Ctrl+C to stop.")

    request_calibration(imu1, "IMU1")
    request_calibration(imu2, "IMU2")

    while True:
        try:
            now = time.time()

            # IMU1
            if not imu_done[1]:
                try:
                    data1 = imu_queues[1].get_nowait()
                except queue.Empty:
                    data1 = None

                if data1:
                    calibration_data.append(("IMU1", data1))
                    if args.verbose and now - print_time >= print_cadence:
                        print("IMU1:", json.dumps(data1, indent=2))
                        print_time = now

                    if data1.get("accel_cal") == 3 and data1.get("mag_cal") == 3 and data1.get("calibrated") == "True":
                        print("IMU1 calibration complete.")
                        imu_done[1] = True
                    else:
                        request_calibration(imu1, data1.get("sensor_name"))

            # IMU2
            if not imu_done[2]:
                try:
                    data2 = imu_queues[2].get_nowait()
                except queue.Empty:
                    data2 = None

                if data2:
                    calibration_data.append(("IMU2", data2))
                    if args.verbose and now - print_time >= print_cadence:
                        print("IMU2:", json.dumps(data2, indent=2))
                        print_time = now

                    if data2.get("accel_cal") == 3 and data2.get("mag_cal") == 3 and data2.get("calibrated") == "True":
                        print("IMU2 calibration complete.")
                        imu_done[2] = True
                    else:
                        request_calibration(imu2, data2.get("sensor_name"))

            # Exit when done
            if imu_done[1] and imu_done[2]:
                print("Both IMUs calibrated. Exiting.")
                break

        except KeyboardInterrupt:
            print("Recording stopped by user.")
            break
