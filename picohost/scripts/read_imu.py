"""
read_imu.py - Read and display IMU data from a Pico running the IMU app.

Usage:
    python3 read_imu.py --port /dev/ttyACM0
    python3 read_imu.py  # auto-detect first Pico
"""

import argparse
import time

from picohost import PicoIMU


def main():
    ap = argparse.ArgumentParser(description="Read IMU data from a Pico device")
    ap.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    ap.add_argument("--interval", type=float, default=0.5, help="Print interval in seconds")
    args = ap.parse_args()

    port = args.port
    if port is None:
        ports = PicoIMU.find_pico_ports()
        if not ports:
            print("No Pico devices found.")
            return
        port = ports[0]
        print(f"Auto-detected: {port}")

    imu = PicoIMU(port)
    print(f"Connected to {port}. Press Ctrl-C to stop.\n")
    print(f"{'Time':>8s}  {'Yaw':>8s}  {'Pitch':>8s}  {'Roll':>8s}  "
          f"{'Ax':>8s}  {'Ay':>8s}  {'Az':>8s}  {'Status':>8s}")
    print("-" * 78)

    t0 = time.time()
    was_connected = True
    try:
        while True:
            if not imu.is_connected:
                if was_connected:
                    t = time.time() - t0
                    print(f"{t:8.1f}  {'DISCONNECTED — waiting for reconnect...':^70s}")
                    was_connected = False
                time.sleep(args.interval)
                continue
            was_connected = True
            s = imu.last_status
            if s:
                t = time.time() - t0
                print(f"{t:8.1f}  {s.get('yaw', 0):8.2f}  {s.get('pitch', 0):8.2f}  "
                      f"{s.get('roll', 0):8.2f}  {s.get('accel_x', 0):8.4f}  "
                      f"{s.get('accel_y', 0):8.4f}  {s.get('accel_z', 0):8.4f}  "
                      f"{s.get('status', '?'):>8s}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        imu.disconnect()


if __name__ == "__main__":
    main()
