"""
Motor control script for PicoMotor
Allows manual control of azimuth and elevation motors with degree inputs
and an infinite scanning mode.
"""

import argparse
import time
import sys
import queue

from picohost.base import PicoMotor


class MotorController(PicoMotor):

    def __init__(self, port):
        super().__init__(port)
        self.status_queue = queue.Queue()
        self.set_response_handler(self.status_queue.put)

    @property
    def is_moving(self):
        """Check if motors are currently moving"""
        if self._running:
            status = self.status_queue.get(timeout=1)
        else:
            status = self.readline(timeout=1)
        if not status:
            print("No status received from motor")
            return False
        az_moving = status.get("az_remaining_steps", 0) != 0
        el_moving = status.get("el_remaining_steps", 0) != 0
        return az_moving or el_moving

    def move(self, deg_az=0, deg_el=0, block=True):
        """Move motors by specified degrees"""
        super().move(deg_az=deg_az, deg_el=deg_el)
        if block:
            self.wait_for_motors()

    def wait_for_motors(self):
        while self.is_moving:
            try:
                print("Waiting for motors to finish moving...")
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.stop()

     def stop(self):
        """Stop all motor movements"""
        print("Stopping motors.")
        self.move(deg_az=0, deg_el=0, block=False)
        self.status_queue.queue.clear()



def infinite_scan_mode(motor, scan_az=True, scan_el=True):

    with motor:
        mode_str = []
        if scan_az:
            mode_str.append("azimuth")
        if scan_el:
            mode_str.append("elevation")
        print(
            f"Starting infinite scan mode for {' and '.join(mode_str)} "
            "(Ctrl+C to stop)"
        )

        # Track motor states with unified logic
        az_direction = 1
        el_direction = 1
        el_total = 0

        while True:
            try:
                if scan_az:
                    # Move azimuth 360 degrees in current direction
                    az_move = 360 * az_direction
                    print(f"Rotating azimuth {az_move}°...")
                    motor.move(deg_az=az_move, deg_el=0)
                    motor.wait_for_motors()

                    # Reverse azimuth direction for next iteration
                    az_direction *= -1

                if scan_el:
                    # Move elevation 10 degrees in current direction
                    el_move = 10 * el_direction
                    print(f"Rotating elevation {el_move}°...")
                    motor.move(deg_az=0, deg_el=el_move)
                    motor.wait_for_motors()

                    # Update elevation total and check for reversal
                    el_total += 10
                    if el_total >= 360:
                        el_direction *= -1
                        el_total = 0
                        print("Elevation direction reversed")

                time.sleep(0.5)  # Small pause between cycles

            except KeyboardInterrupt:
                print("\nStopping infinite scan mode")
                # Send stop command (0 movement)
                motor.move(deg_az=0, deg_el=0)
                break


def main():
    parser = argparse.ArgumentParser(
        description="Control PicoMotor azimuth and elevation"
    )
    parser.add_argument(
        "-p",
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="Serial port for PicoMotor device (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--az", type=float, help="Degrees to move azimuth motor"
    )
    parser.add_argument(
        "--el", type=float, help="Degrees to move elevation motor"
    )
    parser.add_argument(
        "--infinite", action="store_true", help="Run infinite scanning mode"
    )
    parser.add_argument(
        "--infinite-az",
        action="store_true",
        help="Run infinite scanning mode for azimuth only",
    )
    parser.add_argument(
        "--infinite-el",
        action="store_true",
        help="Run infinite scanning mode for elevation only",
    )

    args = parser.parse_args()

    m = MotorController(args.port)
    if args.infinite or args.infinite_az or args.infinite_el:
        scan_az = args.infinite or args.infinite_az
        scan_el = args.infinite or args.infinite_el
        infinite_scan_mode(m, scan_az=scan_az, scan_el=scan_el)
    elif args.az is not None or args.el is not None:
        m.move(az_degrees=args.az, el_degrees=args.el)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
