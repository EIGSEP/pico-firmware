#!/usr/bin/env python3
"""
Test script for motor control using the picohost library.
"""

import argparse
import time
import sys
from picohost import PicoMotor


def main():
    """Test motor control with movement patterns."""
    parser = argparse.ArgumentParser(description="Test Pico motor control")
    parser.add_argument("port", help="Serial port (e.g., /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--delay-az",
        type=int,
        default=600,
        help="Azimuth step delay (microseconds)",
    )
    parser.add_argument(
        "--delay-el",
        type=int,
        default=600,
        help="Elevation step delay (microseconds)",
    )

    args = parser.parse_args()

    # Create motor controller
    motor = PicoMotor(args.port, baudrate=args.baud)

    # Connect and start monitoring
    if not motor.connect():
        print(f"Failed to connect to {args.port}")
        return 1

    print(f"Connected to motor controller on {args.port}")
    motor.start()

    # Wait for user to start
    input("Press Enter to start motor test sequence...")

    try:
        # Test sequence: move in a pattern
        test_positions = [1620, 0, -1620]

        while True:
            for pulses in test_positions:
                print(f"Moving to position: az={pulses}, el={pulses}")
                motor.move(
                    pulses_az=pulses,
                    pulses_el=pulses,
                    delay_us_az=args.delay_az,
                    delay_us_el=args.delay_el,
                )
                time.sleep(3)

    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        motor.disconnect()
        print("Disconnected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
