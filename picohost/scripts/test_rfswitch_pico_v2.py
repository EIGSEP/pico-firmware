#!/usr/bin/env python3
"""
Test script for RF switch control using the picohost library.
"""

import argparse
import time
import sys
from picohost import PicoRFSwitch


def main():
    """Test RF switch control interactively."""
    parser = argparse.ArgumentParser(description='Test Pico RF switch control')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--cycle', action='store_true', help='Cycle through all switch states')
    
    args = parser.parse_args()
    
    # Create RF switch controller
    switch = PicoRFSwitch(args.port, baudrate=args.baud)
    
    # Connect and start monitoring
    if not switch.connect():
        print(f"Failed to connect to {args.port}")
        return 1
    
    print(f"Connected to RF switch controller on {args.port}")
    switch.start()
    
    try:
        if args.cycle:
            # Automatic cycling mode
            print("Cycling through switch states (0-7)...")
            while True:
                for state in range(8):
                    print(f"Setting switch state to {state}")
                    switch.set_switch_state(state)
                    time.sleep(2)
        else:
            # Interactive mode
            print("RF Switch Control - Interactive Mode")
            print("Enter switch state (0-255) or 'q' to quit")
            
            while True:
                try:
                    user_input = input("Switch state: ").strip()
                    
                    if user_input.lower() == 'q':
                        break
                    
                    state = int(user_input)
                    if 0 <= state <= 255:
                        switch.set_switch_state(state)
                    else:
                        print("Error: State must be between 0 and 255")
                        
                except ValueError:
                    print("Error: Please enter a valid number or 'q' to quit")
                except KeyboardInterrupt:
                    print("\n")
                    break
                    
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        switch.disconnect()
        print("Disconnected")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())