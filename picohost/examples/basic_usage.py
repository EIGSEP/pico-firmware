#!/usr/bin/env python3
"""
Basic usage examples for picohost package.
"""

from picohost import PicoDevice, PicoMotor, PicoPeltier, PicoRFSwitch
import time


def discover_devices():
    """Find all connected Pico devices."""
    print("Discovering Pico devices...")
    ports = PicoDevice.find_pico_ports()
    print(f"Found {len(ports)} device(s): {ports}")
    return ports


def motor_example(port):
    """Example of motor control."""
    print(f"\n--- Motor Control Example ---")

    with PicoMotor(port) as motor:
        # Move both motors 1000 steps
        print("Moving motors...")
        motor.move(pulses_az=1000, pulses_el=1000)
        time.sleep(2)

        # Move back to origin
        print("Returning to origin...")
        motor.move(pulses_az=-1000, pulses_el=-1000)


def temperature_example(port):
    """Example of temperature control."""
    print(f"\n--- Temperature Control Example ---")

    with PicoPeltier(port) as peltier:
        # Set temperature to 25C on both channels
        print("Setting temperature to 25C...")
        peltier.set_temperature(25.0, channel=0)

        # Set hysteresis to 0.5C
        print("Setting hysteresis to 0.5C...")
        peltier.set_hysteresis(0.5, channel=0)

        # Enable control
        print("Enabling temperature control...")
        peltier.enable(channel=0)

        # Monitor for a few seconds
        time.sleep(5)

        # Disable control
        print("Disabling temperature control...")
        peltier.disable(channel=0)


def rf_switch_example(port):
    """Example of RF switch control."""
    print(f"\n--- RF Switch Example ---")

    with PicoRFSwitch(port) as switch:
        # Cycle through switch states
        for state in [1, 2, 4, 8, 0]:
            print(f"Setting switch state to {state}")
            switch.set_switch_state(state)
            time.sleep(1)


def main():
    """Main example function."""
    # Discover devices
    ports = discover_devices()

    if not ports:
        print("No Pico devices found. Please connect a device and try again.")
        return

    # Use first available port
    port = ports[0]
    print(f"Using device: {port}")

    # Run examples (uncomment the ones you want to test)
    try:
        motor_example(port)
        # temperature_example(port)
        # rf_switch_example(port)
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure your Pico is running the correct firmware app.")


if __name__ == "__main__":
    main()
