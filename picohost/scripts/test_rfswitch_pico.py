#!/usr/bin/env python3
import sys
import picohost
import time

# Walk every burned EEPROM path, ending on the fail-safe default.
STATES = sorted(
    picohost.PicoRFSwitch.PATHS, key=picohost.PicoRFSwitch.PATHS.get
)
STATES.remove("RFANT")
STATES.append("RFANT")


def main():
    """
    Open the serial port, read until a valid JSON line appears or timeout.
    """
    rfsw = picohost.PicoRFSwitch(sys.argv[-1])
    for state in STATES:
        print(f"RF switch state: {state}")
        rfsw.switch(state)
        time.sleep(2)


if __name__ == "__main__":
    main()
