#!/usr/bin/env python3
import sys
import picohost
import time

STATES = [
    'VNAO',
    'RFANT',
    'VNAS',
    'VNAL',
    'VNAANT',
    'VNANON',
    'VNANOFF',
    'RFNON',
    'RFNOFF',
]

def main():
    """
    Open the serial port, read until a valid JSON line appears or timeout.
    """
    rfsw = picohost.PicoRFSwitch(sys.argv[-1])
    rfsw.connect()  #XXX why??
    for state in STATES:
        print(f"RF switch state: {state}")
        rfsw.switch(state)
        time.sleep(2)


if __name__ == "__main__":
    main()
