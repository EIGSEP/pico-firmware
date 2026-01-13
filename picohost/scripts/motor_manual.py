"""
Motor control script for PicoMotor
Allows manual control of azimuth and elevation motors with degree inputs
and an infinite scanning mode.
"""

import time
import queue
import json
import numpy as np
from eigsep_observing import EigsepRedis

from picohost import PicoMotor

def main(screen):
    curses.noecho()           # optional: wrapper sets cbreak but not noecho
    screen.nodelay(False)     # blocking getch
    import sys
    import argparse

    parser = argparse.ArgumentParser(
        description="Control PicoMotor azimuth and elevation"
    )
    parser.add_argument(
        "-c",
        "--pico_config",
        type=str,
        default="pico_config.json",
        help="Output of flash_picos (pico_config.json)",
    )

    args = parser.parse_args()
    port = None
    with open(args.pico_config, "r", encoding="utf-8") as f:
        records = json.load(f)
        for config in records:
            if config["app_id"] == 0:  # must match pico_multi.h
                port = config["port"]
                break
    assert port is not None  # didn't find app_id 0 in pico_config.json

    r = EigsepRedis()

    c = PicoMotor(port, r, verbose=True)
    c.set_delay(az_up_delay_us=2400, az_dn_delay_us=300, el_up_delay_us=2400, el_dn_delay_us=600)

    def move_up(deg): c.el_move_deg(deg, wait_for_stop=True)
    def move_dn(deg): c.el_move_deg(-deg, wait_for_stop=True)
    def move_lf(deg): c.az_move_deg(deg, wait_for_stop=True)
    def move_rt(deg): c.az_move_deg(-deg, wait_for_stop=True)

    DISPATCH = {
        'u': move_up,
        'd': move_dn,
        'l': move_lf,
        'r': move_rt,
    }
    try:
        deg = 1
        while True:
            ch = screen.getch()
            if ch == -1:
                continue
            if 0 <= ch < 256:
                key = chr(ch).lower()
                if key in DISPATCH:
                    DISPATCH[key](deg)
        #c.el_move_deg(-10, wait_for_stop=True)
    #    c.az_target_deg(180, wait_for_stop=True)
    #    c.az_target_deg(-180, wait_for_stop=True)
    except(KeyboardInterrupt):
        c.stop()
    finally:
        c.stop()


if __name__ == "__main__":
    import curses
    curses.wrapper(main)
