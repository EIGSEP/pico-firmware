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

def main():
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

    c = PicoMotor(port, verbose=True)
    c.set_delay(az_up_delay_us=2400, az_dn_delay_us=300, el_up_delay_us=2400, el_dn_delay_us=600)
    try:
        c.el_move_deg(-10, wait_for_stop=True)
    #    c.az_target_deg(180, wait_for_stop=True)
    #    c.az_target_deg(-180, wait_for_stop=True)
    except(KeyboardInterrupt):
        c.stop()
    finally:
        c.stop()


if __name__ == "__main__":
    main()
