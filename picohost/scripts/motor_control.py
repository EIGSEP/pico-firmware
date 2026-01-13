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
    parser.add_argument(
        "--el_first",
        action="store_true",
        help="Scan el, then az. Otherwise, reverse.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=None,
        help="Run specified number of scans (defauly infinite)",
    )
    parser.add_argument(
        "--pause_s",
        type=float,
        default=None,
        help="Seconds to pause at each pointing",
    )
    parser.add_argument(
        "--sleep_s",
        type=float,
        default=None,
        help="Seconds to sleep between each scan",
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

    try:
        r = EigsepRedis()
        last_status = r.get_live_metadata(keys='motor')
    except(KeyError):
        last_status = None
    c = PicoMotor(port, r, verbose=True)
    #zeroed = c.status['az_pos'] == 0 and c.status['el_pos'] == 0
    #if zeroed and (last_status is not None):
    #    print('Resetting to last known position.')
    #    c.reset_step_position(az_step=last_status['az_pos'], el_step=last_status['el_pos'])
    c.reset_step_position(az_step=0, el_step=0)
    c.set_delay(az_up_delay_us=2400, az_dn_delay_us=300, el_up_delay_us=2400, el_dn_delay_us=600)
    c.stop()
    #try:
    #    #c.el_move_deg(30, wait_for_stop=True)
    #    c.az_move_deg(100, wait_for_stop=True)
    ##    c.az_target_deg(180, wait_for_stop=True)
    ##    c.az_target_deg(-180, wait_for_stop=True)
    #except(KeyboardInterrupt):
    #    c.stop()
    #finally:
    #    c.stop()
    try:
        c.stop()
        c.scan(
            az_range_deg=np.linspace(-180.0, 180.0, 10),
            el_range_deg=np.linspace(-180.0, 180.0, 10),
            el_first=args.el_first,
            repeat_count=args.count,
            pause_s=args.pause_s,
            sleep_between=sleep_s,
        )
    except(KeyboardInterrupt):
        c.stop()
    finally:
        c.stop()


if __name__ == "__main__":
    main()
