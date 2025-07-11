"""
Motor control script for PicoMotor
Allows manual control of azimuth and elevation motors with degree inputs
and an infinite scanning mode.
"""

import time
import queue
import json

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
    c.stop()
    c.scan(el_first=args.el_first, repeat_count=args.count, pause_s=args.pause_s)
    #c.az_move_deg(-360)
    # XXX re-init position from redis
    #c.reset_deg_position(az_deg=0)
    for deg in (90, -90, 0):
        c.az_target_deg(deg)
        try:
            while c.is_moving():
                time.sleep(0.1)
        except(KeyboardInterrupt):
            continue
        finally:
            c.stop()
    c.stop()


if __name__ == "__main__":
    main()
