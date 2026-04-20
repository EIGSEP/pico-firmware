"""
Motor control script for PicoMotor
Allows manual control of azimuth and elevation motors with degree inputs
and an infinite scanning mode.
"""

import json
import logging
import numpy as np
from eigsep_observing import EigsepRedis

from picohost import PicoMotor

logger = logging.getLogger(__name__)


def _try_halt(c):
    """Best-effort halt; log and swallow disconnect."""
    try:
        c.halt()
    except ConnectionError as e:
        logger.warning("halt skipped: %s", e)


def main():
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
        last_status = r.get_live_metadata(keys="motor")
    except KeyError:
        last_status = None
    c = PicoMotor(port, verbose=True)
    try:
        c.set_delay(
            az_up_delay_us=2400,
            az_dn_delay_us=300,
            el_up_delay_us=2400,
            el_dn_delay_us=600,
        )
    except ConnectionError as e:
        logger.error("Could not configure motor: %s", e)
        return
    _try_halt(c)
    # try:
    #    #c.el_move_deg(30, wait_for_stop=True)
    #    c.az_move_deg(100, wait_for_stop=True)
    ##    c.az_target_deg(180, wait_for_stop=True)
    ##    c.az_target_deg(-180, wait_for_stop=True)
    # except(KeyboardInterrupt):
    #    c.halt()
    # finally:
    #    c.halt()
    try:
        _try_halt(c)
        c.scan(
            az_range_deg=np.linspace(-180.0, 180.0, 10),
            el_range_deg=np.linspace(-180.0, 180.0, 10),
            el_first=args.el_first,
            repeat_count=args.count,
            pause_s=args.pause_s,
            sleep_between=args.sleep_s,
        )
    except KeyboardInterrupt:
        _try_halt(c)
    finally:
        _try_halt(c)


if __name__ == "__main__":
    main()
