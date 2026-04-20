"""
Manual potentiometer calibration entry point.

Auto-discovers the potentiometer Pico (by USB VID/PID + probing the
``sensor_name`` field of its status JSON), walks the user through a
voltage-to-angle sweep, computes a linear fit, and publishes the result
to both Redis (canonical, via :class:`picohost.buses.PotCalStore`) and
a timestamped JSON artifact on disk (audit). A rebooted
:class:`picohost.PicoPotentiometer` picks the Redis cal up at
``__init__`` time so no JSON file needs to be accessible on the Pico
host.

Two modes:
  --mode minmax   : collect only at min and max (2-point fit, default)
  --mode turns    : collect at every full turn from min to max (least-squares)

Usage:
    calibrate-pot
    calibrate-pot -p /dev/ttyACM0 --mode turns
    calibrate-pot --no-redis -o pot_cal_bench.json
"""

from argparse import ArgumentParser
import json
import logging
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from eigsep_redis import Transport

from .base import PicoPotentiometer
from .buses import PotCalStore
from .flash_picos import find_pico_ports, read_json_from_serial

logger = logging.getLogger(__name__)

POTMON_SENSOR_NAME = "potmon"


def discover_pot_port(probe_timeout=3.0):
    """Return the serial port of the connected potentiometer Pico.

    Opens each Pico-VID port in turn, reads one JSON status line, and
    matches on ``sensor_name == "potmon"``. Raises if zero or multiple
    pot Picos are found — in those cases the caller must pass ``-p``
    to disambiguate or fix the wiring.
    """
    ports = find_pico_ports()
    if not ports:
        raise RuntimeError("No Raspberry Pi Pico serial ports found.")
    matches = []
    for port_dev in ports:
        try:
            data = read_json_from_serial(port_dev, 115200, probe_timeout)
        except RuntimeError:
            continue
        if data.get("sensor_name") == POTMON_SENSOR_NAME:
            matches.append(port_dev)
    if not matches:
        raise RuntimeError(
            "No potentiometer Pico found. Confirm the pot pico is flashed "
            "with APP_POTMON and connected."
        )
    if len(matches) > 1:
        raise RuntimeError(
            f"Multiple potentiometer Picos found on {matches}. "
            "Pass -p <port> to pick one."
        )
    return matches[0]


def collect_samples(pot, n, interval=0.25):
    """Collect *n* voltage samples and return averages for both pots."""
    samples_0, samples_1 = [], []
    for _ in range(n):
        v = pot.read_voltage()
        samples_0.append(v["pot_el_voltage"])
        samples_1.append(v["pot_az_voltage"])
        time.sleep(interval)
    return np.mean(samples_0), np.mean(samples_1)


def compute_linear_fit(voltages, angles):
    """Compute (m, b) such that angle = m * voltage + b via least-squares.

    Returns None if the voltage range is too small to calibrate.
    """
    voltages = np.asarray(voltages)
    if np.abs(voltages[-1] - voltages[0]) < 1e-6:
        print(
            f"  ERROR: min and max voltages are identical "
            f"({voltages[0]:.4f} V). Cannot calibrate."
        )
        return None
    m, b = np.polyfit(voltages, angles, 1)
    return (float(m), float(b))


def collect_minmax(pot, n_samples, total_degrees):
    """Collect at min and max only (2-point calibration)."""
    input("\nSet both potentiometers to MINIMUM position, then press Enter.")
    v_min_0, v_min_1 = collect_samples(pot, n_samples)
    print(f"  pot_el min voltage: {v_min_0:.4f} V")
    print(f"  pot_az min voltage: {v_min_1:.4f} V")

    input("\nSet both potentiometers to MAXIMUM position, then press Enter.")
    v_max_0, v_max_1 = collect_samples(pot, n_samples)
    print(f"  pot_el max voltage: {v_max_0:.4f} V")
    print(f"  pot_az max voltage: {v_max_1:.4f} V")

    voltages_0 = [v_min_0, v_max_0]
    voltages_1 = [v_min_1, v_max_1]
    angles = [0.0, total_degrees]
    return voltages_0, voltages_1, angles


def collect_per_turn(pot, n_samples, turns):
    """Collect at every full turn from min to max."""
    input(
        "\nSet both potentiometers to MINIMUM position (turn 0), "
        "then press Enter."
    )
    v0, v1 = collect_samples(pot, n_samples)
    print(f"  turn  0: pot_el={v0:.4f} V, pot_az={v1:.4f} V")
    voltages_0 = [v0]
    voltages_1 = [v1]
    angles = [0.0]

    for turn in range(1, turns + 1):
        input(
            f"\nAdvance both pots 1 full turn (to turn {turn}), "
            "then press Enter."
        )
        v0, v1 = collect_samples(pot, n_samples)
        print(f"  turn {turn:2d}: pot_el={v0:.4f} V, pot_az={v1:.4f} V")
        voltages_0.append(v0)
        voltages_1.append(v1)
        angles.append(turn * 360.0)

    return voltages_0, voltages_1, angles


def _default_output_path():
    """Timestamped default filename — never overwrites prior runs."""
    # Use "-" instead of ":" in the time part so the filename is valid
    # on every filesystem (Windows/macOS reject ":" in filenames).
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    return f"pot_cal_{stamp}.json"


def main():
    parser = ArgumentParser(
        description="Calibrate potentiometer voltage-to-angle mapping.",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=str,
        default=None,
        help=(
            "Serial port for the potentiometer Pico. Auto-discovered "
            "when omitted."
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help=(
            "Output JSON artifact path. Defaults to a timestamped file "
            "in the current directory so prior runs aren't overwritten."
        ),
    )
    parser.add_argument(
        "-t",
        "--turns",
        type=int,
        default=10,
        help="Number of full turns from min to max (default: 10)",
    )
    parser.add_argument(
        "-n",
        "--n-samples",
        type=int,
        default=10,
        help=(
            "Number of voltage samples to average at each position "
            "(default: 10)"
        ),
    )
    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        choices=["minmax", "turns"],
        default="minmax",
        help=(
            "Calibration mode: 'minmax' for 2-point, 'turns' for "
            "per-turn least-squares (default: minmax)"
        ),
    )
    parser.add_argument(
        "--redis-host",
        default="localhost",
        help="Redis host for PotCalStore publication",
    )
    parser.add_argument(
        "--redis-port",
        type=int,
        default=6379,
        help="Redis port for PotCalStore publication",
    )
    parser.add_argument(
        "--no-redis",
        action="store_true",
        help=(
            "Skip Redis publication. The JSON artifact is still written; "
            "use this on a bench host without a Redis instance."
        ),
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    if args.port is None:
        try:
            args.port = discover_pot_port()
        except RuntimeError as e:
            print(str(e), file=sys.stderr)
            sys.exit(1)
        print(f"Discovered potentiometer Pico on {args.port}.")

    total_degrees = 360.0 * args.turns

    print(f"Connecting to {args.port}...")
    print(
        f"Mode: {args.mode} ({args.turns} turns, {args.n_samples} "
        f"samples/position)"
    )

    with PicoPotentiometer(args.port) as pot:
        if args.mode == "minmax":
            voltages_0, voltages_1, angles = collect_minmax(
                pot,
                args.n_samples,
                total_degrees,
            )
        else:
            voltages_0, voltages_1, angles = collect_per_turn(
                pot,
                args.n_samples,
                args.turns,
            )

    cal0 = compute_linear_fit(voltages_0, angles)
    cal1 = compute_linear_fit(voltages_1, angles)

    if cal0 is None or cal1 is None:
        print("\nCalibration failed. Exiting.", file=sys.stderr)
        sys.exit(1)

    cal_data = {
        "pot_el": list(cal0),
        "pot_az": list(cal1),
        "metadata": {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "port": args.port,
            "turns": args.turns,
            "total_degrees": total_degrees,
            "mode": args.mode,
            "n_points": len(angles),
            "pot_el_voltages": [float(v) for v in voltages_0],
            "pot_az_voltages": [float(v) for v in voltages_1],
            "angles": [float(a) for a in angles],
            "n_samples": args.n_samples,
        },
    }

    # Redis publication (canonical source for PicoManager-spawned pots)
    if not args.no_redis:
        try:
            transport = Transport(host=args.redis_host, port=args.redis_port)
            PotCalStore(transport).upload(cal_data)
            print(
                f"Published calibration to Redis at "
                f"{args.redis_host}:{args.redis_port} (key: pot_calibration)."
            )
        except Exception as e:
            print(
                f"Redis publication failed: {e}\n"
                "Re-run with --no-redis if Redis is not available.",
                file=sys.stderr,
            )
            # Keep going so the JSON artifact still lands on disk.

    # JSON artifact (audit / bench fallback)
    output_path = Path(args.output) if args.output else Path(_default_output_path())
    with open(output_path, "w") as f:
        json.dump(cal_data, f, indent=2)
    print(f"\nCalibration saved to {output_path}")
    print(f"  pot_el: angle = {cal0[0]:.4f} * V + {cal0[1]:.4f}")
    print(f"  pot_az: angle = {cal1[0]:.4f} * V + {cal1[1]:.4f}")
    if args.mode == "turns":
        print(f"  ({len(angles)} points used for least-squares fit)")


if __name__ == "__main__":
    main()
