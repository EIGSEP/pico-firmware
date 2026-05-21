"""
Manual potentiometer calibration entry point.

Reads voltage samples from the running PicoManager's metadata stream
(``stream:potmon``), walks the user through a voltage-to-angle sweep,
computes a linear fit, and publishes it to three places:

  1. :class:`picohost.buses.PotCalStore` — canonical Redis store, so a
     rebooted :class:`picohost.PicoPotentiometer` picks the cal up at
     ``__init__`` time.
  2. The running pot device via :class:`picohost.proxy.PicoProxy`
     (``set_calibration``) — the new cal takes effect on the next
     status tick without restarting the manager.
  3. A timestamped JSON artifact on disk (audit).

Requires PicoManager to be running and the ``potmon`` device to be
healthy. The script never touches the serial port itself, so it can
be invoked alongside the manager.

Two modes:
  --mode minmax   : collect only at min and max (2-point fit, default)
  --mode turns    : collect at every full turn from min to max, plus a
                    fractional final stop when ``--turns`` isn't an
                    integer (least-squares fit)

Usage:
    calibrate-pot
    calibrate-pot --mode turns --turns 3.75
"""

from argparse import ArgumentParser
import json
import logging
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from eigsep_redis import Transport

from .buses import PotCalStore
from .proxy import PicoProxy

logger = logging.getLogger(__name__)

POTMON_NAME = "potmon"
POTMON_STREAM = f"stream:{POTMON_NAME}"
# Producer cadence is 200 ms (STATUS_CADENCE_MS), so a 5 s budget per
# entry is ~25x — long enough to ride out a single reconnect blip but
# short enough to fail fast when the manager isn't actually publishing.
SAMPLE_TIMEOUT_S = 5.0


def collect_samples(transport, n):
    """Average ``n`` consecutive entries from ``stream:potmon``.

    Reads only entries published after this call starts (``$``), so
    repeated calls within one calibration sweep don't double-count the
    same firmware tick.
    """
    samples_el, samples_az = [], []
    last_id = "$"
    while len(samples_el) < n:
        remaining = n - len(samples_el)
        resp = transport.r.xread(
            {POTMON_STREAM: last_id},
            block=int(SAMPLE_TIMEOUT_S * 1000),
            count=remaining,
        )
        if not resp:
            raise RuntimeError(
                f"No new entries on {POTMON_STREAM} within "
                f"{SAMPLE_TIMEOUT_S}s. Is PicoManager publishing?"
            )
        _stream, messages = resp[0]
        for msg_id, fields in messages:
            value = json.loads(fields[b"value"])
            samples_el.append(value["pot_el_voltage"])
            samples_az.append(value["pot_az_voltage"])
            last_id = msg_id
    return float(np.mean(samples_el)), float(np.mean(samples_az))


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


def collect_minmax(transport, n_samples, total_degrees):
    """Collect at min and max only (2-point calibration)."""
    input("\nSet both potentiometers to MINIMUM position, then press Enter.")
    print("  averaging samples...")
    v_min_0, v_min_1 = collect_samples(transport, n_samples)
    print(f"  pot_el min voltage: {v_min_0:.4f} V")
    print(f"  pot_az min voltage: {v_min_1:.4f} V")

    input("\nSet both potentiometers to MAXIMUM position, then press Enter.")
    print("  averaging samples...")
    v_max_0, v_max_1 = collect_samples(transport, n_samples)
    print(f"  pot_el max voltage: {v_max_0:.4f} V")
    print(f"  pot_az max voltage: {v_max_1:.4f} V")

    voltages_0 = [v_min_0, v_max_0]
    voltages_1 = [v_min_1, v_max_1]
    angles = [0.0, total_degrees]
    return voltages_0, voltages_1, angles


def _per_turn_stops(turns):
    """Stop points for per-turn collection.

    Integer turns 1..floor(turns), plus the exact ``turns`` value when
    it has a fractional part — so a 3.75-turn pot visits 1, 2, 3, 3.75.
    """
    full_turns = int(math.floor(turns))
    stops = [float(t) for t in range(1, full_turns + 1)]
    if not math.isclose(turns, full_turns):
        stops.append(float(turns))
    return stops


def collect_per_turn(transport, n_samples, turns):
    """Collect at every full turn from min to max (plus a fractional final)."""
    input(
        "\nSet both potentiometers to MINIMUM position (turn 0), "
        "then press Enter."
    )
    print("  averaging samples...")
    v0, v1 = collect_samples(transport, n_samples)
    print(f"  turn  0.00: pot_el={v0:.4f} V, pot_az={v1:.4f} V")
    voltages_0 = [v0]
    voltages_1 = [v1]
    angles = [0.0]

    prev = 0.0
    for stop in _per_turn_stops(turns):
        delta = stop - prev
        input(
            f"\nAdvance both pots {delta:.2f} turn(s) (to turn {stop:.2f}), "
            "then press Enter."
        )
        print("  averaging samples...")
        v0, v1 = collect_samples(transport, n_samples)
        print(f"  turn {stop:5.2f}: pot_el={v0:.4f} V, pot_az={v1:.4f} V")
        voltages_0.append(v0)
        voltages_1.append(v1)
        angles.append(stop * 360.0)
        prev = stop

    return voltages_0, voltages_1, angles


def _default_output_path():
    """Timestamped default filename — never overwrites prior runs."""
    # "-" instead of ":" in the time part so the filename is valid on
    # every filesystem (Windows/macOS reject ":" in filenames).
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    return f"pot_cal_{stamp}.json"


def main():
    parser = ArgumentParser(
        description="Calibrate potentiometer voltage-to-angle mapping.",
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
        type=float,
        default=10.0,
        help=(
            "Total turns from min to max. Fractional values are "
            "supported (e.g. 3.75 for the standard 3.75-turn pot). "
            "Default: 10."
        ),
    )
    parser.add_argument(
        "-n",
        "--n-samples",
        type=int,
        default=10,
        help=(
            "Number of voltage samples to average at each position "
            "(default: 10, ~2 s at the 200 ms producer cadence)"
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
        help="Redis host for the running PicoManager",
    )
    parser.add_argument(
        "--redis-port",
        type=int,
        default=6379,
        help="Redis port for the running PicoManager",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    if args.turns <= 0:
        print("turns must be positive.", file=sys.stderr)
        sys.exit(1)

    transport = Transport(host=args.redis_host, port=args.redis_port)
    pot_proxy = PicoProxy(POTMON_NAME, transport, source="calibrate-pot")

    if not pot_proxy.is_available:
        print(
            f"{POTMON_NAME} is not reachable via PicoManager. "
            "Start the manager and confirm the pot Pico is enumerated.",
            file=sys.stderr,
        )
        sys.exit(1)

    total_degrees = 360.0 * args.turns

    print(
        f"Mode: {args.mode} ({args.turns:g} turns, {args.n_samples} "
        f"samples/position)"
    )
    print(f"Reading voltages from {POTMON_STREAM}.")

    try:
        if args.mode == "minmax":
            voltages_0, voltages_1, angles = collect_minmax(
                transport, args.n_samples, total_degrees
            )
        else:
            voltages_0, voltages_1, angles = collect_per_turn(
                transport, args.n_samples, args.turns
            )
    except (RuntimeError, ConnectionError) as exc:
        print(f"Calibration sample collection failed: {exc}", file=sys.stderr)
        sys.exit(1)

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
            "turns": float(args.turns),
            "total_degrees": total_degrees,
            "mode": args.mode,
            "n_points": len(angles),
            "pot_el_voltages": [float(v) for v in voltages_0],
            "pot_az_voltages": [float(v) for v in voltages_1],
            "angles": [float(a) for a in angles],
            "n_samples": args.n_samples,
        },
    }

    # Persist to Redis first — if the live push later fails, the cal is
    # still stored and will load on the next PicoManager restart.
    PotCalStore(transport).upload(cal_data)
    print(
        f"\nPublished calibration to Redis at "
        f"{args.redis_host}:{args.redis_port} (key: pot_calibration)."
    )

    # Push to the running PicoPotentiometer so the new cal takes effect
    # on the next status tick.
    try:
        pot_proxy.send_command(
            "set_calibration",
            pot_el_params=list(cal0),
            pot_az_params=list(cal1),
        )
        print("Live PicoPotentiometer updated with new calibration.")
    except (TimeoutError, RuntimeError) as e:
        print(
            f"Live cal push failed: {e}\n"
            "Calibration is stored in Redis; restart PicoManager to apply.",
            file=sys.stderr,
        )

    # JSON artifact (audit)
    output_path = (
        Path(args.output) if args.output else Path(_default_output_path())
    )
    with open(output_path, "w") as f:
        json.dump(cal_data, f, indent=2)
    print(f"\nCalibration saved to {output_path}")
    print(f"  pot_el: angle = {cal0[0]:.4f} * V + {cal0[1]:.4f}")
    print(f"  pot_az: angle = {cal1[0]:.4f} * V + {cal1[1]:.4f}")
    if args.mode == "turns":
        print(f"  ({len(angles)} points used for least-squares fit)")


if __name__ == "__main__":
    main()
