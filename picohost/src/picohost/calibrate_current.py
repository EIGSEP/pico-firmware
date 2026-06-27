"""
Manual whole-system current-monitor calibration entry point.

Reads raw ADC voltages from the running PicoManager's metadata stream
(``stream:system_current``, produced by the current sensor on the lidar
Pico), walks the user through a **two-point** calibration, and publishes
the fit to two places:

  1. :class:`picohost.buses.CurrentCalStore` — canonical Redis store, so a
     rebooted :class:`picohost.PicoLidar` picks the cal up at ``__init__``
     time. Followed by a ``BGSAVE`` to fsync the RDB snapshot, since the
     default ``save 3600 1`` policy would otherwise leave a single-key
     write unsnapshotted for up to an hour.
  2. The running lidar device via :class:`picohost.proxy.PicoProxy`
     (``set_calibration``) — the new cal takes effect on the next status
     tick without restarting the manager.

The two-point fit folds both the ACS724's untrimmed zero offset and the
divider/resistor tolerance into a single measured ``(V0, slope)`` at the
ADC pin, so ``I = (V_adc - V0) / slope`` — no reliance on nominal resistor
values. Point 1 is the system at 0 A (load off); point 2 is a known
reference current read from an inline ammeter.

Requires PicoManager to be running and the ``lidar`` device healthy. The
script never touches the serial port itself, so it can be invoked
alongside the manager.

Usage:
    calibrate-current
    calibrate-current --n-samples 20
"""

from argparse import ArgumentParser
import json
import logging
import sys
from datetime import datetime, timezone

import numpy as np
from eigsep_redis import Transport

from .buses import CurrentCalStore
from .proxy import PicoProxy

logger = logging.getLogger(__name__)

# The current sensor is hosted on the lidar Pico, so commands target the
# "lidar" device; its raw current voltage is published to a clean,
# app-agnostic stream that never names lidar.
LIDAR_NAME = "lidar"
CURRENT_STREAM = "stream:system_current"
# Producer cadence is 200 ms (STATUS_CADENCE_MS), so a 5 s budget per
# entry is ~25x — long enough to ride out a single reconnect blip but
# short enough to fail fast when the manager isn't actually publishing.
SAMPLE_TIMEOUT_S = 5.0
# Nominal slope at the ADC pin (S * k = 0.2 * 0.5875). A measured slope
# wildly off this hints at reversed sensor wiring or a wrong reference
# current — warn, but don't refuse (the operator may know better).
_NOMINAL_SLOPE = 0.2 * (4.7 / (3.3 + 4.7))


def collect_samples(transport, n):
    """Average ``n`` consecutive ``current_voltage`` entries from the stream.

    Reads only entries published after this call starts (``$``), so
    repeated calls within one calibration sweep don't double-count the
    same firmware tick.
    """
    samples = []
    last_id = "$"
    while len(samples) < n:
        remaining = n - len(samples)
        resp = transport.r.xread(
            {CURRENT_STREAM: last_id},
            block=int(SAMPLE_TIMEOUT_S * 1000),
            count=remaining,
        )
        if not resp:
            raise RuntimeError(
                f"No new entries on {CURRENT_STREAM} within "
                f"{SAMPLE_TIMEOUT_S}s. Is PicoManager publishing?"
            )
        _stream, messages = resp[0]
        for msg_id, fields in messages:
            value = json.loads(fields[b"value"])
            samples.append(value["current_voltage"])
            last_id = msg_id
    return float(np.mean(samples))


def compute_two_point(v0, v1, i_ref):
    """Compute ``(V0, slope)`` for ``I = (V_adc - V0) / slope``.

    ``v0`` is the raw ADC voltage at 0 A, ``v1`` at the known reference
    current ``i_ref``. The slope absorbs the divider ratio and the sensor
    sensitivity. Returns ``None`` (and prints why) when the inputs can't
    define a gain.
    """
    if abs(i_ref) < 1e-9:
        print("  ERROR: reference current is 0 A. Cannot fit a slope.")
        return None
    if abs(v1 - v0) < 1e-6:
        print(
            f"  ERROR: the two voltages are identical ({v0:.4f} V). "
            "No swing to fit a slope — check the reference load."
        )
        return None
    slope = (v1 - v0) / i_ref
    if slope < 0:
        print(
            f"  WARNING: negative slope ({slope:.4f} V/A) — the sensor "
            "output dropped under load. Wire IP+ -> IP- so it rises, or "
            "flip the reference-current sign."
        )
    elif not (0.5 * _NOMINAL_SLOPE <= slope <= 2.0 * _NOMINAL_SLOPE):
        print(
            f"  WARNING: slope {slope:.4f} V/A is far from nominal "
            f"{_NOMINAL_SLOPE:.4f} V/A — double-check the reference current."
        )
    return (float(v0), float(slope))


def collect_two_point(transport, n_samples):
    """Collect the 0 A point and one known-reference-current point."""
    input(
        "\nDisconnect/disable the load so the system draws 0 A, "
        "then press Enter."
    )
    print("  averaging samples...")
    v0 = collect_samples(transport, n_samples)
    print(f"  0 A voltage: {v0:.4f} V")

    raw = input(
        "\nApply a known load and enter the measured current in amps "
        "(from an inline reference ammeter): "
    )
    try:
        i_ref = float(raw)
    except ValueError:
        raise RuntimeError(f"'{raw}' is not a number.")
    print("  averaging samples...")
    v1 = collect_samples(transport, n_samples)
    print(f"  {i_ref:.4f} A voltage: {v1:.4f} V")

    return v0, v1, i_ref


def main():
    parser = ArgumentParser(
        description="Calibrate the whole-system current monitor (two-point).",
    )
    parser.add_argument(
        "-n",
        "--n-samples",
        type=int,
        default=10,
        help=(
            "Number of voltage samples to average at each point "
            "(default: 10, ~2 s at the 200 ms producer cadence)"
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

    transport = Transport(host=args.redis_host, port=args.redis_port)
    lidar_proxy = PicoProxy(LIDAR_NAME, transport, source="calibrate-current")

    if not lidar_proxy.is_available:
        print(
            f"{LIDAR_NAME} is not reachable via PicoManager. "
            "Start the manager and confirm the lidar Pico is enumerated.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(
        f"Two-point current calibration ({args.n_samples} samples/point).\n"
        f"Reading voltages from {CURRENT_STREAM}."
    )

    try:
        v0, v1, i_ref = collect_two_point(transport, args.n_samples)
    except (RuntimeError, ConnectionError) as exc:
        print(f"Calibration sample collection failed: {exc}", file=sys.stderr)
        sys.exit(1)

    cal = compute_two_point(v0, v1, i_ref)
    if cal is None:
        print("\nCalibration failed. Exiting.", file=sys.stderr)
        sys.exit(1)

    cal_data = {
        "system_current": list(cal),
        "metadata": {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "v0": float(v0),
            "v1": float(v1),
            "i_ref": float(i_ref),
            "n_samples": args.n_samples,
        },
    }

    # Persist to Redis first — if the live push later fails, the cal is
    # still stored and will load on the next PicoManager restart.
    CurrentCalStore(transport).upload(cal_data)
    # Force an RDB snapshot now so the cal survives a power loss before
    # the next scheduled save (default policy is `save 3600 1`).
    transport.r.bgsave()
    print(
        f"\nPublished calibration to Redis at "
        f"{args.redis_host}:{args.redis_port} (key: current_calibration); "
        "BGSAVE triggered."
    )

    # Push to the running PicoLidar so the new cal takes effect on the
    # next status tick.
    try:
        lidar_proxy.send_command(
            "set_calibration",
            system_current_params=list(cal),
        )
        print("Live PicoLidar updated with new calibration.")
    except (TimeoutError, RuntimeError) as e:
        print(
            f"Live cal push failed: {e}\n"
            "Calibration is stored in Redis; restart PicoManager to apply.",
            file=sys.stderr,
        )

    print(f"  system_current: I = (V_adc - {cal[0]:.4f}) / {cal[1]:.4f}")


if __name__ == "__main__":
    main()
