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
    calibrate-current                      # two-point (default)
    calibrate-current --mode multi         # N-point, loop until done
    calibrate-current --currents 0,1,2,5,8 # N-point preset sweep
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
    _warn_on_slope(slope)
    return (float(v0), float(slope))


def _warn_on_slope(slope):
    """Print a heuristic warning when the fitted slope looks wrong.

    Shared by the two-point and multi-point fits. Never raises and never
    blocks — the operator may legitimately know better.
    """
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


def _parse_currents(raw):
    """Parse a comma-separated current list like ``'0,1,2,5,8'`` to floats."""
    return [float(x) for x in raw.split(",") if x.strip() != ""]


def _residual_threshold_a(currents):
    """Acceptable fit residual (A): 20 mA floor, or 2% of the max current."""
    max_current = max((abs(c) for c in currents), default=0.0)
    return max(0.020, 0.02 * max_current)


def compute_multi_point(currents, voltages):
    """Least-squares ``(V0, slope)`` fit of ``V_adc = slope * I + V0``.

    Parameters
    ----------
    currents : sequence of float
        Known reference currents (A) at each calibration point.
    voltages : sequence of float
        Averaged raw ADC-pin voltage (V) at each point.

    Returns
    -------
    tuple or None
        ``((V0, slope), quality)`` where ``quality`` is a dict with
        ``residual_rms_v``, ``residual_rms_a`` and ``r_squared``; or
        ``None`` (with a printed reason) when the points cannot define a
        gain. Downstream conversion is ``I = (V_adc - V0) / slope``.
    """
    currents = np.asarray(currents, dtype=float)
    voltages = np.asarray(voltages, dtype=float)
    if np.unique(currents).size < 2:
        print("  ERROR: need at least 2 distinct reference currents to fit.")
        return None
    if np.ptp(voltages) < 1e-6:
        print(
            f"  ERROR: voltage spread < 1 uV ({voltages.min():.4f} V). "
            "No swing to fit a slope — check the reference loads."
        )
        return None
    slope, v0 = np.polyfit(currents, voltages, 1)
    residuals = voltages - (slope * currents + v0)
    residual_rms_v = float(np.sqrt(np.mean(residuals ** 2)))
    ss_res = float(np.sum(residuals ** 2))
    ss_tot = float(np.sum((voltages - voltages.mean()) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    residual_rms_a = (
        abs(residual_rms_v / float(slope)) if slope != 0 else float("inf")
    )
    _warn_on_slope(slope)
    quality = {
        "residual_rms_v": residual_rms_v,
        "residual_rms_a": residual_rms_a,
        "r_squared": r_squared,
    }
    return ((float(v0), float(slope)), quality)


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


def collect_multi_point(transport, n_samples, currents=None):
    """Collect ``(currents, voltages)`` for a multi-point calibration.

    With ``currents`` given, walk that preset list, prompting at each
    target (blank Enter accepts the target as the reading). Otherwise loop
    until the operator enters a blank line, requiring at least 3 points.
    Each point averages ``n_samples`` ADC voltages via ``collect_samples``.
    """
    measured_currents = []
    voltages = []

    if currents is not None:
        for target in currents:
            while True:
                raw = input(
                    f"\nDial load to ~{target:g} A; enter measured current "
                    f"[{target:g}]: "
                ).strip()
                if raw == "":
                    i_ref = target
                    break
                try:
                    i_ref = float(raw)
                    break
                except ValueError:
                    print(f"  '{raw}' is not a number; try again.")
            print("  averaging samples...")
            v = collect_samples(transport, n_samples)
            print(f"  {i_ref:.4f} A: {v:.4f} V")
            measured_currents.append(float(i_ref))
            voltages.append(v)
        return measured_currents, voltages

    print(
        "\nEnter the measured current (A) at each point; blank line when "
        "done (need >= 3 points). Include a 0 A point (load off)."
    )
    while True:
        raw = input(
            f"\nPoint {len(measured_currents) + 1} current [A] "
            "(blank to finish): "
        ).strip()
        if raw == "":
            if len(measured_currents) >= 3:
                break
            print(
                f"  need >= 3 points, have {len(measured_currents)}; "
                "keep going."
            )
            continue
        try:
            i_ref = float(raw)
        except ValueError:
            print(f"  '{raw}' is not a number; try again.")
            continue
        print("  averaging samples...")
        v = collect_samples(transport, n_samples)
        print(f"  {i_ref:.4f} A: {v:.4f} V")
        measured_currents.append(i_ref)
        voltages.append(v)

    return measured_currents, voltages


def _print_point_table(currents, voltages, cal):
    """Print a per-point I / V / residual table so a bad point is obvious."""
    v0, slope = cal
    print("\n  point  I_ref [A]  V_adc [V]  resid [mV]  resid [mA]")
    for idx, (i, v) in enumerate(zip(currents, voltages), start=1):
        fit_v = slope * i + v0
        dv = v - fit_v
        di = dv / slope if slope else float("inf")
        print(
            f"  {idx:>5}{i:11.4f}{v:11.4f}{dv * 1e3:12.2f}{di * 1e3:12.2f}"
        )


def _build_multi_cal_data(cal, n_samples, currents, voltages, quality):
    """Assemble the Redis payload (system_current + metadata) for multi mode."""
    return {
        "system_current": list(cal),
        "metadata": {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "mode": "multi",
            "n_samples": n_samples,
            "n_points": len(currents),
            "currents": [float(c) for c in currents],
            "voltages": [float(v) for v in voltages],
            "residual_rms_v": quality["residual_rms_v"],
            "residual_rms_a": quality["residual_rms_a"],
            "r_squared": quality["r_squared"],
        },
    }


def main():
    parser = ArgumentParser(
        description="Calibrate the whole-system current monitor.",
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
        "-m",
        "--mode",
        choices=["two-point", "multi"],
        default="two-point",
        help=(
            "Calibration mode: 'two-point' (default) for the 0 A + one "
            "reference flow, 'multi' for an N-point least-squares fit."
        ),
    )
    parser.add_argument(
        "--currents",
        default=None,
        help=(
            "Comma-separated known currents (A) for a preset multi-point "
            "sweep, e.g. '0,1,2,5,8'. Implies --mode multi (>= 3 points)."
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

    preset = _parse_currents(args.currents) if args.currents else None
    mode = "multi" if preset is not None else args.mode

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
        f"{mode} current calibration ({args.n_samples} samples/point).\n"
        f"Reading voltages from {CURRENT_STREAM}."
    )

    try:
        if mode == "multi":
            if preset is not None and len(preset) < 3:
                print(
                    "--currents needs >= 3 points for a multi-point fit.",
                    file=sys.stderr,
                )
                sys.exit(1)
            currents, voltages = collect_multi_point(
                transport, args.n_samples, preset
            )
        else:
            v0, v1, i_ref = collect_two_point(transport, args.n_samples)
    except (RuntimeError, ConnectionError) as exc:
        print(f"Calibration sample collection failed: {exc}", file=sys.stderr)
        sys.exit(1)

    if mode == "multi":
        result = compute_multi_point(currents, voltages)
        if result is None:
            print("\nCalibration failed. Exiting.", file=sys.stderr)
            sys.exit(1)
        cal, quality = result
        _print_point_table(currents, voltages, cal)
        print(
            f"\n  residual RMS: {quality['residual_rms_v'] * 1e3:.2f} mV "
            f"({quality['residual_rms_a'] * 1e3:.2f} mA), "
            f"r^2 = {quality['r_squared']:.5f}"
        )
        threshold = _residual_threshold_a(currents)
        if quality["residual_rms_a"] > threshold:
            print(
                f"\n  WARNING: residual "
                f"{quality['residual_rms_a'] * 1e3:.2f} mA exceeds "
                f"{threshold * 1e3:.1f} mA — possible bad point or sensor "
                "nonlinearity."
            )
            ans = input("  Store this calibration anyway? [y/N]: ").strip()
            if ans.lower() not in ("y", "yes"):
                print("Aborted; calibration not stored.", file=sys.stderr)
                sys.exit(1)
        cal_data = _build_multi_cal_data(
            cal, args.n_samples, currents, voltages, quality
        )
    else:
        cal = compute_two_point(v0, v1, i_ref)
        if cal is None:
            print("\nCalibration failed. Exiting.", file=sys.stderr)
            sys.exit(1)
        cal_data = {
            "system_current": list(cal),
            "metadata": {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "mode": "two-point",
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
