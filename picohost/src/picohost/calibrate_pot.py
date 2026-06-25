"""
Manual potentiometer calibration entry point.

Reads voltage samples from the running PicoManager's metadata stream
(``stream:potmon``), walks the user through a voltage-to-angle sweep,
computes a linear fit, and publishes it to two places:

  1. :class:`picohost.buses.PotCalStore` — canonical Redis store, so a
     rebooted :class:`picohost.PicoPotentiometer` picks the cal up at
     ``__init__`` time. Followed by a ``BGSAVE`` to fsync the RDB
     snapshot, since the default ``save 3600 1`` policy would otherwise
     leave a single-key write unsnapshotted for up to an hour.
  2. The running pot device via :class:`picohost.proxy.PicoProxy`
     (``set_calibration``) — the new cal takes effect on the next
     status tick without restarting the manager.

Requires PicoManager to be running and the ``potmon`` device to be
healthy. The script never touches the serial port itself, so it can
be invoked alongside the manager.

Four modes:
  --mode minmax   : bench, collect at min and max (2-point fit, default)
  --mode turns    : bench, collect at every full turn (least-squares)
  --mode azimuth  : in-box, operator drives the motor; sweep over the
                    operating turn, slope fit, zero pinned to motor-home
  --mode rezero   : in-box, re-pin the zero using the stored slope (fast;
                    needs only motor access)

Usage:
    calibrate-pot --mode azimuth
    calibrate-pot --mode rezero
"""

from argparse import ArgumentParser
import json
import logging
import math
import sys
from datetime import datetime, timezone

import numpy as np
from eigsep_redis import Transport

from .buses import PotCalStore
from .motor import steps_to_deg
from .proxy import PicoProxy

logger = logging.getLogger(__name__)

POTMON_NAME = "potmon"
POTMON_STREAM = f"stream:{POTMON_NAME}"
MOTOR_NAME = "motor"
MOTOR_STREAM = f"stream:{MOTOR_NAME}"
# Producer cadence is 200 ms (STATUS_CADENCE_MS), so a 5 s budget per
# entry is ~25x — long enough to ride out a single reconnect blip but
# short enough to fail fast when the manager isn't actually publishing.
SAMPLE_TIMEOUT_S = 5.0
# Pot wiper spans ~0..Vref, so the ADC rails approximate the pot's
# electrical ends. Mirrors firmware POTMON_VREF (src/potmon.h).
ADC_VREF = 3.3
# Warn if the operating window leaves less than this much travel (in az
# degrees) before a rail/electrical end (~0.2 turn).
HEADROOM_WARN_DEG = 72.0
# Motor az should read ~0 at home; warn beyond this if the operator
# forgot to home before pressing Enter.
HOME_AZ_TOL_DEG = 5.0


def collect_samples(transport, n):
    """Average ``n`` consecutive entries from ``stream:potmon``.

    Reads only entries published after this call starts (``$``), so
    repeated calls within one calibration sweep don't double-count the
    same firmware tick.
    """
    samples_az = []
    last_id = "$"
    while len(samples_az) < n:
        remaining = n - len(samples_az)
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
            samples_az.append(value["pot_az_voltage"])
            last_id = msg_id
    return float(np.mean(samples_az))


def read_motor_az_steps(transport, start_id="$"):
    """Return the current az_pos (motor steps) from ``stream:motor``.

    Read-only: calibrate-pot never commands the motor. Mirrors
    :func:`collect_samples`' fail-fast semantics — if PicoManager isn't
    publishing motor status within ``SAMPLE_TIMEOUT_S``, raise rather
    than silently using a stale value.
    """
    resp = transport.r.xread(
        {MOTOR_STREAM: start_id},
        block=int(SAMPLE_TIMEOUT_S * 1000),
        count=1,
    )
    if not resp:
        raise RuntimeError(
            f"No entries on {MOTOR_STREAM} within {SAMPLE_TIMEOUT_S}s. "
            "Is PicoManager publishing motor status? (motor app running "
            "and driven via motor_manual)"
        )
    _stream, messages = resp[0]
    _msg_id, fields = messages[0]
    value = json.loads(fields[b"value"])
    return float(value["az_pos"])


def read_motor_az_deg(
    transport, *, step_angle_deg, gear_teeth, microstep, start_id="$"
):
    """Current motor az in degrees (steps converted via motor geometry)."""
    steps = read_motor_az_steps(transport, start_id=start_id)
    return steps_to_deg(
        steps,
        step_angle_deg=step_angle_deg,
        gear_teeth=gear_teeth,
        microstep=microstep,
    )


def compute_linear_fit(voltages, angles):
    """Compute (m, b) such that angle = m * voltage + b via least-squares.

    Returns None if the voltage range is too small to calibrate.
    """
    voltages = np.asarray(voltages)
    span = voltages.max() - voltages.min()
    if np.abs(span) < 1e-6:
        print(
            f"  ERROR: voltage span is too small to calibrate "
            f"(range {span:.4f} V). Cannot calibrate."
        )
        return None
    m, b = np.polyfit(voltages, angles, 1)
    return (float(m), float(b))


def fit_slope_pin_zero(voltages, angles, v0):
    """Least-squares slope, with the intercept pinned to motor-home.

    Fits the best slope ``m`` over all (voltage, angle) points, then
    overrides the intercept so that ``angle = 0`` exactly at ``v0`` (the
    pot voltage at motor-home): ``b = -m * v0``. Returns ``None`` when
    the voltage span is too small to fit (delegated to
    :func:`compute_linear_fit`).
    """
    fit = compute_linear_fit(voltages, angles)
    if fit is None:
        return None
    m, _b_free = fit
    return (float(m), float(-m * v0))


def compute_fit_residuals(voltages, angles, m, b):
    """Max-abs and RMS residual (in degrees) of points about the line angle = m*V + b."""
    v = np.asarray(voltages, dtype=float)
    a = np.asarray(angles, dtype=float)
    resid = a - (m * v + b)
    return {
        "max_abs_deg": float(np.max(np.abs(resid))),
        "rms_deg": float(np.sqrt(np.mean(resid ** 2))),
    }


def compute_headroom(voltages, m, vref=ADC_VREF):
    """Margin from the swept window's endpoints to the ADC rails.

    The pot wiper spans roughly 0..vref, so distance to the rails is a
    proxy for distance to the pot's electrical ends. Degrees use the
    magnitude of the slope so both directions report positive margin.
    """
    v_lo = min(voltages)
    v_hi = max(voltages)
    deg_per_v = abs(m)
    headroom_low_v = v_lo
    headroom_high_v = vref - v_hi
    return {
        "v_lo": v_lo,
        "v_hi": v_hi,
        "span_v": v_hi - v_lo,
        "headroom_low_v": headroom_low_v,
        "headroom_high_v": headroom_high_v,
        "headroom_low_deg": headroom_low_v * deg_per_v,
        "headroom_high_deg": headroom_high_v * deg_per_v,
    }


def collect_minmax(transport, n_samples, total_degrees):
    """Collect at min and max only (2-point calibration)."""
    input("\nSet the az potentiometer to MINIMUM position, then press Enter.")
    print("  averaging samples...")
    v_min = collect_samples(transport, n_samples)
    print(f"  pot_az min voltage: {v_min:.4f} V")

    input("\nSet the az potentiometer to MAXIMUM position, then press Enter.")
    print("  averaging samples...")
    v_max = collect_samples(transport, n_samples)
    print(f"  pot_az max voltage: {v_max:.4f} V")

    voltages = [v_min, v_max]
    angles = [0.0, total_degrees]
    return voltages, angles


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
        "\nSet the az potentiometer to MINIMUM position (turn 0), "
        "then press Enter."
    )
    print("  averaging samples...")
    v = collect_samples(transport, n_samples)
    print(f"  turn  0.00: pot_az={v:.4f} V")
    voltages = [v]
    angles = [0.0]

    prev = 0.0
    for stop in _per_turn_stops(turns):
        delta = stop - prev
        input(
            f"\nAdvance the az pot {delta:.2f} turn(s) (to turn {stop:.2f}), "
            "then press Enter."
        )
        print("  averaging samples...")
        v = collect_samples(transport, n_samples)
        print(f"  turn {stop:5.2f}: pot_az={v:.4f} V")
        voltages.append(v)
        angles.append(stop * 360.0)
        prev = stop

    return voltages, angles


def collect_azimuth(transport, n_samples, motor_cfg):
    """In-box sweep: operator drives the motor; we record (az, voltage).

    The operator moves the motor with ``motor_manual`` and presses Enter
    at each stop; calibrate-pot reads the current az from ``stream:motor``
    (read-only) and averages the pot voltage. The first stop is motor-home
    and *defines* az=0. Returns ``(voltages, angles, v0)``.
    """
    input("\nDrive the motor to HOME (az 0), stop there, then press Enter.")
    az_home = read_motor_az_deg(transport, **motor_cfg)
    if abs(az_home) > HOME_AZ_TOL_DEG:
        print(
            f"  WARNING: motor az reads {az_home:.1f} deg at 'home' "
            "(expected ~0). Did you home the motor first?"
        )
    print("  averaging samples...")
    v0 = collect_samples(transport, n_samples)
    print(f"  home: az=0.00 deg (motor reads {az_home:.2f}), pot_az={v0:.4f} V")
    voltages = [v0]
    angles = [0.0]

    while True:
        resp = input(
            "\nDrive to the next stop, stop there, then press Enter "
            "(or type 'q' then Enter to finish): "
        ).strip().lower()
        if resp == "q":
            break
        az = read_motor_az_deg(transport, **motor_cfg)
        print("  averaging samples...")
        v = collect_samples(transport, n_samples)
        print(f"  az={az:8.2f} deg: pot_az={v:.4f} V")
        voltages.append(v)
        angles.append(az)

    return voltages, angles, v0


def rezero(transport, n_samples):
    """Re-pin the zero using the stored slope (needs only motor access).

    Loads the slope ``m`` from :class:`PotCalStore`, captures the pot
    voltage at motor-home, and returns ``((m, -m*v0), v0)``. The slope is
    reused verbatim — never re-fit. Raises if no calibration is stored.
    """
    stored = PotCalStore(transport).get()
    if not stored or "pot_az" not in stored:
        raise RuntimeError(
            "No stored calibration to re-zero. Run '--mode azimuth' (or a "
            "bench mode) first to establish the slope."
        )
    m = float(stored["pot_az"][0])
    input("\nDrive the motor to HOME (az 0), stop there, then press Enter.")
    print("  averaging samples...")
    v0 = collect_samples(transport, n_samples)
    b = -m * v0
    print(f"  reused slope m={m:.4f}; V0={v0:.4f} V -> new intercept b={b:.4f}")
    return (m, b), v0


def build_parser():
    parser = ArgumentParser(
        description="Calibrate potentiometer voltage-to-angle mapping.",
    )
    parser.add_argument(
        "-t", "--turns", type=float, default=10.0,
        help=(
            "Total turns from min to max for bench modes. Fractional "
            "values supported (e.g. 3.75). Default: 10."
        ),
    )
    parser.add_argument(
        "-n", "--n-samples", type=int, default=10,
        help=(
            "Voltage samples to average per position "
            "(default: 10, ~2 s at the 200 ms producer cadence)"
        ),
    )
    parser.add_argument(
        "-m", "--mode", type=str,
        choices=["minmax", "turns", "azimuth", "rezero"],
        default="minmax",
        help=(
            "Calibration mode. Bench: 'minmax' (2-point), 'turns' "
            "(per-turn least-squares). In-box (motor-driven): 'azimuth' "
            "(sweep + zero pinned to motor-home), 'rezero' (re-pin zero "
            "with the stored slope). Default: minmax."
        ),
    )
    parser.add_argument(
        "--step-angle-deg", type=float, default=1.8,
        help="Motor step angle in degrees (mirrors PicoMotor; default 1.8).",
    )
    parser.add_argument(
        "--gear-teeth", type=int, default=113,
        help="Motor gear teeth (mirrors PicoMotor; default 113).",
    )
    parser.add_argument(
        "--microstep", type=int, default=1,
        help="Motor microstep divisor (mirrors PicoMotor; default 1). "
             "MUST match the deployed motor or the slope scales wrong.",
    )
    parser.add_argument(
        "--redis-host", default="localhost",
        help="Redis host for the running PicoManager",
    )
    parser.add_argument(
        "--redis-port", type=int, default=6379,
        help="Redis port for the running PicoManager",
    )
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    if args.mode in ("minmax", "turns") and args.turns <= 0:
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

    if args.mode in ("minmax", "turns"):
        print(
            f"Mode: {args.mode} ({args.turns:g} turns, {args.n_samples} "
            f"samples/position)"
        )
    else:
        print(f"Mode: {args.mode} ({args.n_samples} samples/position)")
    print(f"Reading voltages from {POTMON_STREAM}.")

    motor_cfg = {
        "step_angle_deg": args.step_angle_deg,
        "gear_teeth": args.gear_teeth,
        "microstep": args.microstep,
    }
    headroom = None
    free = None
    resid = None
    try:
        if args.mode == "minmax":
            voltages, angles = collect_minmax(
                transport, args.n_samples, total_degrees
            )
            cal = compute_linear_fit(voltages, angles)
        elif args.mode == "turns":
            voltages, angles = collect_per_turn(
                transport, args.n_samples, args.turns
            )
            cal = compute_linear_fit(voltages, angles)
        elif args.mode == "azimuth":
            voltages, angles, v0 = collect_azimuth(
                transport, args.n_samples, motor_cfg
            )
            if len(voltages) < 2:
                print(
                    "\nNeed at least one stop beyond home to fit a slope.",
                    file=sys.stderr,
                )
                sys.exit(1)
            cal = fit_slope_pin_zero(voltages, angles, v0)
            if cal is not None:
                headroom = compute_headroom(voltages, cal[0])
                free = compute_linear_fit(voltages, angles)
                resid = compute_fit_residuals(voltages, angles, free[0], free[1])
        else:  # rezero
            cal, v0 = rezero(transport, args.n_samples)
            voltages, angles = [v0], [0.0]
    except (RuntimeError, ConnectionError) as exc:
        print(f"Calibration sample collection failed: {exc}", file=sys.stderr)
        sys.exit(1)

    if cal is None:
        print("\nCalibration failed. Exiting.", file=sys.stderr)
        sys.exit(1)

    if free is not None and resid is not None:
        print("\nLinearity check (azimuth fit):")
        print(f"  slope m = {cal[0]:.4f} deg/V")
        print(
            f"  intercept: pinned b = {cal[1]:.4f}  (free-fit b = {free[1]:.4f})"
        )
        print(
            f"  residuals about free-fit line: "
            f"max |{resid['max_abs_deg']:.2f}| deg, RMS {resid['rms_deg']:.2f} deg"
        )

    if headroom is not None:
        print("\nHeadroom to the pot's electrical ends (via the ADC rails):")
        print(
            f"  swept window: {headroom['v_lo']:.4f}..{headroom['v_hi']:.4f} V "
            f"(span {headroom['span_v']:.4f} V)"
        )
        print(
            f"  margin to 0 V rail:   {headroom['headroom_low_v']:.4f} V "
            f"~ {headroom['headroom_low_deg']:.0f} deg"
        )
        print(
            f"  margin to {ADC_VREF:.1f} V rail: {headroom['headroom_high_v']:.4f} V "
            f"~ {headroom['headroom_high_deg']:.0f} deg"
        )
        if min(
            headroom["headroom_low_deg"], headroom["headroom_high_deg"]
        ) < HEADROOM_WARN_DEG:
            print(
                f"  WARNING: less than {HEADROOM_WARN_DEG:.0f} deg of margin on "
                "one side — risk of hitting the pot's hard stop in operation."
            )

    cal_data = {
        "pot_az": list(cal),
        "metadata": {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "mode": args.mode,
            "n_points": len(angles),
            "pot_az_voltages": [float(v) for v in voltages],
            "angles": [float(a) for a in angles],
            "n_samples": args.n_samples,
        },
    }
    if args.mode in ("minmax", "turns"):
        cal_data["metadata"]["turns"] = float(args.turns)
        cal_data["metadata"]["total_degrees"] = total_degrees
    if args.mode == "azimuth":
        cal_data["metadata"]["motor_cfg"] = motor_cfg
        cal_data["metadata"]["free_fit_intercept"] = float(free[1])
        cal_data["metadata"]["residual_max_deg"] = resid["max_abs_deg"]
        cal_data["metadata"]["residual_rms_deg"] = resid["rms_deg"]
    if args.mode == "rezero":
        cal_data["metadata"]["slope_reused"] = True

    # Persist to Redis first — if the live push later fails, the cal is
    # still stored and will load on the next PicoManager restart.
    PotCalStore(transport).upload(cal_data)
    # Force an RDB snapshot now so the cal survives a power loss before
    # the next scheduled save (default policy is `save 3600 1`, which
    # would leave this single-key write unsnapshotted for up to an hour).
    transport.r.bgsave()
    print(
        f"\nPublished calibration to Redis at "
        f"{args.redis_host}:{args.redis_port} (key: pot_calibration); "
        "BGSAVE triggered."
    )

    # Push to the running PicoPotentiometer so the new cal takes effect
    # on the next status tick.
    try:
        pot_proxy.send_command(
            "set_calibration",
            pot_az_params=list(cal),
        )
        print("Live PicoPotentiometer updated with new calibration.")
    except (TimeoutError, RuntimeError) as e:
        print(
            f"Live cal push failed: {e}\n"
            "Calibration is stored in Redis; restart PicoManager to apply.",
            file=sys.stderr,
        )

    print(f"  pot_az: angle = {cal[0]:.4f} * V + {cal[1]:.4f}")
    if args.mode == "turns":
        print(f"  ({len(angles)} points used for least-squares fit)")


if __name__ == "__main__":
    main()
