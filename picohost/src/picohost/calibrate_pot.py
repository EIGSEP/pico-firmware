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

Six modes:
  --mode minmax   : bench, collect at min and max (2-point fit)
  --mode turns    : bench, collect at every full turn (least-squares)
  --mode azimuth  : in-box, operator drives the motor; sweep over the
                    operating turn, slope fit, zero pinned to motor-home
                    (default)
  --mode auto     : in-box, calibrate-pot drives the motor itself through
                    the same sweep as azimuth (moves are non-blocking;
                    settle is detected by polling stream:motor) -- no
                    operator needed at each stop, just to home first
  --mode rezero   : in-box, re-pin the zero using the stored slope (fast;
                    needs only motor access)
  --mode manual   : recovery, write a hand-supplied --slope/--intercept
                    directly (no sweep) to restore the cal after a
                    catastrophic Redis loss (e.g. Pi swap); the two numbers
                    are read off a recent correlator .h5. Writes Redis even
                    when the pot is down (loads on the next manager start).

Every mode runs a physical slope sanity check: a 3.75-turn pot over the
~3.3 V ADC range has a slope of ~409 deg/V, so a slope off by more than
1.5x prints a WARNING and requires a typed 'yes' to save.

Usage:
    calibrate-pot --mode azimuth
    calibrate-pot --mode auto
    calibrate-pot --mode rezero
    calibrate-pot --mode manual --slope 409.1 --intercept -400.0 \\
        --note "restored from corr_20260615.h5"
"""

from argparse import ArgumentParser
import json
import logging
import math
import sys
import time
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
# --mode auto: a commanded move is considered settled once two
# consecutive stream:motor reads agree with each other and with the
# target within this many degrees.
SETTLE_TOL_DEG = 2.0
# --mode auto: overall budget to settle a single commanded move.
# Comfortably larger than a full 360 deg turn (~2 min at the installed
# motor's default speed).
SETTLE_TIMEOUT_S = 180.0
# Warn before saving if the new calibration would move the predicted
# azimuth by more than this (deg) anywhere in the swept window, relative
# to the calibration already stored in Redis.
WILDLY_DIFFERENT_WARN_DEG = 30.0
# Physical sanity bound on the slope, independent of the stored cal. A
# full-travel `turns`-turn pot whose wiper spans ADC_VREF has a slope of
# turns*360/Vref deg/V (~409 for the installed 3.75-turn pot over 3.3 V).
# Warn (and escalate the save to a typed 'yes') when the computed or
# operator-supplied |slope| is off by more than this factor — catches
# order-of-magnitude typos (manual mode) and gross sweep/turn-count errors.
# 1.5x -> an in-range window of ~273..614 deg/V for the installed pot.
SLOPE_SANITY_FACTOR = 1.5


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

    Read-only itself (this call never commands the motor), but
    ``--mode auto`` does command the motor elsewhere via
    :func:`collect_auto` and then uses this function to poll for
    settle. Mirrors :func:`collect_samples`' fail-fast semantics — if
    PicoManager isn't publishing motor status within
    ``SAMPLE_TIMEOUT_S``, raise rather than silently using a stale
    value.
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
        "rms_deg": float(np.sqrt(np.mean(resid**2))),
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


def predicted_angle_divergence(new_cal, old_cal, voltages):
    """Max |new(V) - old(V)| in degrees over the swept voltage window.

    Both calibrations are linear (angle = m*V + b), so the two lines
    diverge most at an endpoint of ``[min(voltages), max(voltages)]``.

    Parameters
    ----------
    new_cal : tuple
        The freshly computed ``(slope, intercept)``.
    old_cal : dict or None
        The stored calibration from :meth:`PotCalStore.get`, shaped
        ``{"pot_az": [m, b], ...}``, or ``None``.
    voltages : sequence of float
        The pot voltages swept on this run.

    Returns
    -------
    float or None
        Divergence in degrees, or ``None`` when there is no usable stored
        calibration to compare against (``old_cal`` is ``None``, lacks a
        ``pot_az`` entry, or its ``pot_az`` is not a numeric ``(m, b)``
        pair). The caller then skips the "wildly different" warning.
    """
    if not old_cal or "pot_az" not in old_cal:
        return None
    pair = old_cal["pot_az"]
    try:
        m_old, b_old = float(pair[0]), float(pair[1])
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    m_new, b_new = float(new_cal[0]), float(new_cal[1])
    v_lo, v_hi = min(voltages), max(voltages)
    return max(
        abs((m_new * v + b_new) - (m_old * v + b_old)) for v in (v_lo, v_hi)
    )


def expected_slope_mag(turns, vref=ADC_VREF):
    """Expected |slope| in deg/V for a full-travel ``turns``-turn pot.

    The wiper spans ~0..``vref``, so a pot of ``turns`` full turns
    (``turns*360`` mechanical degrees) gives ``turns*360/vref`` deg/V.
    """
    return turns * 360.0 / vref


def slope_out_of_range(m, turns, vref=ADC_VREF, factor=SLOPE_SANITY_FACTOR):
    """True when ``|m|`` diverges from the expected slope by more than ``factor``.

    Magnitude-only (slope sign is a wiring-direction convention). A zero
    slope, or a non-positive expected slope, is always out of range.
    """
    expected = expected_slope_mag(turns, vref)
    if expected <= 0 or m == 0:
        return True
    ratio = max(abs(m) / expected, expected / abs(m))
    return ratio > factor


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


def _warn_if_not_homed(az_home):
    """Print the "did you home the motor?" warning if ``az_home`` is off.

    Shared by :func:`collect_azimuth` (operator-driven) and
    :func:`collect_auto` (motor-driven) so the message can't drift
    between the two sweep modes.
    """
    if abs(az_home) > HOME_AZ_TOL_DEG:
        print(
            f"  WARNING: motor az reads {az_home:.1f} deg at 'home' "
            "(expected ~0). Did you home the motor first?"
        )


def collect_azimuth(transport, n_samples, motor_cfg):
    """In-box sweep: operator drives the motor; we record (az, voltage).

    The operator moves the motor with ``motor_manual`` and presses Enter
    at each stop; calibrate-pot reads the current az from ``stream:motor``
    (read-only) and averages the pot voltage. The first stop is motor-home
    and *defines* az=0. Returns ``(voltages, angles, v0)``.
    """
    input("\nDrive the motor to HOME (az 0), stop there, then press Enter.")
    az_home = read_motor_az_deg(transport, **motor_cfg)
    _warn_if_not_homed(az_home)
    print("  averaging samples...")
    v0 = collect_samples(transport, n_samples)
    print(
        f"  home: az=0.00 deg (motor reads {az_home:.2f}), pot_az={v0:.4f} V"
    )
    voltages = [v0]
    angles = [0.0]

    while True:
        resp = (
            input(
                "\nDrive to the next stop, stop there, then press Enter "
                "(or type 'q' then Enter to finish): "
            )
            .strip()
            .lower()
        )
        if resp == "q":
            break
        az = read_motor_az_deg(transport, **motor_cfg)
        print("  averaging samples...")
        v = collect_samples(transport, n_samples)
        print(f"  az={az:8.2f} deg: pot_az={v:.4f} V")
        voltages.append(v)
        angles.append(az)

    return voltages, angles, v0


def _wait_for_settle(
    transport,
    motor_cfg,
    target_deg,
    *,
    tol_deg=SETTLE_TOL_DEG,
    timeout_s=SETTLE_TIMEOUT_S,
):
    """Poll ``stream:motor`` until az settles at ``target_deg``.

    Moves are commanded non-blocking (see :func:`collect_auto`), so the
    caller must poll for settle itself rather than waiting on the proxy.
    "Settled" requires two consecutive reads that agree with each other
    and with the target within ``tol_deg`` -- a single sample could catch
    the motor mid-move if it happens to cross the tolerance band. Returns
    the settled az. Raises ``TimeoutError`` if the motor hasn't settled
    within ``timeout_s``.
    """
    deadline = time.monotonic() + timeout_s
    prev_az = None
    while time.monotonic() < deadline:
        az = read_motor_az_deg(transport, start_id="$", **motor_cfg)
        if (
            prev_az is not None
            and abs(az - target_deg) <= tol_deg
            and abs(az - prev_az) <= tol_deg
        ):
            return az
        prev_az = az
    raise TimeoutError(
        f"Motor did not settle at {target_deg:.1f} deg (tol {tol_deg:.1f} "
        f"deg) within {timeout_s:.0f}s (last read: {prev_az})"
    )


def collect_auto(
    transport,
    motor_proxy,
    n_samples,
    motor_cfg,
    n_stops=8,
    settle_tol_deg=SETTLE_TOL_DEG,
    settle_timeout_s=SETTLE_TIMEOUT_S,
):
    """In-box sweep: calibrate-pot drives the motor itself.

    Same sweep as :func:`collect_azimuth`, but instead of an operator
    driving the motor and pressing Enter at each stop, this commands the
    motor through ``n_stops`` stops evenly spaced over one 360 deg
    operating turn (default 8 -> 45 deg spacing). The operator is
    assumed to have homed the motor beforehand (az~=0 at the start of
    this call); the first sample (at the assumed-home position) defines
    az=0 and v0, exactly as in :func:`collect_azimuth`.

    Moves are sent non-blocking (``wait_for_stop=False``) because the
    manager runs routed commands synchronously on its command thread and
    a full-turn move can take ~2 minutes -- blocking there would stall
    every other command against the fleet. Settle is instead detected by
    this function polling ``stream:motor`` (:func:`_wait_for_settle`).

    The motor is soft-claimed via ``motor_proxy`` for the duration of the
    sweep and released in a ``finally`` block (claims are advisory, not
    enforced -- see ``manager.py``). Returns ``(voltages, angles, v0)``,
    matching :func:`collect_azimuth`.

    Raises
    ------
    RuntimeError, TimeoutError
        On any command or settle failure mid-sweep. This function never
        returns a partial result on failure -- the exception propagates
        before ``return``, so the caller cannot mistakenly save a fit
        from an interrupted sweep.
    """
    claim_ttl = int(settle_timeout_s * (n_stops + 2))
    motor_proxy.send_command("claim", ttl=claim_ttl)
    try:
        az_home = read_motor_az_deg(transport, **motor_cfg)
        _warn_if_not_homed(az_home)
        print("  averaging samples...")
        v0 = collect_samples(transport, n_samples)
        print(
            f"  home: az=0.00 deg (motor reads {az_home:.2f}), "
            f"pot_az={v0:.4f} V"
        )
        voltages = [v0]
        angles = [0.0]

        for i in range(1, n_stops + 1):
            target = i * (360.0 / n_stops)
            print(f"\nMoving to az {target:.1f} deg...")
            motor_proxy.send_command(
                "az_target_deg",
                target_deg=target,
                wait_for_start=False,
                wait_for_stop=False,
            )
            az = _wait_for_settle(
                transport,
                motor_cfg,
                target,
                tol_deg=settle_tol_deg,
                timeout_s=settle_timeout_s,
            )
            print("  averaging samples...")
            v = collect_samples(transport, n_samples)
            print(
                f"  az={az:8.2f} deg (target {target:.1f}): pot_az={v:.4f} V"
            )
            voltages.append(v)
            angles.append(az)

        print("\nReturning motor to home (az 0)...")
        motor_proxy.send_command(
            "az_target_deg",
            target_deg=0.0,
            wait_for_start=False,
            wait_for_stop=False,
        )
        _wait_for_settle(
            transport,
            motor_cfg,
            0.0,
            tol_deg=settle_tol_deg,
            timeout_s=settle_timeout_s,
        )
    finally:
        motor_proxy.send_command("release")

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
    print(
        f"  reused slope m={m:.4f}; V0={v0:.4f} V -> new intercept b={b:.4f}"
    )
    return (m, b), v0


def prompt_save(require_confirm):
    """Ask whether to persist the calibration. Returns True to save.

    Safe default: bare Enter (or anything other than y/yes) discards. When
    ``require_confirm`` is True (a flagged calibration — sharply different
    from the stored one, or a slope that fails the physical sanity check)
    the operator must type the full word ``yes`` to confirm.
    """
    if require_confirm:
        resp = input("Type 'yes' to confirm: ").strip().lower()
        return resp == "yes"
    resp = input("Save this calibration? [y/N]: ").strip().lower()
    return resp in ("y", "yes")


def build_parser():
    parser = ArgumentParser(
        description="Calibrate potentiometer voltage-to-angle mapping.",
    )
    parser.add_argument(
        "-t",
        "--turns",
        type=float,
        default=3.75,
        help=(
            "Total turns from min to max for bench modes. Fractional "
            "values supported (e.g. 3.75). Default: 3.75 (the installed pot)."
        ),
    )
    parser.add_argument(
        "-n",
        "--n-samples",
        type=int,
        default=10,
        help=(
            "Voltage samples to average per position "
            "(default: 10, ~2 s at the 200 ms producer cadence)"
        ),
    )
    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        choices=["minmax", "turns", "azimuth", "auto", "rezero", "manual"],
        default="azimuth",
        help=(
            "Calibration mode. Bench: 'minmax' (2-point), 'turns' "
            "(per-turn least-squares). In-box (motor-driven): 'azimuth' "
            "(operator drives the motor; sweep + zero pinned to "
            "motor-home), 'auto' (calibrate-pot drives the motor itself "
            "through the same sweep -- non-blocking moves, settle "
            "detected by polling stream:motor), 'rezero' (re-pin zero "
            "with the stored slope). Recovery: 'manual' (write a "
            "hand-supplied --slope/--intercept directly, no sweep). "
            "Default: azimuth."
        ),
    )
    parser.add_argument(
        "--n-stops",
        type=int,
        default=8,
        help=(
            "Number of stops after home for --mode auto, evenly spaced "
            "over one 360 deg turn (default: 8, i.e. 45 deg spacing)."
        ),
    )
    parser.add_argument(
        "--slope",
        type=float,
        default=None,
        help="Slope m (deg/V) for --mode manual. Required for that mode.",
    )
    parser.add_argument(
        "--intercept",
        type=float,
        default=None,
        help="Intercept b (deg) for --mode manual. Required for that mode.",
    )
    parser.add_argument(
        "--note",
        type=str,
        default=None,
        help=(
            "Free-text provenance recorded in the calibration metadata, "
            "e.g. 'restored from corr_20260615.h5'. Useful with --mode manual."
        ),
    )
    parser.add_argument(
        "--step-angle-deg",
        type=float,
        default=1.8,
        help="Motor step angle in degrees (mirrors PicoMotor; default 1.8).",
    )
    parser.add_argument(
        "--gear-teeth",
        type=int,
        default=113,
        help="Motor gear teeth (mirrors PicoMotor; default 113).",
    )
    parser.add_argument(
        "--microstep",
        type=int,
        default=1,
        help="Motor microstep divisor (mirrors PicoMotor; default 1). "
        "MUST match the deployed motor or the slope scales wrong.",
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

    if args.mode == "auto" and args.n_stops <= 0:
        print("n-stops must be positive.", file=sys.stderr)
        sys.exit(1)

    if args.mode == "manual" and (
        args.slope is None or args.intercept is None
    ):
        print(
            "--mode manual requires both --slope and --intercept.",
            file=sys.stderr,
        )
        sys.exit(1)

    transport = Transport(host=args.redis_host, port=args.redis_port)
    pot_proxy = PicoProxy(POTMON_NAME, transport, source="calibrate-pot")

    # Manual mode is a recovery path: it needs nothing from the pot to
    # compute the cal, so it writes Redis even when the pot is down (the
    # cal then loads on the next PicoManager start). Every other mode
    # samples the live stream, so it still hard-requires the pot.
    if args.mode != "manual" and not pot_proxy.is_available:
        print(
            f"{POTMON_NAME} is not reachable via PicoManager. "
            "Start the manager and confirm the pot Pico is enumerated.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Auto mode additionally commands the motor itself, so it hard-requires
    # the motor's heartbeat too (azimuth mode only reads the motor stream,
    # which the pot-availability check above doesn't cover either, but that
    # mode is operator-driven so a stalled read just blocks on input()).
    motor_proxy = None
    if args.mode == "auto":
        motor_proxy = PicoProxy(MOTOR_NAME, transport, source="calibrate-pot")
        if not motor_proxy.is_available:
            print(
                f"{MOTOR_NAME} is not reachable via PicoManager. "
                "Start the manager and confirm the motor Pico is enumerated.",
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
        elif args.mode in ("azimuth", "auto"):
            if args.mode == "azimuth":
                voltages, angles, v0 = collect_azimuth(
                    transport, args.n_samples, motor_cfg
                )
            else:
                voltages, angles, v0 = collect_auto(
                    transport,
                    motor_proxy,
                    args.n_samples,
                    motor_cfg,
                    n_stops=args.n_stops,
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
                resid = compute_fit_residuals(
                    voltages, angles, free[0], free[1]
                )
        elif args.mode == "manual":
            # Recovery path: no sweep — the operator supplies (m, b) directly.
            cal = (args.slope, args.intercept)
            voltages, angles = [], []
        else:  # rezero
            cal, v0 = rezero(transport, args.n_samples)
            voltages, angles = [v0], [0.0]
    except (RuntimeError, ConnectionError, TimeoutError) as exc:
        print(f"Calibration sample collection failed: {exc}", file=sys.stderr)
        sys.exit(1)

    if cal is None:
        print("\nCalibration failed. Exiting.", file=sys.stderr)
        sys.exit(1)

    if free is not None and resid is not None:
        print(f"\nLinearity check ({args.mode} fit):")
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
        if (
            min(headroom["headroom_low_deg"], headroom["headroom_high_deg"])
            < HEADROOM_WARN_DEG
        ):
            print(
                f"  WARNING: less than {HEADROOM_WARN_DEG:.0f} deg of margin on "
                "one side — risk of hitting the pot's hard stop in operation."
            )

    metadata = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "mode": args.mode,
    }
    # Manual mode has no sweep, so the sample-derived fields are meaningless.
    if args.mode != "manual":
        metadata["n_points"] = len(angles)
        metadata["pot_az_voltages"] = [float(v) for v in voltages]
        metadata["angles"] = [float(a) for a in angles]
        metadata["n_samples"] = args.n_samples
    if args.mode in ("minmax", "turns"):
        metadata["turns"] = float(args.turns)
        metadata["total_degrees"] = total_degrees
    if args.mode in ("azimuth", "auto"):
        metadata["motor_cfg"] = motor_cfg
        metadata["free_fit_intercept"] = float(free[1])
        metadata["residual_max_deg"] = resid["max_abs_deg"]
        metadata["residual_rms_deg"] = resid["rms_deg"]
    if args.mode == "auto":
        metadata["n_stops"] = args.n_stops
    if args.mode == "rezero":
        metadata["slope_reused"] = True
    if args.note:
        metadata["note"] = args.note
    cal_data = {"pot_az": list(cal), "metadata": metadata}

    # Show the operator what was computed, then compare against the stored
    # calibration before writing anything.
    print(f"\nComputed calibration: angle = {cal[0]:.4f} * V + {cal[1]:.4f}")
    if args.mode == "turns":
        print(f"  ({len(angles)} points used for least-squares fit)")

    stored = PotCalStore(transport).get()
    # Manual mode has no swept window to compare; the operator is typing
    # authoritative numbers, so skip the divergence-vs-stored check.
    if args.mode == "manual":
        divergence = None
    else:
        divergence = predicted_angle_divergence(cal, stored, voltages)
    diverged = (
        divergence is not None and divergence > WILDLY_DIFFERENT_WARN_DEG
    )
    if diverged:
        m_old, b_old = float(stored["pot_az"][0]), float(stored["pot_az"][1])
        print(
            f"\nWARNING: this calibration differs from the stored one by up "
            f"to {divergence:.0f} deg over the swept window "
            f"(threshold {WILDLY_DIFFERENT_WARN_DEG:.0f} deg)."
        )
        print(f"  stored: angle = {m_old:.4f} * V + {b_old:.4f}")
        print(f"  new:    angle = {cal[0]:.4f} * V + {cal[1]:.4f}")

    # Physical sanity bound on the slope (all modes), independent of any
    # stored cal — the main guard against a fat-fingered manual --slope.
    slope_bad = slope_out_of_range(cal[0], args.turns)
    if slope_bad:
        expected = expected_slope_mag(args.turns)
        print(
            f"\nWARNING: slope |{cal[0]:.1f}| deg/V is more than "
            f"{SLOPE_SANITY_FACTOR:g}x off the expected ~{expected:.0f} deg/V "
            f"for a {args.turns:g}-turn pot over {ADC_VREF:.1f} V. "
            "Check the wiring, --turns, or a typo in the slope."
        )

    if not prompt_save(diverged or slope_bad):
        print("Discarded. Nothing written to Redis or the live pot.")
        return

    # Persist to Redis first — if the live push later fails, the cal is
    # still stored and will load on the next PicoManager restart.
    PotCalStore(transport).upload(cal_data)
    # Force an RDB snapshot now so the cal survives a power loss before
    # the next scheduled save (default policy is `save 3600 1`, which would
    # leave this single-key write unsnapshotted for up to an hour).
    transport.r.bgsave()
    print(
        f"\nPublished calibration to Redis at "
        f"{args.redis_host}:{args.redis_port} (key: pot_calibration); "
        "BGSAVE triggered."
    )

    # Push to the running PicoPotentiometer so the new cal takes effect on
    # the next status tick. Skip when the pot is known-down (manual recovery
    # path) to avoid a guaranteed ~5 s timeout — the Redis write above already
    # restored it for the next PicoManager start.
    if not pot_proxy.is_available:
        print(
            "Pot not reachable; calibration is stored in Redis and "
            "loads on the next PicoManager restart."
        )
        return
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


if __name__ == "__main__":
    main()
