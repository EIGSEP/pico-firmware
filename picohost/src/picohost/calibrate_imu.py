"""IMU mount calibration from a single elevation sweep.

One full el revolution (auto-driven by default, mirroring
calibrate-pot --mode auto) fits each alive IMU's mount + el-zero via
imu_geometry.fit_el_calibration. El-zero ("home") is DERIVED from the
sweep itself — the pose where the IMUs look most "down"
(NOMINAL_LEVEL_AXIS) — so no operator-eyeballed level pose is needed.
Azimuth estimation from the IMU is retired: at level, az rotation is
rotation about gravity and fundamentally unobservable to an
accelerometer (2026-07-08 field data: 20x noise amplification).
potmon owns azimuth; the IMUs own elevation.

Recipe: calibrate-pot (defines az 0) -> calibrate-imu (defines el 0)
-> home in motor_manual (re-zeros the counters at cal-defined home).
The az turntable must be parked at home during the el sweep for the
imu_az section (checked against the calibrated pot); imu_el alone is
az-invariant and can be calibrated regardless.

Modes:
  auto   : calibrate-imu drives the el sweep itself (default)
  manual : operator drives stop-by-stop (motor stream must publish)
"""

from argparse import ArgumentParser
import json
import logging
import sys
from datetime import datetime, timezone

import numpy as np
from eigsep_redis import Transport

from .buses import ImuCalStore, PotCalStore
from .imu_geometry import fit_el_calibration
from .motor_sweep import (
    SETTLE_TIMEOUT_S,
    motor_command,
    read_motor_pos_deg,
    wait_for_settle,
)
from .proxy import PicoProxy

logger = logging.getLogger(__name__)

SAMPLE_TIMEOUT_S = 5.0
STATUS_SAMPLES = 5  # frames to sample when classifying a stream's liveness
IMU_AZ, IMU_EL, POTMON = "imu_az", "imu_el", "potmon"
MOTOR_NAME = "motor"

# The imu_az section is only meaningful if the turntable is parked at az
# home during the el sweep: the pot (calibrated) must read within this many
# degrees of 0. imu_el is az-invariant and needs no such gate.
AZ_HOME_TOL_DEG = 10.0
# Warn if the derived level sits more than this from motor zero — the motor
# step counter is probably stale and should be re-homed after saving. The
# >90 deg (inverted-mount) case is already a hard ValueError from the fit.
HOME_WARN_DEG = 10.0
# Per-stop imu_el vs imu_az elevation disagreement flag threshold (matches
# the live handler's runtime cross-check tolerance).
CROSS_CHECK_TOL_DEG = 5.0
# Manual-mode sweep-quality floor: the fit needs a full revolution to
# resolve the sign and locate derived level. Auto satisfies both by
# construction (n_stops+1 stops over +/-180).
MIN_STOPS = 5
MIN_SPAN_DEG = 180.0


def collect_vector(transport, name, fields, n, start_id="$", reducer=None):
    """Reduce `n` fresh VALID entries of `fields` from stream:<name>.

    Reads only entries published after this call starts (``start_id``
    defaults to ``"$"``), so repeated calls within one sweep don't
    double-count the same firmware tick. Frames whose ``status`` is
    ``"error"`` carry junk (a faulted IMU streams accel=[0,0,0]) and are
    skipped. To avoid looping forever on a sustained fault, abort once
    ``n`` consecutive error frames have been skipped, naming the stream.
    Mirrors :func:`calibrate_pot.collect_samples`' fail-fast semantics: if
    no new entries arrive within ``SAMPLE_TIMEOUT_S``, raise rather than
    average a stale value. Tests pass ``start_id="0-0"`` to read pre-loaded
    entries.

    ``reducer`` maps the ``(n, len(fields))`` sample array to the reduced
    result; it defaults to a per-field arithmetic mean. Pass a circular
    reducer for angle fields (e.g. yaw) that wrap at +/-180.
    """
    stream = f"stream:{name}"
    rows, last_id, consec_err = [], start_id, 0
    while len(rows) < n:
        resp = transport.r.xread(
            {stream: last_id},
            block=int(SAMPLE_TIMEOUT_S * 1000),
            count=n - len(rows),
        )
        if not resp:
            raise RuntimeError(
                f"No new entries on {stream} within {SAMPLE_TIMEOUT_S}s."
            )
        for _s, msgs in resp:
            for msg_id, f in msgs:
                last_id = msg_id
                value = json.loads(f[b"value"])
                if value.get("status") == "error":
                    consec_err += 1
                    if consec_err >= n:
                        raise RuntimeError(
                            f"{name}: {consec_err} consecutive status=error "
                            f"frames (sensor faulted); collected only "
                            f"{len(rows)}/{n} valid samples."
                        )
                    continue
                consec_err = 0
                rows.append([float(value[k]) for k in fields])
    arr = np.asarray(rows, dtype=float)
    return arr.mean(axis=0) if reducer is None else reducer(arr)


def stream_status(
    transport, name, timeout_s=SAMPLE_TIMEOUT_S, samples=STATUS_SAMPLES
):
    """Classify stream:<name> as 'healthy', 'faulted', or 'dead'.

    Blocks on a ``$`` cursor so a stale stream (old entries, no live
    publisher) reads 'dead' — the graceful-degradation gate must not treat
    a crashed IMU as alive. Samples up to ``samples`` fresh frames within
    ``timeout_s``:

      - no frame arrives           -> 'dead'    (no live publisher)
      - >=1 frame status=='update' -> 'healthy'
      - frames arrive, all error   -> 'faulted' (publisher up, sensor down;
                                       these frames carry accel=[0,0,0])

    A single-frame check is too noisy (one stray error frame on a healthy
    sensor would false-trip), so 'faulted' requires a full window with no
    'update'.
    """
    stream = f"stream:{name}"
    last_id, seen_any, remaining = "$", False, samples
    while remaining > 0:
        resp = transport.r.xread(
            {stream: last_id}, block=int(timeout_s * 1000), count=remaining
        )
        if not resp:
            break
        for _s, msgs in resp:
            for msg_id, f in msgs:
                value = json.loads(f[b"value"])
                if value.get("status") == "update":
                    return "healthy"
                seen_any = True
                last_id = msg_id
                remaining -= 1
    return "faulted" if seen_any else "dead"


def stream_alive(transport, name, timeout_s=SAMPLE_TIMEOUT_S):
    """True only if stream:<name> is publishing valid (status=update) frames.

    Thin wrapper over :func:`stream_status`; a faulted or dead stream is not
    alive for calibration.
    """
    return stream_status(transport, name, timeout_s=timeout_s) == "healthy"


def _read_pot_az_deg(transport, n):
    """Latest averaged pot azimuth angle (deg) from stream:potmon."""
    return float(collect_vector(transport, POTMON, ("pot_az_angle",), n)[0])


def _collect_stop(transport, n_samples, alive):
    """Sample each alive IMU's accel vector at the current stop."""
    out = {}
    for name in (IMU_EL, IMU_AZ):
        if name in alive:
            out[name] = collect_vector(
                transport,
                name,
                ("accel_x", "accel_y", "accel_z"),
                n_samples,
            )
    return out


def _stack_rows(rows):
    """(N,3) array of the collected samples, or None if the IMU was absent."""
    return np.array(rows) if rows else None


def collect_el_auto(transport, motor_proxy, n_samples, n_stops, alive):
    """Drive el through n_stops+1 stops over ±180 deg; sample IMUs.

    Same claim / non-blocking-move / settle-poll / halt-on-failure /
    release pattern as calibrate_pot.collect_auto (see that docstring
    for the rationale). Returns {"motor_el_deg": [...],
    "imu_el": (N,3) array or None, "imu_az": (N,3) array or None}.
    """
    claim_ttl = int(SETTLE_TIMEOUT_S * (n_stops + 3))
    motor_command(motor_proxy, "claim", ttl=claim_ttl)
    rows = {IMU_EL: [], IMU_AZ: []}
    motor_el = []
    try:
        for i in range(n_stops + 1):
            target = -180.0 + i * (360.0 / n_stops)
            print(f"\nMoving to el {target:.1f} deg...")
            motor_command(
                motor_proxy,
                "el_target_deg",
                target_deg=target,
                wait_for_start=False,
                wait_for_stop=False,
            )
            el = wait_for_settle(transport, "el", target)
            samples = _collect_stop(transport, n_samples, alive)
            motor_el.append(el)
            for name, vec in samples.items():
                rows[name].append(vec)
        print("\nReturning el to motor zero...")
        motor_command(
            motor_proxy,
            "el_target_deg",
            target_deg=0.0,
            wait_for_start=False,
            wait_for_stop=False,
        )
        wait_for_settle(transport, "el", 0.0)
    except BaseException:
        # Includes KeyboardInterrupt: an operator's Ctrl-C mid-sweep must
        # stop the motor, not just this process. Moves are non-blocking, so
        # the firmware would otherwise keep driving with nobody watching.
        try:
            motor_proxy.send_command("halt")
        except (TimeoutError, RuntimeError):
            pass  # best-effort; the original failure is what matters
        raise
    finally:
        motor_proxy.send_command("release")
    return {
        "motor_el_deg": motor_el,
        IMU_EL: _stack_rows(rows[IMU_EL]),
        IMU_AZ: _stack_rows(rows[IMU_AZ]),
    }


def collect_el_manual(transport, n_samples, alive):
    """Operator-driven stops; reads settled motor el from stream:motor.

    Returns the same dict shape as :func:`collect_el_auto`. stream:motor
    must be publishing (it feeds the sign resolution and flip guard).
    """
    rows = {IMU_EL: [], IMU_AZ: []}
    motor_el = []
    print("\n== ELEVATION sweep (manual) ==")
    print("Cover a full revolution; any order; level NOT required.")
    while True:
        r = input("Drive to an el stop + Enter (or 'q' to finish): ")
        if r.strip().lower() == "q":
            break
        motor_el.append(read_motor_pos_deg(transport, "el"))
        for name, vec in _collect_stop(transport, n_samples, alive).items():
            rows[name].append(vec)
    return {
        "motor_el_deg": motor_el,
        IMU_EL: _stack_rows(rows[IMU_EL]),
        IMU_AZ: _stack_rows(rows[IMU_AZ]),
    }


def build_parser():
    p = ArgumentParser(
        description="Calibrate IMU mount + el-zero from one el sweep."
    )
    p.add_argument("-m", "--mode", default="auto", choices=["auto", "manual"])
    p.add_argument("-n", "--n-samples", type=int, default=10)
    p.add_argument(
        "--n-stops",
        type=int,
        default=12,
        help="auto-mode el stops (12 -> 30 deg spacing, 13 stops over +/-180)",
    )
    p.add_argument("--redis-host", default="localhost")
    p.add_argument("--redis-port", type=int, default=6379)
    return p


def _print_cross_check(cross):
    """Print the per-stop el_signed/el_abs table, flagging disagreements."""
    print("\nPer-stop cross-check (motor_el, el_signed, el_abs):")
    for m, el_s, el_a in cross:
        flag = ""
        if (
            el_s is not None
            and el_a is not None
            and abs(abs(el_s) - el_a) > CROSS_CHECK_TOL_DEG
        ):
            flag = "  <-- FLAG"
        s_txt = "   --  " if el_s is None else f"{el_s:+7.1f}"
        a_txt = "  --  " if el_a is None else f"{el_a:6.1f}"
        print(f"  {m:+7.1f}  {s_txt}  {a_txt}{flag}")


def _pot_az_home_gate(transport, alive, n_samples):
    """Gate the imu_az section on a calibrated pot parked at az home.

    Returns ``(keep_az, pot_az_home_deg, abort)``: whether to keep the
    imu_az section, the pot angle stamped into metadata (``None`` when
    running imu_el-only), and whether the operator chose to abort.
    """
    cal = PotCalStore(transport).get()
    calibrated = bool(cal and cal.get("pot_az"))
    pot_deg = None
    if calibrated:
        try:
            pot_deg = _read_pot_az_deg(transport, n_samples)
        except RuntimeError:
            # A quiet pot stream reads the same as "not parked": offer el-only.
            pot_deg = None
    if pot_deg is not None and abs(pot_deg) <= AZ_HOME_TOL_DEG:
        return True, pot_deg, False
    print(
        "az not parked at home (or pot uncalibrated) — home az first "
        "(calibrate-pot / motor_manual).",
        file=sys.stderr,
    )
    ans = input("Continue imu_el-only? [y / Enter to abort]: ").strip().lower()
    if ans in ("y", "yes"):
        return False, None, False
    return False, None, True


def main(argv=None):
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    transport = Transport(host=args.redis_host, port=args.redis_port)

    states = {n: stream_status(transport, n) for n in (IMU_AZ, IMU_EL, POTMON)}
    alive = {n for n, s in states.items() if s == "healthy"}
    # A faulted IMU (publisher up, sensor down) streams status=error frames
    # with accel=[0,0,0]. Without this gate it would pass a naive liveness
    # check and poison the fit with a cryptic 'SVD did not converge'. Surface
    # it and let the operator fix wiring (abort) or proceed without it.
    faulted_imus = [n for n in (IMU_AZ, IMU_EL) if states[n] == "faulted"]
    if faulted_imus:
        for n in faulted_imus:
            print(
                f"{n}: stream is publishing only status=error "
                f"(sensor faulted -- check wiring/power).",
                file=sys.stderr,
            )
        ans = (
            input(
                f"Continue calibration without {', '.join(faulted_imus)}? "
                f"[y to continue / Enter to abort]: "
            )
            .strip()
            .lower()
        )
        if ans not in ("y", "yes"):
            print("Aborted -- fix the sensor(s) and rerun.", file=sys.stderr)
            return 1
    if IMU_AZ not in alive and IMU_EL not in alive:
        print("No IMU streams alive; nothing to calibrate.", file=sys.stderr)
        return 1

    # Pot / az-home gate: imu_az's el section is only meaningful if the
    # turntable is parked at az home during the sweep. imu_el is
    # az-invariant, so an off-home / uncalibrated pot only drops imu_az.
    pot_az_home_deg = None
    if IMU_AZ in alive:
        keep_az, pot_az_home_deg, abort = _pot_az_home_gate(
            transport, alive, args.n_samples
        )
        if abort:
            return 1
        if not keep_az:
            alive.discard(IMU_AZ)
            if IMU_EL not in alive:
                print(
                    "imu_el not alive; nothing calibratable.", file=sys.stderr
                )
                return 1

    try:
        if args.mode == "auto":
            motor_proxy = PicoProxy(
                MOTOR_NAME, transport, source="calibrate-imu"
            )
            sweep = collect_el_auto(
                transport,
                motor_proxy,
                n_samples=args.n_samples,
                n_stops=args.n_stops,
                alive=alive,
            )
        else:
            sweep = collect_el_manual(transport, args.n_samples, alive)
    except (RuntimeError, TimeoutError) as e:
        # A fault that begins mid-sweep makes collect_vector abort with a
        # named RuntimeError; a motor that never settles raises TimeoutError.
        # Surface either cleanly rather than as a traceback.
        print(f"Sweep aborted: {e}", file=sys.stderr)
        return 1

    motor_el = sweep["motor_el_deg"]
    span = (max(motor_el) - min(motor_el)) if motor_el else 0.0
    if len(motor_el) < MIN_STOPS or span < MIN_SPAN_DEG:
        print(
            f"Sweep too small to fit ({len(motor_el)} stops spanning "
            f"{span:.0f} deg; need >= {MIN_STOPS} stops over "
            f">= {MIN_SPAN_DEG:.0f} deg). Cover a full revolution.",
            file=sys.stderr,
        )
        return 1

    try:
        sections, report = fit_el_calibration(
            sweep[IMU_EL], sweep[IMU_AZ], motor_el
        )
    except ValueError as e:
        # Backstop: a degenerate/zero-norm fit or an inverted mount names its
        # cause here instead of an opaque SVD failure.
        print(f"Fit failed: {e}", file=sys.stderr)
        return 1
    if not sections:
        print("Fit produced no sections.", file=sys.stderr)
        return 1

    for name, sec in sections.items():
        print(
            f"\n{name}: mount_perm={sec.get('mount_perm')} "
            f"misalign={sec.get('mount_misalign_deg'):.2f} deg "
            f"accel_scale={sec['accel_scale']:.3f}"
        )
    home = report["home_offset_motor_deg"]
    print(
        f"\nDerived level (home) at motor {home:+.1f} deg "
        f"(anchor: {report['anchor']})."
    )
    if abs(home) > HOME_WARN_DEG:
        print(
            f"WARNING: motor zero is >{HOME_WARN_DEG:.0f} deg from IMU level "
            "— motor zero may be stale; home after saving."
        )
    _print_cross_check(report["cross_check"])

    if input("\nSave this calibration? [y/N]: ").strip().lower() not in (
        "y",
        "yes",
    ):
        print("Discarded.")
        return 0

    payload = dict(sections)
    payload["metadata"] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "mode": args.mode,
        "n_samples": args.n_samples,
        "n_stops": args.n_stops,
        "derived_home_motor_deg": home,
        "pot_az_home_deg": pot_az_home_deg,
    }
    ImuCalStore(transport).upload(payload)
    transport.r.bgsave()
    print("Published to Redis (key: imu_calibration); BGSAVE triggered.")

    for name, sec in sections.items():
        proxy = PicoProxy(name, transport, source="calibrate-imu")
        try:
            proxy.send_command("set_calibration", **{name: sec})
            print(f"Live {name} updated.")
        except (TimeoutError, RuntimeError) as e:
            print(f"Live push to {name} failed: {e}", file=sys.stderr)

    print(
        "\nNow run home (h + confirm) in motor_manual to re-zero the "
        "counters at the new cal-defined home."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
