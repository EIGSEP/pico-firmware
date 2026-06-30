"""Manual IMU mount calibration.

Reads accel/yaw from stream:imu_az / stream:imu_el and az truth from
stream:potmon (pot is the azimuth standard; the motor is a mover only and is
never used as fit truth). Drives the operator through elevation + azimuth
sweeps, fits each IMU's mount + zero via imu_geometry, then persists to
ImuCalStore (BGSAVE) and live-pushes to each IMU via PicoProxy.

Modes:
  elevation : elevation sweep -> el calibration for alive IMUs
  azimuth   : azimuth sweeps (near-level + tilted) -> imu_az az+el; needs pot
  all       : guided full run (default)
(A fast rezero-el re-level is a deferred follow-on; see the design doc.)
"""

from argparse import ArgumentParser
import json
import logging
import sys
from datetime import datetime, timezone

import numpy as np
from eigsep_redis import Transport

from .buses import ImuCalStore, PotCalStore
from .imu_geometry import (
    DEFAULT_THETA_DEAD_DEG,
    DEFAULT_THETA_SAT_DEG,
    circular_mean_deg,
    fit_calibration_from_sweeps,
)
from .proxy import PicoProxy

logger = logging.getLogger(__name__)

SAMPLE_TIMEOUT_S = 5.0
STATUS_SAMPLES = 5  # frames to sample when classifying a stream's liveness
IMU_AZ, IMU_EL, POTMON = "imu_az", "imu_el", "potmon"


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


class Calibrator:
    """Operator-driven sweep collection (separated so tests can stub it)."""

    def __init__(self, transport, n_samples, alive, mode):
        self.transport = transport
        self.n = n_samples
        self.alive = alive  # set of alive stream names
        self.mode = mode

    def _accel(self, name):
        return collect_vector(
            self.transport, name, ("accel_x", "accel_y", "accel_z"), self.n
        )

    def _pot(self):
        return float(
            collect_vector(self.transport, POTMON, ("pot_az_angle",), self.n)[
                0
            ]
        )

    def _yaw(self):
        # Yaw wraps at +/-180, so average it circularly, not linearly.
        return collect_vector(
            self.transport,
            IMU_AZ,
            ("yaw",),
            self.n,
            reducer=lambda r: circular_mean_deg(r[:, 0]),
        )

    def run_sweeps(self):
        """Return (el_sweep, az_level, az_tilt) dicts, gated by self.mode.

        Prompts the operator stop-by-stop; records pot/accel/yaw at rest.
        """
        el_sweep = {
            "imu_el": None,
            "imu_az": None,
            "level_index": 0,
            "direction": 1,
        }
        az_level = {"imu_az": None, "yaw_deg": None, "pot_deg": None}
        az_tilt = {"imu_az": None, "pot_deg": None, "imu_el": None}
        if self.mode in ("elevation", "all"):
            el_sweep = self._elevation_sweep()
        if self.mode in ("azimuth", "all"):
            if IMU_AZ in self.alive and POTMON in self.alive:
                az_level = self._az_sweep("near-LEVEL", want_yaw=True)
                az_tilt = self._az_sweep("TILTED (~20-45 deg)", want_yaw=False)
        return el_sweep, az_level, az_tilt

    def _elevation_sweep(self):
        el_el, el_az = [], []
        print("\n== ELEVATION sweep ==")
        input("Drive to the LEVEL pose (el 0), stop, press Enter.")
        if IMU_EL in self.alive:
            el_el.append(self._accel(IMU_EL))
        if IMU_AZ in self.alive:
            el_az.append(self._accel(IMU_AZ))
        while True:
            r = input("Next el stop + Enter (or 'q' to finish): ").strip()
            if r.lower() == "q":
                break
            if IMU_EL in self.alive:
                el_el.append(self._accel(IMU_EL))
            if IMU_AZ in self.alive:
                el_az.append(self._accel(IMU_AZ))
        return {
            "imu_el": np.array(el_el) if el_el else None,
            "imu_az": np.array(el_az) if el_az else None,
            "level_index": 0,
            "direction": 1,
        }

    def _az_sweep(self, label, want_yaw):
        print(f"\n== AZIMUTH sweep ({label}) ==")
        acc, yaw, pot, el = [], [], [], []
        input("Drive to the first az stop, stop, press Enter.")
        while True:
            acc.append(self._accel(IMU_AZ))
            pot.append(self._pot())
            if want_yaw:
                yaw.append(self._yaw())
            if IMU_EL in self.alive:
                el.append(self._accel(IMU_EL))
            r = input("Next az stop + Enter (or 'q' to finish): ").strip()
            if r.lower() == "q":
                break
        out = {"imu_az": np.array(acc), "pot_deg": np.array(pot)}
        out["yaw_deg"] = np.array(yaw) if want_yaw else None
        out["imu_el"] = np.array(el) if el else None
        return out


def build_parser():
    p = ArgumentParser(description="Calibrate IMU mount -> az/el conversion.")
    p.add_argument(
        "-m", "--mode", default="all", choices=["elevation", "azimuth", "all"]
    )
    p.add_argument("-n", "--n-samples", type=int, default=10)
    p.add_argument(
        "--theta-sat-deg", type=float, default=DEFAULT_THETA_SAT_DEG
    )
    p.add_argument(
        "--theta-dead-deg", type=float, default=DEFAULT_THETA_DEAD_DEG
    )
    p.add_argument("--redis-host", default="localhost")
    p.add_argument("--redis-port", type=int, default=6379)
    return p


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
    # The pot must be CALIBRATED, not merely alive, to serve as the az
    # standard: an uncalibrated pot streams pot_az_angle=None, which would
    # otherwise crash the az sweep mid-run. Drop it from `alive` so the check
    # below treats it like a missing pot (azimuth aborts; all skips az).
    if args.mode in ("azimuth", "all") and POTMON in alive:
        pot_cal = PotCalStore(transport).get()
        if not (pot_cal and pot_cal.get("pot_az")):
            print(
                "pot is alive but uncalibrated; run calibrate_pot first.",
                file=sys.stderr,
            )
            alive.discard(POTMON)
    if args.mode in ("azimuth", "all") and POTMON not in alive:
        print(
            "pot not alive; azimuth needs the pot standard.", file=sys.stderr
        )
        if args.mode == "azimuth":
            return 1

    cal = Calibrator(transport, args.n_samples, alive, args.mode)
    try:
        el_sweep, az_level, az_tilt = cal.run_sweeps()
    except RuntimeError as e:
        # A fault that begins mid-sweep makes collect_vector abort with a
        # named RuntimeError; surface it cleanly rather than as a traceback.
        print(f"Sweep aborted: {e}", file=sys.stderr)
        return 1
    try:
        sections = fit_calibration_from_sweeps(
            el_sweep,
            az_level,
            az_tilt,
            theta_sat_deg=args.theta_sat_deg,
            theta_dead_deg=args.theta_dead_deg,
        )
    except ValueError as e:
        # Backstop: a degenerate/zero-norm fit (e.g. a fault that slipped in
        # mid-sweep) names its cause here instead of an opaque SVD failure.
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
