"""Shared motor-drive helpers for self-driving calibration sweeps.

Extracted from calibrate_pot --mode auto so calibrate_imu can drive the
elevation axis with identical settle semantics. Moves are commanded
non-blocking (the manager runs routed commands synchronously on its
command thread; a full-turn move takes ~2 min), so callers poll
stream:motor for settle via wait_for_settle.
"""

import json
import time

from .motor import GEAR_TEETH, MICROSTEP, STEP_ANGLE_DEG, steps_to_deg

MOTOR_NAME = "motor"
MOTOR_STREAM = "stream:motor"
SAMPLE_TIMEOUT_S = 5.0
# A commanded move is settled once two consecutive stream:motor reads
# agree with each other and with the target within this many degrees.
SETTLE_TOL_DEG = 2.0
# Overall budget to settle a single commanded move (> full 360 turn).
SETTLE_TIMEOUT_S = 180.0
MOTOR_GEOMETRY = {
    "step_angle_deg": STEP_ANGLE_DEG,
    "gear_teeth": GEAR_TEETH,
    "microstep": MICROSTEP,
}
_POS_FIELD = {"az": "az_pos", "el": "el_pos"}


def read_motor_pos_steps(
    transport, axis, start_id="$", timeout_s=SAMPLE_TIMEOUT_S
):
    """Current az_pos/el_pos (steps) from stream:motor; fail-fast.

    Read-only itself (this call never commands the motor). Mirrors
    ``calibrate_pot.collect_samples``' fail-fast semantics -- if
    PicoManager isn't publishing motor status within ``timeout_s``,
    raise rather than silently using a stale value.
    """
    if axis not in _POS_FIELD:
        raise ValueError(f"axis must be 'az' or 'el', got {axis!r}")
    resp = transport.r.xread(
        {MOTOR_STREAM: start_id}, block=int(timeout_s * 1000), count=1
    )
    if not resp:
        raise RuntimeError(
            f"No entries on {MOTOR_STREAM} within {timeout_s}s. "
            "Is PicoManager publishing motor status?"
        )
    _stream, messages = resp[0]
    _msg_id, fields = messages[0]
    return float(json.loads(fields[b"value"])[_POS_FIELD[axis]])


def read_motor_pos_deg(
    transport, axis, start_id="$", timeout_s=SAMPLE_TIMEOUT_S
):
    """Current motor position in degrees (steps converted via
    MOTOR_GEOMETRY)."""
    steps = read_motor_pos_steps(
        transport, axis, start_id=start_id, timeout_s=timeout_s
    )
    return steps_to_deg(steps, **MOTOR_GEOMETRY)


def motor_command(motor_proxy, action, **kwargs):
    """Send a motor command, failing fast if the motor went away.

    ``PicoProxy.send_command`` silently no-ops (returns ``None``) when
    the device heartbeat is down. For a sweep that would otherwise wait
    out a ~3 min settle timeout on a move that was never dispatched,
    that must be an immediate, clearly-attributed error instead.
    """
    if not motor_proxy.is_available:
        raise RuntimeError(
            f"{MOTOR_NAME} became unreachable (heartbeat down); "
            f"'{action}' not sent."
        )
    return motor_proxy.send_command(action, **kwargs)


def wait_for_settle(
    transport,
    axis,
    target_deg,
    *,
    tol_deg=SETTLE_TOL_DEG,
    timeout_s=SETTLE_TIMEOUT_S,
    mid_move_check=None,
    start_id="$",
    read_pos_deg=read_motor_pos_deg,
):
    """Poll ``stream:motor`` until ``axis`` settles at ``target_deg``.

    Moves are commanded non-blocking, so the caller must poll for
    settle itself rather than waiting on the proxy. "Settled" requires
    two consecutive reads that agree with each other and with the
    target within ``tol_deg`` -- a single sample could catch the motor
    mid-move if it happens to cross the tolerance band. This only
    discriminates "arrived" from "still at the previous stop" when the
    stops are more than ``2 * tol_deg`` apart: closer than that, the
    stationary pre-move position already satisfies both conditions.
    Returns the settled position. Raises ``TimeoutError`` if the motor
    hasn't settled within ``timeout_s``.

    ``mid_move_check(transport, pos_deg)``, if given, is invoked every
    poll before the settle test -- so a check that raises (e.g.
    calibrate_pot's pot-rail guard) aborts before a railed/faulted
    sample can ever be reported as a clean arrival.

    ``start_id`` and ``read_pos_deg`` are internal seams: ``start_id``
    lets a caller pin the stream read to a fixed position (tests, to
    avoid racing "$" new-entries-only semantics against fakeredis);
    ``read_pos_deg`` lets calibrate_pot delegate to its own
    ``read_motor_az_deg`` wrapper (signature
    ``(transport, axis, start_id) -> float``) so its existing test
    suite's monkeypatching of that wrapper keeps working unchanged.
    Callers driving a fresh axis (e.g. calibrate_imu / elevation) can
    ignore both and rely on the defaults.
    """
    deadline = time.monotonic() + timeout_s
    prev = None
    while time.monotonic() < deadline:
        pos = read_pos_deg(transport, axis, start_id=start_id)
        if mid_move_check is not None:
            mid_move_check(transport, pos)
        if (
            prev is not None
            and abs(pos - target_deg) <= tol_deg
            and abs(pos - prev) <= tol_deg
        ):
            return pos
        prev = pos
    raise TimeoutError(
        f"Motor {axis} did not settle at {target_deg:.1f} deg "
        f"(tol {tol_deg:.1f} deg) within {timeout_s:.0f}s "
        f"(last read: {prev})"
    )
