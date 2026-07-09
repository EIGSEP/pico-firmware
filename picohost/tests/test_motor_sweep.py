"""Unit tests for picohost.motor_sweep."""

import json

import pytest
from eigsep_redis.testing import DummyTransport

from picohost import motor_sweep


def _xadd_motor(transport, az_pos=0, el_pos=0):
    transport.r.xadd(
        "stream:motor",
        {"value": json.dumps({"az_pos": az_pos, "el_pos": el_pos})},
    )


def test_read_motor_pos_deg_el_axis():
    t = DummyTransport()
    _xadd_motor(t, el_pos=11300)  # 11300 steps * 1.8/113 = 180 deg
    assert motor_sweep.read_motor_pos_deg(
        t, "el", start_id="0-0"
    ) == pytest.approx(180.0)


def test_read_motor_pos_deg_az_axis():
    t = DummyTransport()
    _xadd_motor(t, az_pos=22600)
    assert motor_sweep.read_motor_pos_deg(
        t, "az", start_id="0-0"
    ) == pytest.approx(360.0)


def test_read_motor_pos_deg_rejects_bad_axis():
    with pytest.raises(ValueError, match="axis"):
        motor_sweep.read_motor_pos_deg(DummyTransport(), "up")


def test_wait_for_settle_runs_mid_move_check():
    t = DummyTransport()
    _xadd_motor(t, el_pos=0)
    _xadd_motor(t, el_pos=0)
    seen = []
    # start_id="0-0" so the poll reads the two pre-loaded entries instead
    # of racing "$" (new-entries-only) against fakeredis's blocking xread
    # -- mirrors the start_id test hook on read_motor_az_steps.
    motor_sweep.wait_for_settle(
        t,
        "el",
        0.0,
        timeout_s=5.0,
        mid_move_check=lambda tr, pos: seen.append(pos),
        start_id="0-0",
    )
    assert seen  # callback fired at least once


def test_motor_command_fails_fast_when_unavailable():
    class DeadProxy:
        is_available = False

    with pytest.raises(RuntimeError, match="unreachable"):
        motor_sweep.motor_command(DeadProxy(), "halt")
