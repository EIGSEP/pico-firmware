"""Unit tests for picohost.calibrate_pot."""

import json

import pytest
from eigsep_redis.testing import DummyTransport

from picohost import calibrate_pot
from picohost.buses import PotCalStore


def test_fit_slope_pin_zero_pins_intercept_to_v0():
    # angle = 100*(V - 1.0): zero at V=1.0
    voltages = [1.0, 1.5, 2.0]
    angles = [0.0, 50.0, 100.0]
    m, b = calibrate_pot.fit_slope_pin_zero(voltages, angles, v0=1.0)
    assert m == pytest.approx(100.0)
    assert b == pytest.approx(-100.0)
    # exact zero at the home voltage
    assert m * 1.0 + b == pytest.approx(0.0)


def test_fit_slope_pin_zero_uses_v0_not_freefit_intercept():
    # Even when v0 differs from any sampled point, b must pin to v0.
    voltages = [1.0, 2.0]
    angles = [0.0, 100.0]
    m, b = calibrate_pot.fit_slope_pin_zero(voltages, angles, v0=1.2)
    assert b == pytest.approx(-m * 1.2)


def test_fit_slope_pin_zero_returns_none_for_flat_voltages():
    assert calibrate_pot.fit_slope_pin_zero([1.0, 1.0], [0.0, 100.0], 1.0) is None


def test_compute_headroom_basic():
    h = calibrate_pot.compute_headroom([1.0, 1.9], m=100.0, vref=3.3)
    assert h["v_lo"] == pytest.approx(1.0)
    assert h["v_hi"] == pytest.approx(1.9)
    assert h["span_v"] == pytest.approx(0.9)
    assert h["headroom_low_v"] == pytest.approx(1.0)
    assert h["headroom_high_v"] == pytest.approx(1.4)
    assert h["headroom_low_deg"] == pytest.approx(100.0)
    assert h["headroom_high_deg"] == pytest.approx(140.0)


def test_compute_headroom_uses_abs_slope():
    # Negative slope (voltage falls as az rises) must still give
    # positive degree headroom.
    h = calibrate_pot.compute_headroom([1.0, 1.9], m=-100.0, vref=3.3)
    assert h["headroom_low_deg"] == pytest.approx(100.0)
    assert h["headroom_high_deg"] == pytest.approx(140.0)


def _xadd_motor(transport, az_pos):
    transport.r.xadd(
        calibrate_pot.MOTOR_STREAM,
        {"value": json.dumps({"sensor_name": "motor", "az_pos": az_pos})},
    )


def test_read_motor_az_steps_reads_latest():
    t = DummyTransport()
    _xadd_motor(t, 22600)
    # start_id="0-0" reads from the beginning so the test doesn't have to
    # race the "$" (new-entries-only) production default.
    assert calibrate_pot.read_motor_az_steps(t, start_id="0-0") == 22600.0


def test_read_motor_az_deg_converts_with_geometry():
    t = DummyTransport()
    _xadd_motor(t, 22600)
    deg = calibrate_pot.read_motor_az_deg(
        t, step_angle_deg=1.8, gear_teeth=113, microstep=1, start_id="0-0"
    )
    assert deg == pytest.approx(360.0)


def test_read_motor_az_steps_raises_when_silent():
    t = DummyTransport()
    with pytest.raises(RuntimeError, match="motor"):
        # Nothing ever published; "$" returns after the block timeout.
        calibrate_pot.read_motor_az_steps(t, start_id="$")


def _seq(values):
    it = iter(values)
    return lambda *a, **k: next(it)


def test_collect_azimuth_pairs_voltage_with_motor_az(monkeypatch):
    monkeypatch.setattr(
        calibrate_pot, "collect_samples", _seq([1.0, 1.5, 2.0])
    )
    monkeypatch.setattr(
        calibrate_pot, "read_motor_az_deg", _seq([0.5, 180.0, 360.0])
    )
    # home prompt, two stop prompts, then 'q' to finish
    monkeypatch.setattr("builtins.input", _seq(["", "", "", "q"]))

    cfg = {"step_angle_deg": 1.8, "gear_teeth": 113, "microstep": 1}
    voltages, angles, v0 = calibrate_pot.collect_azimuth(
        DummyTransport(), n_samples=10, motor_cfg=cfg
    )

    assert v0 == pytest.approx(1.0)
    assert voltages == [1.0, 1.5, 2.0]
    # home is pinned to az=0 regardless of the 0.5 deg read
    assert angles == [0.0, 180.0, 360.0]


def test_collect_azimuth_warns_when_not_homed(monkeypatch, capsys):
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0, 2.0]))
    monkeypatch.setattr(
        calibrate_pot, "read_motor_az_deg", _seq([45.0, 360.0])
    )
    monkeypatch.setattr("builtins.input", _seq(["", "", "q"]))
    cfg = {"step_angle_deg": 1.8, "gear_teeth": 113, "microstep": 1}
    calibrate_pot.collect_azimuth(DummyTransport(), 10, cfg)
    assert "WARNING" in capsys.readouterr().out
