"""Unit tests for picohost.calibrate_pot."""

import json
import sys

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
    assert (
        calibrate_pot.fit_slope_pin_zero([1.0, 1.0], [0.0, 100.0], 1.0) is None
    )


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


def test_rezero_reuses_stored_slope(monkeypatch):
    t = DummyTransport()
    PotCalStore(t).upload({"pot_az": [100.0, -50.0]})
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0]))
    monkeypatch.setattr("builtins.input", _seq([""]))

    (m, b), v0 = calibrate_pot.rezero(t, n_samples=10)

    assert m == pytest.approx(100.0)  # slope reused, not refit
    assert v0 == pytest.approx(1.0)
    assert b == pytest.approx(-100.0)  # b = -m * v0


def test_rezero_without_stored_cal_raises(monkeypatch):
    t = DummyTransport()
    monkeypatch.setattr("builtins.input", _seq([""]))
    with pytest.raises(RuntimeError, match="No stored calibration"):
        calibrate_pot.rezero(t, n_samples=10)


def test_default_turns_matches_installed_pot():
    """The installed pot is 3.75-turn, so that is the bench-mode default."""
    p = calibrate_pot.build_parser()
    assert p.parse_args([]).turns == pytest.approx(3.75)
    # explicit override still works
    assert p.parse_args(["-t", "10"]).turns == pytest.approx(10.0)


def test_build_parser_accepts_new_modes_and_motor_cfg():
    p = calibrate_pot.build_parser()

    a = p.parse_args(["--mode", "azimuth", "--gear-teeth", "200"])
    assert a.mode == "azimuth"
    assert a.gear_teeth == 200

    assert p.parse_args(["--mode", "rezero"]).mode == "rezero"

    d = p.parse_args([])
    assert d.mode == "minmax"  # unchanged default
    assert d.step_angle_deg == pytest.approx(1.8)
    assert d.gear_teeth == 113
    assert d.microstep == 1


# ---------------------------------------------------------------------------
# main() integration tests — drive the two new modes without hardware
# ---------------------------------------------------------------------------


def _make_fake_proxy():
    """Return a fake PicoProxy class whose instances record send_command calls."""

    class FakePicoProxy:
        def __init__(self, *args, **kwargs):
            self.is_available = True
            self.calls = []

        def send_command(self, action, **kwargs):
            self.calls.append((action, kwargs))

    instance = FakePicoProxy()
    return instance, lambda *a, **k: instance


def test_main_azimuth_mode(monkeypatch):
    """main() --mode azimuth: collects, fits, stores, and pushes cal."""
    dummy_transport = DummyTransport()
    fake_proxy, proxy_factory = _make_fake_proxy()

    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    # Return canned (voltages, angles, v0) — no hardware or user input needed.
    monkeypatch.setattr(
        calibrate_pot,
        "collect_azimuth",
        lambda *a, **k: ([1.0, 1.9], [0.0, 360.0], 1.0),
    )
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "azimuth"])

    calibrate_pot.main()

    stored = PotCalStore(dummy_transport).get()
    expected_m, expected_b = calibrate_pot.fit_slope_pin_zero(
        [1.0, 1.9], [0.0, 360.0], 1.0
    )

    # Stored calibration coefficients match the fit.
    assert stored["pot_az"][0] == pytest.approx(expected_m)
    assert stored["pot_az"][1] == pytest.approx(expected_b)
    # Metadata carries mode and motor geometry.
    assert stored["metadata"]["mode"] == "azimuth"
    assert "motor_cfg" in stored["metadata"]

    # Live proxy received exactly one set_calibration call with the right params.
    assert len(fake_proxy.calls) == 1
    action, kwargs = fake_proxy.calls[0]
    assert action == "set_calibration"
    assert kwargs["pot_az_params"] == pytest.approx([expected_m, expected_b])


def test_main_rezero_mode(monkeypatch):
    """main() --mode rezero: reuses stored slope, repins intercept, stores, pushes."""
    dummy_transport = DummyTransport()
    # Pre-seed an existing calibration so rezero() can load the slope.
    PotCalStore(dummy_transport).upload({"pot_az": [100.0, -50.0]})

    fake_proxy, proxy_factory = _make_fake_proxy()

    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    # rezero() calls collect_samples() once (for v0) and input() once.
    monkeypatch.setattr(calibrate_pot, "collect_samples", lambda *a, **k: 1.0)
    monkeypatch.setattr("builtins.input", lambda *a, **k: "")
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "rezero"])

    calibrate_pot.main()

    stored = PotCalStore(dummy_transport).get()

    # Slope reused (100.0), intercept re-pinned: b = -m * v0 = -100.0 * 1.0 = -100.0
    assert stored["pot_az"] == pytest.approx([100.0, -100.0])
    assert stored["metadata"]["slope_reused"] is True

    # Live proxy updated with the new coefficients.
    assert len(fake_proxy.calls) == 1
    action, kwargs = fake_proxy.calls[0]
    assert action == "set_calibration"
    assert kwargs["pot_az_params"] == pytest.approx([100.0, -100.0])


# ---------------------------------------------------------------------------
# compute_fit_residuals
# ---------------------------------------------------------------------------


def test_compute_fit_residuals_perfect_linear():
    """Points that lie exactly on the line produce zero residuals."""
    voltages = [0.0, 1.0, 2.0]
    angles = [10.0, 20.0, 30.0]
    r = calibrate_pot.compute_fit_residuals(voltages, angles, m=10.0, b=10.0)
    assert r["max_abs_deg"] == pytest.approx(0.0)
    assert r["rms_deg"] == pytest.approx(0.0)


def test_compute_fit_residuals_known_nonlinear():
    """A point 10 deg off the line yields a hand-computable max residual."""
    # Line: angle = 10*V + 10  => at V=2 perfect would be 30, but we give 40
    voltages = [0.0, 1.0, 2.0]
    angles = [10.0, 20.0, 40.0]  # last point is 10 deg above the line
    r = calibrate_pot.compute_fit_residuals(voltages, angles, m=10.0, b=10.0)
    # residuals: 0, 0, +10  → max_abs = 10.0
    assert r["max_abs_deg"] == pytest.approx(10.0)
    # rms = sqrt((0 + 0 + 100) / 3) = sqrt(100/3)
    import math

    assert r["rms_deg"] == pytest.approx(math.sqrt(100.0 / 3.0))


# ---------------------------------------------------------------------------
# azimuth main() — linearity report and new metadata fields
# ---------------------------------------------------------------------------


def test_main_azimuth_prints_linearity_report(monkeypatch, capsys):
    """main() --mode azimuth must print a 'Linearity check' block."""
    dummy_transport = DummyTransport()
    _fake_proxy, proxy_factory = _make_fake_proxy()

    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(
        calibrate_pot,
        "collect_azimuth",
        lambda *a, **k: ([1.0, 1.9], [0.0, 360.0], 1.0),
    )
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "azimuth"])

    calibrate_pot.main()

    out = capsys.readouterr().out
    assert "Linearity check" in out
    assert "pinned b" in out
    assert "free-fit b" in out
    assert "residuals about free-fit line" in out


def test_main_azimuth_metadata_has_residual_fields(monkeypatch):
    """main() --mode azimuth must store free_fit_intercept, residual_max_deg,
    and residual_rms_deg in the calibration metadata."""
    dummy_transport = DummyTransport()
    _fake_proxy, proxy_factory = _make_fake_proxy()

    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(
        calibrate_pot,
        "collect_azimuth",
        lambda *a, **k: ([1.0, 1.9], [0.0, 360.0], 1.0),
    )
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "azimuth"])

    calibrate_pot.main()

    stored = PotCalStore(dummy_transport).get()
    meta = stored["metadata"]
    assert "free_fit_intercept" in meta
    assert "residual_max_deg" in meta
    assert "residual_rms_deg" in meta
    # Values must be finite floats (not None / NaN)
    import math

    assert math.isfinite(meta["free_fit_intercept"])
    assert math.isfinite(meta["residual_max_deg"])
    assert math.isfinite(meta["residual_rms_deg"])
    # For perfectly linear 2-point data the residuals must be exactly zero.
    assert meta["residual_max_deg"] == pytest.approx(0.0)
    assert meta["residual_rms_deg"] == pytest.approx(0.0)
