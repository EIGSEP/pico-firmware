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


# ---------------------------------------------------------------------------
# collect_auto (--mode auto): motor-driven sweep
# ---------------------------------------------------------------------------


class _FakeMotorProxy:
    """Records send_command calls; can be told to fail on a given action."""

    def __init__(self, fail_on=None, fail_exc=RuntimeError("boom")):
        self.is_available = True
        self.calls = []
        self._fail_on = fail_on
        self._fail_exc = fail_exc

    def send_command(self, action, **kwargs):
        self.calls.append((action, kwargs))
        if action == self._fail_on:
            raise self._fail_exc


def test_collect_auto_commands_expected_targets_and_reads_actual_az(
    monkeypatch,
):
    # n_stops=2 -> 180 deg spacing: commanded targets 180, 360, then home (0).
    # Actual settled reads are offset from the commanded target (within the
    # settle tolerance) to prove angles come from the stream, not the target.
    monkeypatch.setattr(
        calibrate_pot,
        "read_motor_az_deg",
        _seq(
            [
                1.0,  # home read
                181.0,
                181.0,  # settle at stop 1 (target 180)
                361.0,
                361.0,  # settle at stop 2 (target 360)
                0.5,
                0.5,  # settle back at home (target 0)
            ]
        ),
    )
    monkeypatch.setattr(
        calibrate_pot, "collect_samples", _seq([1.0, 1.5, 2.0])
    )

    proxy = _FakeMotorProxy()
    cfg = {"step_angle_deg": 1.8, "gear_teeth": 113, "microstep": 1}
    voltages, angles, v0 = calibrate_pot.collect_auto(
        DummyTransport(), proxy, n_samples=10, motor_cfg=cfg, n_stops=2
    )

    assert v0 == pytest.approx(1.0)
    assert voltages == [1.0, 1.5, 2.0]
    # Recorded angles are the actual stream reads (181, 361), not the
    # commanded targets (180, 360).
    assert angles == [0.0, 181.0, 361.0]

    move_calls = [c for c in proxy.calls if c[0] == "az_target_deg"]
    assert [c[1]["target_deg"] for c in move_calls] == [180.0, 360.0, 0.0]
    # Moves must be non-blocking -- the manager's command thread cannot
    # afford to wait on a multi-minute move.
    for _action, kwargs in move_calls:
        assert kwargs["wait_for_start"] is False
        assert kwargs["wait_for_stop"] is False

    # Claimed at the start, released at the end.
    assert proxy.calls[0][0] == "claim"
    assert proxy.calls[-1][0] == "release"


def test_collect_auto_warns_when_not_homed(monkeypatch, capsys):
    monkeypatch.setattr(
        calibrate_pot,
        "read_motor_az_deg",
        _seq([10.0, 360.5, 360.5, 0.2, 0.2]),
    )
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0, 2.0]))

    proxy = _FakeMotorProxy()
    cfg = {"step_angle_deg": 1.8, "gear_teeth": 113, "microstep": 1}
    calibrate_pot.collect_auto(
        DummyTransport(), proxy, n_samples=10, motor_cfg=cfg, n_stops=1
    )

    assert "WARNING" in capsys.readouterr().out


def test_collect_auto_releases_claim_on_command_failure(monkeypatch):
    monkeypatch.setattr(calibrate_pot, "read_motor_az_deg", _seq([0.0]))
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0]))

    proxy = _FakeMotorProxy(fail_on="az_target_deg")
    cfg = {"step_angle_deg": 1.8, "gear_teeth": 113, "microstep": 1}
    with pytest.raises(RuntimeError, match="boom"):
        calibrate_pot.collect_auto(
            DummyTransport(), proxy, n_samples=10, motor_cfg=cfg, n_stops=1
        )

    assert proxy.calls[0][0] == "claim"
    assert proxy.calls[-1][0] == "release"


def test_collect_auto_raises_on_settle_timeout(monkeypatch):
    # The motor never reaches the commanded target -- read_motor_az_deg
    # always reports the same stale value.
    monkeypatch.setattr(
        calibrate_pot, "read_motor_az_deg", lambda *a, **k: 0.0
    )
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0]))

    proxy = _FakeMotorProxy()
    cfg = {"step_angle_deg": 1.8, "gear_teeth": 113, "microstep": 1}
    with pytest.raises(TimeoutError, match="did not settle"):
        calibrate_pot.collect_auto(
            DummyTransport(),
            proxy,
            n_samples=10,
            motor_cfg=cfg,
            n_stops=1,
            settle_timeout_s=0.05,
        )

    # Claim still released after the settle timeout propagates.
    assert proxy.calls[0][0] == "claim"
    assert proxy.calls[-1][0] == "release"


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


def test_build_parser_accepts_manual_mode_and_args():
    p = calibrate_pot.build_parser()
    a = p.parse_args(
        [
            "--mode",
            "manual",
            "--slope",
            "409.0",
            "--intercept",
            "-400.0",
            "--note",
            "restored from corr_20260615.h5",
        ]
    )
    assert a.mode == "manual"
    assert a.slope == pytest.approx(409.0)
    assert a.intercept == pytest.approx(-400.0)
    assert a.note == "restored from corr_20260615.h5"
    # Defaults when omitted: slope/intercept None, note None.
    d = p.parse_args([])
    assert d.slope is None
    assert d.intercept is None
    assert d.note is None


def test_build_parser_accepts_new_modes_and_motor_cfg():
    p = calibrate_pot.build_parser()

    a = p.parse_args(["--mode", "azimuth", "--gear-teeth", "200"])
    assert a.mode == "azimuth"
    assert a.gear_teeth == 200

    assert p.parse_args(["--mode", "rezero"]).mode == "rezero"

    d = p.parse_args([])
    assert d.mode == "azimuth"  # in-box calibration is the common case
    assert d.step_angle_deg == pytest.approx(1.8)
    assert d.gear_teeth == 113
    assert d.microstep == 1


def test_build_parser_accepts_auto_mode():
    p = calibrate_pot.build_parser()

    a = p.parse_args(["--mode", "auto"])
    assert a.mode == "auto"
    assert a.n_stops == 8  # ~45 deg spacing over one 360 deg turn

    a = p.parse_args(["--mode", "auto", "--n-stops", "4"])
    assert a.n_stops == 4


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
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
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
    monkeypatch.setattr("builtins.input", _seq(["", "yes"]))
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


def _make_named_fake_proxy_factory(unavailable=frozenset()):
    """Fake PicoProxy factory keyed by device name.

    Unlike ``_make_fake_proxy`` (a single shared instance), --mode auto
    constructs two proxies (pot and motor) that must be independently
    controllable and independently record their calls.
    """

    class FakePicoProxy:
        def __init__(self, name, *args, **kwargs):
            self.name = name
            self.is_available = name not in unavailable
            self.calls = []

        def send_command(self, action, **kwargs):
            self.calls.append((action, kwargs))

    proxies = {}

    def factory(name, *args, **kwargs):
        proxy = FakePicoProxy(name, *args, **kwargs)
        proxies[name] = proxy
        return proxy

    return proxies, factory


def test_main_auto_mode(monkeypatch):
    """main() --mode auto: collects via collect_auto, fits, stores, pushes."""
    dummy_transport = DummyTransport()
    proxies, proxy_factory = _make_named_fake_proxy_factory()

    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(
        calibrate_pot,
        "collect_auto",
        lambda *a, **k: ([1.0, 1.9], [0.0, 360.0], 1.0),
    )
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "auto"])

    calibrate_pot.main()

    stored = PotCalStore(dummy_transport).get()
    expected_m, expected_b = calibrate_pot.fit_slope_pin_zero(
        [1.0, 1.9], [0.0, 360.0], 1.0
    )
    assert stored["pot_az"][0] == pytest.approx(expected_m)
    assert stored["pot_az"][1] == pytest.approx(expected_b)
    assert stored["metadata"]["mode"] == "auto"
    assert "motor_cfg" in stored["metadata"]
    assert stored["metadata"]["n_stops"] == 8

    # Only the pot proxy gets the live push; the motor proxy is used only
    # inside collect_auto (which is monkeypatched away here).
    pot_calls = proxies[calibrate_pot.POTMON_NAME].calls
    assert len(pot_calls) == 1
    action, kwargs = pot_calls[0]
    assert action == "set_calibration"
    assert kwargs["pot_az_params"] == pytest.approx([expected_m, expected_b])


def test_main_auto_mode_requires_motor_available(monkeypatch):
    """--mode auto exits(1) up front if the motor isn't reachable."""
    dummy_transport = DummyTransport()
    _proxies, proxy_factory = _make_named_fake_proxy_factory(
        unavailable={calibrate_pot.MOTOR_NAME}
    )
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "auto"])

    with pytest.raises(SystemExit) as exc:
        calibrate_pot.main()
    assert exc.value.code == 1
    assert PotCalStore(dummy_transport).get() is None


def test_main_auto_mode_rejects_nonpositive_n_stops(monkeypatch):
    dummy_transport = DummyTransport()
    _proxies, proxy_factory = _make_named_fake_proxy_factory()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(
        sys, "argv", ["calibrate-pot", "--mode", "auto", "--n-stops", "0"]
    )

    with pytest.raises(SystemExit) as exc:
        calibrate_pot.main()
    assert exc.value.code == 1


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
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
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
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
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


# ---------------------------------------------------------------------------
# predicted_angle_divergence
# ---------------------------------------------------------------------------


def test_predicted_angle_divergence_none_when_no_stored():
    assert (
        calibrate_pot.predicted_angle_divergence(
            (100.0, -100.0), None, [1.0, 2.0]
        )
        is None
    )


def test_predicted_angle_divergence_none_when_missing_pot_az():
    assert (
        calibrate_pot.predicted_angle_divergence(
            (100.0, -100.0), {"foo": 1}, [1.0, 2.0]
        )
        is None
    )


def test_predicted_angle_divergence_none_when_malformed_pot_az():
    # pot_az present but not a numeric (m, b) pair -> treated as unusable
    assert (
        calibrate_pot.predicted_angle_divergence(
            (100.0, -100.0), {"pot_az": ["x"]}, [1.0, 2.0]
        )
        is None
    )


def test_predicted_angle_divergence_slope_and_zero_change():
    # old(V) = 100V - 100 ; new(V) = 120V - 150 ; window [1.0, 2.0]
    #   V=1: |(-30) - 0|   = 30
    #   V=2: |90 - 100|    = 10   -> max = 30
    d = calibrate_pot.predicted_angle_divergence(
        (120.0, -150.0), {"pot_az": [100.0, -100.0]}, [1.0, 2.0]
    )
    assert d == pytest.approx(30.0)


def test_predicted_angle_divergence_constant_offset_rezero():
    # equal slopes (rezero) -> divergence is the pure zero shift |b_new - b_old|
    d = calibrate_pot.predicted_angle_divergence(
        (100.0, -160.0), {"pot_az": [100.0, -100.0]}, [1.5]
    )
    assert d == pytest.approx(60.0)


def test_main_discard_writes_nothing(monkeypatch):
    """A 'no' at the prompt persists nothing and never pushes to the pot."""
    dummy_transport = DummyTransport()  # fresh -> no stored cal
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(
        calibrate_pot,
        "collect_azimuth",
        lambda *a, **k: ([1.0, 1.9], [0.0, 360.0], 1.0),
    )
    monkeypatch.setattr("builtins.input", lambda *a, **k: "n")
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "azimuth"])

    calibrate_pot.main()

    assert PotCalStore(dummy_transport).get() is None  # nothing stored
    assert fake_proxy.calls == []  # nothing pushed live


def test_main_no_stored_cal_no_divergence_warning(monkeypatch, capsys):
    """First-ever calibration (no stored cal) is never flagged as divergent."""
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
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "azimuth"])

    calibrate_pot.main()

    out = capsys.readouterr().out
    assert "differs from the stored one" not in out
    assert PotCalStore(dummy_transport).get() is not None  # saved normally


def test_main_diverged_warns_and_full_yes_saves(monkeypatch, capsys):
    """A far-off stored cal triggers the warning; 'yes' persists the new fit."""
    dummy_transport = DummyTransport()
    # Stored cal far from the new fit (new fit is angle = 400V - 400).
    PotCalStore(dummy_transport).upload({"pot_az": [50.0, -50.0]})
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(
        calibrate_pot,
        "collect_azimuth",
        lambda *a, **k: ([1.0, 1.9], [0.0, 360.0], 1.0),
    )
    monkeypatch.setattr("builtins.input", lambda *a, **k: "yes")
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "azimuth"])

    calibrate_pot.main()

    out = capsys.readouterr().out
    assert "WARNING" in out
    assert "differs from the stored one" in out
    expected_m, expected_b = calibrate_pot.fit_slope_pin_zero(
        [1.0, 1.9], [0.0, 360.0], 1.0
    )
    stored = PotCalStore(dummy_transport).get()
    assert stored["pot_az"][0] == pytest.approx(expected_m)
    assert stored["pot_az"][1] == pytest.approx(expected_b)
    assert len(fake_proxy.calls) == 1


def test_main_diverged_y_alone_discards(monkeypatch):
    """Under the divergence warning, a bare 'y' is not enough — it discards."""
    dummy_transport = DummyTransport()
    PotCalStore(dummy_transport).upload({"pot_az": [50.0, -50.0]})
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr(
        calibrate_pot,
        "collect_azimuth",
        lambda *a, **k: ([1.0, 1.9], [0.0, 360.0], 1.0),
    )
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "azimuth"])

    calibrate_pot.main()

    # Stored cal is unchanged (still the seeded values) and nothing pushed.
    stored = PotCalStore(dummy_transport).get()
    assert stored["pot_az"] == pytest.approx([50.0, -50.0])
    assert fake_proxy.calls == []


def test_prompt_save_default_path(monkeypatch):
    cases = [
        ("y", True),
        ("yes", True),
        ("Y", True),
        ("n", False),
        ("", False),
        ("nope", False),
    ]
    for ans, expected in cases:
        monkeypatch.setattr("builtins.input", lambda *a, **k: ans)
        assert calibrate_pot.prompt_save(False) is expected


def test_prompt_save_diverged_requires_full_yes(monkeypatch):
    cases = [
        ("yes", True),
        ("YES", True),
        (" yes ", True),
        ("y", False),
        ("", False),
        ("no", False),
    ]
    for ans, expected in cases:
        monkeypatch.setattr("builtins.input", lambda *a, **k: ans)
        assert calibrate_pot.prompt_save(True) is expected


# ---------------------------------------------------------------------------
# slope sanity check (all modes)
# ---------------------------------------------------------------------------


def test_expected_slope_mag_installed_pot():
    # 3.75-turn pot whose wiper spans the 3.3 V ADC range:
    #   3.75 * 360 / 3.3 = 409.09... deg/V
    assert calibrate_pot.expected_slope_mag(3.75) == pytest.approx(
        3.75 * 360.0 / 3.3
    )
    assert calibrate_pot.expected_slope_mag(3.75) == pytest.approx(
        409.09, abs=0.1
    )


def test_slope_out_of_range_in_window():
    # Expected ~409 deg/V; factor 1.5 -> in-range ~273..614 deg/V.
    assert calibrate_pot.slope_out_of_range(409.0, 3.75) is False
    assert calibrate_pot.slope_out_of_range(300.0, 3.75) is False
    assert calibrate_pot.slope_out_of_range(600.0, 3.75) is False
    # Sign is irrelevant — magnitude only.
    assert calibrate_pot.slope_out_of_range(-409.0, 3.75) is False


def test_slope_out_of_range_flags_gross_errors():
    # An order-of-magnitude typo in either direction is flagged.
    assert calibrate_pot.slope_out_of_range(40.9, 3.75) is True
    assert calibrate_pot.slope_out_of_range(4090.0, 3.75) is True
    # Just outside the 1.5x window on each side.
    assert calibrate_pot.slope_out_of_range(200.0, 3.75) is True
    assert calibrate_pot.slope_out_of_range(700.0, 3.75) is True
    # A zero slope is never sane.
    assert calibrate_pot.slope_out_of_range(0.0, 3.75) is True


# ---------------------------------------------------------------------------
# manual mode main()
# ---------------------------------------------------------------------------


def _manual_argv(slope, intercept, *extra):
    return [
        "calibrate-pot",
        "--mode",
        "manual",
        "--slope",
        str(slope),
        "--intercept",
        str(intercept),
        *extra,
    ]


def test_main_manual_stores_and_pushes(monkeypatch):
    """--mode manual writes the typed slope/intercept and pushes live."""
    dummy_transport = DummyTransport()
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(
        sys,
        "argv",
        _manual_argv(409.0, -400.0, "--note", "restored from corr_x.h5"),
    )

    calibrate_pot.main()

    stored = PotCalStore(dummy_transport).get()
    assert stored["pot_az"] == pytest.approx([409.0, -400.0])
    assert stored["metadata"]["mode"] == "manual"
    assert stored["metadata"]["note"] == "restored from corr_x.h5"
    # No sweep happened, so no sample arrays in the metadata.
    assert "pot_az_voltages" not in stored["metadata"]
    assert "angles" not in stored["metadata"]

    assert len(fake_proxy.calls) == 1
    action, kwargs = fake_proxy.calls[0]
    assert action == "set_calibration"
    assert kwargs["pot_az_params"] == pytest.approx([409.0, -400.0])


def test_main_manual_requires_slope_and_intercept(monkeypatch):
    """--mode manual without both numbers exits(1) and writes nothing."""
    dummy_transport = DummyTransport()
    _fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    # --intercept given but --slope omitted.
    monkeypatch.setattr(
        sys,
        "argv",
        ["calibrate-pot", "--mode", "manual", "--intercept", "-400.0"],
    )

    with pytest.raises(SystemExit) as exc:
        calibrate_pot.main()
    assert exc.value.code == 1
    assert PotCalStore(dummy_transport).get() is None


def test_main_manual_redis_first_when_pot_unavailable(monkeypatch, capsys):
    """Manual mode writes Redis even when the pot is down; no live push."""
    dummy_transport = DummyTransport()
    fake_proxy, proxy_factory = _make_fake_proxy()
    fake_proxy.is_available = False  # pot not reachable (e.g. fresh Pi)
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", _manual_argv(409.0, -400.0))

    calibrate_pot.main()

    # Durable restore happened despite the pot being down.
    stored = PotCalStore(dummy_transport).get()
    assert stored["pot_az"] == pytest.approx([409.0, -400.0])
    # No live push was attempted (would have timed out).
    assert fake_proxy.calls == []
    assert "loads on" in capsys.readouterr().out.lower()


def test_main_manual_overwrites_existing_without_divergence(
    monkeypatch, capsys
):
    """Manual mode skips the divergence check (no swept window) and overwrites
    any existing stored cal without crashing."""
    dummy_transport = DummyTransport()
    # A wildly different stored cal must NOT trigger the divergence warning,
    # and the empty swept window must NOT crash predicted_angle_divergence.
    PotCalStore(dummy_transport).upload({"pot_az": [50.0, -50.0]})
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", _manual_argv(409.0, -400.0))

    calibrate_pot.main()

    out = capsys.readouterr().out
    assert "differs from the stored one" not in out
    stored = PotCalStore(dummy_transport).get()
    assert stored["pot_az"] == pytest.approx([409.0, -400.0])


def test_main_manual_bad_slope_warns_and_requires_full_yes(
    monkeypatch, capsys
):
    """An off-by-10x --slope prints a WARNING and a bare 'y' is not enough."""
    dummy_transport = DummyTransport()
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    # slope 4090 deg/V is ~10x the expected ~409 -> out of range.
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", _manual_argv(4090.0, -400.0))

    calibrate_pot.main()

    out = capsys.readouterr().out
    assert "WARNING" in out
    assert "expected" in out.lower()
    # Bare 'y' must NOT save under the escalated (typed 'yes') gate.
    assert PotCalStore(dummy_transport).get() is None
    assert fake_proxy.calls == []


def test_main_manual_bad_slope_full_yes_saves(monkeypatch):
    """The same off-slope save goes through when the operator types 'yes'."""
    dummy_transport = DummyTransport()
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr("builtins.input", lambda *a, **k: "yes")
    monkeypatch.setattr(sys, "argv", _manual_argv(4090.0, -400.0))

    calibrate_pot.main()

    assert PotCalStore(dummy_transport).get()["pot_az"] == pytest.approx(
        [4090.0, -400.0]
    )
    assert len(fake_proxy.calls) == 1


def test_main_manual_sane_slope_no_escalation(monkeypatch, capsys):
    """A sane --slope (~409) prints no slope warning and saves on a bare 'y'."""
    dummy_transport = DummyTransport()
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", _manual_argv(409.0, -400.0))

    calibrate_pot.main()

    out = capsys.readouterr().out
    assert "off the expected" not in out.lower()
    assert PotCalStore(dummy_transport).get()["pot_az"] == pytest.approx(
        [409.0, -400.0]
    )


def test_main_manual_triggers_bgsave(monkeypatch):
    """Saving in manual mode forces an RDB snapshot (durable restore)."""
    dummy_transport = DummyTransport()
    _fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    bgsave_calls = []
    monkeypatch.setattr(
        dummy_transport.r, "bgsave", lambda *a, **k: bgsave_calls.append(1)
    )
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", _manual_argv(409.0, -400.0))

    calibrate_pot.main()

    assert bgsave_calls == [1]
