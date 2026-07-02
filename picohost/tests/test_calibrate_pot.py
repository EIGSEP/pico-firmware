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
    # 22600 steps = one full turn for the installed drive (MOTOR_GEOMETRY).
    t = DummyTransport()
    _xadd_motor(t, 22600)
    deg = calibrate_pot.read_motor_az_deg(t, start_id="0-0")
    assert deg == pytest.approx(360.0)


def test_read_motor_az_steps_raises_when_silent():
    t = DummyTransport()
    with pytest.raises(RuntimeError, match="motor"):
        # Nothing ever published; "$" returns after the block timeout.
        calibrate_pot.read_motor_az_steps(t, start_id="$")


def _seq(values):
    it = iter(values)
    return lambda *a, **k: next(it)


class FakePicoProxy:
    """Records send_command calls; the one fake proxy for every test here.

    ``fail_on`` raises ``fail_exc`` when that action is sent; flip
    ``is_available`` to simulate a heartbeat drop.
    """

    def __init__(self, name="", available=True, fail_on=None, fail_exc=None):
        self.name = name
        self.is_available = available
        self.calls = []
        self._fail_on = fail_on
        self._fail_exc = fail_exc or RuntimeError("boom")

    def send_command(self, action, **kwargs):
        self.calls.append((action, kwargs))
        if action == self._fail_on:
            raise self._fail_exc


def test_collect_azimuth_pairs_voltage_with_motor_az(monkeypatch):
    monkeypatch.setattr(
        calibrate_pot, "collect_samples", _seq([1.0, 1.5, 2.0])
    )
    monkeypatch.setattr(
        calibrate_pot, "read_motor_az_deg", _seq([0.5, 180.0, 360.0])
    )
    # home prompt, two stop prompts, then 'q' to finish
    monkeypatch.setattr("builtins.input", _seq(["", "", "", "q"]))

    voltages, angles, v0 = calibrate_pot.collect_azimuth(
        DummyTransport(), n_samples=10
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
    calibrate_pot.collect_azimuth(DummyTransport(), 10)
    assert "WARNING" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# collect_auto (--mode auto): motor-driven sweep
# ---------------------------------------------------------------------------


def test_collect_auto_commands_expected_targets_and_reads_actual_az(
    monkeypatch,
):
    # The sweep must cover the production operating window (-180..+180
    # deg around home), not 0..360: n_stops=2 -> 180 deg spacing gives
    # commanded targets -180, 0, +180, then home (0). Actual settled
    # reads are offset from the commanded target (within the settle
    # tolerance) to prove angles come from the stream, not the target.
    monkeypatch.setattr(
        calibrate_pot,
        "read_motor_az_deg",
        _seq(
            [
                1.0,  # home read
                -179.0,
                -179.0,  # settle at stop 1 (target -180)
                0.5,
                0.5,  # settle at stop 2 (target 0)
                179.5,
                179.5,  # settle at stop 3 (target +180)
                0.2,
                0.2,  # settle back at home (target 0)
            ]
        ),
    )
    monkeypatch.setattr(
        calibrate_pot, "collect_samples", _seq([1.0, 1.4, 1.5, 1.6])
    )

    proxy = FakePicoProxy()
    voltages, angles, v0 = calibrate_pot.collect_auto(
        DummyTransport(), proxy, n_samples=10, n_stops=2
    )

    assert v0 == pytest.approx(1.0)
    assert voltages == [1.0, 1.4, 1.5, 1.6]
    # Recorded angles are the actual stream reads (-179, 0.5, 179.5),
    # not the commanded targets (-180, 0, 180).
    assert angles == [0.0, -179.0, 0.5, 179.5]

    move_calls = [c for c in proxy.calls if c[0] == "az_target_deg"]
    assert [c[1]["target_deg"] for c in move_calls] == [
        -180.0,
        0.0,
        180.0,
        0.0,
    ]
    # Moves must be non-blocking -- the manager's command thread cannot
    # afford to wait on a multi-minute move.
    for _action, kwargs in move_calls:
        assert kwargs["wait_for_start"] is False
        assert kwargs["wait_for_stop"] is False

    # Claimed at the start, released at the end; no halt on success.
    assert proxy.calls[0][0] == "claim"
    assert proxy.calls[-1][0] == "release"
    assert "halt" not in [c[0] for c in proxy.calls]


def test_collect_auto_warns_when_not_homed(monkeypatch, capsys):
    # n_stops=1 -> targets -180, +180, then home (0).
    monkeypatch.setattr(
        calibrate_pot,
        "read_motor_az_deg",
        _seq([10.0, -180.2, -180.2, 179.8, 179.8, 0.2, 0.2]),
    )
    monkeypatch.setattr(
        calibrate_pot, "collect_samples", _seq([1.5, 1.0, 2.0])
    )

    proxy = FakePicoProxy()
    calibrate_pot.collect_auto(
        DummyTransport(), proxy, n_samples=10, n_stops=1
    )

    assert "WARNING" in capsys.readouterr().out


def test_collect_auto_halts_and_releases_on_command_failure(monkeypatch):
    monkeypatch.setattr(calibrate_pot, "read_motor_az_deg", _seq([0.0]))
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0]))

    proxy = FakePicoProxy(fail_on="az_target_deg")
    with pytest.raises(RuntimeError, match="boom"):
        calibrate_pot.collect_auto(
            DummyTransport(), proxy, n_samples=10, n_stops=1
        )

    # Moves are non-blocking, so the failure path must stop the motor
    # before giving up the claim.
    assert proxy.calls[0][0] == "claim"
    assert [c[0] for c in proxy.calls[-2:]] == ["halt", "release"]


def test_collect_auto_halts_and_releases_on_settle_timeout(monkeypatch):
    # The motor never reaches the commanded target -- read_motor_az_deg
    # always reports the same stale value.
    monkeypatch.setattr(
        calibrate_pot, "read_motor_az_deg", lambda *a, **k: 0.0
    )
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0]))

    proxy = FakePicoProxy()
    with pytest.raises(TimeoutError, match="did not settle"):
        calibrate_pot.collect_auto(
            DummyTransport(),
            proxy,
            n_samples=10,
            n_stops=1,
            settle_timeout_s=0.05,
        )

    # The motor may still be en route after a settle timeout: halt it,
    # then release the claim.
    assert proxy.calls[0][0] == "claim"
    assert [c[0] for c in proxy.calls[-2:]] == ["halt", "release"]


def test_collect_auto_fails_fast_when_motor_drops_mid_sweep(monkeypatch):
    proxy = FakePicoProxy()

    def _sample_and_drop_heartbeat(*a, **k):
        # Heartbeat lapses while the home stop is being sampled — the
        # next move must raise immediately, not silently no-op and then
        # burn the full settle timeout.
        proxy.is_available = False
        return 1.0

    monkeypatch.setattr(calibrate_pot, "read_motor_az_deg", _seq([0.0]))
    monkeypatch.setattr(
        calibrate_pot, "collect_samples", _sample_and_drop_heartbeat
    )

    with pytest.raises(RuntimeError, match="unreachable"):
        calibrate_pot.collect_auto(
            DummyTransport(), proxy, n_samples=10, n_stops=1
        )

    # No move was dispatched; cleanup still ran.
    assert "az_target_deg" not in [c[0] for c in proxy.calls]
    assert proxy.calls[-1][0] == "release"


# ---------------------------------------------------------------------------
# rail guards: near the ADC rails the wiper clips and the pot silently
# stops being an absolute azimuth reference, so in-box sampling and the
# auto sweep must abort rather than record clipped voltages.
# ---------------------------------------------------------------------------


def test_check_off_rails_bounds():
    # Mid-range passes silently.
    calibrate_pot._check_off_rails(1.65, where="test")
    # Within the guard margin of either rail raises.
    with pytest.raises(RuntimeError, match="rail"):
        calibrate_pot._check_off_rails(0.05, where="test")
    with pytest.raises(RuntimeError, match="rail"):
        calibrate_pot._check_off_rails(3.25, where="test")


def test_sample_stop_aborts_on_railed_voltage(monkeypatch):
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([0.05]))
    with pytest.raises(RuntimeError, match="rail"):
        calibrate_pot._sample_stop(DummyTransport(), 10, 45.0)


def test_collect_auto_halts_on_railed_sample(monkeypatch):
    # Home samples fine; the first sweep stop reads a clipped voltage.
    # The sweep must halt the motor and release the claim, not keep
    # collecting a fit from clipped data.
    monkeypatch.setattr(
        calibrate_pot,
        "read_motor_az_deg",
        _seq([0.0, -179.5, -179.5]),
    )
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([1.0, 3.28]))

    proxy = FakePicoProxy()
    with pytest.raises(RuntimeError, match="rail"):
        calibrate_pot.collect_auto(
            DummyTransport(), proxy, n_samples=10, n_stops=2
        )

    assert [c[0] for c in proxy.calls[-2:]] == ["halt", "release"]


def test_collect_auto_preflight_aborts_before_moving_when_home_near_rail(
    monkeypatch,
):
    # v0=0.4 V: at the field-measured slope (~320 deg/V) the -180 deg
    # half of the sweep needs ~0.56 V of travel, which would cross the
    # 0 V rail. The preflight must abort before any move is commanded.
    monkeypatch.setattr(calibrate_pot, "read_motor_az_deg", _seq([0.0]))
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([0.4]))

    proxy = FakePicoProxy()
    with pytest.raises(RuntimeError, match="headroom"):
        calibrate_pot.collect_auto(
            DummyTransport(), proxy, n_samples=10, n_stops=8
        )

    assert "az_target_deg" not in [c[0] for c in proxy.calls]
    assert proxy.calls[-1][0] == "release"


def test_collect_auto_preflight_uses_stored_slope(monkeypatch):
    # A stored steep slope (1000 deg/V -> 0.18 V per 180 deg) makes the
    # same 0.4 V home viable; the empirical fallback alone would abort.
    t = DummyTransport()
    PotCalStore(t).upload({"pot_az": [1000.0, -400.0]})
    monkeypatch.setattr(
        calibrate_pot,
        "read_motor_az_deg",
        _seq([0.0, -179.8, -179.8, 179.9, 179.9, 0.1, 0.1]),
    )
    monkeypatch.setattr(
        calibrate_pot, "collect_samples", _seq([0.4, 0.3, 0.55])
    )

    proxy = FakePicoProxy()
    voltages, angles, v0 = calibrate_pot.collect_auto(
        t, proxy, n_samples=10, n_stops=1
    )

    assert v0 == pytest.approx(0.4)
    move_calls = [c for c in proxy.calls if c[0] == "az_target_deg"]
    assert [c[1]["target_deg"] for c in move_calls] == [-180.0, 180.0, 0.0]


def test_wait_for_settle_aborts_when_pot_rails_mid_move(monkeypatch):
    # The motor is settling happily on target, but the pot hit a rail
    # mid-move: the settle poll must abort (upstream halts the motor)
    # rather than report a clean arrival.
    t = DummyTransport()
    t.r.xadd(
        calibrate_pot.POTMON_STREAM,
        {"value": json.dumps({"pot_az_voltage": 0.05})},
    )
    monkeypatch.setattr(
        calibrate_pot, "read_motor_az_deg", lambda *a, **k: 180.0
    )

    with pytest.raises(RuntimeError, match="rail"):
        calibrate_pot._wait_for_settle(t, 180.0, timeout_s=1.0)


def test_wait_for_settle_passes_with_healthy_pot(monkeypatch):
    t = DummyTransport()
    t.r.xadd(
        calibrate_pot.POTMON_STREAM,
        {"value": json.dumps({"pot_az_voltage": 1.5})},
    )
    monkeypatch.setattr(
        calibrate_pot, "read_motor_az_deg", lambda *a, **k: 180.0
    )

    az = calibrate_pot._wait_for_settle(t, 180.0, timeout_s=1.0)
    assert az == pytest.approx(180.0)


def test_rezero_aborts_on_railed_home(monkeypatch):
    # Re-pinning zero to a clipped voltage would poison the intercept.
    t = DummyTransport()
    PotCalStore(t).upload({"pot_az": [320.0, -50.0]})
    monkeypatch.setattr(calibrate_pot, "collect_samples", _seq([3.25]))
    monkeypatch.setattr("builtins.input", _seq([""]))
    with pytest.raises(RuntimeError, match="rail"):
        calibrate_pot.rezero(t, n_samples=10)


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


def test_build_parser_accepts_new_modes():
    p = calibrate_pot.build_parser()

    assert p.parse_args(["--mode", "azimuth"]).mode == "azimuth"
    assert p.parse_args(["--mode", "rezero"]).mode == "rezero"
    assert p.parse_args([]).mode == "azimuth"  # in-box is the common case


def test_build_parser_rejects_motor_geometry_flags():
    # Geometry is a fixed hardware property (picohost.motor constants),
    # deliberately not CLI-tunable: a client-side override would desync
    # deg->steps (manager) from steps->deg (calibrate-pot) and make
    # --mode auto's settle detection unreachable.
    p = calibrate_pot.build_parser()
    for flag in ("--step-angle-deg", "--gear-teeth", "--microstep"):
        with pytest.raises(SystemExit):
            p.parse_args([flag, "2"])


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
    """Return a shared FakePicoProxy instance and a factory that yields it."""
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
    """FakePicoProxy factory keyed by device name.

    Unlike ``_make_fake_proxy`` (a single shared instance), --mode auto
    constructs two proxies (pot and motor) that must be independently
    controllable and independently record their calls.
    """
    proxies = {}

    def factory(name, *args, **kwargs):
        proxy = FakePicoProxy(name, available=name not in unavailable)
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


def test_main_auto_mode_rejects_spacing_within_settle_tolerance(monkeypatch):
    """Stops closer than 2x the settle tolerance would false-settle at the
    previous stop (the stationary pre-move position already matches the
    target within tolerance), silently corrupting the calibration."""
    dummy_transport = DummyTransport()
    _proxies, proxy_factory = _make_named_fake_proxy_factory()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    # 90 stops -> 4.0 deg spacing == 2 * SETTLE_TOL_DEG: not strictly
    # greater, so it must be rejected (89 -> ~4.04 deg is the max).
    monkeypatch.setattr(
        sys, "argv", ["calibrate-pot", "--mode", "auto", "--n-stops", "90"]
    )

    with pytest.raises(SystemExit) as exc:
        calibrate_pot.main()
    assert exc.value.code == 1
    assert PotCalStore(dummy_transport).get() is None


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
    # Expected 320 deg/V (field-measured); factor 1.5 -> ~213..480 deg/V.
    assert calibrate_pot.slope_out_of_range(320.0, 320.0) is False
    assert calibrate_pot.slope_out_of_range(409.0, 320.0) is False
    assert calibrate_pot.slope_out_of_range(250.0, 320.0) is False
    # Sign is irrelevant — magnitude only.
    assert calibrate_pot.slope_out_of_range(-320.0, 320.0) is False


def test_slope_out_of_range_flags_gross_errors():
    # An order-of-magnitude typo in either direction is flagged.
    assert calibrate_pot.slope_out_of_range(32.0, 320.0) is True
    assert calibrate_pot.slope_out_of_range(3200.0, 320.0) is True
    # Just outside the 1.5x window on each side.
    assert calibrate_pot.slope_out_of_range(200.0, 320.0) is True
    assert calibrate_pot.slope_out_of_range(500.0, 320.0) is True
    # A zero slope is never sane, nor is a non-positive expectation.
    assert calibrate_pot.slope_out_of_range(0.0, 320.0) is True
    assert calibrate_pot.slope_out_of_range(320.0, 0.0) is True


def test_main_manual_sanity_centered_on_empirical_slope(monkeypatch):
    """In-box modes check the slope against the field-measured ~320 deg/V.

    500 deg/V sat comfortably inside the old physically-derived window
    (409 * 1.5 = 614) but is >1.5x off the measured slope, so it must
    escalate the save — a bare 'y' discards."""
    dummy_transport = DummyTransport()
    fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", _manual_argv(500.0, -400.0))

    calibrate_pot.main()

    assert PotCalStore(dummy_transport).get() is None
    assert fake_proxy.calls == []


def test_main_bench_mode_keeps_physical_expectation(monkeypatch, capsys):
    """Bench modes keep the physical turns*360/vref expectation.

    A 500 deg/V fit is in range for a 3.75-turn pot on the bench (where
    the wiper genuinely spans the full range) even though it would fail
    the in-box empirical window."""
    dummy_transport = DummyTransport()
    _fake_proxy, proxy_factory = _make_fake_proxy()
    monkeypatch.setattr(
        calibrate_pot, "Transport", lambda *a, **k: dummy_transport
    )
    monkeypatch.setattr(calibrate_pot, "PicoProxy", proxy_factory)
    # minmax fit: 0..1350 deg over 0.3..3.0 V -> slope = 1350/2.7 = 500
    monkeypatch.setattr(
        calibrate_pot,
        "collect_minmax",
        lambda *a, **k: ([0.3, 3.0], [0.0, 1350.0]),
    )
    monkeypatch.setattr("builtins.input", lambda *a, **k: "y")
    monkeypatch.setattr(sys, "argv", ["calibrate-pot", "--mode", "minmax"])

    calibrate_pot.main()

    out = capsys.readouterr().out
    assert "off the expected" not in out.lower()
    assert PotCalStore(dummy_transport).get() is not None


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
