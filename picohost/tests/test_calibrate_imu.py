import json
import numpy as np
import pytest
from eigsep_redis.testing import DummyTransport

from picohost import calibrate_imu
from picohost.buses import ImuCalStore, PotCalStore


def test_build_parser_modes():
    p = calibrate_imu.build_parser()
    args = p.parse_args(["--mode", "elevation"])
    assert args.mode == "elevation"
    for m in ("elevation", "azimuth", "all"):
        assert p.parse_args(["--mode", m]).mode == m


def test_collect_vector_averages_named_fields(monkeypatch):
    t = DummyTransport()
    # publish 3 entries on stream:imu_az
    for ax in (1.0, 2.0, 3.0):
        t.r.xadd(
            "stream:imu_az",
            {
                "value": json.dumps(
                    {"accel_x": ax, "accel_y": 0.0, "accel_z": 9.0}
                )
            },
        )
    # start_id="0-0" reads from the beginning so the test doesn't race the
    # "$" (new-entries-only) production default.
    v = calibrate_imu.collect_vector(
        t, "imu_az", ("accel_x", "accel_y", "accel_z"), n=3, start_id="0-0"
    )
    assert np.allclose(v, [2.0, 0.0, 9.0])  # mean of 1,2,3 -> 2


def test_yaw_collected_with_circular_mean(monkeypatch):
    """_yaw must reduce its samples circularly (robust to the +/-180 wrap),
    not with the linear mean that collect_vector applies by default."""
    cal = calibrate_imu.Calibrator(DummyTransport(), 2, {"imu_az"}, "azimuth")

    def fake_collect(transport, name, fields, n, start_id="$", reducer=None):
        assert reducer is not None  # _yaw must supply a circular reducer
        return reducer(np.array([[179.0], [-179.0]]))

    monkeypatch.setattr(calibrate_imu, "collect_vector", fake_collect)
    assert abs(cal._yaw()) == pytest.approx(180.0, abs=1e-6)  # linear -> ~0


def test_main_azimuth_uncalibrated_pot_aborts(monkeypatch):
    """An alive-but-uncalibrated pot is not a usable az standard: azimuth
    mode must abort up front, not crash mid-sweep on a None pot angle."""
    transport = DummyTransport()
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: transport)
    monkeypatch.setattr(
        calibrate_imu,
        "stream_alive",
        lambda t, n, timeout_s=5.0: n in ("imu_az", "potmon"),
    )
    monkeypatch.setattr(
        calibrate_imu.Calibrator,
        "run_sweeps",
        lambda self: pytest.fail("must abort before sweeps"),
    )
    # no PotCalStore uploaded -> pot alive but uncalibrated
    assert calibrate_imu.main(["--mode", "azimuth"]) == 1
    assert ImuCalStore(transport).get() is None


def test_stream_alive_true_false(monkeypatch):
    """Liveness via blocking "$": True only when a NEW entry arrives.

    Monkeypatch xread directly so the test is deterministic and doesn't
    depend on fakeredis racing the "$" cursor against pre-loaded entries
    (a "$" read correctly sees nothing for pre-existing entries).
    """
    t = DummyTransport()

    alive = [("stream:imu_az", [(b"1-0", {b"value": b"{}"})])]
    monkeypatch.setattr(t.r, "xread", lambda *a, **k: alive)
    assert calibrate_imu.stream_alive(t, "imu_az", timeout_s=0.1) is True

    monkeypatch.setattr(t.r, "xread", lambda *a, **k: [])
    assert calibrate_imu.stream_alive(t, "imu_el", timeout_s=0.1) is False


def test_main_persists_and_pushes(monkeypatch):
    """End-to-end main(): synthetic sweeps -> fit -> ImuCalStore + proxy push."""
    from picohost import imu_geometry as ig

    transport = DummyTransport()
    PotCalStore(transport).upload({"pot_az": [1.0, 0.0]})  # az standard ready

    # Fake operator-driven collection: return forward-model sweeps.
    M_az = ig.R_z(-0.3)
    phi_degs = np.linspace(0, 359, 24)

    def fake_run_sweeps(self):
        return (
            {  # el_sweep
                "imu_el": None,
                "imu_az": None,
                "level_index": 12,
                "direction": 1,
            },
            {  # az_level
                "imu_az": np.array(
                    [ig.R_z(np.radians(p)).T @ [0, 0, 1.0] for p in phi_degs]
                ),
                "yaw_deg": -phi_degs,
                "pot_deg": -phi_degs + 40.0,
            },
            {  # az_tilt
                "imu_az": np.array(
                    [
                        M_az.T
                        @ (
                            ig.R_z(np.radians(p)).T
                            @ [
                                0,
                                np.sin(np.radians(40)),
                                np.cos(np.radians(40)),
                            ]
                        )
                        for p in phi_degs
                    ]
                )
                * ig.GRAVITY,
                "pot_deg": -phi_degs + 40.0,
                "imu_el": None,
            },
        )

    captured = {}

    class FakeProxy:
        def __init__(self, *a, **k):
            pass

        @property
        def is_available(self):
            return True

        def send_command(self, action, **kw):
            captured[action] = kw
            return {}

    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: transport)
    monkeypatch.setattr(calibrate_imu, "PicoProxy", FakeProxy)
    monkeypatch.setattr(
        calibrate_imu.Calibrator, "run_sweeps", fake_run_sweeps
    )
    monkeypatch.setattr(
        calibrate_imu,
        "stream_alive",
        lambda t, n, timeout_s=5.0: n == "imu_az" or n == "potmon",
    )
    monkeypatch.setattr("builtins.input", lambda *a: "y")  # confirm save

    rc = calibrate_imu.main(["--mode", "azimuth"])
    assert rc == 0
    stored = ImuCalStore(transport).get()
    assert "imu_az" in stored
    assert "set_calibration" in captured  # live push happened
    assert "imu_az" in captured["set_calibration"]


def test_main_discard_writes_nothing(monkeypatch):
    """Declining the save prompt persists nothing and pushes nothing."""
    from picohost import imu_geometry as ig

    transport = DummyTransport()
    PotCalStore(transport).upload({"pot_az": [1.0, 0.0]})  # az standard ready

    M_az = ig.R_z(-0.3)
    phi_degs = np.linspace(0, 359, 24)

    def fake_run_sweeps(self):
        return (
            {
                "imu_el": None,
                "imu_az": None,
                "level_index": 12,
                "direction": 1,
            },
            {
                "imu_az": np.array(
                    [ig.R_z(np.radians(p)).T @ [0, 0, 1.0] for p in phi_degs]
                ),
                "yaw_deg": -phi_degs,
                "pot_deg": -phi_degs + 40.0,
            },
            {
                "imu_az": np.array(
                    [
                        M_az.T
                        @ (
                            ig.R_z(np.radians(p)).T
                            @ [
                                0,
                                np.sin(np.radians(40)),
                                np.cos(np.radians(40)),
                            ]
                        )
                        for p in phi_degs
                    ]
                )
                * ig.GRAVITY,
                "pot_deg": -phi_degs + 40.0,
                "imu_el": None,
            },
        )

    captured = {}

    class FakeProxy:
        def __init__(self, *a, **k):
            pass

        @property
        def is_available(self):
            return True

        def send_command(self, action, **kw):
            captured[action] = kw
            return {}

    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: transport)
    monkeypatch.setattr(calibrate_imu, "PicoProxy", FakeProxy)
    monkeypatch.setattr(
        calibrate_imu.Calibrator, "run_sweeps", fake_run_sweeps
    )
    monkeypatch.setattr(
        calibrate_imu,
        "stream_alive",
        lambda t, n, timeout_s=5.0: n == "imu_az" or n == "potmon",
    )
    monkeypatch.setattr("builtins.input", lambda *a: "n")  # decline save

    rc = calibrate_imu.main(["--mode", "azimuth"])
    assert rc == 0  # clean discard
    assert ImuCalStore(transport).get() is None  # nothing persisted
    assert "set_calibration" not in captured  # no live push
