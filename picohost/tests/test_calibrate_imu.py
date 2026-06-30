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
        "stream_status",
        lambda t, n, **k: "healthy" if n in ("imu_az", "potmon") else "dead",
    )
    monkeypatch.setattr(
        calibrate_imu.Calibrator,
        "run_sweeps",
        lambda self: pytest.fail("must abort before sweeps"),
    )
    # no PotCalStore uploaded -> pot alive but uncalibrated
    assert calibrate_imu.main(["--mode", "azimuth"]) == 1
    assert ImuCalStore(transport).get() is None




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
        "stream_status",
        lambda t, n, **k: "healthy" if n in ("imu_az", "potmon") else "dead",
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
        "stream_status",
        lambda t, n, **k: "healthy" if n in ("imu_az", "potmon") else "dead",
    )
    monkeypatch.setattr("builtins.input", lambda *a: "n")  # decline save

    rc = calibrate_imu.main(["--mode", "azimuth"])
    assert rc == 0  # clean discard
    assert ImuCalStore(transport).get() is None  # nothing persisted
    assert "set_calibration" not in captured  # no live push


def _xadd(t, stream, **fields):
    t.r.xadd(stream, {"value": json.dumps(fields)})


def test_collect_vector_skips_error_frames(monkeypatch):
    """status=error frames carry junk (accel=[0,0,0]); collect_vector must
    drop them and average only the valid samples."""
    t = DummyTransport()
    _xadd(t, "stream:imu_el", status="error", accel_x=0.0, accel_y=0.0, accel_z=0.0)
    _xadd(t, "stream:imu_el", status="update", accel_x=2.0, accel_y=0.0, accel_z=8.0)
    _xadd(t, "stream:imu_el", status="error", accel_x=0.0, accel_y=0.0, accel_z=0.0)
    _xadd(t, "stream:imu_el", status="update", accel_x=4.0, accel_y=0.0, accel_z=10.0)
    v = calibrate_imu.collect_vector(
        t, "imu_el", ("accel_x", "accel_y", "accel_z"), n=2, start_id="0-0"
    )
    assert np.allclose(v, [3.0, 0.0, 9.0])  # mean of the two valid frames


def test_collect_vector_all_error_raises_named(monkeypatch):
    """A faulted IMU (every frame status=error) must abort with a message
    naming the stream and the valid/required counts — not block or NaN."""
    t = DummyTransport()
    for _ in range(3):
        _xadd(t, "stream:imu_el", status="error", accel_x=0.0, accel_y=0.0, accel_z=0.0)
    with pytest.raises(RuntimeError, match="imu_el.*status=error"):
        calibrate_imu.collect_vector(
            t, "imu_el", ("accel_x", "accel_y", "accel_z"), n=3, start_id="0-0"
        )


def test_stream_status_healthy(monkeypatch):
    """An update frame -> healthy."""
    t = DummyTransport()
    resp = [("stream:imu_az", [(b"1-0", {b"value": json.dumps(
        {"status": "update", "accel_x": 0.0}).encode()})])]
    monkeypatch.setattr(t.r, "xread", lambda *a, **k: resp)
    assert calibrate_imu.stream_status(t, "imu_az", timeout_s=0.1) == "healthy"


def test_stream_status_faulted(monkeypatch):
    """Frames arriving but all status=error -> faulted (publisher up, sensor
    down). Drained one window then an empty read ends the loop."""
    t = DummyTransport()
    err = [("stream:imu_el", [(b"1-0", {b"value": json.dumps(
        {"status": "error", "accel_x": 0.0}).encode()})])]
    calls = [err, []]  # first read: an error frame; second: nothing more
    monkeypatch.setattr(t.r, "xread", lambda *a, **k: calls.pop(0) if calls else [])
    assert calibrate_imu.stream_status(t, "imu_el", timeout_s=0.1) == "faulted"


def test_stream_status_dead(monkeypatch):
    """No frame within the window -> dead."""
    t = DummyTransport()
    monkeypatch.setattr(t.r, "xread", lambda *a, **k: [])
    assert calibrate_imu.stream_status(t, "imu_el", timeout_s=0.1) == "dead"


def test_stream_alive_delegates_to_status(monkeypatch):
    """stream_alive is True only for a healthy stream."""
    monkeypatch.setattr(calibrate_imu, "stream_status",
                        lambda *a, **k: "healthy")
    assert calibrate_imu.stream_alive(DummyTransport(), "imu_az") is True
    monkeypatch.setattr(calibrate_imu, "stream_status",
                        lambda *a, **k: "faulted")
    assert calibrate_imu.stream_alive(DummyTransport(), "imu_el") is False


def _answers(*seq):
    """Return an input() stand-in that yields the given answers in order."""
    it = iter(seq)
    return lambda *a, **k: next(it)


def test_main_faulted_imu_aborts_by_default(monkeypatch):
    """imu_el publishing only status=error: main names it and aborts (return
    1) when the operator does not opt to continue — never reaching the fit."""
    transport = DummyTransport()
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: transport)
    monkeypatch.setattr(
        calibrate_imu, "stream_status",
        lambda t, n, **k: "faulted" if n == "imu_el" else "healthy",
    )
    monkeypatch.setattr(
        calibrate_imu.Calibrator, "run_sweeps",
        lambda self: pytest.fail("must abort before sweeps"),
    )
    monkeypatch.setattr("builtins.input", _answers(""))  # Enter = abort
    assert calibrate_imu.main(["--mode", "elevation"]) == 1
    assert ImuCalStore(transport).get() is None


def test_main_faulted_imu_continue_skips_it(monkeypatch):
    """Operator opts to continue without the faulted imu_el: main proceeds and
    imu_el is absent from the alive set handed to the Calibrator."""
    transport = DummyTransport()
    PotCalStore(transport).upload({"pot_az": [1.0, 0.0]})
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: transport)
    monkeypatch.setattr(
        calibrate_imu, "stream_status",
        lambda t, n, **k: "faulted" if n == "imu_el" else "healthy",
    )
    seen = {}

    def fake_run_sweeps(self):
        seen["alive"] = set(self.alive)
        # Minimal empty sweeps -> no sections -> main returns 1 cleanly.
        return (
            {"imu_el": None, "imu_az": None, "level_index": 0, "direction": 1},
            {"imu_az": None, "yaw_deg": None, "pot_deg": None},
            {"imu_az": None, "pot_deg": None, "imu_el": None},
        )

    monkeypatch.setattr(calibrate_imu.Calibrator, "run_sweeps", fake_run_sweeps)
    monkeypatch.setattr("builtins.input", _answers("y"))  # continue past fault
    calibrate_imu.main(["--mode", "all"])
    assert "imu_el" not in seen["alive"]
    assert "imu_az" in seen["alive"]


def test_main_fit_valueerror_is_clean(monkeypatch):
    """A backstop ValueError from the fit surfaces as a clean return 1, not an
    uncaught traceback."""
    transport = DummyTransport()
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: transport)
    monkeypatch.setattr(
        calibrate_imu, "stream_status", lambda t, n, **k: "healthy"
    )
    monkeypatch.setattr(
        calibrate_imu.Calibrator, "run_sweeps",
        lambda self: ({"imu_el": None, "imu_az": None, "level_index": 0,
                       "direction": 1},
                      {"imu_az": None, "yaw_deg": None, "pot_deg": None},
                      {"imu_az": None, "pot_deg": None, "imu_el": None}),
    )

    def boom(*a, **k):
        raise ValueError("degenerate accel sphere")

    monkeypatch.setattr(calibrate_imu, "fit_calibration_from_sweeps", boom)
    monkeypatch.setattr("builtins.input", _answers("y"))
    assert calibrate_imu.main(["--mode", "elevation"]) == 1
