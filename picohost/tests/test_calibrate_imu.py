import json

import numpy as np
import pytest
from eigsep_redis.testing import DummyTransport

from picohost import calibrate_imu
from picohost import imu_geometry as ig
from picohost.buses import ImuCalStore, PotCalStore


# --- synthetic el-sweep helpers (mirrored from tests/test_imu_geometry.py) --
# Duplicated (not imported) so this suite doesn't depend on another test
# module's import surface; kept minimal and in sync with the originals.
def _el_sweep_units(motor_deg, M_true, level_offset_deg=0.0):
    """Sensor-frame accel unit vectors for an el sweep (see test_imu_geometry).

    Physical el at each stop = motor + level_offset; M_true maps sensor
    a_unit -> host frame, host gravity at el t = [0, sin t, cos t].
    """
    ts = np.radians(np.asarray(motor_deg, float) + level_offset_deg)
    host = np.array([[0.0, np.sin(t), np.cos(t)] for t in ts])
    return host @ M_true


MOTOR_STOPS = np.arange(-180.0, 181.0, 30.0)  # 13 stops, 360 deg span
M_EL_TRUE = ig.R_z(np.radians(3.0)) @ ig.R_x(np.pi)  # a_unit -> -z at level
M_AZ_TRUE = ig.R_y(-np.pi / 2) @ ig.R_x(np.radians(2.0))  # a_unit -> +x


def test_build_parser_modes():
    p = calibrate_imu.build_parser()
    args = p.parse_args([])
    assert args.mode == "auto"
    assert args.n_stops == 12
    args = p.parse_args(["-m", "manual"])
    assert args.mode == "manual"
    with pytest.raises(SystemExit):
        p.parse_args(["-m", "azimuth"])  # retired mode


class FakeProxy:
    def __init__(self, *a, **k):
        self.is_available = True
        self.commands = []

    def send_command(self, action, **kw):
        self.commands.append((action, kw))
        return {}


# ---------------------------------------------------------------------------
# collect_vector / stream_status: unchanged behavior, kept from the old suite
# ---------------------------------------------------------------------------


def test_collect_vector_averages_named_fields():
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


def _xadd(t, stream, **fields):
    t.r.xadd(stream, {"value": json.dumps(fields)})


def test_collect_vector_skips_error_frames():
    """status=error frames carry junk (accel=[0,0,0]); collect_vector must
    drop them and average only the valid samples."""
    t = DummyTransport()
    _xadd(
        t,
        "stream:imu_el",
        status="error",
        accel_x=0.0,
        accel_y=0.0,
        accel_z=0.0,
    )
    _xadd(
        t,
        "stream:imu_el",
        status="update",
        accel_x=2.0,
        accel_y=0.0,
        accel_z=8.0,
    )
    _xadd(
        t,
        "stream:imu_el",
        status="error",
        accel_x=0.0,
        accel_y=0.0,
        accel_z=0.0,
    )
    _xadd(
        t,
        "stream:imu_el",
        status="update",
        accel_x=4.0,
        accel_y=0.0,
        accel_z=10.0,
    )
    v = calibrate_imu.collect_vector(
        t, "imu_el", ("accel_x", "accel_y", "accel_z"), n=2, start_id="0-0"
    )
    assert np.allclose(v, [3.0, 0.0, 9.0])  # mean of the two valid frames


def test_collect_vector_all_error_raises_named():
    """A faulted IMU (every frame status=error) must abort with a message
    naming the stream and the valid/required counts — not block or NaN."""
    t = DummyTransport()
    for _ in range(3):
        _xadd(
            t,
            "stream:imu_el",
            status="error",
            accel_x=0.0,
            accel_y=0.0,
            accel_z=0.0,
        )
    with pytest.raises(RuntimeError, match="imu_el.*status=error"):
        calibrate_imu.collect_vector(
            t, "imu_el", ("accel_x", "accel_y", "accel_z"), n=3, start_id="0-0"
        )


def test_stream_status_healthy(monkeypatch):
    """An update frame -> healthy."""
    t = DummyTransport()
    resp = [
        (
            "stream:imu_az",
            [
                (
                    b"1-0",
                    {
                        b"value": json.dumps(
                            {"status": "update", "accel_x": 0.0}
                        ).encode()
                    },
                )
            ],
        )
    ]
    monkeypatch.setattr(t.r, "xread", lambda *a, **k: resp)
    assert calibrate_imu.stream_status(t, "imu_az", timeout_s=0.1) == "healthy"


def test_stream_status_faulted(monkeypatch):
    """Frames arriving but all status=error -> faulted (publisher up, sensor
    down). Drained one window then an empty read ends the loop."""
    t = DummyTransport()
    err = [
        (
            "stream:imu_el",
            [
                (
                    b"1-0",
                    {
                        b"value": json.dumps(
                            {"status": "error", "accel_x": 0.0}
                        ).encode()
                    },
                )
            ],
        )
    ]
    calls = [err, []]  # first read: an error frame; second: nothing more
    monkeypatch.setattr(
        t.r, "xread", lambda *a, **k: calls.pop(0) if calls else []
    )
    assert calibrate_imu.stream_status(t, "imu_el", timeout_s=0.1) == "faulted"


def test_stream_status_dead(monkeypatch):
    """No frame within the window -> dead."""
    t = DummyTransport()
    monkeypatch.setattr(t.r, "xread", lambda *a, **k: [])
    assert calibrate_imu.stream_status(t, "imu_el", timeout_s=0.1) == "dead"


def test_stream_alive_delegates_to_status(monkeypatch):
    """stream_alive is True only for a healthy stream."""
    monkeypatch.setattr(
        calibrate_imu, "stream_status", lambda *a, **k: "healthy"
    )
    assert calibrate_imu.stream_alive(DummyTransport(), "imu_az") is True
    monkeypatch.setattr(
        calibrate_imu, "stream_status", lambda *a, **k: "faulted"
    )
    assert calibrate_imu.stream_alive(DummyTransport(), "imu_el") is False


# ---------------------------------------------------------------------------
# collect_el_auto: motor-driven sweep (claim -> stops -> halt/release)
# ---------------------------------------------------------------------------


def test_collect_el_auto_commands_stops_and_halts_on_failure(monkeypatch):
    t = DummyTransport()
    proxy = FakeProxy()
    settled = []

    def fake_settle(transport, axis, target, **kw):
        assert axis == "el"
        settled.append(target)
        return target

    monkeypatch.setattr(calibrate_imu, "wait_for_settle", fake_settle)
    monkeypatch.setattr(
        calibrate_imu,
        "collect_vector",
        lambda tr, name, fields, n, **kw: np.array([0.0, 0.0, -9.81]),
    )
    stops = calibrate_imu.collect_el_auto(
        t, proxy, n_samples=2, n_stops=4, alive={"imu_el"}
    )
    # 5 sweep stops over +/-180 plus the return-to-zero move
    assert settled == [-180.0, -90.0, 0.0, 90.0, 180.0, 0.0]
    assert [a for a, _ in proxy.commands].count("el_target_deg") == 6
    assert proxy.commands[0][0] == "claim"
    assert proxy.commands[-1][0] == "release"
    assert len(stops["motor_el_deg"]) == 5
    assert stops["imu_el"].shape == (5, 3)
    assert stops["imu_az"] is None


def test_collect_el_auto_halts_and_releases_on_settle_timeout(monkeypatch):
    t = DummyTransport()
    proxy = FakeProxy()

    def boom(transport, axis, target, **kw):
        raise TimeoutError("no settle")

    monkeypatch.setattr(calibrate_imu, "wait_for_settle", boom)
    with pytest.raises(TimeoutError):
        calibrate_imu.collect_el_auto(
            t, proxy, n_samples=2, n_stops=4, alive={"imu_el"}
        )
    actions = [a for a, _ in proxy.commands]
    assert "halt" in actions
    assert actions[-1] == "release"


def test_collect_el_manual_reads_motor_and_samples(monkeypatch):
    """Manual mode reads settled motor el from stream:motor at each stop and
    samples each alive IMU; 'q' ends the sweep."""
    t = DummyTransport()
    reads = iter([-90.0, 0.0, 90.0])
    # read_motor_pos_deg's production default (start_id="$") reads only NEW
    # entries; monkeypatch it so the test doesn't race that on fakeredis.
    monkeypatch.setattr(
        calibrate_imu,
        "read_motor_pos_deg",
        lambda tr, axis, **kw: next(reads),
    )
    monkeypatch.setattr(
        calibrate_imu,
        "collect_vector",
        lambda tr, name, fields, n, **kw: np.array([0.0, 0.0, -9.81]),
    )
    answers = iter(["", "", "", "q"])
    monkeypatch.setattr("builtins.input", lambda *a: next(answers))
    sweep = calibrate_imu.collect_el_manual(t, n_samples=2, alive={"imu_el"})
    assert sweep["motor_el_deg"] == [-90.0, 0.0, 90.0]
    assert sweep["imu_el"].shape == (3, 3)
    assert sweep["imu_az"] is None


# ---------------------------------------------------------------------------
# main(): gating, sweep-quality, fit, persist/push
# ---------------------------------------------------------------------------


def test_main_aborts_without_pot_home_unless_el_only(monkeypatch, capsys):
    """Calibrated pot parked off-home -> offer imu_el-only, Enter aborts."""
    t = DummyTransport()
    calibrate_imu.PotCalStore(t).upload({"pot_az": [320.0, -400.0]})
    t.r.xadd(  # pot parked at 60 deg
        "stream:potmon",
        {"value": json.dumps({"status": "update", "pot_az_angle": 60.0})},
    )
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(
        calibrate_imu, "stream_status", lambda *a, **k: "healthy"
    )
    monkeypatch.setattr("builtins.input", lambda *a: "")  # abort
    assert calibrate_imu.main([]) == 1
    assert "home az" in capsys.readouterr().err.lower()


def _synthetic_sweep(alive, scale=9.81, bias=(0.05, -0.02, 0.1)):
    b = np.asarray(bias)
    return {
        "motor_el_deg": list(MOTOR_STOPS),
        "imu_el": (
            _el_sweep_units(MOTOR_STOPS, M_EL_TRUE) * scale + b
            if "imu_el" in alive
            else None
        ),
        "imu_az": (
            _el_sweep_units(MOTOR_STOPS, M_AZ_TRUE) * scale + b
            if "imu_az" in alive
            else None
        ),
    }


def test_main_persists_and_pushes(monkeypatch):
    """Stubbed sweep -> fit -> save 'y' -> ImuCalStore + one live push/IMU."""
    t = DummyTransport()
    PotCalStore(t).upload({"pot_az": [320.0, -400.0]})  # az standard ready

    def fake_collect(transport, motor_proxy, n_samples, n_stops, alive):
        return _synthetic_sweep(alive)

    pushes = []

    class RecordingProxy:
        def __init__(self, *a, **k):
            self.is_available = True

        def send_command(self, action, **kw):
            pushes.append((action, kw))
            return {}

    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(calibrate_imu, "PicoProxy", RecordingProxy)
    monkeypatch.setattr(calibrate_imu, "collect_el_auto", fake_collect)
    monkeypatch.setattr(
        calibrate_imu, "stream_status", lambda *a, **k: "healthy"
    )
    # Pot parked at home so the imu_az section is kept.
    monkeypatch.setattr(calibrate_imu, "_read_pot_az_deg", lambda *a, **k: 0.0)
    monkeypatch.setattr("builtins.input", lambda *a: "y")  # confirm save

    assert calibrate_imu.main([]) == 0
    stored = ImuCalStore(t).get()
    assert "imu_el" in stored and "imu_az" in stored
    assert "derived_home_motor_deg" in stored["metadata"]
    assert stored["metadata"]["mode"] == "auto"
    sc = [kw for act, kw in pushes if act == "set_calibration"]
    assert len(sc) == 2  # one live push per IMU
    pushed = set()
    for kw in sc:
        pushed |= set(kw)
    assert pushed == {"imu_el", "imu_az"}


def test_main_discard_writes_nothing(monkeypatch):
    """Declining the save prompt persists nothing and pushes nothing."""
    t = DummyTransport()
    PotCalStore(t).upload({"pot_az": [320.0, -400.0]})

    def fake_collect(transport, motor_proxy, n_samples, n_stops, alive):
        return _synthetic_sweep(alive)

    pushes = []

    class RecordingProxy:
        def __init__(self, *a, **k):
            self.is_available = True

        def send_command(self, action, **kw):
            pushes.append((action, kw))
            return {}

    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(calibrate_imu, "PicoProxy", RecordingProxy)
    monkeypatch.setattr(calibrate_imu, "collect_el_auto", fake_collect)
    monkeypatch.setattr(
        calibrate_imu, "stream_status", lambda *a, **k: "healthy"
    )
    monkeypatch.setattr(calibrate_imu, "_read_pot_az_deg", lambda *a, **k: 0.0)
    monkeypatch.setattr("builtins.input", lambda *a: "n")  # decline save

    assert calibrate_imu.main([]) == 0  # clean discard
    assert ImuCalStore(t).get() is None  # nothing persisted
    assert not [p for p in pushes if p[0] == "set_calibration"]


def _answers(*seq):
    """Return an input() stand-in that yields the given answers in order."""
    it = iter(seq)
    return lambda *a, **k: next(it)


def test_main_faulted_imu_aborts_by_default(monkeypatch):
    """imu_el publishing only status=error: main names it and aborts (return
    1) when the operator does not opt to continue — never reaching the fit."""
    t = DummyTransport()
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(
        calibrate_imu,
        "stream_status",
        lambda tr, n, **k: "faulted" if n == "imu_el" else "healthy",
    )
    monkeypatch.setattr(
        calibrate_imu,
        "collect_el_manual",
        lambda *a, **k: pytest.fail("must abort before collection"),
    )
    monkeypatch.setattr("builtins.input", _answers(""))  # Enter = abort
    assert calibrate_imu.main(["-m", "manual"]) == 1
    assert ImuCalStore(t).get() is None


def test_main_faulted_imu_continue_skips_it(monkeypatch):
    """Operator opts to continue without the faulted imu_el: main proceeds and
    imu_el is absent from the alive set handed to the collector."""
    t = DummyTransport()
    PotCalStore(t).upload({"pot_az": [320.0, -400.0]})
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(
        calibrate_imu,
        "stream_status",
        lambda tr, n, **k: "faulted" if n == "imu_el" else "healthy",
    )
    monkeypatch.setattr(calibrate_imu, "_read_pot_az_deg", lambda *a, **k: 0.0)
    seen = {}

    def fake_collect(transport, n_samples, alive):
        seen["alive"] = set(alive)
        # Empty sweep -> quality gate returns 1 cleanly.
        return {"motor_el_deg": [], "imu_el": None, "imu_az": None}

    monkeypatch.setattr(calibrate_imu, "collect_el_manual", fake_collect)

    prompts = []

    def spy_input(prompt=""):
        prompts.append(prompt)
        return "y"

    monkeypatch.setattr("builtins.input", spy_input)
    assert calibrate_imu.main(["-m", "manual"]) == 1
    assert "imu_el" not in seen["alive"]
    assert "imu_az" in seen["alive"]
    assert any("imu_el" in p for p in prompts)  # operator was prompted


def test_main_fit_valueerror_is_clean(monkeypatch):
    """A backstop ValueError from the fit surfaces as a clean return 1, not an
    uncaught traceback (and no prompt fires before the fit)."""
    t = DummyTransport()
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(
        calibrate_imu,
        "stream_status",
        lambda tr, n, **k: "healthy" if n == "imu_el" else "dead",
    )
    el_el = _el_sweep_units(MOTOR_STOPS, M_EL_TRUE) * 9.81
    monkeypatch.setattr(
        calibrate_imu,
        "collect_el_manual",
        lambda *a, **k: {
            "motor_el_deg": list(MOTOR_STOPS),
            "imu_el": el_el,
            "imu_az": None,
        },
    )

    def boom(*a, **k):
        raise ValueError("degenerate accel sphere")

    monkeypatch.setattr(calibrate_imu, "fit_el_calibration", boom)
    monkeypatch.setattr(
        "builtins.input",
        lambda *a, **k: pytest.fail("no prompt expected before fit error"),
    )
    assert calibrate_imu.main(["-m", "manual"]) == 1


def test_main_sweep_runtimeerror_is_clean(monkeypatch):
    """A RuntimeError raised during collection (sustained fault mid-sweep)
    must surface as a clean return 1, not an uncaught traceback."""
    t = DummyTransport()
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(
        calibrate_imu,
        "stream_status",
        lambda tr, n, **k: "healthy" if n == "imu_el" else "dead",
    )

    def boom(*a, **k):
        raise RuntimeError(
            "imu_el: 3 consecutive status=error frames (sensor faulted); "
            "collected only 0/10 valid samples."
        )

    monkeypatch.setattr(calibrate_imu, "collect_el_manual", boom)
    assert calibrate_imu.main(["-m", "manual"]) == 1


def test_main_sweep_timeouterror_is_clean(monkeypatch):
    """A TimeoutError from the auto sweep (motor never settled) also surfaces
    as a clean return 1."""
    t = DummyTransport()
    monkeypatch.setattr(calibrate_imu, "Transport", lambda **kw: t)
    monkeypatch.setattr(calibrate_imu, "PicoProxy", FakeProxy)
    monkeypatch.setattr(
        calibrate_imu,
        "stream_status",
        lambda tr, n, **k: "healthy" if n == "imu_el" else "dead",
    )

    def boom(*a, **k):
        raise TimeoutError("motor el did not settle")

    monkeypatch.setattr(calibrate_imu, "collect_el_auto", boom)
    assert calibrate_imu.main([]) == 1
