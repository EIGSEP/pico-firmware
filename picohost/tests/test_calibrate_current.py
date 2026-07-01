"""
Tests for the whole-system current-monitor calibration.

Covers three layers, mirroring the potentiometer calibration suite:
  - ``compute_two_point``: the amps-vs-volts (slope, intercept) from a 0 A
    point and a known-reference-current point.
  - ``CurrentCalStore``: the Redis-backed cal store round-trip / error paths.
  - ``PicoLidar``: cal loaded at __init__, applied in ``_current_fields`` and
    the redis handler, and updated live via ``set_calibration``.
"""

import pytest
from eigsep_redis.testing import DummyTransport

import picohost.calibrate_current as cc
from picohost.buses import CurrentCalStore
from picohost.calibrate_current import (
    _build_multi_cal_data,
    _parse_currents,
    _residual_threshold_a,
    collect_multi_point,
    compute_multi_point,
    compute_two_point,
)
from picohost.keys import CURRENT_CAL_KEY
from picohost.testing import DummyPicoLidar


class TestComputeTwoPoint:
    """The two-point fit folds offset and gain into (slope, intercept)."""

    def test_fit_returns_slope_and_intercept(self):
        # 0 A → 1.0 V, 5 A → 2.0 V ⇒ V0=1.0, slope=0.2 V/A ⇒
        # slope_a = 1/0.2 = 5.0 A/V, intercept_a = -1.0/0.2 = -5.0 A
        cal = compute_two_point(1.0, 2.0, 5.0)
        assert cal == pytest.approx((5.0, -5.0))

    def test_zero_reference_current_rejected(self):
        """A 0 A 'reference' can't define a slope → None (caller aborts)."""
        assert compute_two_point(1.0, 2.0, 0.0) is None

    def test_identical_voltages_rejected(self):
        """No voltage swing between the two points → can't fit gain → None."""
        assert compute_two_point(1.5, 1.5, 5.0) is None


class TestComputeMultiPoint:
    """Least-squares fit converted to the stored (slope, intercept)."""

    def _clean_line(self):
        # V = 0.1175 * I + 1.469  (nominal ACS724 + divider line)
        currents = [0.0, 1.0, 2.0, 5.0, 8.0]
        voltages = [1.469 + 0.1175 * i for i in currents]
        return currents, voltages

    def test_recovers_slope_and_intercept(self):
        currents, voltages = self._clean_line()
        # V=1.469+0.1175*I ⇒ slope_a=1/0.1175≈8.5106, intercept_a=-1.469/0.1175≈-12.5021
        (slope_a, intercept_a), quality = compute_multi_point(
            currents, voltages
        )
        assert slope_a == pytest.approx(8.5106, abs=1e-3)
        assert intercept_a == pytest.approx(-12.5021, abs=1e-3)
        assert quality["r_squared"] == pytest.approx(1.0, abs=1e-9)
        assert quality["residual_rms_v"] == pytest.approx(0.0, abs=1e-9)
        assert quality["residual_rms_a"] == pytest.approx(0.0, abs=1e-9)

    def test_residual_flags_a_bent_dataset(self):
        # Bend the middle point well off the line.
        currents = [0.0, 1.0, 2.0, 5.0, 8.0]
        voltages = [1.469 + 0.1175 * i for i in currents]
        voltages[2] += 0.05  # 50 mV kink
        (slope_a, _intercept), quality = compute_multi_point(
            currents, voltages
        )
        assert quality["residual_rms_v"] > 0.0
        # residual_rms_a = residual_rms_v / slope_VperA = residual_rms_v * slope_a
        assert quality["residual_rms_a"] == pytest.approx(
            quality["residual_rms_v"] * abs(slope_a), rel=1e-9
        )
        assert quality["r_squared"] < 1.0

    def test_single_distinct_current_rejected(self):
        assert compute_multi_point([2.0, 2.0, 2.0], [1.7, 1.7, 1.7]) is None

    def test_zero_voltage_spread_rejected(self):
        assert compute_multi_point([0.0, 1.0, 2.0], [1.5, 1.5, 1.5]) is None

    def test_point_table_zero_residuals_on_clean_line(self, capsys):
        """The per-point QA table reads the cal as (slope_a, intercept_a):
        a clean line prints ~0 residuals. Guards against the old (V0, slope)
        misbinding, which prints garbage residuals for a perfect line."""
        currents, voltages = self._clean_line()
        cal, _quality = compute_multi_point(currents, voltages)
        cc._print_point_table(currents, voltages, cal)
        out = capsys.readouterr().out
        rows = 0
        for line in out.splitlines():
            parts = line.split()
            if len(parts) == 5 and parts[0].isdigit():
                rows += 1
                assert abs(float(parts[3])) < 0.01  # resid mV
                assert abs(float(parts[4])) < 0.01  # resid mA
        assert rows == len(currents)


class TestCurrentCalStore:
    """Round-trip and error-path coverage for the Redis cal store."""

    def _cal_payload(self):
        return {
            "system_current": [1.0, 0.2],
            "metadata": {"i_ref": 5.0, "n_samples": 10},
        }

    def test_empty_store_returns_none(self):
        assert CurrentCalStore(DummyTransport()).get() is None

    def test_upload_get_round_trip_preserves_fields(self):
        store = CurrentCalStore(DummyTransport())
        payload = self._cal_payload()
        store.upload(payload)
        loaded = store.get()
        assert loaded["system_current"] == payload["system_current"]
        assert loaded["metadata"] == payload["metadata"]
        # Transport.upload_dict injects an upload_time field on every write.
        assert "upload_time" in loaded

    def test_corrupt_json_returns_none(self):
        transport = DummyTransport()
        transport.r.set(CURRENT_CAL_KEY, b"not-json-{")
        assert CurrentCalStore(transport).get() is None

    def test_clear_removes_key(self):
        store = CurrentCalStore(DummyTransport())
        store.upload(self._cal_payload())
        assert store.get() is not None
        store.clear()
        assert store.get() is None


class TestPicoLidarCurrentCal:
    """PicoLidar loads, applies, and live-updates the current cal."""

    def test_uncalibrated_returns_none(self):
        """No cal store → no nominal fallback; all three fields are None."""
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            assert not lidar.is_current_calibrated
            assert lidar._current_fields(1.46) == (None, None, None)
        finally:
            lidar.disconnect()

    def test_cal_from_redis_applied_at_init(self):
        transport = DummyTransport()
        store = CurrentCalStore(transport)
        store.upload(
            {"system_current": [1.0, 0.2]}
        )  # slope_a=1.0, intercept_a=0.2
        lidar = DummyPicoLidar("/dev/dummy", current_cal_store=store)
        try:
            assert lidar.is_current_calibrated
            assert lidar._current_cal == (1.0, 0.2)
            # I = slope*V + intercept = 1.0*1.2 + 0.2 = 1.4 A
            assert lidar._current_fields(1.2)[0] == pytest.approx(
                1.4, abs=1e-9
            )
            assert lidar._current_fields(1.0)[0] == pytest.approx(
                1.2, abs=1e-9
            )
        finally:
            lidar.disconnect()

    def test_set_calibration_updates_live(self):
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            lidar.set_calibration(system_current_params=[1.0, 0.2])
            assert lidar.is_current_calibrated
            # 1.0*1.4 + 0.2 = 1.6 A
            assert lidar._current_fields(1.4)[0] == pytest.approx(
                1.6, abs=1e-9
            )
        finally:
            lidar.disconnect()

    def test_handler_uses_calibrated_conversion(self):
        """The system_current publish reflects the loaded cal, not nominal."""
        transport = DummyTransport()
        store = CurrentCalStore(transport)
        store.upload({"system_current": [1.0, 0.2]})
        lidar = DummyPicoLidar("/dev/dummy", current_cal_store=store)
        try:
            published = []
            lidar._base_redis_handler = lambda d: published.append(dict(d))
            lidar._lidar_redis_handler(
                {
                    "sensor_name": "lidar",
                    "status": "update",
                    "distance_m": 1.0,
                    "current_voltage": 1.2,
                }
            )
            current = published[1]
            assert current["sensor_name"] == "system_current"
            # 1.0*1.2 + 0.2 = 1.4 A
            assert current["current_a"] == pytest.approx(1.4, abs=1e-9)
        finally:
            lidar.disconnect()


class TestMultiPointHelpers:
    """Pure CLI/threshold helpers for the multi-point flow."""

    def test_parse_currents_basic(self):
        assert _parse_currents("0,1,2,5,8") == [0.0, 1.0, 2.0, 5.0, 8.0]

    def test_parse_currents_tolerates_spaces_and_trailing_comma(self):
        assert _parse_currents(" 0 , 1.5 , 3 ,") == [0.0, 1.5, 3.0]

    def test_threshold_uses_2_percent_above_floor(self):
        # 2% of 8 A = 0.16 A > 20 mA floor.
        assert _residual_threshold_a([0.0, 1.0, 8.0]) == pytest.approx(0.16)

    def test_threshold_applies_20mA_floor_for_small_currents(self):
        # 2% of 0.5 A = 10 mA < 20 mA floor.
        assert _residual_threshold_a([0.0, 0.5]) == pytest.approx(0.020)


class TestMultiPointCollection:
    """Interactive collection and payload assembly (no hardware/Redis)."""

    def test_preset_walks_the_list(self, monkeypatch):
        # collect_samples returns a voltage per point; feed deterministic values.
        volts = iter([1.47, 1.59, 1.70, 2.06])
        monkeypatch.setattr(cc, "collect_samples", lambda t, n: next(volts))
        # Preset currents; blank Enter accepts each target as the reading.
        inputs = iter(["", "", "", ""])
        monkeypatch.setattr("builtins.input", lambda *a, **k: next(inputs))
        currents, voltages = collect_multi_point(
            transport=None, n_samples=5, currents=[0.0, 1.0, 2.0, 5.0]
        )
        assert currents == [0.0, 1.0, 2.0, 5.0]
        assert voltages == [1.47, 1.59, 1.70, 2.06]

    def test_preset_override_uses_typed_reading(self, monkeypatch):
        monkeypatch.setattr(cc, "collect_samples", lambda t, n: 1.6)
        # Operator overrides the second target with a measured 1.05 A.
        inputs = iter(["", "1.05", ""])
        monkeypatch.setattr("builtins.input", lambda *a, **k: next(inputs))
        currents, _v = collect_multi_point(
            transport=None, n_samples=5, currents=[0.0, 1.0, 2.0]
        )
        assert currents == [0.0, 1.05, 2.0]

    def test_loop_until_blank_requires_three_points(self, monkeypatch):
        monkeypatch.setattr(cc, "collect_samples", lambda t, n: 1.5)
        # Blank too early is rejected; then three points, then blank to finish.
        inputs = iter(["", "0", "1", "2", ""])
        monkeypatch.setattr("builtins.input", lambda *a, **k: next(inputs))
        currents, voltages = collect_multi_point(
            transport=None, n_samples=5, currents=None
        )
        assert currents == [0.0, 1.0, 2.0]
        assert len(voltages) == 3

    def test_build_multi_cal_data_shape(self):
        cal = (1.469, 0.1175)
        quality = {
            "residual_rms_v": 0.001,
            "residual_rms_a": 0.0085,
            "r_squared": 0.999,
        }
        payload = _build_multi_cal_data(
            cal,
            n_samples=10,
            currents=[0.0, 1.0, 2.0],
            voltages=[1.469, 1.587, 1.704],
            quality=quality,
        )
        assert payload["system_current"] == [1.469, 0.1175]
        meta = payload["metadata"]
        assert meta["mode"] == "multi"
        assert meta["n_points"] == 3
        assert meta["currents"] == [0.0, 1.0, 2.0]
        assert meta["voltages"] == [1.469, 1.587, 1.704]
        assert meta["residual_rms_a"] == pytest.approx(0.0085)
        assert meta["r_squared"] == pytest.approx(0.999)
        assert "timestamp" in meta


class _FakeProxy:
    """Stand-in for PicoProxy: always available, send_command is a no-op."""

    is_available = True

    def __init__(self, *args, **kwargs):
        pass

    def send_command(self, *args, **kwargs):
        return None


class _FakeProxyDown(_FakeProxy):
    """PicoProxy stand-in reporting the device unreachable. Pushing to a
    down device is a bug — the live push must be skipped — so send_command
    fails loudly if the guard is ever removed."""

    is_available = False

    def send_command(self, *args, **kwargs):
        raise AssertionError(
            "send_command must not be called when the device is down"
        )


class _FakeTransport:
    """Stand-in for Transport with a no-op ``.r.bgsave()``."""

    def __init__(self, *args, **kwargs):
        self.r = self

    def bgsave(self):
        return None


def _spy_store_factory(recorder):
    """Build a CurrentCalStore stand-in that records uploaded payloads."""

    class _SpyStore:
        def __init__(self, transport):
            pass

        def upload(self, payload):
            recorder.append(payload)

    return _SpyStore


class TestMainDispatch:
    """Drive ``main()`` to lock the storage-gating control flow."""

    def test_bad_fit_prompt_blocks_storage_on_no(self, monkeypatch):
        """Residual over threshold + operator 'n' aborts before upload."""
        uploaded = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxy)
        monkeypatch.setattr(
            cc,
            "collect_multi_point",
            lambda t, n, c: ([0.0, 1.0, 2.0], [1.0, 1.1, 1.2]),
        )
        # Force a residual well above the threshold so the gate fires.
        monkeypatch.setattr(
            cc,
            "compute_multi_point",
            lambda c, v: (
                (1.0, 0.1175),
                {
                    "residual_rms_v": 1.0,
                    "residual_rms_a": 1.0,
                    "r_squared": 0.5,
                },
            ),
        )
        monkeypatch.setattr(
            cc, "CurrentCalStore", _spy_store_factory(uploaded)
        )
        monkeypatch.setattr("builtins.input", lambda *a, **k: "n")
        monkeypatch.setattr(
            "sys.argv", ["calibrate-current", "--mode", "multi"]
        )
        with pytest.raises(SystemExit):
            cc.main()
        assert uploaded == []

    def test_currents_under_three_errors_before_collection(self, monkeypatch):
        """``--currents 0,1`` (2 points) exits before any sample collection."""
        called = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxy)
        monkeypatch.setattr(
            cc,
            "collect_multi_point",
            lambda *a, **k: called.append("collect_multi_point"),
        )
        monkeypatch.setattr(
            cc,
            "collect_samples",
            lambda *a, **k: called.append("collect_samples"),
        )
        monkeypatch.setattr(
            "sys.argv", ["calibrate-current", "--currents", "0,1"]
        )
        with pytest.raises(SystemExit):
            cc.main()
        assert called == []

    def test_two_point_metadata_tags_mode(self, monkeypatch):
        """Default-mode payload carries ``metadata['mode'] == 'two-point'``."""
        uploaded = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxy)
        monkeypatch.setattr(
            cc, "collect_two_point", lambda t, n: (1.0, 2.0, 5.0)
        )
        monkeypatch.setattr(
            cc, "CurrentCalStore", _spy_store_factory(uploaded)
        )
        monkeypatch.setattr("builtins.input", lambda *a, **k: "")
        monkeypatch.setattr("sys.argv", ["calibrate-current"])
        cc.main()
        assert len(uploaded) == 1
        assert uploaded[0]["metadata"]["mode"] == "two-point"

    def test_multi_happy_path_uploads_calibration(self, monkeypatch):
        """A clean multi fit reaches upload with the right payload shape.

        Uses the real ``compute_multi_point`` so the residual/threshold
        path is genuinely exercised; the points sit on a clean line, so the
        residual stays below threshold and no bad-fit prompt fires.
        """
        uploaded = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxy)
        monkeypatch.setattr(
            cc,
            "collect_multi_point",
            lambda t, n, c: (
                [0.0, 1.0, 2.0, 5.0],
                [1.469, 1.587, 1.704, 2.056],
            ),
        )
        monkeypatch.setattr(
            cc, "CurrentCalStore", _spy_store_factory(uploaded)
        )
        monkeypatch.setattr(
            "sys.argv", ["calibrate-current", "--mode", "multi"]
        )
        cc.main()
        assert len(uploaded) == 1
        assert len(uploaded[0]["system_current"]) == 2
        meta = uploaded[0]["metadata"]
        assert meta["mode"] == "multi"
        assert meta["n_points"] == 4

    def test_manual_uploads_slope_intercept_verbatim(self, monkeypatch):
        """--mode manual writes the supplied (slope, intercept) verbatim
        with a 'manual' mode tag and the note."""
        uploaded = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxy)
        monkeypatch.setattr(
            cc, "CurrentCalStore", _spy_store_factory(uploaded)
        )
        monkeypatch.setattr(
            "sys.argv",
            [
                "calibrate-current",
                "--mode",
                "manual",
                "--slope",
                "8.4223",
                "--intercept",
                "-12.5248",
                "--note",
                "restored from corr_20260629.h5",
            ],
        )
        cc.main()
        assert len(uploaded) == 1
        assert uploaded[0]["system_current"] == [8.4223, -12.5248]
        meta = uploaded[0]["metadata"]
        assert meta["mode"] == "manual"
        assert meta["note"] == "restored from corr_20260629.h5"

    def test_manual_writes_even_when_device_down(self, monkeypatch):
        """Recovery path: manual writes Redis and skips the live push when
        the lidar Pico is unreachable."""
        uploaded = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxyDown)
        monkeypatch.setattr(
            cc, "CurrentCalStore", _spy_store_factory(uploaded)
        )
        monkeypatch.setattr(
            "sys.argv",
            [
                "calibrate-current",
                "--mode",
                "manual",
                "--slope",
                "8.4223",
                "--intercept",
                "-12.5248",
            ],
        )
        cc.main()
        assert len(uploaded) == 1
        assert uploaded[0]["system_current"] == [8.4223, -12.5248]

    def test_manual_missing_intercept_exits_without_upload(self, monkeypatch):
        uploaded = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxy)
        monkeypatch.setattr(
            cc, "CurrentCalStore", _spy_store_factory(uploaded)
        )
        monkeypatch.setattr(
            "sys.argv",
            ["calibrate-current", "--mode", "manual", "--slope", "8.4"],
        )
        with pytest.raises(SystemExit):
            cc.main()
        assert uploaded == []

    def test_manual_zero_slope_exits_without_upload(self, monkeypatch):
        uploaded = []
        monkeypatch.setattr(cc, "Transport", _FakeTransport)
        monkeypatch.setattr(cc, "PicoProxy", _FakeProxy)
        monkeypatch.setattr(
            cc, "CurrentCalStore", _spy_store_factory(uploaded)
        )
        monkeypatch.setattr(
            "sys.argv",
            [
                "calibrate-current",
                "--mode",
                "manual",
                "--slope",
                "0",
                "--intercept",
                "1.0",
            ],
        )
        with pytest.raises(SystemExit):
            cc.main()
        assert uploaded == []
