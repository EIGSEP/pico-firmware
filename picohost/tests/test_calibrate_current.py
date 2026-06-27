"""
Tests for the whole-system current-monitor calibration.

Covers three layers, mirroring the potentiometer calibration suite:
  - ``compute_two_point``: the (V0, slope) fit from a 0 A point and a
    known-reference-current point.
  - ``CurrentCalStore``: the Redis-backed cal store round-trip / error paths.
  - ``PicoLidar``: cal loaded at __init__, applied in ``_v_to_current`` and
    the redis handler, and updated live via ``set_calibration``.
"""

import pytest
from eigsep_redis.testing import DummyTransport

from picohost.buses import CurrentCalStore
from picohost.calibrate_current import compute_two_point
from picohost.keys import CURRENT_CAL_KEY
from picohost.testing import DummyPicoLidar


class TestComputeTwoPoint:
    """The two-point fit folds offset and gain into (V0, slope)."""

    def test_fit_returns_v0_and_slope(self):
        # 0 A → 1.0 V, 5 A → 2.0 V  ⇒  V0 = 1.0, slope = (2-1)/5 = 0.2 V/A
        cal = compute_two_point(1.0, 2.0, 5.0)
        assert cal == pytest.approx((1.0, 0.2))

    def test_zero_reference_current_rejected(self):
        """A 0 A 'reference' can't define a slope → None (caller aborts)."""
        assert compute_two_point(1.0, 2.0, 0.0) is None

    def test_identical_voltages_rejected(self):
        """No voltage swing between the two points → can't fit gain → None."""
        assert compute_two_point(1.5, 1.5, 5.0) is None


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

    def test_uncalibrated_uses_nominal_conversion(self):
        """No cal store → nominal ACS724 + divider transfer function."""
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            assert not lidar.is_current_calibrated
            # V_adc at 0 A is Vq*k = 2.5 * 0.5875 = 1.46875
            assert lidar._v_to_current(1.46875) == pytest.approx(0.0, abs=1e-6)
        finally:
            lidar.disconnect()

    def test_cal_from_redis_applied_at_init(self):
        transport = DummyTransport()
        store = CurrentCalStore(transport)
        store.upload({"system_current": [1.0, 0.2]})
        lidar = DummyPicoLidar("/dev/dummy", current_cal_store=store)
        try:
            assert lidar.is_current_calibrated
            assert lidar._current_cal == (1.0, 0.2)
            # I = (V_adc - V0) / slope = (1.2 - 1.0)/0.2 = 1.0 A
            assert lidar._v_to_current(1.2) == pytest.approx(1.0, abs=1e-9)
            assert lidar._v_to_current(1.0) == pytest.approx(0.0, abs=1e-9)
        finally:
            lidar.disconnect()

    def test_set_calibration_updates_live(self):
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            lidar.set_calibration(system_current_params=[1.0, 0.2])
            assert lidar.is_current_calibrated
            assert lidar._v_to_current(1.4) == pytest.approx(2.0, abs=1e-9)
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
            assert current["current_a"] == pytest.approx(1.0, abs=1e-9)
        finally:
            lidar.disconnect()
