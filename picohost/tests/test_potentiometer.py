"""
Unit tests for the PicoPotentiometer class.

Tests use DummyPicoPotentiometer which wires a PotMonEmulator to MockSerial.
"""

import json
import tempfile

import pytest
from eigsep_redis.testing import DummyTransport

from conftest import wait_for_condition
from picohost.buses import PotCalStore
from picohost.keys import POT_CAL_KEY
from picohost.testing import DummyPicoPotentiometer


def _make_pot():
    """Create a DummyPicoPotentiometer and wait for first status."""
    pot = DummyPicoPotentiometer("/dev/dummy")
    wait_for_condition(lambda: len(pot.last_status) > 0)
    return pot


class TestPicoPotentiometer:
    def test_status_has_potmon_fields(self):
        """Potentiometer status should contain voltage fields from emulator."""
        pot = _make_pot()
        assert pot.last_status.get("sensor_name") == "potmon"
        assert "pot_az_voltage" in pot.last_status
        pot.disconnect()

    def test_read_voltage(self):
        """read_voltage() returns dict with az voltage reading."""
        pot = _make_pot()
        volts = pot.read_voltage()
        assert "pot_az_voltage" in volts
        assert 0.0 <= volts["pot_az_voltage"] <= 3.3
        pot.disconnect()

    def test_read_angle_without_calibration_raises(self):
        """read_angle() raises RuntimeError when uncalibrated."""
        pot = _make_pot()
        with pytest.raises(RuntimeError, match="No calibration"):
            pot.read_angle()
        pot.disconnect()

    def test_set_calibration_and_read_angle(self):
        """After set_calibration(), read_angle() returns computed angle."""
        pot = _make_pot()
        pot.set_calibration(pot_az_params=(1000.0, 0.0))
        angles = pot.read_angle()
        assert "pot_az" in angles
        # Emulator base voltage is ~1.5V, so angle should be ~1500
        assert 1000.0 < angles["pot_az"] < 2000.0
        pot.disconnect()

    def test_is_calibrated_property(self):
        """is_calibrated reflects whether the az pot has parameters."""
        pot = _make_pot()
        assert pot.is_calibrated is False
        pot.set_calibration(pot_az_params=(1.0, 0.0))
        assert pot.is_calibrated is True
        pot.disconnect()

    def test_load_calibration_from_file(self):
        """load_calibration() reads (m, b) from a JSON file."""
        pot = _make_pot()
        cal_data = {"pot_az": [200.0, -100.0]}
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False
        ) as f:
            json.dump(cal_data, f)
            f.flush()
            pot.load_calibration(f.name)
        assert pot.is_calibrated is True
        assert pot._cal["pot_az"] == (200.0, -100.0)
        pot.disconnect()

    def test_angle_math_is_linear(self):
        """Verify angle = m * voltage + b."""
        pot = _make_pot()
        m, b = 1000.0, -500.0
        pot.set_calibration(pot_az_params=(m, b))
        v = pot.last_status["pot_az_voltage"]
        angles = pot.read_angle()
        assert angles["pot_az"] == pytest.approx(m * v + b, abs=0.01)
        pot.disconnect()


class TestPotRedisHandler:
    """Verify that _pot_redis_handler publishes a flat, scalar-only dict.

    The published payload (what reaches the Redis consumer) must conform
    to the scalar-only contract documented on
    :func:`picohost.base.redis_handler`: every field is ``str``, ``int``,
    ``float``, ``bool``, or ``None``. Calibration parameters are flattened
    into per-component slope/intercept scalars rather than emitted as
    ``[m, b]`` lists, so downstream schemas can validate them per-field
    and HDF5 attribute storage works without special-casing.
    """

    _SCALAR_TYPES = (str, int, float, bool, type(None))

    def _capture(self, pot):
        """Run the redis handler against the latest emulator status and
        return the dict that the base handler would receive."""
        captured = {}
        pot._base_redis_handler = lambda d: captured.update(d)
        pot._pot_redis_handler(dict(pot.last_status))
        return captured

    def test_uncalibrated_publishes_none_cal_and_angle(self):
        """Uncalibrated pot publishes voltage + None for cal/angle."""
        pot = _make_pot()
        published = self._capture(pot)
        assert isinstance(published["pot_az_voltage"], float)
        assert published["pot_az_cal_slope"] is None
        assert published["pot_az_cal_intercept"] is None
        assert published["pot_az_angle"] is None
        pot.disconnect()

    def test_calibrated_publishes_scalar_slope_intercept_angle(self):
        """After set_calibration, slope/intercept/angle are floats."""
        pot = _make_pot()
        pot.set_calibration(pot_az_params=(200.0, -100.0))
        published = self._capture(pot)
        assert published["pot_az_cal_slope"] == 200.0
        assert published["pot_az_cal_intercept"] == -100.0
        assert isinstance(published["pot_az_cal_slope"], float)
        assert isinstance(published["pot_az_cal_intercept"], float)
        assert isinstance(published["pot_az_angle"], float)
        v_az = published["pot_az_voltage"]
        assert published["pot_az_angle"] == pytest.approx(
            200.0 * v_az - 100.0, abs=1e-6
        )
        pot.disconnect()

    def test_published_dict_is_scalar_only(self):
        """Every value in the published dict is a permitted scalar type.

        This is the structural enforcement of the scalar-only contract:
        no lists, tuples, dicts, or numpy arrays may sneak through.
        Both calibrated and uncalibrated states are checked.
        """
        pot = _make_pot()
        for cal in (None, (200.0, -100.0)):
            if cal is not None:
                pot.set_calibration(pot_az_params=cal)
            published = self._capture(pot)
            for k, v in published.items():
                assert isinstance(v, self._SCALAR_TYPES), (
                    f"field {k!r} has non-scalar type {type(v).__name__}"
                )
        pot.disconnect()

    def test_published_shape_stable_across_calibration_state(self):
        """Field set is identical whether calibrated or not."""
        pot = _make_pot()
        before = set(self._capture(pot))
        pot.set_calibration(pot_az_params=(1.0, 0.0))
        after = set(self._capture(pot))
        assert before == after
        # And specifically the new flat field names are present
        expected_added = {
            "pot_az_cal_slope",
            "pot_az_cal_intercept",
            "pot_az_angle",
            "pot_az_near_rail",
        }
        assert expected_added.issubset(before)
        pot.disconnect()

    def test_near_rail_flag_false_mid_range(self):
        """The emulator's ~1.5 V baseline is far from both rails."""
        pot = _make_pot()
        published = self._capture(pot)
        assert published["pot_az_near_rail"] is False
        pot.disconnect()

    def test_near_rail_flag_true_near_either_rail(self):
        """A wiper within the margin of 0 V or vref publishes True.

        A railed pot reports a steady, plausible voltage, so this flag
        is the only stream-level tell that the absolute azimuth
        reference is compromised (e.g. accumulated motor slip during a
        multi-day scan)."""
        pot = _make_pot()
        captured = {}
        pot._base_redis_handler = lambda d: captured.update(d)
        for v in (0.05, 3.25):
            captured.clear()
            pot._pot_redis_handler({"pot_az_voltage": v})
            assert captured["pot_az_near_rail"] is True, f"{v} V"
        # Missing voltage keeps the stable-shape contract: field present,
        # value None.
        captured.clear()
        pot._pot_redis_handler({})
        assert captured["pot_az_near_rail"] is None
        pot.disconnect()


class TestPotCalStore:
    """Round-trip and error-path coverage for the Redis cal store."""

    def _cal_payload(self):
        return {
            "pot_az": [200.0, -100.0],
            "metadata": {"port": "/dev/ttyACM0", "turns": 10},
        }

    def test_empty_store_returns_none(self):
        store = PotCalStore(DummyTransport())
        assert store.get() is None

    def test_upload_get_round_trip_preserves_fields(self):
        """upload()/get() round-trips all fields including metadata."""
        store = PotCalStore(DummyTransport())
        payload = self._cal_payload()
        store.upload(payload)
        loaded = store.get()
        assert loaded["pot_az"] == payload["pot_az"]
        assert loaded["metadata"] == payload["metadata"]
        # Transport.upload_dict injects an upload_time field on every write.
        assert "upload_time" in loaded

    def test_corrupt_json_returns_none(self):
        """Garbage in Redis → get() returns None so callers fall back."""
        transport = DummyTransport()
        transport.r.set(POT_CAL_KEY, b"not-json-{")
        assert PotCalStore(transport).get() is None

    def test_clear_removes_key(self):
        store = PotCalStore(DummyTransport())
        store.upload(self._cal_payload())
        assert store.get() is not None
        store.clear()
        assert store.get() is None


class TestPicoPotentiometerCalSource:
    """Precedence: Redis wins, JSON fallback, uncalibrated if neither.

    Validates the init-time source selection added for the Redis-as-
    canonical-cal-store migration. Each test asserts on ``_cal`` directly
    because the emulator sends voltages that the scalar-only contract
    tests already cover; here we only care where the (m, b) pair came
    from.
    """

    def _redis_cal(self):
        return {
            "pot_az": [200.0, -100.0],
        }

    def _json_cal_file(self, tmp_path, cal):
        path = tmp_path / "pot_cal.json"
        with open(path, "w") as f:
            json.dump(cal, f)
        return str(path)

    def test_redis_wins_over_json(self, tmp_path):
        """Both sources present → Redis cal is applied, JSON is ignored."""
        transport = DummyTransport()
        store = PotCalStore(transport)
        store.upload(self._redis_cal())
        file_cal = {"pot_az": [3.0, 4.0]}
        pot = DummyPicoPotentiometer(
            "/dev/dummy",
            calibration_file=self._json_cal_file(tmp_path, file_cal),
            pot_cal_store=store,
        )
        try:
            assert pot.is_calibrated
            assert pot._cal["pot_az"] == (200.0, -100.0)
        finally:
            pot.disconnect()

    def test_json_fallback_when_redis_empty(self, tmp_path):
        """Redis miss + JSON present → JSON cal is applied."""
        store = PotCalStore(DummyTransport())
        file_cal = {"pot_az": [3.0, 4.0]}
        pot = DummyPicoPotentiometer(
            "/dev/dummy",
            calibration_file=self._json_cal_file(tmp_path, file_cal),
            pot_cal_store=store,
        )
        try:
            assert pot.is_calibrated
            assert pot._cal["pot_az"] == (3.0, 4.0)
        finally:
            pot.disconnect()

    def test_uncalibrated_when_both_missing(self):
        """No store and no file → pot is uncalibrated (current behavior)."""
        pot = DummyPicoPotentiometer("/dev/dummy")
        try:
            assert pot.is_calibrated is False
            assert pot._cal == {"pot_az": None}
        finally:
            pot.disconnect()

    def test_corrupt_redis_falls_back_to_json(self, tmp_path):
        """Garbage in Redis → PotCalStore.get returns None → JSON wins."""
        transport = DummyTransport()
        transport.r.set(POT_CAL_KEY, b"not-json-{")
        file_cal = {"pot_az": [3.0, 4.0]}
        pot = DummyPicoPotentiometer(
            "/dev/dummy",
            calibration_file=self._json_cal_file(tmp_path, file_cal),
            pot_cal_store=PotCalStore(transport),
        )
        try:
            assert pot.is_calibrated
            assert pot._cal["pot_az"] == (3.0, 4.0)
        finally:
            pot.disconnect()


class TestSp1Termination:
    """SP1 failsafe termination: command method + handler name field."""

    def test_set_sp1_termination_drives_emulator(self):
        pot = _make_pot()
        pot.set_sp1_termination("OPEN")
        wait_for_condition(lambda: pot.last_status.get("sp1_term") == 1)
        pot.set_sp1_termination("SHORT")
        wait_for_condition(lambda: pot.last_status.get("sp1_term") == 0)
        pot.disconnect()

    def test_set_sp1_termination_invalid_raises(self):
        pot = _make_pot()
        with pytest.raises(ValueError, match="Invalid SP1 termination"):
            pot.set_sp1_termination("open")  # case-sensitive
        with pytest.raises(ValueError, match="Invalid SP1 termination"):
            pot.set_sp1_termination(1)
        pot.disconnect()

    def test_handler_adds_sp1_term_name(self):
        from picohost.base import PicoPotentiometer

        pot = PicoPotentiometer.__new__(PicoPotentiometer)
        pot._cal = {"pot_az": None}
        captured = {}
        pot._base_redis_handler = lambda d: captured.update(d)
        pot._pot_redis_handler(
            {"sensor_name": "potmon", "status": "update", "sp1_term": 1}
        )
        # Additive: raw int kept, name added.
        assert captured["sp1_term"] == 1
        assert captured["sp1_term_name"] == "OPEN"

    def test_handler_sp1_term_name_none_when_missing(self):
        from picohost.base import PicoPotentiometer

        pot = PicoPotentiometer.__new__(PicoPotentiometer)
        pot._cal = {"pot_az": None}
        captured = {}
        pot._base_redis_handler = lambda d: captured.update(d)
        pot._pot_redis_handler({"sensor_name": "potmon", "status": "update"})
        assert captured["sp1_term_name"] is None
