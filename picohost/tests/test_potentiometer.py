"""
Unit tests for the PicoPotentiometer class.

Tests use DummyPicoPotentiometer which wires a PotMonEmulator to MockSerial.
"""

import json
import tempfile

import pytest

from conftest import wait_for_condition
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
        assert "pot_el_voltage" in pot.last_status
        assert "pot_az_voltage" in pot.last_status
        pot.disconnect()

    def test_read_voltage(self):
        """read_voltage() returns dict with both voltage readings."""
        pot = _make_pot()
        volts = pot.read_voltage()
        assert "pot_el_voltage" in volts
        assert "pot_az_voltage" in volts
        assert 0.0 <= volts["pot_el_voltage"] <= 3.3
        assert 0.0 <= volts["pot_az_voltage"] <= 3.3
        pot.disconnect()

    def test_read_angle_without_calibration_raises(self):
        """read_angle() raises RuntimeError when uncalibrated."""
        pot = _make_pot()
        with pytest.raises(RuntimeError, match="No calibration"):
            pot.read_angle()
        pot.disconnect()

    def test_set_calibration_and_read_angle(self):
        """After set_calibration(), read_angle() returns computed angles."""
        pot = _make_pot()
        pot.set_calibration(
            pot_el_params=(1000.0, 0.0),
            pot_az_params=(1000.0, 0.0),
        )
        angles = pot.read_angle()
        assert "pot_el" in angles
        assert "pot_az" in angles
        # Emulator base voltage is ~1.5V, so angle should be ~1500
        assert 1000.0 < angles["pot_el"] < 2000.0
        assert 1000.0 < angles["pot_az"] < 2000.0
        pot.disconnect()

    def test_is_calibrated_property(self):
        """is_calibrated reflects whether both pots have parameters."""
        pot = _make_pot()
        assert pot.is_calibrated is False
        pot.set_calibration(pot_el_params=(1.0, 0.0))
        assert pot.is_calibrated is False  # only pot_el set
        pot.set_calibration(pot_az_params=(1.0, 0.0))
        assert pot.is_calibrated is True
        pot.disconnect()

    def test_load_calibration_from_file(self):
        """load_calibration() reads (m, b) from a JSON file."""
        pot = _make_pot()
        cal_data = {"pot_el": [100.0, -50.0], "pot_az": [200.0, -100.0]}
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False
        ) as f:
            json.dump(cal_data, f)
            f.flush()
            pot.load_calibration(f.name)
        assert pot.is_calibrated is True
        assert pot._cal["pot_el"] == (100.0, -50.0)
        assert pot._cal["pot_az"] == (200.0, -100.0)
        pot.disconnect()

    def test_angle_math_is_linear(self):
        """Verify angle = m * voltage + b."""
        pot = _make_pot()
        m, b = 1000.0, -500.0
        pot.set_calibration(pot_el_params=(m, b), pot_az_params=(m, b))
        v = pot.last_status["pot_el_voltage"]
        angles = pot.read_angle()
        assert angles["pot_el"] == pytest.approx(m * v + b, abs=0.01)
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
        for key in ("pot_el", "pot_az"):
            assert isinstance(published[f"{key}_voltage"], float)
            assert published[f"{key}_cal_slope"] is None
            assert published[f"{key}_cal_intercept"] is None
            assert published[f"{key}_angle"] is None
        pot.disconnect()

    def test_calibrated_publishes_scalar_slope_intercept_angle(self):
        """After set_calibration, slope/intercept/angle are floats."""
        pot = _make_pot()
        pot.set_calibration(
            pot_el_params=(100.0, -50.0),
            pot_az_params=(200.0, -100.0),
        )
        published = self._capture(pot)
        assert published["pot_el_cal_slope"] == 100.0
        assert published["pot_el_cal_intercept"] == -50.0
        assert published["pot_az_cal_slope"] == 200.0
        assert published["pot_az_cal_intercept"] == -100.0
        for key in ("pot_el", "pot_az"):
            assert isinstance(published[f"{key}_cal_slope"], float)
            assert isinstance(published[f"{key}_cal_intercept"], float)
            assert isinstance(published[f"{key}_angle"], float)
        # angle = m * v + b for each pot
        v_el = published["pot_el_voltage"]
        v_az = published["pot_az_voltage"]
        assert published["pot_el_angle"] == pytest.approx(
            100.0 * v_el - 50.0, abs=1e-6
        )
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
        for cal in (None, ((100.0, -50.0), (200.0, -100.0))):
            if cal is not None:
                pot.set_calibration(
                    pot_el_params=cal[0], pot_az_params=cal[1]
                )
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
        pot.set_calibration(
            pot_el_params=(1.0, 0.0), pot_az_params=(1.0, 0.0)
        )
        after = set(self._capture(pot))
        assert before == after
        # And specifically the new flat field names are present
        expected_added = {
            "pot_el_cal_slope", "pot_el_cal_intercept", "pot_el_angle",
            "pot_az_cal_slope", "pot_az_cal_intercept", "pot_az_angle",
        }
        assert expected_added.issubset(before)
        pot.disconnect()
