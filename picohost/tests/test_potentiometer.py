"""
Unit tests for the PicoPotentiometer class.

Tests use DummyPicoPotentiometer which wires a PotMonEmulator to MockSerial.
"""

import json
import time
import tempfile

import pytest

from picohost.testing import DummyPicoPotentiometer


def _make_pot():
    """Create a DummyPicoPotentiometer and wait for first status."""
    pot = DummyPicoPotentiometer("/dev/dummy")
    # No wait_for_updates — give the emulator time to stream first status
    time.sleep(0.2)
    return pot


class TestPicoPotentiometer:

    def test_status_has_potmon_fields(self):
        """Potentiometer status should contain voltage fields from emulator."""
        pot = _make_pot()
        assert pot.last_status.get("sensor_name") == "potmon"
        assert "pot0_voltage" in pot.last_status
        assert "pot1_voltage" in pot.last_status
        pot.disconnect()

    def test_read_voltage(self):
        """read_voltage() returns dict with both voltage readings."""
        pot = _make_pot()
        volts = pot.read_voltage()
        assert "pot0_voltage" in volts
        assert "pot1_voltage" in volts
        assert 0.0 <= volts["pot0_voltage"] <= 3.3
        assert 0.0 <= volts["pot1_voltage"] <= 3.3
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
            pot0_params=(1000.0, 0.0),
            pot1_params=(1000.0, 0.0),
        )
        angles = pot.read_angle()
        assert "pot0" in angles
        assert "pot1" in angles
        # Emulator base voltage is ~1.5V, so angle should be ~1500
        assert 1000.0 < angles["pot0"] < 2000.0
        assert 1000.0 < angles["pot1"] < 2000.0
        pot.disconnect()

    def test_is_calibrated_property(self):
        """is_calibrated reflects whether both pots have parameters."""
        pot = _make_pot()
        assert pot.is_calibrated is False
        pot.set_calibration(pot0_params=(1.0, 0.0))
        assert pot.is_calibrated is False  # only pot0 set
        pot.set_calibration(pot1_params=(1.0, 0.0))
        assert pot.is_calibrated is True
        pot.disconnect()

    def test_load_calibration_from_file(self):
        """load_calibration() reads (m, b) from a JSON file."""
        pot = _make_pot()
        cal_data = {"pot0": [100.0, -50.0], "pot1": [200.0, -100.0]}
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False
        ) as f:
            json.dump(cal_data, f)
            f.flush()
            pot.load_calibration(f.name)
        assert pot.is_calibrated is True
        assert pot._cal["pot0"] == (100.0, -50.0)
        assert pot._cal["pot1"] == (200.0, -100.0)
        pot.disconnect()

    def test_angle_math_is_linear(self):
        """Verify angle = m * voltage + b."""
        pot = _make_pot()
        m, b = 1000.0, -500.0
        pot.set_calibration(pot0_params=(m, b), pot1_params=(m, b))
        v = pot.last_status["pot0_voltage"]
        angles = pot.read_angle()
        assert angles["pot0"] == pytest.approx(m * v + b, abs=0.01)
        pot.disconnect()
