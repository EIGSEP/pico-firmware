"""Tests for the simple_therm thermistor logger (emulator + host class)."""

import pytest

from conftest import wait_for_condition
from picohost.emulators import SimpleThermEmulator
from picohost.emulators.simple_therm import (
    NUM_CHANNELS,
    SIMPLE_THERM_PULLUP_OHMS,
    THERMISTOR_SUPPLY_VOLTS,
    _thermistor_resistance,
    _thermistor_temp_c,
    _thermistor_voltage,
)
from picohost.testing import DummyPicoSimpleTherm


@pytest.mark.parametrize("temp_c", [-40.0, 0.0, 25.0, 60.0, 125.0])
def test_conversion_round_trip(temp_c):
    v = _thermistor_voltage(_thermistor_resistance(temp_c))
    r = SIMPLE_THERM_PULLUP_OHMS * v / (THERMISTOR_SUPPLY_VOLTS - v)
    assert _thermistor_temp_c(r) == pytest.approx(temp_c, abs=1e-6)


@pytest.mark.parametrize(
    "temp_c, r_ohms",
    # Vishay NTCLE100E3103 datasheet R-T table (document 29049).
    [(0.0, 32554.0), (25.0, 10000.0), (50.0, 3605.0), (100.0, 680.0)],
)
def test_matches_vishay_table(temp_c, r_ohms):
    assert _thermistor_resistance(temp_c) == pytest.approx(r_ohms, rel=0.01)


def test_status_fields():
    emu = SimpleThermEmulator()
    status = emu.get_status()
    assert status["sensor_name"] == "simple_therm"
    assert status["app_id"] == 7
    for i in range(NUM_CHANNELS):
        assert status[f"therm{i}_T"] == pytest.approx(22.0, abs=0.5)
        assert 0 < status[f"therm{i}_voltage"] < THERMISTOR_SUPPLY_VOLTS
        assert status[f"therm{i}_resistance"] > 0
    assert "die_T" in status


def test_open_channel_reports_none():
    emu = SimpleThermEmulator()
    emu.base_temps_c[1] = None
    emu.op()
    status = emu.get_status()
    assert status["therm1_T"] is None
    assert status["therm1_resistance"] is None
    assert status["therm0_T"] is not None


def test_dummy_device_reads_temperatures():
    dev = DummyPicoSimpleTherm("/dev/dummy")
    try:
        wait_for_condition(
            lambda: dev.last_status.get("sensor_name") == "simple_therm"
        )
        temps = dev.read_temperatures()
        assert len(temps) == NUM_CHANNELS
        assert all(t == pytest.approx(22.0, abs=0.5) for t in temps)
    finally:
        dev.disconnect()
