"""Unit tests for picohost.calibrate_pot."""

import json

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
    assert calibrate_pot.fit_slope_pin_zero([1.0, 1.0], [0.0, 100.0], 1.0) is None


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
