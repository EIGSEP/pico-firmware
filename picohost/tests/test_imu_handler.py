import numpy as np
import pytest
from eigsep_redis.testing import DummyTransport

from picohost import imu_geometry as ig
from picohost.buses import ImuCalStore
from picohost.testing import DummyPicoIMU

_SCALARS = (str, int, float, bool, type(None))

# An imu_az calibration with identity mount and a +30 deg pot offset on both
# az channels, so az = phi + 30 (accel) and az = yaw + 30 (yaw).
_AZ_CAL = {
    "accel_bias": [0.0, 0.0, 0.0],
    "accel_scale": 1.0,
    "M": [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]],
    "az_accel_offset_deg": 30.0,
    "az_sign": 1.0,
    "az_yaw_offset_deg": 30.0,
    "az_yaw_sign": 1.0,
    "theta_cross_deg": 1.6,
}
_EL_CAL = {
    "accel_bias": [0.0, 0.0, 0.0],
    "accel_scale": 1.0,
    "M": [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]],
}


def _accel_az(theta_deg, phi_deg):
    th, ph = np.radians(theta_deg), np.radians(phi_deg)
    return ig.R_z(ph).T @ np.array([0.0, np.sin(th), np.cos(th)])


def _capture(dev, data):
    captured = {}
    dev._base_redis_handler = lambda d: captured.update(d)
    dev._imu_redis_handler(data)
    return captured


def _az_status(theta_deg, phi_deg, yaw_deg):
    ax, ay, az = _accel_az(theta_deg, phi_deg)
    return {
        "sensor_name": "imu_az",
        "status": "update",
        "app_id": 6,
        "yaw": yaw_deg,
        "pitch": 0.0,
        "roll": 0.0,
        "accel_x": float(ax),
        "accel_y": float(ay),
        "accel_z": float(az),
    }


def test_imu_az_uncalibrated_publishes_none_fields():
    dev = DummyPicoIMU("/dev/dummy")
    pub = _capture(dev, _az_status(30, 70, 100))
    for k in (
        "az_deg",
        "el_deg",
        "az_from_accel_deg",
        "az_from_yaw_deg",
        "az_blend_weight",
    ):
        assert pub[k] is None
    # raw fields preserved
    assert pub["yaw"] == 100
    dev.disconnect()


def test_imu_az_calibrated_reports_az_and_el():
    dev = DummyPicoIMU("/dev/dummy")
    dev.set_calibration(imu_az=_AZ_CAL)
    pub = _capture(dev, _az_status(40.0, 70.0, 100.0))
    assert pub["el_deg"] == pytest.approx(40.0, abs=1e-3)
    # tilted -> accel regime -> az = phi + 30 = 100
    assert pub["az_from_accel_deg"] == pytest.approx(100.0, abs=1e-3)
    assert pub["az_from_yaw_deg"] == pytest.approx(130.0, abs=1e-3)
    assert pub["az_blend_weight"] == pytest.approx(1.0)
    assert pub["az_deg"] == pytest.approx(100.0, abs=1e-3)
    dev.disconnect()


def test_imu_az_near_level_uses_yaw():
    dev = DummyPicoIMU("/dev/dummy")
    dev.set_calibration(imu_az=_AZ_CAL)
    pub = _capture(dev, _az_status(0.0, 0.0, 100.0))
    assert pub["az_blend_weight"] == pytest.approx(0.0)
    assert pub["az_deg"] == pytest.approx(130.0, abs=1e-3)  # yaw + 30
    dev.disconnect()


def test_imu_el_reports_el_only_no_az_fields():
    dev = DummyPicoIMU("/dev/dummy", name="imu_el")
    dev.set_calibration(imu_el=_EL_CAL)
    th = np.radians(25.0)
    data = {
        "sensor_name": "imu_el",
        "status": "update",
        "app_id": 3,
        "yaw": 0.0,
        "pitch": 0.0,
        "roll": 0.0,
        "accel_x": 0.0,
        "accel_y": float(np.sin(th)),
        "accel_z": float(np.cos(th)),
    }
    pub = _capture(dev, data)
    assert pub["el_deg"] == pytest.approx(25.0, abs=1e-3)
    assert "az_deg" not in pub  # imu_el structurally never reports az
    dev.disconnect()


def test_published_dict_is_scalar_only_both_states():
    dev = DummyPicoIMU("/dev/dummy")
    for cal in (None, _AZ_CAL):
        if cal is not None:
            dev.set_calibration(imu_az=cal)
        pub = _capture(dev, _az_status(30, 70, 100))
        for k, v in pub.items():
            assert isinstance(v, _SCALARS), f"{k!r} is {type(v).__name__}"
    dev.disconnect()


def test_imu_az_field_set_stable_across_cal_state():
    dev = DummyPicoIMU("/dev/dummy")
    before = set(_capture(dev, _az_status(30, 70, 100)))
    dev.set_calibration(imu_az=_AZ_CAL)
    after = set(_capture(dev, _az_status(30, 70, 100)))
    assert before == after
    dev.disconnect()


def test_imu_cal_store_loads_at_init():
    """ImuCalStore pre-populated before construction is applied at init time."""
    transport = DummyTransport()
    store = ImuCalStore(transport)
    store.upload({"imu_az": _AZ_CAL})
    dev = DummyPicoIMU("/dev/dummy", imu_cal_store=store)
    try:
        assert dev._imu_cal.get("imu_az") == _AZ_CAL
    finally:
        dev.disconnect()


def test_imu_el_uncalibrated_publishes_none_el_no_az_keys():
    """Uncalibrated imu_el: el_deg is None and no az keys are emitted."""
    dev = DummyPicoIMU("/dev/dummy", name="imu_el")
    th = np.radians(25.0)
    data = {
        "sensor_name": "imu_el",
        "status": "update",
        "app_id": 3,
        "yaw": 0.0,
        "pitch": 0.0,
        "roll": 0.0,
        "accel_x": 0.0,
        "accel_y": float(np.sin(th)),
        "accel_z": float(np.cos(th)),
    }
    pub = _capture(dev, data)
    assert pub["el_deg"] is None
    assert "az_deg" not in pub
    dev.disconnect()


def test_imu_el_calibrated_published_dict_is_scalar_only():
    """Calibrated imu_el: every published value is a scalar (no lists/dicts)."""
    dev = DummyPicoIMU("/dev/dummy", name="imu_el")
    dev.set_calibration(imu_el=_EL_CAL)
    th = np.radians(25.0)
    data = {
        "sensor_name": "imu_el",
        "status": "update",
        "app_id": 3,
        "yaw": 0.0,
        "pitch": 0.0,
        "roll": 0.0,
        "accel_x": 0.0,
        "accel_y": float(np.sin(th)),
        "accel_z": float(np.cos(th)),
    }
    pub = _capture(dev, data)
    for k, v in pub.items():
        assert isinstance(v, _SCALARS), f"{k!r} is {type(v).__name__}"
    dev.disconnect()


def test_emulator_to_handler_roundtrip_identity_mount():
    """Forward-model status dict at a known pose; handler recovers az/el
    with a cal whose mount matches the forward model's (identity)."""
    dev = DummyPicoIMU(
        "/dev/dummy"
    )  # handler keys off data["sensor_name"], not the device name
    dev.set_calibration(imu_az=_AZ_CAL)
    # craft an imu_az status straight from the forward model at (el=35, az=80)
    pub = _capture(dev, _az_status(35.0, 80.0 - 30.0, 80.0 - 30.0))
    # az_accel_offset is +30 so phi=50 -> az 80; yaw 50 -> az 80
    assert pub["el_deg"] == pytest.approx(35.0, abs=1e-3)
    assert pub["az_deg"] == pytest.approx(80.0, abs=1e-3)
    dev.disconnect()
