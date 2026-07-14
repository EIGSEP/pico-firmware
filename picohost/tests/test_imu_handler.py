import numpy as np
import pytest
from eigsep_redis.testing import DummyTransport

from picohost import imu_geometry as ig
from picohost.buses import ImuCalStore
from picohost.testing import DummyPicoIMU

_SCALARS = (str, int, float, bool, type(None))

# An imu_az calibration with identity mount (el-only; azimuth is owned by
# potmon since the 2026-07-09 descope).
_AZ_CAL = {
    "accel_bias": [0.0, 0.0, 0.0],
    "accel_scale": 1.0,
    "M": [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]],
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


def test_imu_az_uncalibrated_publishes_none_el():
    dev = DummyPicoIMU("/dev/dummy")
    pub = _capture(dev, _az_status(30, 70, 100))
    assert pub["el_deg"] is None
    for k in (
        "az_deg",
        "az_from_accel_deg",
        "az_from_yaw_deg",
        "az_blend_weight",
    ):
        assert k not in pub
    # raw fields preserved
    assert pub["yaw"] == 100
    dev.disconnect()


def test_imu_az_calibrated_reports_el_only():
    dev = DummyPicoIMU("/dev/dummy")
    dev.set_calibration(imu_az=_AZ_CAL)
    pub = _capture(dev, _az_status(60.0, 70.0, 100.0))
    assert pub["el_deg"] == pytest.approx(60.0, abs=1e-3)
    for k in (
        "az_deg",
        "az_from_accel_deg",
        "az_from_yaw_deg",
        "az_blend_weight",
    ):
        assert k not in pub
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


def test_imu_az_malformed_cal_still_publishes_raw():
    """A partial/broken cal section must never suppress the raw firmware tick.

    The handler derives el before its single publish; a missing key in the
    cal must degrade el_deg to None, not drop the whole record.
    """
    dev = DummyPicoIMU("/dev/dummy")
    # cal missing "M" -> derivation raises mid-handler
    dev.set_calibration(imu_az={"accel_bias": [0.0, 0.0, 0.0]})
    pub = _capture(dev, _az_status(30, 70, 100))
    assert pub["yaw"] == 100  # raw tick survived
    assert pub["accel_x"] is not None
    assert (
        pub["el_deg"] is None
    )  # derived degraded to None, shape stays stable
    for k in (
        "az_deg",
        "az_from_accel_deg",
        "az_from_yaw_deg",
        "az_blend_weight",
    ):
        assert k not in pub
    dev.disconnect()


def test_imu_el_malformed_cal_still_publishes_raw():
    """imu_el counterpart: a broken cal yields el_deg=None, raw preserved."""
    dev = DummyPicoIMU("/dev/dummy", name="imu_el")
    dev.set_calibration(imu_el={"accel_bias": [0.0, 0.0, 0.0]})  # missing M
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
    assert pub["accel_y"] is not None  # raw survived
    assert pub["el_deg"] is None
    dev.disconnect()


def test_emulator_to_handler_roundtrip_identity_mount():
    """Forward-model status dict at a known pose; handler recovers el
    with a cal whose mount matches the forward model's (identity)."""
    dev = DummyPicoIMU(
        "/dev/dummy"
    )  # handler keys off data["sensor_name"], not the device name
    dev.set_calibration(imu_az=_AZ_CAL)
    # craft an imu_az status straight from the forward model at el=35 (phi is
    # irrelevant to el_abs_from_imu_az -- rotation about the az spin axis)
    pub = _capture(dev, _az_status(35.0, 80.0, 0.0))
    assert pub["el_deg"] == pytest.approx(35.0, abs=1e-3)
    dev.disconnect()


def _imu_standby_status(app_id):
    """Raw firmware/emulator standby tick: status='error', standby=true,
    and the sensor data fields omitted (imu.c imu_status standby branch)."""
    return {
        "sensor_name": "imu_el" if app_id == 3 else "imu_az",
        "status": "error",
        "app_id": app_id,
        "standby": True,
    }


@pytest.mark.parametrize("app_id", [3, 6])
def test_imu_standby_publishes_full_shape_with_none_data(app_id):
    """A standby tick must publish the full field set (data None) plus
    standby=True, so the consumer contract sees no missing/extra keys."""
    dev = DummyPicoIMU("/dev/dummy")
    pub = _capture(dev, _imu_standby_status(app_id))
    assert pub["standby"] is True
    assert pub["status"] == "error"
    for f in ("yaw", "pitch", "roll", "accel_x", "accel_y", "accel_z"):
        assert f in pub and pub[f] is None
    assert pub["el_deg"] is None
    dev.disconnect()


def test_imu_normal_tick_carries_standby_false():
    """Normal ticks gain standby=False so the published shape is stable
    across normal/standby (the consumer schema requires the key)."""
    dev = DummyPicoIMU("/dev/dummy")
    pub = _capture(dev, _az_status(30, 70, 100))
    assert pub["standby"] is False
    dev.disconnect()
