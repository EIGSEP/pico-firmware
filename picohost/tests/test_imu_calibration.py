from eigsep_redis.testing import DummyTransport

from picohost.buses import ImuCalStore
from picohost.keys import IMU_CAL_KEY


def _payload():
    return {
        "imu_az": {
            "accel_bias": [0.1, 0.0, -0.1],
            "accel_scale": 12.2,
            "M": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "az_accel_offset_deg": 30.0,
            "az_sign": 1,
            "az_yaw_offset_deg": 30.0,
            "az_yaw_sign": 1,
            "theta_cross_deg": 1.6,
            "mount_perm": ["+x", "+y", "+z"],
            "mount_misalign_deg": 0.5,
        },
        "metadata": {"timestamp": "t", "mode": "all"},
    }


def test_empty_store_returns_none():
    assert ImuCalStore(DummyTransport()).get() is None


def test_round_trip_preserves_sections():
    store = ImuCalStore(DummyTransport())
    store.upload(_payload())
    loaded = store.get()
    assert loaded["imu_az"]["az_accel_offset_deg"] == 30.0
    assert loaded["imu_az"]["M"] == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    assert "upload_time" in loaded  # injected by Transport.upload_dict


def test_corrupt_json_returns_none():
    t = DummyTransport()
    t.r.set(IMU_CAL_KEY, b"not-json-{")
    assert ImuCalStore(t).get() is None


def test_clear_removes_key():
    store = ImuCalStore(DummyTransport())
    store.upload(_payload())
    assert store.get() is not None
    store.clear()
    assert store.get() is None
