from eigsep_redis.testing import DummyTransport

from picohost.buses import ImuCalStore
from picohost.keys import IMU_CAL_KEY


def _payload():
    # El-only cal section shape (fit_el_calibration output) since the
    # 2026-07-09 azimuth descope; ImuCalStore itself is opaque to the keys.
    return {
        "imu_az": {
            "accel_bias": [0.1, 0.0, -0.1],
            "accel_scale": 12.2,
            "M": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
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
    assert loaded["imu_az"]["mount_misalign_deg"] == 0.5
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


def test_upload_merges_preserving_other_sections():
    # A --mode elevation run uploads only imu_el; a prior imu_az section
    # must survive (read-modify-write merge, not whole-key replace).
    store = ImuCalStore(DummyTransport())
    store.upload(_payload())  # imu_az only

    store.upload(
        {
            "imu_el": {"accel_bias": [0, 0, 0], "el_sign": 1},
            "metadata": {"timestamp": "t2", "mode": "elevation"},
        }
    )

    loaded = store.get()
    assert loaded["imu_az"]["mount_misalign_deg"] == 0.5  # survived
    assert loaded["imu_el"]["el_sign"] == 1  # newly added
    assert loaded["metadata"]["mode"] == "elevation"  # latest run wins


def test_upload_replaces_same_section():
    # Re-calibrating the same IMU overwrites its section (no stale merge).
    store = ImuCalStore(DummyTransport())
    store.upload(_payload())
    store.upload({"imu_az": {"mount_misalign_deg": 99.0}})
    assert store.get()["imu_az"] == {"mount_misalign_deg": 99.0}
