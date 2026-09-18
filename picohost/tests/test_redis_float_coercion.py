"""Regression tests for issue #148: whole-valued floats must not reach
Redis as ints.

Firmware cJSON prints a whole-valued double with no decimal point
(``30.0`` -> ``30``), so ``json.loads`` on the host yields Python
``int`` for fields the firmware emitted as ``KV_FLOAT``. Two halves of
the fix are pinned here:

1. Emulators serialize numbers the way cJSON does, so the dummy-device
   pipeline exercises the same int-collapsed shapes as real firmware
   (previously ``json.dumps`` preserved ``30.0`` and masked the bug).
2. The redis publish path coerces the firmware's float-typed fields
   back to ``float`` before ``MetadataWriter.add``, so the published
   types satisfy the consumer metadata schemas regardless of value.
"""

import json

from conftest import wait_for_condition

from picohost.base import redis_handler
from picohost.emulators.base import PicoEmulator
from picohost.testing import (
    DummyPicoIMU,
    DummyPicoLidar,
    DummyPicoTempCtrl,
    DummyPicoPotentiometer,
    DummyPicoRFSwitch,
)


class FakeMetadataWriter:
    def __init__(self):
        self.received = []

    def add(self, name, data):
        self.received.append((name, dict(data)))


# Emulator-less dummies: the handler-chain tests below drive
# dev.redis_handler() directly with firmware-shaped payloads, so a live
# emulator would only interleave unrelated entries into the writer.
class _QuietTempCtrl(DummyPicoTempCtrl):
    def _make_emulator(self):
        return None


class _QuietIMU(DummyPicoIMU):
    def _make_emulator(self):
        return None


class _QuietLidar(DummyPicoLidar):
    def _make_emulator(self):
        return None


class _QuietRFSwitch(DummyPicoRFSwitch):
    def _make_emulator(self):
        return None


class _QuietPotentiometer(DummyPicoPotentiometer):
    def _make_emulator(self):
        return None


# --- 1. Emulator serialization matches cJSON print_number ---


class _FakePeer:
    def __init__(self):
        self.chunks = []

    def write(self, data):
        self.chunks.append(data)


def _emit(payload):
    emu = PicoEmulator()
    peer = _FakePeer()
    emu.attach(peer)
    emu._write_json(payload)
    return json.loads(b"".join(peer.chunks).decode())


class TestEmulatorCJSONNumbers:
    def test_whole_float_collapses_to_int(self):
        parsed = _emit({"a": 30.0})
        assert parsed["a"] == 30
        assert isinstance(parsed["a"], int)

    def test_fractional_float_survives(self):
        parsed = _emit({"a": 30.5})
        assert parsed["a"] == 30.5
        assert isinstance(parsed["a"], float)

    def test_nan_and_inf_become_null(self):
        parsed = _emit({"a": float("nan"), "b": float("inf")})
        assert parsed["a"] is None
        assert parsed["b"] is None

    def test_bool_int_str_none_untouched(self):
        parsed = _emit({"a": True, "b": 7, "c": "x", "d": None})
        assert parsed["a"] is True
        assert parsed["b"] == 7 and isinstance(parsed["b"], int)
        assert parsed["c"] == "x"
        assert parsed["d"] is None

    def test_whole_float_beyond_int32_still_collapses(self):
        # cJSON prints these via %1.15g, which also drops the decimal
        # point for whole values ("4000000000"), so json.loads -> int.
        parsed = _emit({"a": 4000000000.0})
        assert parsed["a"] == 4000000000
        assert isinstance(parsed["a"], int)

    def test_exponent_notation_range_stays_float(self):
        # At 1e15 %1.15g switches to exponent notation ("1e+15"),
        # which parses back as float — mirror that boundary.
        parsed = _emit({"a": 1e15})
        assert parsed["a"] == 1e15
        assert isinstance(parsed["a"], float)


# --- 2. redis_handler factory coercion ---


class TestRedisHandlerFloatFields:
    def test_listed_int_fields_publish_as_float(self):
        writer = FakeMetadataWriter()
        handler = redis_handler(writer, float_fields=("a", "b"))
        handler({"sensor_name": "s", "a": 30, "b": 2.5, "c": 7})
        ((name, data),) = writer.received
        assert name == "s"
        assert data["a"] == 30.0 and isinstance(data["a"], float)
        assert data["b"] == 2.5 and isinstance(data["b"], float)
        # Unlisted fields keep their parsed type.
        assert data["c"] == 7 and isinstance(data["c"], int)

    def test_none_and_missing_fields_untouched(self):
        writer = FakeMetadataWriter()
        handler = redis_handler(writer, float_fields=("a", "b"))
        handler({"sensor_name": "s", "a": None})
        ((_, data),) = writer.received
        assert data["a"] is None
        assert "b" not in data

    def test_bool_never_coerced(self):
        # bool is a subclass of int; a listed field that arrives as a
        # bool (KV_BOOL mixup upstream) must not silently become 0.0/1.0.
        writer = FakeMetadataWriter()
        handler = redis_handler(writer, float_fields=("a",))
        handler({"sensor_name": "s", "a": True})
        ((_, data),) = writer.received
        assert data["a"] is True

    def test_caller_dict_not_mutated(self):
        # The same dict is stored as device.last_status; coercion must
        # not alias into it.
        writer = FakeMetadataWriter()
        handler = redis_handler(writer, float_fields=("a",))
        payload = {"sensor_name": "s", "a": 30}
        handler(payload)
        assert payload["a"] == 30 and isinstance(payload["a"], int)


# --- 3. Device handler chains publish float-typed firmware fields ---


def _tempctrl_payload():
    """Firmware-shaped tempctrl status (src/tempctrl.c) with every
    routinely whole-valued KV_FLOAT collapsed to int, as cJSON emits.

    LOAD is a single low-side FET heater (on/off hysteresis, no PI
    loop), so it has no cooling_enabled/clamp/Kp/Ki/integral fields at
    all — firmware never sends them.
    """
    payload = {
        "sensor_name": "tempctrl",
        "app_id": 1,
        "watchdog_tripped": False,
        "watchdog_timeout_ms": 30000,
    }
    payload.update(
        {
            "LOAD_status": "update",
            "LOAD_T_now": 29.87,
            "LOAD_voltage": 1.234,
            "LOAD_resistance": 10234.5,
            "LOAD_timestamp": 1234,  # uint32 cast: whole on every tick
            "LOAD_T_target": 30,
            "LOAD_drive_level": 0,
            "LOAD_installed": True,
            "LOAD_enabled": True,
            "LOAD_active": False,
            "LOAD_sensor_tripped": False,
            "LOAD_stall_tripped": False,
            "LOAD_runaway_tripped": False,
            "LOAD_hysteresis": 1,
            # LNA1/LNA2: read-only thermistors, whole-valued to exercise
            # the same int->float coercion as LOAD's fields.
            "LNA1_status": "update",
            "LNA1_T_now": 24,
            "LNA1_voltage": 1,
            "LNA1_resistance": 32000,
            "LNA1_timestamp": 1234,
            "LNA2_status": "update",
            "LNA2_T_now": 26,
            "LNA2_voltage": 1,
            "LNA2_resistance": 30500,
            "LNA2_timestamp": 1234,
        }
    )
    return payload


TEMPCTRL_FLOAT_FIELDS = (
    "T_now",
    "voltage",
    "resistance",
    "timestamp",
    "T_target",
    "drive_level",
    "hysteresis",
)

# LNA1/LNA2 publish the same field names as LOAD but without the
# control-only ones (no T_target/drive_level/hysteresis to report).
LNA_FLOAT_FIELDS = ("T_now", "voltage", "resistance", "timestamp")


class TestDeviceHandlersCoerce:
    def test_tempctrl_stream_publishes_floats(self):
        writer = FakeMetadataWriter()
        dev = _QuietTempCtrl(
            "/dev/dummy", metadata_writer=writer, keepalive_interval=0
        )
        try:
            dev.redis_handler(_tempctrl_payload())
        finally:
            dev.disconnect()
        published = dict(writer.received)
        assert set(published) == {
            "tempctrl_load",
            "tempctrl_lna1",
            "tempctrl_lna2",
        }
        load = published["tempctrl_load"]
        for key in TEMPCTRL_FLOAT_FIELDS:
            assert isinstance(load[key], float), f"tempctrl_load:{key}"
        # Non-float fields keep their firmware types.
        assert load["watchdog_tripped"] is False
        assert isinstance(load["watchdog_timeout_ms"], int)
        assert load["enabled"] is True
        assert load["status"] == "update"
        for stream in ("tempctrl_lna1", "tempctrl_lna2"):
            data = published[stream]
            for key in LNA_FLOAT_FIELDS:
                assert isinstance(data[key], float), f"{stream}:{key}"
            assert data["watchdog_tripped"] is False
            assert isinstance(data["watchdog_timeout_ms"], int)
            assert data["status"] == "update"

    def test_imu_publishes_floats(self):
        writer = FakeMetadataWriter()
        dev = _QuietIMU("/dev/dummy", metadata_writer=writer)
        try:
            dev.redis_handler(
                {
                    "sensor_name": "imu_el",
                    "status": "update",
                    "app_id": 3,
                    "yaw": 0,
                    "pitch": 0,
                    "roll": 0,
                    "accel_x": 0,
                    "accel_y": 0,
                    "accel_z": 1,
                }
            )
        finally:
            dev.disconnect()
        ((name, data),) = writer.received
        assert name == "imu_el"
        for key in ("yaw", "pitch", "roll", "accel_x", "accel_y", "accel_z"):
            assert isinstance(data[key], float), key
        assert isinstance(data["app_id"], int)

    def test_lidar_and_system_current_publish_floats(self):
        writer = FakeMetadataWriter()
        dev = _QuietLidar("/dev/dummy", metadata_writer=writer)
        try:
            dev.redis_handler(
                {
                    "sensor_name": "lidar",
                    "status": "update",
                    "app_id": 4,
                    "distance_m": 5,
                    "current_voltage": 1,
                }
            )
        finally:
            dev.disconnect()
        published = dict(writer.received)
        assert set(published) == {"lidar", "system_current"}
        assert isinstance(published["lidar"]["distance_m"], float)
        assert isinstance(
            published["system_current"]["current_voltage"], float
        )

    def test_rfswitch_therm_stream_publishes_floats(self):
        writer = FakeMetadataWriter()
        dev = _QuietRFSwitch("/dev/dummy", metadata_writer=writer)
        try:
            dev.redis_handler(
                {
                    "sensor_name": "rfswitch",
                    "status": "update",
                    "app_id": 5,
                    "sw_state": 0,
                    "volt_therm0": 1,
                    "volt_therm1": 2,
                    "volt_therm2": 3,
                }
            )
        finally:
            dev.disconnect()
        published = dict(writer.received)
        assert set(published) == {"rfswitch", "rfswitch_therm"}
        therm = published["rfswitch_therm"]
        for i in range(3):
            assert isinstance(therm[f"volt_therm{i}"], float), i
        # Switch state is categorical and must stay an int.
        assert isinstance(published["rfswitch"]["sw_state"], int)

    def test_potmon_publishes_float_voltage(self):
        writer = FakeMetadataWriter()
        dev = _QuietPotentiometer("/dev/dummy", metadata_writer=writer)
        try:
            dev.redis_handler(
                {
                    "sensor_name": "potmon",
                    "app_id": 2,
                    "status": "update",
                    "pot_az_voltage": 2,
                }
            )
        finally:
            dev.disconnect()
        ((name, data),) = writer.received
        assert name == "potmon"
        assert isinstance(data["pot_az_voltage"], float)
        assert data["pot_az_near_rail"] is False


# --- 4. End-to-end: emulator collapse -> parse -> coerce -> publish ---


def test_dummy_tempctrl_round_trip_publishes_floats():
    """Full-pipeline #148 regression. The tempctrl emulator's default
    T_target=30.0 is exactly the whole value that cJSON collapses; after
    the round trip it must still publish as a float."""
    writer = FakeMetadataWriter()
    dev = DummyPicoTempCtrl(
        "/dev/dummy", metadata_writer=writer, keepalive_interval=0
    )
    try:
        wait_for_condition(
            lambda: any(n == "tempctrl_load" for n, _ in writer.received),
            cadence_ms=dev.EMULATOR_CADENCE_MS,
        )
    finally:
        dev.disconnect()
    data = next(d for n, d in writer.received if n == "tempctrl_load")
    assert data["T_target"] == 30.0
    for key in ("T_target", "drive_level"):
        assert isinstance(data[key], float), key
