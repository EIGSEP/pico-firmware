"""Tests for the motor position checkpoint / boot-detection feature.

Firmware step counters live in RAM and reset to 0 on power-up, so the
host persists ``(az_pos, el_pos, boot_id)`` to
:class:`~picohost.buses.MotorPositionStore` on every position change
and pushes the stored positions back down (``az_set_pos`` /
``el_set_pos``) when an incoming status packet carries a ``boot_id``
the checkpoint doesn't match — i.e. the pico rebooted since the
checkpoint was written.

Two layers of coverage:

- ``TestMotorPositionStore`` mirrors ``TestPotCalStore`` in
  ``test_potentiometer.py`` (empty / round-trip / corrupt / clear).
- ``TestCheckpointAndSeed`` drives ``_checkpoint_and_seed`` directly
  with firmware-shaped status dicts on a ``__new__``-bypass instance
  (same pattern as eigsep_observing's producer-contract tests) for
  deterministic branch coverage; ``TestEndToEnd`` runs the real
  ``DummyPicoMotor`` + ``MotorEmulator`` stack through reboot
  scenarios.
"""

import time

from eigsep_redis import MetadataWriter
from eigsep_redis.testing import DummyTransport

from picohost.buses import MotorPositionStore
from picohost.keys import MOTOR_POS_KEY
from picohost.motor import PicoMotor
from picohost.testing import DummyPicoMotor

from conftest import wait_for_condition

# Firmware boot ids are random 30-bit non-negative ints
# (get_rand_32() & 0x3fffffff), so a negative sentinel in a primed
# checkpoint is guaranteed to differ from any live emulator boot_id.
STALE_BOOT_ID = -1


def _status(boot_id, az=0, el=0):
    """A firmware-shaped motor status packet (see motor_status() in
    motor.c). Targets mirror positions — the seed/checkpoint logic
    reads only positions and boot_id.
    """
    return {
        "sensor_name": "motor",
        "status": "update",
        "app_id": 0,
        "boot_id": boot_id,
        "az_pos": az,
        "az_target_pos": az,
        "el_pos": el,
        "el_target_pos": el,
    }


class TestMotorPositionStore:
    def test_get_empty_returns_none(self):
        store = MotorPositionStore(DummyTransport())
        assert store.get() is None

    def test_round_trip(self):
        store = MotorPositionStore(DummyTransport())
        store.upload(az_pos=500, el_pos=-300, boot_id=7)
        blob = store.get()
        assert blob["az_pos"] == 500
        assert blob["el_pos"] == -300
        assert blob["boot_id"] == 7
        # Canonical timestamp injected by Transport.upload_dict.
        assert "upload_time" in blob

    def test_corrupt_json_returns_none(self):
        transport = DummyTransport()
        store = MotorPositionStore(transport)
        transport.r.set(MOTOR_POS_KEY, "not json{")
        assert store.get() is None

    def test_missing_field_returns_none(self):
        transport = DummyTransport()
        store = MotorPositionStore(transport)
        # boot_id missing: a blob the seed logic can't fully validate
        # must read back as "no checkpoint".
        transport.upload_dict({"az_pos": 1, "el_pos": 2}, MOTOR_POS_KEY)
        assert store.get() is None

    def test_non_int_field_returns_none(self):
        transport = DummyTransport()
        store = MotorPositionStore(transport)
        transport.upload_dict(
            {"az_pos": "abc", "el_pos": 2, "boot_id": 7}, MOTOR_POS_KEY
        )
        assert store.get() is None

    def test_clear(self):
        store = MotorPositionStore(DummyTransport())
        store.upload(az_pos=1, el_pos=2, boot_id=3)
        store.clear()
        assert store.get() is None


def _bare_motor(store):
    """A serial-less PicoMotor carrying only the checkpoint/seed state.

    The ``__new__`` bypass (no serial I/O, no reader thread) mirrors
    eigsep_observing's producer-contract tests; it lets the tests call
    ``_checkpoint_and_seed`` with deterministic status sequences. The
    instance-level ``reset_step_position`` shadow records seed sends
    instead of writing to a (nonexistent) serial port.
    """
    m = PicoMotor.__new__(PicoMotor)
    m.name = "motor"
    m._motor_pos_store = store
    m._seen_boot_id = None
    m._last_checkpoint = None
    m._await_seed = None
    m._seed_sent_time = None
    m._warned_no_boot_id = False
    m.seeds = []
    m.reset_step_position = lambda az_step, el_step: m.seeds.append(
        (az_step, el_step)
    )
    return m


class TestCheckpointAndSeed:
    def test_checkpoint_uploaded_and_deduped(self):
        store = MotorPositionStore(DummyTransport())
        uploads = []
        orig_upload = store.upload
        store.upload = lambda **kw: (uploads.append(kw), orig_upload(**kw))
        m = _bare_motor(store)
        m._checkpoint_and_seed(_status(7, az=0, el=0))
        m._checkpoint_and_seed(_status(7, az=0, el=0))
        assert len(uploads) == 1
        # A position change re-checkpoints.
        m._checkpoint_and_seed(_status(7, az=10, el=-20))
        assert len(uploads) == 2
        blob = store.get()
        assert (blob["az_pos"], blob["el_pos"], blob["boot_id"]) == (
            10,
            -20,
            7,
        )

    def test_same_boot_never_seeds(self):
        store = MotorPositionStore(DummyTransport())
        store.upload(az_pos=500, el_pos=300, boot_id=7)
        m = _bare_motor(store)
        m._checkpoint_and_seed(_status(7, az=0, el=0))
        # Checkpoint written under this very boot: firmware position is
        # live truth (a manager restart against a running pico) — no
        # seed, and the checkpoint follows the live report.
        assert m.seeds == []
        assert store.get()["az_pos"] == 0

    def test_reboot_seeds_and_suppresses_checkpoint(self):
        store = MotorPositionStore(DummyTransport())
        store.upload(az_pos=500, el_pos=-300, boot_id=7)
        m = _bare_motor(store)
        # First post-reboot packet: all-zero counters, new boot_id.
        m._checkpoint_and_seed(_status(8, az=0, el=0))
        assert m.seeds == [(500, -300)]
        # The good checkpoint must NOT be overwritten by the pre-seed
        # zeros...
        blob = store.get()
        assert (blob["az_pos"], blob["boot_id"]) == (500, 7)
        # ...nor by in-transit positions while the seed lands...
        m._checkpoint_and_seed(_status(8, az=250, el=-150))
        assert store.get()["boot_id"] == 7
        # ...until the seeded position is reflected in a status packet,
        # at which point checkpointing resumes under the new boot.
        m._checkpoint_and_seed(_status(8, az=500, el=-300))
        blob = store.get()
        assert (blob["az_pos"], blob["el_pos"], blob["boot_id"]) == (
            500,
            -300,
            8,
        )
        # The reboot is handled exactly once.
        assert m.seeds == [(500, -300)]

    def test_seed_timeout_resumes_checkpoints(self, caplog):
        store = MotorPositionStore(DummyTransport())
        store.upload(az_pos=500, el_pos=-300, boot_id=7)
        m = _bare_motor(store)
        m._checkpoint_and_seed(_status(8, az=0, el=0))
        assert m.seeds == [(500, -300)]
        # Seed never reflected (lost command / concurrent move): after
        # the timeout, checkpointing resumes from the live position so
        # the store can't stay frozen on a dead boot_id forever.
        m._seed_sent_time = time.time() - 2 * PicoMotor._SEED_TIMEOUT_S
        with caplog.at_level("ERROR", logger="picohost.motor"):
            m._checkpoint_and_seed(_status(8, az=42, el=0))
        assert "not reflected" in caplog.text
        blob = store.get()
        assert (blob["az_pos"], blob["boot_id"]) == (42, 8)

    def test_empty_store_never_seeds(self):
        store = MotorPositionStore(DummyTransport())
        m = _bare_motor(store)
        m._checkpoint_and_seed(_status(7, az=5, el=6))
        assert m.seeds == []
        assert store.get()["az_pos"] == 5

    def test_corrupt_store_treated_as_empty(self, caplog):
        transport = DummyTransport()
        store = MotorPositionStore(transport)
        transport.r.set(MOTOR_POS_KEY, "not json{")
        m = _bare_motor(store)
        with caplog.at_level("WARNING", logger="picohost.buses"):
            m._checkpoint_and_seed(_status(8, az=0, el=0))
        assert "Corrupted" in caplog.text
        assert m.seeds == []
        # The corrupt blob is replaced by a valid live checkpoint.
        assert store.get()["boot_id"] == 8

    def test_missing_boot_id_warns_once_and_disables(self, caplog):
        store = MotorPositionStore(DummyTransport())
        m = _bare_motor(store)
        old_firmware = {
            "sensor_name": "motor",
            "status": "update",
            "app_id": 0,
            "az_pos": 5,
            "az_target_pos": 5,
            "el_pos": 6,
            "el_target_pos": 6,
        }
        with caplog.at_level("WARNING", logger="picohost.motor"):
            m._checkpoint_and_seed(old_firmware)
            m._checkpoint_and_seed(old_firmware)
        assert caplog.text.count("position checkpointing disabled") == 1
        assert store.get() is None
        assert m.seeds == []


class TestEndToEnd:
    """Real DummyPicoMotor + MotorEmulator + DummyTransport pipeline."""

    def _build(self, store=None, transport=None):
        transport = transport or DummyTransport()
        store = store or MotorPositionStore(transport)
        motor = DummyPicoMotor(
            port="/dev/ttyUSB0",
            metadata_writer=MetadataWriter(transport),
            motor_pos_store=store,
        )
        return motor, store

    def test_checkpoint_tracks_settled_position(self):
        motor, store = self._build()
        cadence = motor.EMULATOR_CADENCE_MS
        try:
            wait_for_condition(
                lambda: (
                    (s := store.get()) is not None
                    and s["boot_id"] == motor._emulator.boot_id
                ),
                cadence_ms=cadence,
            )
            motor.az_target_steps(120)
            wait_for_condition(
                lambda: store.get()["az_pos"] == 120,
                cadence_ms=cadence,
            )
        finally:
            motor.disconnect()

    def test_cold_boot_reseed(self):
        """Manager start against a freshly booted pico: store holds a
        checkpoint from a previous boot (sentinel boot_id), the
        emulator reports zeros — the handler must push the checkpoint
        back down and re-pair the store with the live boot_id.
        """
        transport = DummyTransport()
        store = MotorPositionStore(transport)
        store.upload(az_pos=500, el_pos=-300, boot_id=STALE_BOOT_ID)
        motor, _ = self._build(store=store, transport=transport)
        cadence = motor.EMULATOR_CADENCE_MS
        try:
            wait_for_condition(
                lambda: (
                    motor.last_status.get("az_pos") == 500
                    and motor.last_status.get("el_pos") == -300
                ),
                cadence_ms=cadence,
            )
            wait_for_condition(
                lambda: store.get()["boot_id"] == motor._emulator.boot_id,
                cadence_ms=cadence,
            )
            blob = store.get()
            assert (blob["az_pos"], blob["el_pos"]) == (500, -300)
        finally:
            motor.disconnect()

    def test_midrun_power_cycle_reseed(self):
        """Pico power-cycles while the manager keeps running: emulator
        init() models the firmware reboot (counters zeroed, new
        boot_id) and the handler restores the last checkpoint.
        """
        motor, store = self._build()
        cadence = motor.EMULATOR_CADENCE_MS
        try:
            # az_target_steps waits on is_moving, which requires a
            # first status packet to have arrived.
            wait_for_condition(
                lambda: motor.last_status.get("sensor_name") == "motor",
                cadence_ms=cadence,
            )
            motor.az_target_steps(120)
            wait_for_condition(
                lambda: (
                    store.get() is not None and store.get()["az_pos"] == 120
                ),
                cadence_ms=cadence,
            )
            old_boot = motor._emulator.boot_id
            motor._emulator.init()
            assert motor._emulator.boot_id != old_boot
            wait_for_condition(
                lambda: (
                    motor._emulator.azimuth.position == 120
                    and store.get()["boot_id"] == motor._emulator.boot_id
                ),
                cadence_ms=cadence,
            )
        finally:
            motor.disconnect()
