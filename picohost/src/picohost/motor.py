"""
Base class for Pico device communication.
Provides common functionality for serial communication with Pico devices.
"""

import logging
import time
import numpy as np
from .base import PicoDevice

logger = logging.getLogger(__name__)

# Geometry of the installed az drive. Step angle and gear teeth are fixed
# properties of the hardware; microstep is a physical driver switch that
# is very unlikely to change. Single source for both PicoMotor (deg->steps
# on the manager side) and calibrate-pot (steps->deg on the client side) —
# the two conversions must always use the same numbers or --mode auto's
# settle detection can never match the commanded target.
STEP_ANGLE_DEG = 1.8
GEAR_TEETH = 113
MICROSTEP = 1


def steps_to_deg(steps, *, step_angle_deg, gear_teeth, microstep):
    """Convert motor pulses to degrees (pure; no device state).

    Shared by :meth:`PicoMotor.steps_to_deg` and ``calibrate-pot`` so the
    geometry formula lives in exactly one place.
    """
    s = steps / microstep / gear_teeth
    return float(s * step_angle_deg)


class PicoMotor(PicoDevice):
    """Specialized class for motor control Pico devices."""

    #: Seconds to wait for a re-seeded position to show up in firmware
    #: status before giving up and resuming checkpoints (see
    #: :meth:`_checkpoint_and_seed`).
    _SEED_TIMEOUT_S = 5.0

    def __init__(
        self,
        port,
        step_angle_deg=STEP_ANGLE_DEG,
        gear_teeth=GEAR_TEETH,
        microstep=MICROSTEP,
        verbose=False,
        timeout=5.0,
        name=None,
        metadata_writer=None,
        usb_serial="",
        motor_pos_store=None,
    ):
        self.step_angle_deg = step_angle_deg
        self.gear_teeth = gear_teeth
        self.microstep = microstep
        self.commands = {
            "az_set_pos": int,
            "el_set_pos": int,
            "az_set_target_pos": int,
            "el_set_target_pos": int,
            "halt": int,
            "az_up_delay_us": int,
            "az_dn_delay_us": int,
            "el_up_delay_us": int,
            "el_dn_delay_us": int,
        }
        self._delay_kwargs = None
        # Position checkpoint / boot-detection state. Touched only by
        # _checkpoint_and_seed, which runs on the reader thread; set up
        # before super().__init__ because the reader may start in there.
        # The whole feature is inert when motor_pos_store is None (or
        # when there is no metadata_writer — the redis-handler wrapper
        # is the hook, so no publishing means no checkpointing).
        self._motor_pos_store = motor_pos_store
        self._seen_boot_id = None
        self._last_checkpoint = None
        self._await_seed = None
        self._seed_sent_time = None
        self._warned_no_boot_id = False
        super().__init__(
            port,
            timeout=timeout,
            name=name,
            metadata_writer=metadata_writer,
            usb_serial=usb_serial,
            verbose=verbose,
        )
        # Wrap the base redis handler to checkpoint positions and
        # detect reboots. See _motor_redis_handler. Install this
        # immediately after super().__init__ because the base class may
        # already have started the reader thread.
        if self.redis_handler is not None:
            self._base_redis_handler = self.redis_handler
            self.redis_handler = self._motor_redis_handler
        self.set_delay()

    _POSITION_FIELDS = (
        "az_pos",
        "az_target_pos",
        "el_pos",
        "el_target_pos",
    )

    # The C firmware emits position fields with ``KV_INT`` (raw step
    # counts), which the JSON parser surfaces as Python ``int``.
    # Position values legitimately change within a single consumer
    # integration window during a scan, so the consumer-side
    # per-integration reduction needs the float→mean policy rather
    # than the int→min "invariant" policy. Listing them here casts
    # them at the publish boundary (see PicoDevice._REDIS_FLOAT_FIELDS)
    # without requiring a firmware reflash.
    _REDIS_FLOAT_FIELDS = _POSITION_FIELDS

    def _motor_redis_handler(self, data):
        """Checkpoint the raw integer positions, then publish.

        The position checkpoint / boot-detection logic
        (:meth:`_checkpoint_and_seed`) runs off the raw integer
        payload; the float cast for the published position fields
        happens downstream in the base handler (``_REDIS_FLOAT_FIELDS``).
        """
        self._checkpoint_and_seed(data)
        self._base_redis_handler(data)

    def _checkpoint_and_seed(self, data):
        """Persist the step position; re-seed it after a pico reboot.

        Runs on the reader thread for every firmware status packet
        (safe: the serial write path used by the re-seed has its own
        lock and never waits on status). Two jobs:

        1. **Checkpoint.** Upload ``(az_pos, el_pos, boot_id)`` to the
           :class:`~picohost.buses.MotorPositionStore` whenever the
           triple changes — during a move that is one upload per
           status tick, at rest none. The checkpoint is what survives
           a full-system power cut (Redis on the host persists it).
        2. **Boot detection + re-seed.** Firmware step counters live
           in RAM and reset to 0 at power-up; ``boot_id`` is a random
           per-boot constant in every status packet. When the
           incoming ``boot_id`` differs from the checkpointed one,
           the pico has rebooted since the checkpoint was written, so
           the stored positions are pushed back down via
           :meth:`reset_step_position`. Checkpointing is suppressed
           until the seeded position is reflected in a status packet
           (else the first post-reboot all-zero status would
           overwrite the good checkpoint), bounded by
           :data:`_SEED_TIMEOUT_S` so a lost seed command cannot
           freeze checkpointing forever.

        A matching ``boot_id`` means the firmware position is live
        truth — a manager restart against a still-running pico never
        re-seeds.
        """
        store = self._motor_pos_store
        if store is None:
            return
        boot_id = data.get("boot_id")
        az = data.get("az_pos")
        el = data.get("el_pos")
        if boot_id is None or az is None or el is None:
            if not self._warned_no_boot_id:
                self._warned_no_boot_id = True
                logger.warning(
                    f"{self.name}: status packet lacks boot_id or "
                    "position fields; position checkpointing disabled "
                    "(firmware predates boot_id?)"
                )
            return
        az, el, boot_id = int(az), int(el), int(boot_id)

        if boot_id != self._seen_boot_id:
            self._handle_new_boot(boot_id)

        if self._await_seed is not None:
            if (az, el) == self._await_seed:
                self._await_seed = None
            elif time.time() - self._seed_sent_time > self._SEED_TIMEOUT_S:
                logger.error(
                    f"{self.name}: re-seeded position "
                    f"{self._await_seed} not reflected in status "
                    f"within {self._SEED_TIMEOUT_S}s; resuming "
                    "checkpoints from live position"
                )
                self._await_seed = None
            else:
                # Pre-seed positions are the reset-to-zero counters,
                # not real state — don't checkpoint them.
                return

        checkpoint = (az, el, boot_id)
        if checkpoint != self._last_checkpoint:
            store.upload(az_pos=az, el_pos=el, boot_id=boot_id)
            self._last_checkpoint = checkpoint

    def _handle_new_boot(self, boot_id):
        """React to a never-seen ``boot_id``: re-seed if the stored
        checkpoint belongs to a different boot.

        ``_seen_boot_id`` is committed only after the branch completes,
        so a failed seed send is retried on the next status packet.
        """
        stored = self._motor_pos_store.get()
        if stored is None or stored["boot_id"] == boot_id:
            # No checkpoint to restore, or the checkpoint was written
            # under this very boot (manager restart, pico kept running):
            # firmware position is authoritative.
            self._seen_boot_id = boot_id
            return
        az, el = stored["az_pos"], stored["el_pos"]
        age = time.time() - stored.get("upload_time", time.time())
        logger.warning(
            f"{self.name}: pico rebooted (boot_id "
            f"{stored['boot_id']} -> {boot_id}); re-seeding step "
            f"position az={az} el={el} from checkpoint ({age:.0f}s old)"
        )
        try:
            self.reset_step_position(az_step=az, el_step=el)
        except ConnectionError as e:
            # Status packets arrive over the same serial link, so this
            # should be unreachable; log and leave _seen_boot_id unset
            # so the next packet retries.
            logger.error(f"{self.name}: position re-seed failed: {e}")
            return
        self._await_seed = (az, el)
        self._seed_sent_time = time.time()
        self._seen_boot_id = boot_id

    def on_reconnect(self):
        """Re-apply delay configuration after a serial reconnect."""
        if self._delay_kwargs is not None:
            self.set_delay(**self._delay_kwargs)

    def deg_to_steps(self, degrees: float) -> int:
        """Convert degrees to motor pulses."""
        s = degrees / self.step_angle_deg
        return int(s * self.microstep * self.gear_teeth)

    def steps_to_deg(self, steps: int) -> float:
        """Convert motor pulses to degrees."""
        return steps_to_deg(
            steps,
            step_angle_deg=self.step_angle_deg,
            gear_teeth=self.gear_teeth,
            microstep=self.microstep,
        )

    def motor_command(self, **kwargs):
        """Send a json motor command with specified keys."""
        # check commands
        cmd = {}
        for k, v in kwargs.items():
            if k not in self.commands:
                raise ValueError(f"command {k} not in {self.commands}")
            cmd[k] = self.commands[k](v)
        self.send_command(cmd)

    def reset_step_position(self, az_step=None, el_step=None):
        """Set az and el position to specified count."""
        cmd = {}
        if az_step is not None:
            cmd["az_set_pos"] = az_step
        if el_step is not None:
            cmd["el_set_pos"] = el_step
        self.motor_command(**cmd)

    def reset_deg_position(self, az_deg=None, el_deg=None):
        """Set az and el position to specified count."""
        az_pos = None if az_deg is None else self.deg_to_steps(az_deg)
        el_pos = None if el_deg is None else self.deg_to_steps(el_deg)
        self.reset_step_position(az_step=az_pos, el_step=el_pos)

    def set_delay(
        self,
        az_up_delay_us=2300,
        az_dn_delay_us=2300,
        el_up_delay_us=2300,
        el_dn_delay_us=2300,
    ):
        self._delay_kwargs = {
            "az_up_delay_us": az_up_delay_us,
            "az_dn_delay_us": az_dn_delay_us,
            "el_up_delay_us": el_up_delay_us,
            "el_dn_delay_us": el_dn_delay_us,
        }
        self.motor_command(**self._delay_kwargs)

    def halt(self):
        """Hard stop on both motors."""
        self.motor_command(halt=0)

    def _do_wait(self, wait_for_start, wait_for_stop):
        if wait_for_start:
            self.wait_for_start()
        if wait_for_stop:
            self.wait_for_stop()

    def az_target_steps(
        self, target_steps, wait_for_start=True, wait_for_stop=False
    ):
        """Move az to target step position."""
        self.motor_command(az_set_target_pos=target_steps)
        self._do_wait(wait_for_start, wait_for_stop)

    def az_target_deg(
        self, target_deg, wait_for_start=True, wait_for_stop=False
    ):
        """Move az to target deg position."""
        self.az_target_steps(
            self.deg_to_steps(target_deg),
            wait_for_start=wait_for_start,
            wait_for_stop=wait_for_stop,
        )

    def _require_status(self):
        """Raise if no firmware status has been received yet."""
        if not self.last_status:
            raise RuntimeError(f"No status from {self.name} yet")

    def az_move_steps(
        self, delta_steps, wait_for_start=True, wait_for_stop=False
    ):
        """Move az in specified number of steps from current target."""
        self._require_status()
        new_target = self.last_status["az_target_pos"] + delta_steps
        self.az_target_steps(
            new_target,
            wait_for_start=wait_for_start,
            wait_for_stop=wait_for_stop,
        )

    def az_move_deg(self, delta_deg, wait_for_start=True, wait_for_stop=False):
        """Move az in specified number of degs from current target."""
        self.az_move_steps(
            self.deg_to_steps(delta_deg),
            wait_for_start=wait_for_start,
            wait_for_stop=wait_for_stop,
        )

    def el_target_steps(
        self, target_steps, wait_for_start=True, wait_for_stop=False
    ):
        """Move el to target step position."""
        self.motor_command(el_set_target_pos=target_steps)
        self._do_wait(wait_for_start, wait_for_stop)

    def el_target_deg(
        self, target_deg, wait_for_start=True, wait_for_stop=False
    ):
        """Move el to target deg position."""
        self.el_target_steps(
            self.deg_to_steps(target_deg),
            wait_for_start=wait_for_start,
            wait_for_stop=wait_for_stop,
        )

    def el_move_steps(
        self, delta_steps, wait_for_start=True, wait_for_stop=False
    ):
        """Move el in specified number of steps from current target."""
        self._require_status()
        new_target = self.last_status["el_target_pos"] + delta_steps
        self.el_target_steps(
            new_target,
            wait_for_start=wait_for_start,
            wait_for_stop=wait_for_stop,
        )

    def el_move_deg(self, delta_deg, wait_for_start=True, wait_for_stop=False):
        """Move el in specified number of degs from current target."""
        self.el_move_steps(
            self.deg_to_steps(delta_deg),
            wait_for_start=wait_for_start,
            wait_for_stop=wait_for_stop,
        )

    def is_moving(self):
        self._require_status()
        return (
            self.last_status["az_target_pos"] != self.last_status["az_pos"]
            or self.last_status["el_target_pos"] != self.last_status["el_pos"]
        )

    def wait_for_start(self, timeout=0.3):
        t = time.time()
        while not self.is_moving() and time.time() < t + timeout:
            time.sleep(0.1)

    def wait_for_stop(self, stall_timeout=30):
        if self.verbose:
            logger.debug("Waiting for stop.")
        self._require_status()
        last_pos = (self.last_status["az_pos"], self.last_status["el_pos"])
        t = time.time()
        while self.is_moving():
            pos = (self.last_status["az_pos"], self.last_status["el_pos"])
            if pos != last_pos:
                last_pos = pos
                t = time.time()
            elif time.time() - t >= stall_timeout:
                raise TimeoutError(
                    f"Motor stalled for {stall_timeout}s without progress"
                )
            time.sleep(0.1)

    def scan(
        self,
        az_range_deg=np.arange(-180.0, 180.0, 5),
        el_range_deg=np.arange(-180.0, 180.0, 5),
        el_first=False,
        repeat_count=None,
        pause_s=None,
        sleep_between=None,
    ):
        """
        Perform beam scanning strategy.

        Homes motors to (0, 0) before starting and after normal
        completion.  Use ``reset_step_position`` beforehand to define
        where home is.

        Parameters
        ---------
        az_range_deg : array_like
        el_range_deg : array_like
        el_first : bool
        repeat_count : int
        pause_s : float
            Pause time at each position.
        sleep_between : float
            Sleep between every scan (if `repeat_count` is not None).
        """
        # home before scanning
        self.az_target_steps(0, wait_for_stop=True)
        self.el_target_steps(0, wait_for_stop=True)
        # set order of scanning
        if el_first:
            mv_axis1, mv_axis2 = self.az_target_deg, self.el_target_deg
            axis1_rng, axis2_rng = az_range_deg.copy(), el_range_deg.copy()
        else:
            mv_axis2, mv_axis1 = self.az_target_deg, self.el_target_deg
            axis2_rng, axis1_rng = az_range_deg.copy(), el_range_deg.copy()

        i = 0
        try:
            while True:
                if repeat_count is not None and i >= repeat_count:
                    break
                for val1 in axis1_rng:
                    if self.verbose:
                        logger.info("MOVE AXIS 1 TO %s", val1)
                    mv_axis1(val1, wait_for_stop=True)
                    if pause_s is None:
                        if self.verbose:
                            logger.info(
                                "MOVE AXIS 2 FROM %s TO %s",
                                axis2_rng[0],
                                axis2_rng[-1],
                            )
                        # continuous motion
                        mv_axis2(axis2_rng[0], wait_for_stop=True)
                        mv_axis2(axis2_rng[-1], wait_for_stop=True)
                    else:
                        # pause at each position
                        for val2 in axis2_rng:
                            mv_axis2(val2, wait_for_stop=True)
                            time.sleep(pause_s)
                    axis2_rng = axis2_rng[::-1]  # reverse direction each time
                axis1_rng = axis1_rng[::-1]  # reverse direction each time
                i += 1
                if sleep_between is not None:
                    if self.verbose:
                        logger.info("Sleeping for %s s", sleep_between)
                    time.sleep(sleep_between)
        finally:
            self.halt()

        # home motors one at a time after normal completion
        self.az_target_steps(0, wait_for_stop=True)
        self.el_target_steps(0, wait_for_stop=True)
