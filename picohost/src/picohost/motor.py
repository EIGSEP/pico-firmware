"""
Base class for Pico device communication.
Provides common functionality for serial communication with Pico devices.
"""

import logging
import time
import numpy as np
from .base import PicoDevice

logger = logging.getLogger(__name__)


class PicoMotor(PicoDevice):
    """Specialized class for motor control Pico devices."""

    def __init__(
        self,
        port,
        step_angle_deg=1.8,
        gear_teeth=113,
        microstep=1,
        verbose=False,
        timeout=5.0,
        name=None,
        metadata_writer=None,
        usb_serial="",
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
        super().__init__(
            port,
            timeout=timeout,
            name=name,
            metadata_writer=metadata_writer,
            usb_serial=usb_serial,
            verbose=verbose,
        )
        self.set_delay()

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
        s = steps / self.microstep / self.gear_teeth
        deg = s * self.step_angle_deg
        return float(deg)

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
