"""
Base class for Pico device communication.
Provides common functionality for serial communication with Pico devices.
"""

import json
import logging
import time
import queue
import numpy as np
from typing import Dict, Any, Optional, Callable
from .base import PicoDevice, logger, redis_handler


class PicoMotor(PicoDevice):
    """Specialized class for motor control Pico devices."""

    def __init__(self, port, step_angle_deg=1.8, gear_teeth=113, microstep=1, verbose=False):
        super().__init__(port)
        self.verbose = verbose
        self.step_angle_deg = step_angle_deg
        self.gear_teeth = gear_teeth
        self.microstep = microstep
        self.commands = {
            'az_set_pos': int,
            'el_set_pos': int,
            'az_set_target_pos': int,
            'el_set_target_pos': int,
            'halt': int,
            'az_delay_us': int,
            'el_delay_us': int,
        }
        self.status = {}
        self.set_response_handler(self.update_status)
        self.set_delay()
        self.wait_for_updates()

    def update_status(self, data):
        """Update internal status based on unpacked json packets from picos."""
        if self.verbose:
            print(json.dumps(data, indent=2, sort_keys=True))
        self.status.update(data)

    def wait_for_updates(self, timeout=10):
        t = time.time()
        while True:
            if len(self.status) != 0:
                break
            assert time.time() - t < timeout
            time.sleep(0.1)

    def deg_to_steps(self, degrees: float) -> int:
        """Convert degrees to motor pulses."""
        s = degrees / self.step_angle_deg
        return int(s * self.microstep * self.gear_teeth)

    def steps_to_deg(self, steps: int) -> float:
        """Convert degrees to motor pulses."""
        s = steps / self.microstep / self.gear_teeth
        deg = s * self.step_angle_deg
        return float(deg)

    def motor_command(self, **kwargs):
        """Send a json motor command with specified keys."""
        # check commands
        cmd = {}
        for k, v in kwargs.items():
            if not k in self.commands:
                raise ValueError(f"command {k} not in {self.commands}")
            cmd[k] = self.commands[k](v)
        self.send_command(cmd)
        
    def reset_step_position(self, az_pos=None, el_pos=None):
        """Set az and el position to specified count."""
        cmd = {}
        if az_pos is not None:
            cmd['az_set_pos'] = az_pos
        if el_pos is not None:
            cmd['el_set_pos'] = el_pos
        self.motor_command(**cmd)

    def reset_deg_position(self, az_deg=None, el_deg=None):
        """Set az and el position to specified count."""
        az_pos = None if az_deg is None else self.deg_to_steps(az_deg)
        el_pos = None if el_deg is None else self.deg_to_steps(el_deg)
        self.reset_step_position(az_pos=az_pos, el_pos=el_pos)

    def set_delay(self, az_delay_us=2300, el_delay_us=2300):
        self.motor_command(az_delay_us=az_delay_us, el_delay_us=el_delay_us)

    def stop(self, az=True, el=True):
        """Hard stop on motors. Default: both."""
        cmd = {'halt': 0}
        self.motor_command(**cmd)

    def _do_wait(self, wait_for_start, wait_for_stop):
        if wait_for_start:
            self.wait_for_start()
        if wait_for_stop:
            self.wait_for_stop()

    def az_target_steps(self, target_steps, wait_for_start=True, wait_for_stop=False):
        """Move az to target step position."""
        self.motor_command(az_set_target_pos=target_steps)
        self._do_wait(wait_for_start, wait_for_stop)

    def az_target_deg(self, target_deg, wait_for_start=True, wait_for_stop=False):
        """Move az to target deg position."""
        self.az_target_steps(self.deg_to_steps(target_deg),
                wait_for_start=wait_for_start, wait_for_stop=wait_for_stop)

    def az_move_steps(self, delta_steps, wait_for_start=True, wait_for_stop=False):
        """Move az in specified number of steps from current target."""
        new_target = self.status['az_target_pos'] + delta_steps
        self.az_target_steps(new_target,
                wait_for_start=wait_for_start, wait_for_stop=wait_for_stop)

    def az_move_deg(self, delta_deg, wait_for_start=True, wait_for_stop=False):
        """Move az in specified number of degs from current target."""
        self.az_move_steps(self.deg_to_steps(delta_deg),
                wait_for_start=wait_for_start, wait_for_stop=wait_for_stop)

    def el_target_steps(self, target_steps, wait_for_start=True, wait_for_stop=False):
        """Move el to target step position."""
        self.motor_command(el_set_target_pos=target_steps)
        self._do_wait(wait_for_start, wait_for_stop)

    def el_target_deg(self, target_deg, wait_for_start=True, wait_for_stop=False):
        """Move el to target deg position."""
        self.el_target_steps(self.deg_to_steps(target_deg),
                wait_for_start=wait_for_start, wait_for_stop=wait_for_stop)

    def el_move_steps(self, delta_steps, wait_for_start=True, wait_for_stop=False):
        """Move el in specified number of steps from current target."""
        new_target = self.status['el_target_pos'] + delta_steps
        self.el_target_steps(new_target,
                wait_for_start=wait_for_start, wait_for_stop=wait_for_stop)

    def el_move_deg(self, delta_deg, wait_for_start=True, wait_for_stop=False):
        """Move el in specified number of degs from current target."""
        self.el_move_steps(self.deg_to_steps(delta_deg),
                wait_for_start=wait_for_start, wait_for_stop=wait_for_stop)

    def is_moving(self):
        return self.status['az_target_pos'] != self.status['az_pos'] or \
               self.status['el_target_pos'] != self.status['el_pos']

    def wait_for_start(self):
        while not self.is_moving():
            time.sleep(.1)

    def wait_for_stop(self):
        if self.verbose:
            print('Waiting for stop.')
        while self.is_moving():
            time.sleep(.1)
        
    def scan(self,
            az_range_deg=np.arange(-180.0, 180.0, 5),
            el_range_deg=np.arange(-180.0, 180.0, 5),
            el_first=False, repeat_count=None, pause_s=None, reset_pos=False):
        """Perform beam scanning strategy."""
        if reset_pos:
            self.reset_deg_position(az_deg=0.0, el_deg=0.0)
        # set order of scanning
        if el_first:
            mv_axis1, mv_axis2 = self.az_target_deg, self.el_target_deg
            axis1_rng, axis2_rng = az_range_deg, el_range_deg
        else:
            mv_axis2, mv_axis1 = self.az_target_deg, self.el_target_deg
            axis2_rng, axis1_rng = az_range_deg, el_range_deg

        i = 0
        try:
            while True:
                if repeat_count is not None and i >= repeat_count:
                    break
                for val1 in axis1_rng:
                    mv_axis1(val1)
                    self.wait_for_stop()
                    if pause_s is None:
                        # continuous motion
                        mv_axis2(axis2_rng[0])
                        self.wait_for_stop()
                        mv_axis2(axis2_rng[-1])
                        self.wait_for_stop()
                    else:
                        # pause at each position
                        for val2 in axis2_rng:
                            mv_axis2(val2)
                            self.wait_for_stop()
                            time.sleep(pause_s)
        finally:
            self.stop()
            

