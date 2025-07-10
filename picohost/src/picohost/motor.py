"""
Base class for Pico device communication.
Provides common functionality for serial communication with Pico devices.
"""

import json
import logging
import time
import queue
from typing import Dict, Any, Optional, Callable
from .base import PicoDevice, logger, redis_handler


class PicoMotor(PicoDevice):
    """Specialized class for motor control Pico devices."""

    def __init__(self, port, step_angle_deg=1.8, gear_teeth=113, microstep=1, verbose=False):
        super().__init__(port)
        self.status_queue = queue.Queue()
        self.set_response_handler(self.status_queue.put)
        self.step_angle_deg = step_angle
        self.gear_teeth = gear_teeth
        self.microstep = microstep
        self.commands = {
            'az_set_pos': int,
            'el_set_pos': int,
            'az_add_pulses': int,
            'el_add_pulses': int,
            'az_delay_us': int,
            'el_delay_us': int,
        }
        self.status = {}
        self.set_response_handler(self.update_status)
        self.set_delay()
        self.verbose = verbose

    def update_status(self, data):
        """Update internal status based on unpacked json packets from picos."""
        self.status.update(data)
        if self.verbose:
            print(json.dumps(data, indent=2, sort_keys=True))

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
        if az_pos != None:
            cmd['az_set_pos'] = az_pos
        if el_pos != None:
            cmd['el_set_pos'] = el_pos
        self.motor_command(**cmd)

    def reset_deg_position(self, az_deg=None, el_deg=None):
        """Set az and el position to specified count."""
        az_pos = az_deg is None ? None : self.steps_to_deg(az_deg)
        el_pos = el_deg is None ? None : self.steps_to_deg(el_deg)
        self.reset_step_pos(az_pos=az_pos, el_pos=el_pos)

    def set_delay(self, az_delay_us=2300, el_delay_us=2300):
        self.send_motor_command(az_delay_us=az_delay_us, el_delay_us=el_delay_us)

    def stop(self, az=True, el=True):
        """Hard stop on motors. Default: both."""
        cmd = {}
        if az:
            cmd['az_add_pulses'] = 0
        if el:
            cmd['el_add_pulses'] = 0
        self.motor_command(**cmd)

    def az_move_steps(self, delta_steps):
        self.send_motor_command(az_add_pulses=delta_steps)

    def az_move_deg(self, delta_deg):
        self.az_incmove_steps(self.deg_to_steps(delta_deg))

    def az_target_steps(self, target_steps):
        cur_steps = self.status['az_position'] + self.status['az_remaining_steps']
        self.az_move_steps(target_steps - cur_steps)

    def az_target_deg(self, target_deg):
        cur_steps = self.status['az_position'] + self.status['az_remaining_steps']
        target_steps = self.deg_to_steps(target_deg)
        self.az_move_steps(target_steps - cur_steps)

    def el_move_steps(self, delta_steps):
        self.send_motor_command(el_add_pulses=delta_steps)

    def el_move_deg(self, delta_deg):
        self.el_incmove_steps(self.deg_to_steps(delta_deg))

    def el_target_steps(self, target_steps):
        cur_steps = self.status['el_position'] + self.status['el_remaining_steps']
        self.el_move_steps(target_steps - cur_steps)

    def el_target_deg(self, target_deg):
        cur_steps = self.status['el_position'] + self.status['el_remaining_steps']
        target_steps = self.deg_to_steps(target_deg)
        self.el_move_steps(target_steps - cur_steps)

    def is_moving(self):
        return self.status['az_remaining_steps'] != 0 or \
               self.status['el_remaining_steps'] != 0

    def wait_for_stop(self):
        if self.verbose:
            print('Waiting for stop.')
        while self.is_moving():
            time.sleep(.1)
        
    def scan(self,
            az_range_deg=np.arange(-180.0, 180.0, 5),
            el_range_deg=np.arange(-180.0, 180.0, 5),
            az_first=True, repeat_count=1, pause_s=1):
        if az_first:
            mv_axis1, mv_axis2 = self.az_target_deg, self.el_target_deg
            axis1_rng, axis2_rng = az_range_deg, el_range_deg
        else:
            mv_axis2, mv_axis1 = self.az_target_deg, self.el_target_deg
            axis2_rng, axis1_rng = az_range_deg, el_range_deg

        i = 0
        try:
            while True:
                if repeat_count > 0 and i >= repeat_count:
                    break
                for val1 in axis1_rng:
                    mv_axis1(val1)
                    self.wait_for_stop()
                    for val2 in axis2_rng:
                        mv_axis2(val2)
                        self.wait_for_stop()
                        time.sleep(pause_s)
        except(KeyboardInterrupt):
            self.stop()
            

