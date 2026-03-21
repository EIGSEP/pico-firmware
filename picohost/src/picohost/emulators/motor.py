from .base import PicoEmulator

DEFAULT_DELAY_US = 600
SLOWDOWN_FACTOR = 2
SLOW_ZONE = 100


class StepperState:
    """Models the Stepper struct from motor.h."""

    def __init__(self):
        self.position = 0
        self.target_pos = 0
        self.dir = 0
        self.steps_in_direction = 0
        self.up_delay_us = DEFAULT_DELAY_US
        self.dn_delay_us = DEFAULT_DELAY_US
        self.slowdown_factor = SLOWDOWN_FACTOR
        self.slow_zone = SLOW_ZONE
        self.max_pulses = 60


def stepper_op(m):
    """Pure position model of stepper_op() from motor.c.

    Moves min(max_pulses, abs(remaining)) steps per call.
    No timing delays in emulation.
    """
    remaining = m.target_pos - m.position
    abs_steps = abs(remaining)
    nsteps = min(m.max_pulses, abs_steps)

    if remaining > 0:
        new_dir = 1
    elif remaining < 0:
        new_dir = -1
    else:
        new_dir = 0

    if new_dir != m.dir:
        m.steps_in_direction = 0
    m.dir = new_dir

    for _ in range(nsteps):
        m.position += m.dir

    m.steps_in_direction += nsteps


class MotorEmulator(PicoEmulator):
    """Emulates src/motor.c firmware."""

    def __init__(self, app_id=0, **kwargs):
        self.azimuth = StepperState()
        self.elevation = StepperState()
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.azimuth = StepperState()
        self.elevation = StepperState()

    def server(self, cmd):
        az = self.azimuth
        el = self.elevation

        # az_set_pos resets both position and target (matching C behavior)
        if "az_set_pos" in cmd:
            az.position = int(cmd["az_set_pos"])
            az.target_pos = az.position
        if "el_set_pos" in cmd:
            el.position = int(cmd["el_set_pos"])
            el.target_pos = el.position

        # target overrides (processed after set_pos, matching C order)
        if "az_set_target_pos" in cmd:
            az.target_pos = int(cmd["az_set_target_pos"])
        if "el_set_target_pos" in cmd:
            el.target_pos = int(cmd["el_set_target_pos"])

        # halt sets target = current position
        if "halt" in cmd:
            az.target_pos = az.position
            el.target_pos = el.position

        # delay settings
        if "az_up_delay_us" in cmd:
            az.up_delay_us = int(cmd["az_up_delay_us"])
        if "az_dn_delay_us" in cmd:
            az.dn_delay_us = int(cmd["az_dn_delay_us"])
        if "el_up_delay_us" in cmd:
            el.up_delay_us = int(cmd["el_up_delay_us"])
        if "el_dn_delay_us" in cmd:
            el.dn_delay_us = int(cmd["el_dn_delay_us"])

    def op(self):
        stepper_op(self.azimuth)
        stepper_op(self.elevation)

    def get_status(self):
        return {
            "sensor_name": "motor",
            "status": "update",
            "app_id": self.app_id,
            "az_pos": self.azimuth.position,
            "az_target_pos": self.azimuth.target_pos,
            "el_pos": self.elevation.position,
            "el_target_pos": self.elevation.target_pos,
        }
