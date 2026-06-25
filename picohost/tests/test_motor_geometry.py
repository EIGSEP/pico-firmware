"""Tests for the shared motor step<->degree geometry helper."""

import types

from picohost.motor import PicoMotor, steps_to_deg


def test_steps_to_deg_known_values():
    # (steps / microstep / gear_teeth) * step_angle_deg
    # 226 steps, 113 teeth, microstep 1, 1.8 deg/step -> 2 * 1.8 = 3.6
    assert steps_to_deg(
        226, step_angle_deg=1.8, gear_teeth=113, microstep=1
    ) == 3.6
    # one full az turn: 22600 steps -> 360 deg
    assert steps_to_deg(
        22600, step_angle_deg=1.8, gear_teeth=113, microstep=1
    ) == 360.0


def test_steps_to_deg_microstep_scaling():
    # doubling microstep halves degrees per step
    assert steps_to_deg(
        452, step_angle_deg=1.8, gear_teeth=113, microstep=2
    ) == 3.6


def test_method_delegates_to_helper():
    # Duck-typed self avoids opening a serial port; the method must
    # produce exactly what the module function does for the same config.
    fake = types.SimpleNamespace(
        step_angle_deg=1.8, gear_teeth=113, microstep=1
    )
    assert PicoMotor.steps_to_deg(fake, 226) == steps_to_deg(
        226, step_angle_deg=1.8, gear_teeth=113, microstep=1
    )
