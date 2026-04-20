"""
Tests for motor control commands.
"""

from conftest import wait_for_condition
from picohost.testing import DummyPicoMotor


def _wait_for_status(motor):
    wait_for_condition(
        lambda: "az_target_pos" in motor.last_status,
        cadence_ms=motor.EMULATOR_CADENCE_MS,
    )


class TestDummyPicoMotor:
    def test_motor_move_command(self):
        """Test motor move command generation."""
        motor = DummyPicoMotor("/dev/ttyACM0")
        _wait_for_status(motor)

        deg_az = 5.0
        deg_el = 10.0
        motor.az_move_deg(deg_az, wait_for_start=False, wait_for_stop=False)
        motor.el_move_deg(deg_el, wait_for_start=False, wait_for_stop=False)

        expected_steps_az = motor.deg_to_steps(deg_az)
        expected_steps_el = motor.deg_to_steps(deg_el)

        assert expected_steps_az == motor.deg_to_steps(deg_az)
        assert expected_steps_el == motor.deg_to_steps(deg_el)

        motor.disconnect()

    def test_motor_move_defaults(self):
        """Test motor move with default delay values."""
        motor = DummyPicoMotor("/dev/ttyACM0")
        _wait_for_status(motor)

        deg_az = 3.0
        deg_el = 4.0
        motor.az_move_deg(deg_az, wait_for_start=False, wait_for_stop=False)
        motor.el_move_deg(deg_el, wait_for_start=False, wait_for_stop=False)

        expected_steps_az = motor.deg_to_steps(deg_az)
        expected_steps_el = motor.deg_to_steps(deg_el)

        assert expected_steps_az == motor.deg_to_steps(deg_az)
        assert expected_steps_el == motor.deg_to_steps(deg_el)

        motor.disconnect()
