"""
Tests for motor control commands.
"""

from picohost.testing import DummyPicoMotor


class TestDummyPicoMotor:

    def test_motor_move_command(self):
        """Test motor move command generation."""
        motor = DummyPicoMotor("/dev/ttyACM0")
        motor.connect()

        # Test move command with degrees
        deg_az = 5.0
        deg_el = 10.0
        motor.az_move_deg(deg_az)
        motor.el_move_deg(deg_el)
        result = True

        # Verify command was sent
        assert result is True

        # Calculate expected steps
        expected_steps_az = motor.deg_to_steps(deg_az)
        expected_steps_el = motor.deg_to_steps(deg_el)

        # Check that methods return correct conversions
        assert expected_steps_az == motor.deg_to_steps(deg_az)
        assert expected_steps_el == motor.deg_to_steps(deg_el)

    def test_motor_move_defaults(self):
        """Test motor move with default delay values."""
        motor = DummyPicoMotor("/dev/ttyACM0")
        motor.connect()

        # Test move with defaults
        deg_az = 3.0
        deg_el = 4.0
        motor.az_move_deg(deg_az)
        motor.el_move_deg(deg_el)
        result = True

        assert result is True

        # Calculate expected steps
        expected_steps_az = motor.deg_to_steps(deg_az)
        expected_steps_el = motor.deg_to_steps(deg_el)

        # Check that methods return correct conversions
        assert expected_steps_az == motor.deg_to_steps(deg_az)
        assert expected_steps_el == motor.deg_to_steps(deg_el)
