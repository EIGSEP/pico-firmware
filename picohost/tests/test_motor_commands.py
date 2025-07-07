"""
Tests for motor control commands.
"""

from unittest.mock import patch
from mockserial import MockSerial
from picohost import PicoMotor


class TestPicoMotor:

    @patch("picohost.base.Serial")
    def test_motor_move_command(self, mock_serial):
        """Test motor move command generation."""
        mock_serial_instance = MockSerial()
        mock_serial_instance.add_peer(MockSerial())  # Make it 'open'
        mock_serial.return_value = mock_serial_instance

        motor = PicoMotor("/dev/ttyACM0")
        motor.connect()

        # Test move command with degrees
        deg_az = 5.0
        deg_el = 10.0
        result = motor.move(
            deg_az=deg_az, deg_el=deg_el, delay_us_az=500, delay_us_el=700
        )

        # Verify command was sent
        assert result is True

        # Check the JSON command that was sent
        command_str = mock_serial_instance.peer._read_buffer.decode(
            "utf-8"
        ).strip()

        # Calculate expected pulses
        expected_pulses_az = motor.deg_to_pulses(deg_az)
        expected_pulses_el = motor.deg_to_pulses(deg_el)

        # Should contain the expected JSON structure
        assert f'"pulses_az":{expected_pulses_az}' in command_str
        assert f'"pulses_el":{expected_pulses_el}' in command_str
        assert '"delay_us_az":500' in command_str
        assert '"delay_us_el":700' in command_str

    @patch("picohost.base.Serial")
    def test_motor_move_defaults(self, mock_serial):
        """Test motor move with default delay values."""
        mock_serial_instance = MockSerial()
        mock_serial_instance.add_peer(MockSerial())  # Make it 'open'
        mock_serial.return_value = mock_serial_instance

        motor = PicoMotor("/dev/ttyACM0")
        motor.connect()

        # Test move with defaults
        deg_az = 3.0
        deg_el = 4.0
        result = motor.move(deg_az=deg_az, deg_el=deg_el)

        assert result is True

        command_str = mock_serial_instance.peer._read_buffer.decode(
            "utf-8"
        ).strip()

        # Calculate expected pulses
        expected_pulses_az = motor.deg_to_pulses(deg_az)
        expected_pulses_el = motor.deg_to_pulses(deg_el)

        # Should contain the correct pulses
        assert f'"pulses_az":{expected_pulses_az}' in command_str
        assert f'"pulses_el":{expected_pulses_el}' in command_str
        # Should use default delays (600)
        assert '"delay_us_az":600' in command_str
        assert '"delay_us_el":600' in command_str
