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

        # Test move command
        result = motor.move(
            pulses_az=100, pulses_el=200, delay_us_az=500, delay_us_el=700
        )

        # Verify command was sent
        assert result is True

        # Check the JSON command that was sent
        command_str = mock_serial_instance.peer._read_buffer.decode(
            "utf-8"
        ).strip()

        # Should contain the expected JSON structure
        assert '"pulses_az":100' in command_str
        assert '"pulses_el":200' in command_str
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
        result = motor.move(pulses_az=50, pulses_el=75)

        assert result is True

        command_str = mock_serial_instance.peer._read_buffer.decode(
            "utf-8"
        ).strip()

        # Should use default delays (600)
        assert '"delay_us_az":600' in command_str
        assert '"delay_us_el":600' in command_str
