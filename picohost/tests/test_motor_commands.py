"""
Tests for motor control commands.
"""

import pytest
from unittest.mock import Mock, patch
from picohost import PicoMotor


class TestPicoMotor:
    
    @patch('picohost.base.Serial')
    def test_motor_move_command(self, mock_serial):
        """Test motor move command generation."""
        mock_serial_instance = Mock()
        mock_serial.return_value = mock_serial_instance
        
        motor = PicoMotor('/dev/ttyACM0')
        motor.connect()
        
        # Test move command
        result = motor.move(pulses_az=100, pulses_el=200, delay_us_az=500, delay_us_el=700)
        
        # Verify command was sent
        assert result is True
        mock_serial_instance.write.assert_called_once()
        
        # Check the JSON command that was sent
        call_args = mock_serial_instance.write.call_args[0][0]
        command_str = call_args.decode('utf-8').strip()
        
        # Should contain the expected JSON structure
        assert '"pulses_az":100' in command_str
        assert '"pulses_el":200' in command_str
        assert '"delay_us_az":500' in command_str
        assert '"delay_us_el":700' in command_str
    
    @patch('picohost.base.Serial')
    def test_motor_move_defaults(self, mock_serial):
        """Test motor move with default delay values."""
        mock_serial_instance = Mock()
        mock_serial.return_value = mock_serial_instance
        
        motor = PicoMotor('/dev/ttyACM0')
        motor.connect()
        
        # Test move with defaults
        result = motor.move(pulses_az=50, pulses_el=75)
        
        assert result is True
        mock_serial_instance.write.assert_called_once()
        
        call_args = mock_serial_instance.write.call_args[0][0]
        command_str = call_args.decode('utf-8').strip()
        
        # Should use default delays (600)
        assert '"delay_us_az":600' in command_str
        assert '"delay_us_el":600' in command_str