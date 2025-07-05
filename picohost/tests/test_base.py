"""
Unit tests for the picohost base classes.
"""

import json
import pytest
from unittest.mock import Mock, patch, MagicMock
from picohost import PicoDevice, PicoMotor, PicoRFSwitch, PicoPeltier


class TestPicoDevice:
    """Test the base PicoDevice class."""
    
    def test_find_pico_ports(self):
        """Test finding Pico ports."""
        # Mock serial port info
        mock_port = Mock()
        mock_port.vid = 0x2E8A
        mock_port.pid = 0x0009
        mock_port.device = '/dev/ttyACM0'
        
        with patch('serial.tools.list_ports.comports', return_value=[mock_port]):
            ports = PicoDevice.find_pico_ports()
            assert ports == ['/dev/ttyACM0']
    
    def test_connect_success(self):
        """Test successful connection."""
        with patch('serial.Serial') as mock_serial:
            device = PicoDevice('/dev/ttyACM0')
            assert device.connect() is True
            mock_serial.assert_called_once_with('/dev/ttyACM0', 115200, timeout=1.0)
    
    def test_connect_failure(self):
        """Test connection failure."""
        with patch('serial.Serial', side_effect=Exception("Connection failed")):
            device = PicoDevice('/dev/ttyACM0')
            assert device.connect() is False
    
    def test_send_command(self):
        """Test sending a command."""
        mock_serial = MagicMock()
        device = PicoDevice('/dev/ttyACM0')
        device.ser = mock_serial
        
        cmd = {"cmd": "test", "value": 42}
        assert device.send_command(cmd) is True
        
        expected_data = json.dumps(cmd, separators=(',', ':')) + '\n'
        mock_serial.write.assert_called_once_with(expected_data.encode('utf-8'))
        mock_serial.flush.assert_called_once()
    
    def test_parse_response(self):
        """Test parsing JSON responses."""
        device = PicoDevice('/dev/ttyACM0')
        
        # Valid JSON
        data = device.parse_response('{"status": "ok", "value": 123}')
        assert data == {"status": "ok", "value": 123}
        
        # Invalid JSON
        assert device.parse_response('not json') is None
    
    def test_context_manager(self):
        """Test context manager functionality."""
        with patch('serial.Serial'):
            with PicoDevice('/dev/ttyACM0') as device:
                assert device.ser is not None
                assert device._running is True
            
            # After exiting context, should be disconnected
            assert device.ser is None
            assert device._running is False


class TestPicoMotor:
    """Test the PicoMotor class."""
    
    def test_move_command(self):
        """Test motor move command."""
        motor = PicoMotor('/dev/ttyACM0')
        motor.ser = MagicMock()
        
        # Test move command
        motor.move(1000, -500, 600, 800)
        
        # Verify the command was sent
        motor.ser.write.assert_called_once()
        sent_data = motor.ser.write.call_args[0][0].decode('utf-8').strip()
        sent_json = json.loads(sent_data)
        
        assert sent_json == {
            "pulses_az": 1000,
            "pulses_el": -500,
            "delay_us_az": 600,
            "delay_us_el": 800
        }


class TestPicoRFSwitch:
    """Test the PicoRFSwitch class."""
    
    def test_set_switch_state(self):
        """Test RF switch state command."""
        switch = PicoRFSwitch('/dev/ttyACM0')
        switch.ser = MagicMock()
        
        # Test switch state command
        switch.set_switch_state(5)
        
        # Verify the command was sent
        switch.ser.write.assert_called_once()
        sent_data = switch.ser.write.call_args[0][0].decode('utf-8').strip()
        sent_json = json.loads(sent_data)
        
        assert sent_json == {"sw_state": 5}


class TestPicoPeltier:
    """Test the PicoPeltier class."""
    
    def test_temperature_commands(self):
        """Test temperature control commands."""
        peltier = PicoPeltier('/dev/ttyACM0')
        peltier.ser = MagicMock()
        
        # Test set temperature
        peltier.set_temperature(25.5, channel=1)
        sent_data = peltier.ser.write.call_args[0][0].decode('utf-8').strip()
        assert json.loads(sent_data) == {
            "cmd": "set_temp",
            "temperature": 25.5,
            "channel": 1
        }
        
        # Test enable
        peltier.ser.reset_mock()
        peltier.enable(channel=2)
        sent_data = peltier.ser.write.call_args[0][0].decode('utf-8').strip()
        assert json.loads(sent_data) == {
            "cmd": "enable",
            "channel": 2
        }
        
        # Test disable
        peltier.ser.reset_mock()
        peltier.disable(channel=0)
        sent_data = peltier.ser.write.call_args[0][0].decode('utf-8').strip()
        assert json.loads(sent_data) == {
            "cmd": "disable",
            "channel": 0
        }
        
        # Test set hysteresis
        peltier.ser.reset_mock()
        peltier.set_hysteresis(0.5, channel=1)
        sent_data = peltier.ser.write.call_args[0][0].decode('utf-8').strip()
        assert json.loads(sent_data) == {
            "cmd": "set_hysteresis",
            "hysteresis": 0.5,
            "channel": 1
        }