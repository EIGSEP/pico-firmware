"""
Test suite for dummy Pico device classes using mockserial.

This module tests the DummyPicoDevice, DummyPicoMotor, DummyPicoRFSwitch, 
and DummyPicoPeltier classes to ensure they properly simulate real device behavior.
"""

import json
import time
import unittest
from unittest.mock import patch
import mockserial

from picohost.testing import (
    DummyPicoDevice, 
    DummyPicoMotor, 
    DummyPicoRFSwitch, 
    DummyPicoPeltier
)


class TestDummyPicoDevice(unittest.TestCase):
    """Test base DummyPicoDevice functionality."""
    
    def setUp(self):
        """Set up test device."""
        self.device = DummyPicoDevice(port="/dev/ttyUSB0")
    
    def tearDown(self):
        """Clean up after tests."""
        if self.device.ser:
            self.device.disconnect()
    
    def test_connect(self):
        """Test device connection."""
        result = self.device.connect()
        self.assertTrue(result)
        self.assertIsInstance(self.device.ser, mockserial.MockSerial)
        self.assertTrue(self.device.ser.is_open)
    
    def test_disconnect(self):
        """Test device disconnection."""
        self.device.connect()
        self.device.disconnect()
        self.assertIsNone(self.device.ser)
    
    def test_send_command(self):
        """Test sending JSON commands."""
        self.device.connect()
        
        # Test successful command
        cmd = {"cmd": "test", "value": 123}
        result = self.device.send_command(cmd)
        self.assertTrue(result)
        
        # Check data was written to mock serial peer
        expected = json.dumps(cmd, separators=(",", ":")) + "\n"
        self.assertEqual(self.device.ser.peer.read(len(expected)), expected.encode())
    
    def test_send_command_no_connection(self):
        """Test sending command without connection."""
        cmd = {"cmd": "test"}
        self.device.disconnect()  # Ensure disconnected
        result = self.device.send_command(cmd)
        self.assertFalse(result)
    
    def test_read_line(self):
        """Test reading lines from mock serial."""
        self.device.connect()
        
        # Write test data to mock serial input
        test_data = "test line\n"
        self.device.ser.peer.write(test_data.encode())
        
        # Read the line back
        line = self.device.read_line()
        self.assertEqual(line, "test line")
    
    def test_parse_response(self):
        """Test JSON response parsing."""
        # Valid JSON
        json_str = '{"status": "ok", "data": 123}'
        result = self.device.parse_response(json_str)
        expected = {"status": "ok", "data": 123}
        self.assertEqual(result, expected)
        
        # Invalid JSON
        result = self.device.parse_response("not json")
        self.assertIsNone(result)
    
    def test_context_manager(self):
        """Test using device as context manager."""
        with self.device as dev:
            self.assertIsInstance(dev.ser, mockserial.MockSerial)
            self.assertTrue(dev.ser.is_open)
        
        # Should be disconnected after context
        self.assertIsNone(self.device.ser)
    
    def test_context_manager_connection_failure(self):
        """Test context manager with connection failure."""
        # Mock connect to fail
        with patch.object(self.device, 'connect', return_value=False):
            with self.assertRaises(RuntimeError):
                with self.device:
                    pass


class TestDummyPicoMotor(unittest.TestCase):
    """Test DummyPicoMotor functionality."""
    
    def setUp(self):
        """Set up test motor device."""
        self.motor = DummyPicoMotor(port="/dev/ttyUSB0")
        self.motor.connect()
    
    def tearDown(self):
        """Clean up after tests."""
        self.motor.disconnect()
    
    def test_deg_to_pulses(self):
        """Test degree to pulse conversion."""
        # Test known values
        self.assertEqual(DummyPicoMotor.deg_to_pulses(0), 0)
        self.assertEqual(DummyPicoMotor.deg_to_pulses(1.8), 113)  # One step
        self.assertEqual(DummyPicoMotor.deg_to_pulses(360), 22600)  # Full rotation
    
    def test_move_command(self):
        """Test motor movement command."""
        result = self.motor.move(deg_az=10.0, deg_el=5.0, delay_us_az=500, delay_us_el=700)
        self.assertTrue(result)
        
        # Check the command was formatted correctly
        expected_cmd = {
            "pulses_az": self.motor.deg_to_pulses(10.0),
            "pulses_el": self.motor.deg_to_pulses(5.0),
            "delay_us_az": 500,
            "delay_us_el": 700
        }
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.motor.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())
    
    def test_move_default_values(self):
        """Test motor movement with default values."""
        result = self.motor.move()
        self.assertTrue(result)
        
        # Should send zero movement with default delays
        expected_cmd = {
            "pulses_az": 0,
            "pulses_el": 0,
            "delay_us_az": 600,
            "delay_us_el": 600
        }
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.motor.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())


class TestDummyPicoRFSwitch(unittest.TestCase):
    """Test DummyPicoRFSwitch functionality."""
    
    def setUp(self):
        """Set up test RF switch device."""
        self.switch = DummyPicoRFSwitch(port="/dev/ttyUSB0")
        self.switch.connect()
    
    def tearDown(self):
        """Clean up after tests."""
        self.switch.disconnect()
    
    def test_rbin_function(self):
        """Test reverse binary conversion function."""
        # Test known conversions
        self.assertEqual(DummyPicoRFSwitch.rbin("10000000"), 1)  # LSB first
        self.assertEqual(DummyPicoRFSwitch.rbin("01000000"), 2)
        self.assertEqual(DummyPicoRFSwitch.rbin("11000000"), 3)
        self.assertEqual(DummyPicoRFSwitch.rbin("00100000"), 4)
    
    def test_paths_property(self):
        """Test that paths are properly converted."""
        paths = self.switch.paths
        self.assertIsInstance(paths, dict)
        self.assertIn("VNAO", paths)
        self.assertIn("RFANT", paths)
        
        # Check specific conversions
        self.assertEqual(paths["VNAO"], 1)  # "10000000" reversed = 1
        self.assertEqual(paths["RFANT"], 0)  # "00000000" = 0
    
    def test_switch_valid_state(self):
        """Test switching to valid states."""
        for state in self.switch.paths.keys():
            result = self.switch.switch(state)
            self.assertTrue(result, f"Failed to switch to state: {state}")
            
            # Verify correct command was sent
            expected_cmd = {"sw_state": self.switch.paths[state]}
            expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
            sent_data = self.switch.ser.peer.read(len(expected_json))
            self.assertEqual(sent_data, expected_json.encode())
    
    def test_switch_invalid_state(self):
        """Test switching to invalid state raises ValueError."""
        with self.assertRaises(ValueError) as context:
            self.switch.switch("INVALID_STATE")
        
        self.assertIn("Invalid switch state", str(context.exception))
        self.assertIn("INVALID_STATE", str(context.exception))


class TestDummyPicoPeltier(unittest.TestCase):
    """Test DummyPicoPeltier functionality."""
    
    def setUp(self):
        """Set up test Peltier device."""
        self.peltier = DummyPicoPeltier(port="/dev/ttyUSB0")
        self.peltier.connect()
    
    def tearDown(self):
        """Clean up after tests."""
        self.peltier.disconnect()
    
    def test_set_temperature(self):
        """Test setting target temperature."""
        result = self.peltier.set_temperature(25.5, channel=1)
        self.assertTrue(result)
        
        expected_cmd = {"cmd": "set_temp", "temperature": 25.5, "channel": 1}
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.peltier.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())
    
    def test_set_temperature_default_channel(self):
        """Test setting temperature with default channel."""
        result = self.peltier.set_temperature(30.0)
        self.assertTrue(result)
        
        expected_cmd = {"cmd": "set_temp", "temperature": 30.0, "channel": 0}
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.peltier.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())
    
    def test_set_hysteresis(self):
        """Test setting hysteresis value."""
        result = self.peltier.set_hysteresis(2.0, channel=2)
        self.assertTrue(result)
        
        expected_cmd = {"cmd": "set_hysteresis", "hysteresis": 2.0, "channel": 2}
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.peltier.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())
    
    def test_enable(self):
        """Test enabling temperature control."""
        result = self.peltier.enable(channel=1)
        self.assertTrue(result)
        
        expected_cmd = {"cmd": "enable", "channel": 1}
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.peltier.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())
    
    def test_disable(self):
        """Test disabling temperature control."""
        result = self.peltier.disable(channel=2)
        self.assertTrue(result)
        
        expected_cmd = {"cmd": "disable", "channel": 2}
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.peltier.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())
    
    def test_enable_disable_default_channel(self):
        """Test enable/disable with default channel."""
        # Test enable
        result = self.peltier.enable()
        self.assertTrue(result)
        
        expected_cmd = {"cmd": "enable", "channel": 0}
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.peltier.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())
        
        # Test disable
        result = self.peltier.disable()
        self.assertTrue(result)
        
        expected_cmd = {"cmd": "disable", "channel": 0}
        expected_json = json.dumps(expected_cmd, separators=(",", ":")) + "\n"
        sent_data = self.peltier.ser.peer.read(len(expected_json))
        self.assertEqual(sent_data, expected_json.encode())


class TestMockSerialIntegration(unittest.TestCase):
    """Test integration with mockserial library features."""
    
    def test_mock_serial_read_write(self):
        """Test basic mockserial read/write functionality."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        device.connect()
        
        # Test writing and reading back
        test_data = b"hello world\n"
        device.ser.write(test_data)
        read_data = device.ser.peer.read(len(test_data))
        self.assertEqual(read_data, test_data)
        
        device.disconnect()
    
    def test_mock_serial_readline(self):
        """Test mockserial readline functionality."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        device.connect()
        
        # Write data to be read back
        lines = [b"line1\n", b"line2\n", b"line3\n"]
        for line in lines:
            device.ser.peer.write(line)
        
        # Read lines back
        for expected_line in lines:
            read_line = device.ser.readline()
            self.assertEqual(read_line, expected_line)
        
        device.disconnect()
    
    def test_mock_serial_properties(self):
        """Test mockserial properties and state."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        device.connect()
        
        # Test basic properties
        self.assertTrue(device.ser.is_open)
        # Note: MockSerial timeout is None by default, unlike real Serial
        
        # Test buffer operations
        device.ser.reset_input_buffer()
        self.assertEqual(device.ser.in_waiting(), 0)
        
        device.disconnect()


if __name__ == "__main__":
    unittest.main(verbosity=2)
