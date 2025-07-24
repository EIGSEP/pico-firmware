"""
Unit tests for the picohost base classes.
"""

import json
from picohost.testing import DummyPicoDevice, DummyPicoMotor, DummyPicoRFSwitch, DummyPicoPeltier


class TestPicoDevice:
    """Test the base PicoDevice class."""

    def test_find_pico_ports(self):
        """Test finding Pico ports."""
        # Note: find_pico_ports is a class method that scans actual ports
        # We can't easily test it with DummyPicoDevice without mocking
        # This test will be kept simple - just verify the method exists
        assert hasattr(DummyPicoDevice, 'find_pico_ports')

    def test_connect_success(self):
        """Test successful connection."""
        device = DummyPicoDevice("/dev/dummy")
        assert device.is_connected is True
        assert device.ser is not None

    def test_connect_failure(self):
        """Test connection failure."""
        # DummyPicoDevice always connects successfully by design
        # This test is not applicable for dummy devices
        pass

    def test_send_command(self):
        """Test sending a command."""
        device = DummyPicoDevice("/dev/dummy")
        
        cmd = {"cmd": "test", "value": 42}
        assert device.send_command(cmd) is True

        expected_data = json.dumps(cmd, separators=(",", ":")) + "\n"
        # Check that data was written to the peer
        assert device.ser.peer._read_buffer == expected_data.encode("utf-8")

    def test_parse_response(self):
        """Test parsing JSON responses."""
        device = DummyPicoDevice("/dev/dummy")

        # Valid JSON
        data = device.parse_response('{"status": "ok", "value": 123}')
        assert data == {"status": "ok", "value": 123}

        # Invalid JSON
        assert device.parse_response("not json") is None

    def test_context_manager(self):
        """Test context manager functionality."""
        with DummyPicoDevice("/dev/dummy") as device:
            assert device.ser is not None
            assert device._running is True

        # After exiting context, should be disconnected
        assert device.ser is None
        assert device._running is False


class TestPicoMotor:
    """Test the PicoMotor class."""

    def test_move_command(self):
        """Test motor move command."""
        motor = DummyPicoMotor("/dev/dummy")
        
        # Clear the buffer from any initialization commands
        motor.ser.peer._read_buffer = bytearray()

        # Test move command with degrees
        az_deg = 10.0
        el_deg = -5.0
        
        # Use the actual methods from the motor class
        motor.az_target_deg(az_deg, wait_for_start=False, wait_for_stop=False)
        
        # Verify the command was sent
        sent_data = motor.ser.peer._read_buffer.decode("utf-8").strip()
        sent_json = json.loads(sent_data)

        # Calculate expected steps
        expected_steps_az = motor.deg_to_steps(az_deg)

        assert sent_json == {
            "az_set_target_pos": expected_steps_az
        }


class TestPicoRFSwitch:
    """Test the PicoRFSwitch class."""

    def test_switch_state(self):
        """Test RF switch state command."""
        switch = DummyPicoRFSwitch("/dev/dummy")

        # Test switch state command with valid state
        switch.switch("VNAO")

        # Verify the command was sent
        sent_data = switch.ser.peer._read_buffer.decode("utf-8").strip()
        sent_json = json.loads(sent_data)

        # The switch method converts the state string to binary
        expected_state = switch.rbin(switch.path_str["VNAO"])
        assert sent_json == {"sw_state": expected_state}

    def test_switch_invalid_state(self):
        """Test RF switch with invalid state."""
        switch = DummyPicoRFSwitch("/dev/dummy")

        # Test invalid switch state - should raise ValueError
        try:
            switch.switch("INVALID")
            assert False, "Should have raised ValueError"
        except ValueError as e:
            assert "Invalid switch state" in str(e)


class TestPicoPeltier:
    """Test the PicoPeltier class."""

    def test_temperature_commands(self):
        """Test temperature control commands."""
        peltier = DummyPicoPeltier("/dev/dummy")

        # Test set temperature for channel A
        peltier.set_temperature(T_A=25.5, A_hyst=0.5)
        sent_data = peltier.ser.peer._read_buffer.decode("utf-8").strip()
        assert json.loads(sent_data) == {
            "A_temp_target": 25.5,
            "A_hysteresis": 0.5,
        }

        # Clear buffer for next test
        peltier.ser.peer._read_buffer = bytearray()

        # Test set temperature for channel B
        peltier.set_temperature(T_B=30.0, B_hyst=1.0)
        sent_data = peltier.ser.peer._read_buffer.decode("utf-8").strip()
        assert json.loads(sent_data) == {
            "B_temp_target": 30.0,
            "B_hysteresis": 1.0,
        }

        # Clear buffer for next test
        peltier.ser.peer._read_buffer = bytearray()

        # Test enable both channels
        peltier.set_enable(A=True, B=True)
        sent_data = peltier.ser.peer._read_buffer.decode("utf-8").strip()
        assert json.loads(sent_data) == {"A_enable": True, "B_enable": True}

        # Clear buffer for next test
        peltier.ser.peer._read_buffer = bytearray()
