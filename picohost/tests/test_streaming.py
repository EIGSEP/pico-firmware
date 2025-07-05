"""
Tests for streaming data handling.
"""

from unittest.mock import patch
from mockserial import MockSerial
from picohost import PicoDevice


class TestStreamingData:

    @patch("picohost.base.Serial")
    def test_read_line_basic(self, mock_serial):
        """Test basic read_line functionality."""
        mock_serial_instance = MockSerial()
        peer = MockSerial()
        mock_serial_instance.add_peer(peer)
        mock_serial.return_value = mock_serial_instance

        device = PicoDevice("/dev/ttyACM0")
        device.connect()

        # Simulate data coming from peer
        peer.write(b'{"test": "data"}\n')

        result = device.read_line()
        assert result == '{"test": "data"}'

    @patch("picohost.base.Serial")
    def test_parse_response_valid_json(self, mock_serial):
        """Test parsing valid JSON responses."""
        mock_serial_instance = MockSerial()
        mock_serial_instance.add_peer(MockSerial())
        mock_serial.return_value = mock_serial_instance

        device = PicoDevice("/dev/ttyACM0")
        device.connect()

        # Test valid JSON
        result = device.parse_response('{"status": "ok", "value": 123}')
        assert result == {"status": "ok", "value": 123}

    @patch("picohost.base.Serial")
    def test_parse_response_invalid_json(self, mock_serial):
        """Test parsing invalid JSON responses."""
        mock_serial_instance = MockSerial()
        mock_serial_instance.add_peer(MockSerial())
        mock_serial.return_value = mock_serial_instance

        device = PicoDevice("/dev/ttyACM0")
        device.connect()

        # Test invalid JSON
        result = device.parse_response("not json")
        assert result is None

    @patch("picohost.base.Serial")
    def test_read_line_handles_empty_data(self, mock_serial):
        """Test that read_line handles empty data gracefully."""
        mock_serial_instance = MockSerial()
        mock_serial_instance.add_peer(MockSerial())
        mock_serial_instance.timeout = 0.1
        mock_serial.return_value = mock_serial_instance

        device = PicoDevice("/dev/ttyACM0")
        device.connect()

        # No data written to peer, should return None
        result = device.read_line()
        assert result is None
