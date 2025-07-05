"""
Tests for streaming data handling.
"""

import pytest
from unittest.mock import Mock, patch
from picohost import PicoDevice


class TestStreamingData:
    
    @patch('picohost.base.Serial')
    def test_read_latest_line_drains_buffer(self, mock_serial):
        """Test that read_latest_line drains the buffer and returns newest data."""
        mock_serial_instance = Mock()
        mock_serial.return_value = mock_serial_instance
        
        # Simulate multiple lines in buffer (old to new)
        mock_serial_instance.readline.side_effect = [
            b'{"old": "data1"}\n',
            b'{"old": "data2"}\n', 
            b'{"newest": "data3"}\n',
            b''  # End of buffer
        ]
        mock_serial_instance.timeout = 1.0
        
        device = PicoDevice('/dev/ttyACM0')
        device.connect()
        
        result = device.read_latest_line()
        
        # Should return the newest line
        assert result == '{"newest": "data3"}'
        
        # Should have called readline multiple times to drain buffer
        assert mock_serial_instance.readline.call_count == 4
    
    @patch('picohost.base.Serial')
    def test_get_latest_status_returns_newest_json(self, mock_serial):
        """Test that get_latest_status returns the most recent parsed JSON."""
        mock_serial_instance = Mock()
        mock_serial.return_value = mock_serial_instance
        
        # Simulate multiple status updates in buffer
        mock_serial_instance.readline.side_effect = [
            b'{"status": "old", "value": 1}\n',
            b'{"status": "newer", "value": 2}\n',
            b'{"status": "newest", "value": 3}\n',
            b''  # End of buffer
        ]
        mock_serial_instance.timeout = 1.0
        
        device = PicoDevice('/dev/ttyACM0')
        device.connect()
        
        result = device.get_latest_status()
        
        # Should return the newest parsed JSON
        assert result == {"status": "newest", "value": 3}
        
        # Should have drained the buffer
        assert mock_serial_instance.readline.call_count == 4
    
    @patch('picohost.base.Serial')
    def test_read_latest_line_handles_empty_buffer(self, mock_serial):
        """Test that read_latest_line handles empty buffer gracefully."""
        mock_serial_instance = Mock()
        mock_serial.return_value = mock_serial_instance
        
        # Empty buffer
        mock_serial_instance.readline.return_value = b''
        mock_serial_instance.timeout = 1.0
        
        device = PicoDevice('/dev/ttyACM0')
        device.connect()
        
        result = device.read_latest_line()
        
        assert result is None
        assert mock_serial_instance.readline.call_count == 1
    
    @patch('picohost.base.Serial')
    def test_read_latest_line_preserves_timeout(self, mock_serial):
        """Test that read_latest_line preserves original timeout."""
        mock_serial_instance = Mock()
        mock_serial.return_value = mock_serial_instance
        mock_serial_instance.timeout = 5.0  # Original timeout
        
        # Single line in buffer
        mock_serial_instance.readline.side_effect = [
            b'{"test": "data"}\n',
            b''  # End of buffer
        ]
        
        device = PicoDevice('/dev/ttyACM0')
        device.connect()
        
        device.read_latest_line()
        
        # Should restore original timeout
        assert mock_serial_instance.timeout == 5.0