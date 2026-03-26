"""
Tests for streaming data handling.

Uses DummyPicoDevice (MockSerial pair) instead of monkeypatching Serial,
so the tests exercise the same code paths as real devices.
"""

from conftest import wait_for_condition
from picohost.testing import DummyPicoDevice


class TestStreamingData:
    """Test that the reader thread correctly processes streaming JSON data."""

    def test_read_line_returns_decoded_string(self):
        """read_line() returns a stripped UTF-8 string from the peer."""
        device = DummyPicoDevice("/dev/dummy")
        # Stop reader thread so it doesn't consume our data
        device.stop()

        device.ser.peer.write(b'{"test": "data"}\n')
        result = device.read_line()
        assert result == '{"test": "data"}'
        device.disconnect()

    def test_reader_thread_parses_json_into_last_status(self):
        """The background reader thread parses JSON and stores it in last_status."""
        device = DummyPicoDevice("/dev/dummy")
        device.ser.peer.write(b'{"status": "ok", "value": 123}\n')
        wait_for_condition(
            lambda: len(device.last_status) > 0,
            cadence_ms=device.EMULATOR_CADENCE_MS,
        )
        assert device.last_status == {"status": "ok", "value": 123}
        device.disconnect()

    def test_parse_response_valid_json(self):
        """parse_response() returns a dict for valid JSON."""
        device = DummyPicoDevice("/dev/dummy")
        result = device.parse_response('{"status": "ok", "value": 123}')
        assert result == {"status": "ok", "value": 123}
        device.disconnect()

    def test_parse_response_invalid_json(self):
        """parse_response() returns None for malformed input."""
        device = DummyPicoDevice("/dev/dummy")
        assert device.parse_response("not json") is None
        device.disconnect()

    def test_read_line_returns_none_on_empty_buffer(self):
        """read_line() returns None when no data is available (timeout)."""
        device = DummyPicoDevice("/dev/dummy")
        device.stop()
        result = device.read_line()
        assert result is None
        device.disconnect()

    def test_multiple_status_updates_overwrites_last_status(self):
        """Each new JSON line from the peer overwrites last_status."""
        device = DummyPicoDevice("/dev/dummy")
        device.ser.peer.write(b'{"sensor_name":"first","v":1}\n')
        wait_for_condition(
            lambda: device.last_status.get("sensor_name") == "first",
            cadence_ms=device.EMULATOR_CADENCE_MS,
        )
        assert device.last_status["v"] == 1

        device.ser.peer.write(b'{"sensor_name":"second","v":2}\n')
        wait_for_condition(
            lambda: device.last_status.get("sensor_name") == "second",
            cadence_ms=device.EMULATOR_CADENCE_MS,
        )
        assert device.last_status["v"] == 2
        device.disconnect()
