"""
Base class for Pico device communication.
Provides common functionality for serial communication with Pico devices.
"""

import json
import logging
import threading
import time
from typing import Dict, Any, Optional, Callable
from serial import Serial
from serial.tools import list_ports

logger = logging.getLogger(__name__)

# USB IDs for Raspberry Pi Pico
PICO_VID = 0x2E8A
PICO_PID_CDC = 0x0009  # CDC mode (serial)
PICO_PID_BOOTSEL = 0x0003  # BOOTSEL mode


class PicoDevice:
    """
    Base class for communicating with Pico devices running custom firmware.
    """

    def __init__(
        self, port: str, baudrate: int = 115200, timeout: float = 1.0
    ):
        """
        Initialize a Pico device connection.

        Args:
            port: Serial port device (e.g., '/dev/ttyACM0' or 'COM3')
            baudrate: Serial baud rate (default: 115200)
            timeout: Serial read timeout in seconds (default: 1.0)
        """
        self.logger = logger
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[Serial] = None
        self._running = False
        self._reader_thread: Optional[threading.Thread] = None
        self._response_handler: Optional[Callable[[Dict[str, Any]], None]] = (
            None
        )
        self._raw_handler: Optional[Callable[[str], None]] = None
        self.last_status: Dict[str, Any] = {}

    @staticmethod
    def find_pico_ports() -> list[str]:
        """
        Find all connected Pico devices in CDC mode.

        Returns:
            List of serial port paths for connected Pico devices
        """
        ports = []
        for info in list_ports.comports():
            if info.vid == PICO_VID and info.pid == PICO_PID_CDC:
                ports.append(info.device)
        return ports

    def connect(self) -> bool:
        """
        Connect to the Pico device.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.ser = Serial(self.port, self.baudrate, timeout=self.timeout)
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from the device and clean up resources."""
        self.stop()
        if self.ser:
            self.ser.close()
            self.ser = None

    def send_command(self, cmd_dict: Dict[str, Any]) -> bool:
        """
        Send a JSON command to the device.

        Args:
            cmd_dict: Dictionary to be JSON-encoded and sent

        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.ser:
            return False

        try:
            json_str = json.dumps(cmd_dict, separators=(",", ":"))
            self.ser.write((json_str + "\n").encode("utf-8"))
            self.ser.flush()
            return True
        except Exception as e:
            self.logger.error(f"Failed to send command: {e}")
            return False

    def read_line(self) -> Optional[str]:
        """
        Read a line from the serial port.

        Returns:
            Decoded string without newline, or None if no data/error
        """
        if not self.ser:
            return None

        try:
            line = self.ser.readline()
            if line:
                return line.decode("utf-8", errors="ignore").strip()
        except Exception:
            pass
        return None

    def parse_response(self, line: str) -> Optional[Dict[str, Any]]:
        """
        Parse JSON response from device.

        Args:
            line: Raw string from serial port

        Returns:
            Parsed JSON as dictionary, or None if parsing fails
        """
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return None

    def _reader_thread_func(self):
        """Background thread function for reading serial data."""
        while self._running:
            line = self.read_line()
            if line:
                # Try to parse as JSON
                data = self.parse_response(line)
                if data:  # is json
                    self.last_status = data
                    # Call response handler if set
                    if self._response_handler:
                        self._response_handler(data)
                    else:
                        # Default: print the response
                        print(json.dumps(data))
                # Call raw handler on non-json if set
                elif self._raw_handler:
                    self._raw_handler(line)

    def set_response_handler(self, handler: Callable[[Dict[str, Any]], None]):
        """
        Set a custom handler for parsed JSON responses.

        Args:
            handler: Function that takes a dictionary (parsed JSON response)
        """
        self._response_handler = handler

    def set_raw_handler(self, handler: Callable[[str], None]):
        """
        Set a custom handler for raw string responses.

        Args:
            handler: Function that takes a string (raw line from serial)
        """
        self._raw_handler = handler

    def start(self):
        """Start the background reader thread."""
        if not self._running:
            self._running = True
            self._reader_thread = threading.Thread(
                target=self._reader_thread_func, daemon=True
            )
            self._reader_thread.start()

    def stop(self):
        """Stop the background reader thread."""
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None

    def wait_for_response(
        self, timeout: float = 5.0
    ) -> Optional[Dict[str, Any]]:
        """
        Send a command and wait for a single response.
        Useful for request-response patterns.

        Args:
            timeout: Maximum time to wait for response

        Returns:
            Parsed response or None if timeout/error
        """
        # Temporarily store the serial timeout
        old_timeout = self.ser.timeout if self.ser else None

        try:
            if self.ser:
                self.ser.timeout = timeout

            start_time = time.time()
            while time.time() - start_time < timeout:
                line = self.read_line()
                if line:
                    data = self.parse_response(line)
                    if data:
                        return data
            return None

        finally:
            # Restore the original timeout
            if self.ser and old_timeout is not None:
                self.ser.timeout = old_timeout

    def __enter__(self):
        """Context manager entry."""
        if self.connect():
            self.start()
            return self
        raise RuntimeError(f"Failed to connect to {self.port}")

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


class PicoMotor(PicoDevice):
    """Specialized class for motor control Pico devices."""

    STEP_ANGLE = 1.8  # degrees per step
    MICROSTEP = 1
    GEAR_TEETH = 113

    @staticmethod
    def deg_to_pulses(degrees: float) -> int:
        """
        Convert degrees to motor pulses.

        Args:
            degrees: Angle in degrees

        Returns:
            Number of pulses for the given angle
        """
        steps = degrees / PicoMotor.STEP_ANGLE
        return int(steps * PicoMotor.MICROSTEP * PicoMotor.GEAR_TEETH)

    def move(
        self,
        deg_az: Optional[float] = 0,
        deg_el: Optional[float] = 0,
        delay_us_az: int = 600,
        delay_us_el: int = 600,
    ) -> bool:
        """
        Send motor movement command.

        Args:
            deg_az: Azimuth angle in degrees (0 for no movement)
            deg_el: Elevation angle in degrees (0 for no movement)
            delay_us_az: Microseconds between azimuth steps
            delay_us_el: Microseconds between elevation steps

        Returns:
            True if command sent successfully
        """
        pulses_az = self.deg_to_pulses(deg_az)
        pulses_el = self.deg_to_pulses(deg_el)
        return self.send_command(
            {
                "pulses_az": pulses_az,
                "pulses_el": pulses_el,
                "delay_us_az": delay_us_az,
                "delay_us_el": delay_us_el,
            }
        )


class PicoRFSwitch(PicoDevice):
    """Specialized class for RF switch control Pico devices."""

    path_str = {
        "VNAO": "10000000",
        "VNAS": "11000000",
        "VNAL": "00100000",
        "VNAANT": "00000100",
        "VNANON": "00000111",
        "VNANOFF": "00000110",
        "VNARF": "00011000",
        "RFNON": "00000011",
        "RFNOFF": "00000010",
        "RFANT": "00000000",
    }

    @staticmethod
    def rbin(s):
        """
        Convert a str of 0s and 1s to binary, where the first char is the LSB.

        Parameters
        ----------
        s : str
            String of 0s and 1s.

        Returns
        -------
        int
            Integer representation of the binary string.

        """
        return int(s[::-1], 2)  # reverse the string and convert to int

    @property
    def paths(self):
        return {k: self.rbin(v) for k, v in self.path_str.items()}

    def switch(self, state: str) -> bool:
        """
        Set RF switch state.

        Parameters
        ----------
        state: str
            Switch state path, see self.PATHS for valid keys

        Returns
        -------
        bool
            True if command sent successfully

        Raises
        -------
        ValueError
            If an invalid switch state is provided

        """
        try:
            s = self.paths[state]
        except KeyError as e:
            raise ValueError(
                f"Invalid switch state '{state}'. Valid states: "
                "{list(self.PATHS.keys())}"
            ) from e
        return self.send_command({"sw_state": s})


class PicoPeltier(PicoDevice):
    """Specialized class for Peltier temperature control Pico devices."""

    def set_temperature(self, temperature: float, channel: int = 0) -> bool:
        """
        Set target temperature.

        Args:
            temperature: Target temperature in Celsius
            channel: Channel number (0=both, 1/2=individual)

        Returns:
            True if command sent successfully
        """
        return self.send_command(
            {"cmd": "set_temp", "temperature": temperature, "channel": channel}
        )

    def set_hysteresis(self, hysteresis: float, channel: int = 0) -> bool:
        """
        Set temperature hysteresis band.

        Args:
            hysteresis: Hysteresis value in Celsius
            channel: Channel number (0=both, 1/2=individual)

        Returns:
            True if command sent successfully
        """
        return self.send_command(
            {
                "cmd": "set_hysteresis",
                "hysteresis": hysteresis,
                "channel": channel,
            }
        )

    def enable(self, channel: int = 0) -> bool:
        """
        Enable temperature control.

        Args:
            channel: Channel number (0=both, 1/2=individual)

        Returns:
            True if command sent successfully
        """
        return self.send_command({"cmd": "enable", "channel": channel})

    def disable(self, channel: int = 0) -> bool:
        """
        Disable temperature control.

        Args:
            channel: Channel number (0=both, 1/2=individual)

        Returns:
            True if command sent successfully
        """
        return self.send_command({"cmd": "disable", "channel": channel})
