"""
Base class for Pico device communication.
Provides common functionality for serial communication with Pico devices.

XXX TODO: make sure everyone has redis handlers
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


def redis_handler(redis):
    def handler(data):
        try:
            name = data["sensor_name"]
        except KeyError:
            logger.error("Data does not contain 'sensor_name' key")
            return
        redis.add_metadata(name, data)

    return handler


class PicoDevice:
    """
    Base class for communicating with Pico devices running custom firmware.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 5.0,
        name=None,
        eig_redis=None,
        response_handler=None,
    ):
        """
        Initialize a Pico device connection.

        Args:
            port: Serial port device (e.g., '/dev/ttyACM0' or 'COM3')
            baudrate: Serial baud rate (default: 115200)
            timeout: Serial read timeout in seconds (default: 1.0)
            name: str
            eig_redis: EigsepRedis instance
        """
        self.logger = logger
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self._running = False
        self._reader_thread = None
        self._response_handler = None
        self._raw_handler = None
        self.last_status = {}
        if name is None:
            self.name = port.split("/")[-1] if "/" in port else port
        else:
            self.name = name

        self.connect()
        if eig_redis is not None:
            self.redis_handler = redis_handler(eig_redis)
        else:
            self.redis_handler = None

        if response_handler is not None:
            self.set_response_handler(response_handler)

        self.start()

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

    @property
    def is_connected(self) -> bool:
        """
        Check if the device is currently connected.

        Returns:
            True if connected, False otherwise
        """
        return self.ser is not None and self.ser.is_open

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
        if self.is_connected:
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
        if not self.is_connected:
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
        if not self.is_connected:
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
                    # upload to redis
                    if self.redis_handler:
                        self.redis_handler(data)
                    # Call response handler if set
                    if self._response_handler:
                        self._response_handler(data)
                    # else:
                        # Default: print the response
                    #    print(json.dumps(data))
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
        if not self.is_connected:
            return None

        old_timeout = self.ser.timeout
        try:
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
            if old_timeout is not None:
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



class PicoRFSwitch(PicoDevice):
    """Specialized class for RF switch control Pico devices."""

    path_str = {
        "VNAO": "10000000",  # checked 7/7/25
        "VNAS": "11000000",  # checked 7/7/25
        "VNAL": "00100000",  # checked 7/7/25
        "VNAANT": "00000001",  # checked 7/7/25
        "VNANON": "00000111",  # checked 7/7/25
        "VNANOFF": "00000101",  # checked 7/7/25
        "VNARF": "00011000",  # checked 7/7/25
        "RFNON": "00000110",  # checked 7/7/25
        "RFNOFF": "00000100",  # checked 7/7/25
        "RFANT": "00000000",  # checked 7/7/25
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
                f"{list(self.paths.keys())}"
            ) from e
        c = self.send_command({"sw_state": s})
        if c:
            time.sleep(0.05)  # allow time for switch to settle
            self.logger.info(f"Switched to {state}.")
        else:
            self.logger.error(f"Failed to switch to {state}.")
        return c

class PicoStatus(PicoDevice):
    """Adds status monitoring to PicoDevice."""
    def __init__(
        self, port, verbose=False, timeout=5., name="", eig_redis=None
    ):
        """ kwargs passed to super()"""
        super().__init__(
            port, timeout=timeout, name=name, eig_redis=eig_redis
        )
        self.verbose = verbose
        self.status = {}
        self.set_response_handler(self.update_status)
        self.wait_for_updates()

    def update_status(self, data):
        """Update internal status based on unpacked json packets from picos."""
        if self.verbose:
            print(json.dumps(data, indent=2, sort_keys=True))
        self.status.update(data)

    def wait_for_updates(self, timeout=3):
        t = time.time()
        while True:
            if len(self.status) != 0:
                break
            assert time.time() - t < timeout
            time.sleep(0.1)

class PicoPeltier(PicoStatus):
    """Specialized class for Peltier temperature control Pico devices."""

    def set_temperature(self, T_A=None, A_hyst=0.5, T_B=None, B_hyst=0.5):
        """Set target temperature."""
        cmd = {}
        if T_A is not None:
            cmd['A_temp_target'] = T_A
            cmd['A_hysteresis'] = A_hyst
        if T_B is not None:
            cmd['B_temp_target'] = T_B
            cmd['B_hysteresis'] = B_hyst
        return self.send_command(cmd)

    def set_enable(self, A=True, B=True):
        """Enable temperature control."""
        return self.send_command({'A_enable': A, 'B_enable': B})


class PicoIMU(PicoDevice):
    """Specialized class for IMU calibration control."""

    def calibrate(self) -> bool:
        """
        Send request to calibrate IMU.

        Args:
            channel: Channel number (0=both, 1/2=individual)

        Returns:
            True if command sent successfully
        """
        return self.send_command({"cmd": "calibrate"})
