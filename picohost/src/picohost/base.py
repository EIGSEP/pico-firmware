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

from .flash_picos import find_pico_ports

logger = logging.getLogger(__name__)

# USB IDs for Raspberry Pi Pico
PICO_VID = 0x2E8A
PICO_PID_CDC = 0x0009  # CDC mode (serial)
PICO_PID_BOOTSEL = 0x0003  # BOOTSEL mode


def redis_handler(writer):
    """
    Create a handler function that publishes a status dict via a
    :class:`eigsep_redis.MetadataWriter`.

    Parameters
    ----------
    writer : eigsep_redis.MetadataWriter
        The metadata bus writer to publish through. The ``sensor_name``
        field on each data dict is used as the metadata key (so
        ``stream:{sensor_name}`` carries the per-sensor history and
        ``metadata[sensor_name]`` holds the live snapshot).

    Returns
    -------
    handler : callable
        Function that takes a data dictionary and publishes it.

    Notes
    -----
    **Scalar-only contract.** The data dict published to Redis must
    contain only scalar values: ``str``, ``int``, ``float``, ``bool``,
    or ``None``. Compound values — vectors, tuples, calibration
    parameters, etc. — must be flattened into per-component scalar
    fields with descriptive suffixes (e.g. ``quat_i/j/k/real`` for a
    quaternion, ``accel_x/y/z`` for an acceleration vector,
    ``pot_el_cal_slope`` / ``pot_el_cal_intercept`` for a linear
    calibration). This invariant lets downstream consumers validate
    every field with a per-key schema, lands the data cleanly in HDF5
    attribute storage, and gives every field a meaningful per-type
    reduction policy when readings are averaged within an integration.
    Compound values cannot be validated per-field, do not have a
    well-defined averaging semantic, and require special-case readers
    downstream — so they are forbidden at the producer boundary.

    Subclasses that wrap this handler (e.g. ``PicoPotentiometer``)
    must preserve the scalar-only contract for any fields they add.
    """

    def handler(data):
        try:
            name = data["sensor_name"]
        except KeyError:
            logger.error("Data does not contain 'sensor_name' key")
            return
        writer.add(name, data)

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
        metadata_writer=None,
        response_handler=None,
        usb_serial: str = "",
        verbose: bool = False,
    ):
        """
        Initialize a Pico device connection.

        Args:
            port: Serial port device (e.g., '/dev/ttyACM0' or 'COM3')
            baudrate: Serial baud rate (default: 115200)
            timeout: Serial read timeout in seconds (default: 5.0)
            name: str
            metadata_writer: eigsep_redis.MetadataWriter instance, or
                ``None`` to disable Redis publication.
            usb_serial: USB serial number for port re-discovery
            verbose: log each received status packet at DEBUG level
        """
        self.logger = logger
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.usb_serial = usb_serial
        self.verbose = verbose
        self.ser = None
        self._running = False
        self._reader_thread = None
        self._response_handler = None
        self._raw_handler = None
        self.last_status = {}
        self.last_status_time = None
        if name is None:
            self.name = port.split("/")[-1] if "/" in port else port
        else:
            self.name = name

        if metadata_writer is not None:
            self.redis_handler = redis_handler(metadata_writer)
        else:
            self.redis_handler = None
        self.connect()

        if response_handler is not None:
            self.set_response_handler(response_handler)

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

    def _open_serial(self) -> bool:
        """Open the serial port without starting the reader thread."""
        try:
            self.ser = Serial(self.port, self.baudrate, timeout=self.timeout)
            self.ser.reset_input_buffer()
            self.last_status_time = time.time()
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to {self.port}: {e}")
            return False

    def _close_serial(self):
        """Close the serial port if open."""
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def connect(self) -> bool:
        """
        Open the serial connection and start the background reader thread.

        Returns:
            True if connection successful, False otherwise
        """
        if self.is_connected:
            return True
        if not self._open_serial():
            return False
        self._start_reader()
        return True

    def disconnect(self):
        """Stop the reader thread, close the serial port, and clean up."""
        self._stop_reader()
        self.ser = None

    def reconnect(self) -> bool:
        """
        Disconnect and reconnect to the device.

        If *usb_serial* is set the current USB port mapping is checked
        first so that the device is found even after a USB re-enumeration.

        Calls ``on_reconnect()`` after a successful reconnect so that
        subclasses can re-send any configuration that the firmware loses
        across a serial drop (e.g. PicoMotor's delay settings).

        Returns
        -------
        bool
            True if reconnection succeeded, False otherwise.
        """
        self.disconnect()
        if self.usb_serial:
            self._rediscover_port()
        if self.connect():
            self.on_reconnect()
            return True
        return False

    def _rediscover_port(self):
        """Update ``self.port`` if the USB serial maps to a new device."""
        try:
            ports = find_pico_ports()
        except Exception as e:
            self.logger.warning(f"Port re-discovery failed: {e}")
            return
        for dev, ser in ports.items():
            if ser == self.usb_serial and dev != self.port:
                self.logger.info(
                    f"{self.name}: port changed {self.port} -> {dev}"
                )
                self.port = dev
                return

    def on_reconnect(self):
        """
        Hook invoked after a successful ``reconnect()``.

        Default is a no-op; subclasses override to re-apply firmware
        state that doesn't persist across a USB serial drop.
        """
        pass

    def send_command(self, cmd_dict: Dict[str, Any]) -> None:
        """
        Send a JSON command to the device.

        Args:
            cmd_dict: Dictionary to be JSON-encoded and sent

        Raises:
            ConnectionError: device is not connected, or the underlying
                write failed.
        """
        if not self.is_connected:
            raise ConnectionError(f"{self.name} not connected")

        json_str = json.dumps(cmd_dict, separators=(",", ":"))
        try:
            self.ser.write((json_str + "\n").encode("utf-8"))
            self.ser.flush()
        except Exception as e:
            raise ConnectionError(
                f"{self.name} write failed: {e}"
            ) from e

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
            # Serial error (likely device unplugged) — close the dead handle
            # so is_connected becomes False and reconnection can be attempted.
            self.logger.warning(
                f"Serial read error on {self.port}, closing connection"
            )
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
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

    _RECONNECT_INTERVAL = 2.0  # seconds between reconnection attempts

    def _reader_thread_func(self):
        """Background thread function for reading serial data."""
        while self._running:
            if not self.is_connected:
                # Try to reopen the serial port (don't call connect() which
                # would spawn a second reader thread).
                self.logger.info(f"Attempting to reconnect to {self.port}...")
                if self._open_serial():
                    self.logger.info(f"Reconnected to {self.port}")
                else:
                    time.sleep(self._RECONNECT_INTERVAL)
                continue

            line = self.read_line()
            if line:
                # Try to parse as JSON
                data = self.parse_response(line)
                if data:  # is json
                    self.last_status = data
                    self.last_status_time = time.time()
                    if self.verbose:
                        self.logger.debug(
                            json.dumps(data, sort_keys=True)
                        )
                    # upload to redis
                    if self.redis_handler:
                        try:
                            self.redis_handler(data)
                        except Exception as e:
                            self.logger.error(f"Redis publish failed: {e}")
                    # Call response handler if set
                    if self._response_handler:
                        self._response_handler(data)
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

    def _start_reader(self):
        """Start the background reader thread."""
        if not self._running:
            self._running = True
            self._reader_thread = threading.Thread(
                target=self._reader_thread_func, daemon=True
            )
            self._reader_thread.start()

    def _stop_reader(self):
        """Stop the background reader thread and close the serial port."""
        self._running = False
        # Close the serial port first so that readline() unblocks
        # immediately, rather than waiting for the serial timeout.
        self._close_serial()
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
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
        if not self.is_connected:
            if not self.connect():
                raise RuntimeError(f"Failed to connect to {self.port}")
        return self

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

    def switch(self, state: str) -> None:
        """
        Set RF switch state.

        Parameters
        ----------
        state: str
            Switch state path, see self.PATHS for valid keys

        Raises
        -------
        ValueError
            If an invalid switch state is provided
        ConnectionError
            If the device is not connected or the write failed.

        """
        try:
            s = self.paths[state]
        except KeyError as e:
            raise ValueError(
                f"Invalid switch state '{state}'. Valid states: "
                f"{list(self.paths.keys())}"
            ) from e
        self.send_command({"sw_state": s})
        time.sleep(0.05)  # allow time for switch to settle
        self.logger.info(f"Switched to {state}.")


class PicoPeltier(PicoDevice):
    """Specialized class for Peltier temperature control Pico devices.

    Sends periodic keepalive commands to prevent the firmware communication
    watchdog from tripping and disabling the peltiers.
    """

    def __init__(
        self,
        port,
        verbose=False,
        timeout=5.0,
        name=None,
        metadata_writer=None,
        keepalive_interval=10.0,
        usb_serial="",
    ):
        """
        Parameters
        ----------
        port : str
            Serial port device.
        metadata_writer : eigsep_redis.MetadataWriter, optional
            Metadata bus writer. ``None`` disables Redis publication.
        verbose : bool, optional
            Enable verbose output.
        timeout : float, optional
            Serial read timeout in seconds.
        name : str, optional
            Logical device name.
        keepalive_interval : float, optional
            Seconds between keepalive commands sent to the firmware.
            Must be less than the firmware watchdog timeout (default 30s).
            Set to 0 to disable keepalive. Default: 10.0.
        """
        self._keepalive_running = False
        self._keepalive_thread = None
        self._keepalive_interval = keepalive_interval
        super().__init__(
            port,
            timeout=timeout,
            name=name,
            metadata_writer=metadata_writer,
            usb_serial=usb_serial,
            verbose=verbose,
        )
        self._start_keepalive()

    def _start_keepalive(self):
        """Start the background keepalive thread."""
        if self._keepalive_interval > 0:
            self._keepalive_running = True
            self._keepalive_thread = threading.Thread(
                target=self._keepalive_thread_func, daemon=True
            )
            self._keepalive_thread.start()

    def _keepalive_thread_func(self):
        """Send empty commands periodically to reset the firmware watchdog."""
        while self._keepalive_running:
            try:
                self.send_command({})
            except ConnectionError:
                # Reader thread owns reconnection; keepalive survives drops.
                pass
            # Sleep in small increments so thread stops promptly
            for _ in range(max(1, int(self._keepalive_interval * 10))):
                if not self._keepalive_running:
                    break
                time.sleep(0.1)

    def disconnect(self):
        """Stop keepalive thread, then reader thread and serial port."""
        self._keepalive_running = False
        if self._keepalive_thread:
            self._keepalive_thread.join(timeout=2.0)
            self._keepalive_thread = None
        super().disconnect()

    def on_reconnect(self):
        """Restart the keepalive thread after a reconnect."""
        self._start_keepalive()

    @property
    def watchdog_tripped(self):
        """Whether the firmware watchdog has tripped and disabled the peltiers."""
        return self.last_status.get("watchdog_tripped", False)

    def set_watchdog_timeout(self, timeout_ms):
        """
        Configure the firmware communication watchdog timeout.

        Parameters
        ----------
        timeout_ms : int
            Watchdog timeout in milliseconds. 0 disables the watchdog.

        Raises
        ------
        ConnectionError
            If the device is not connected or the write failed.
        """
        self.send_command({"watchdog_timeout_ms": int(timeout_ms)})

    def set_temperature(
        self, T_LNA=None, LNA_hyst=0.5, T_LOAD=None, LOAD_hyst=0.5
    ):
        """Set target temperature."""
        cmd = {}
        if T_LNA is not None:
            cmd["LNA_temp_target"] = T_LNA
            cmd["LNA_hysteresis"] = LNA_hyst
        if T_LOAD is not None:
            cmd["LOAD_temp_target"] = T_LOAD
            cmd["LOAD_hysteresis"] = LOAD_hyst
        self.send_command(cmd)

    def set_enable(self, LNA=True, LOAD=True):
        """Enable temperature control."""
        self.send_command({"LNA_enable": LNA, "LOAD_enable": LOAD})

    def set_clamp(self, LNA=None, LOAD=None):
        """Set maximum drive level [0.0, 1.0], 0.6 default."""
        cmd = {}
        if LNA is not None:
            cmd["LNA_clamp"] = LNA
        if LOAD is not None:
            cmd["LOAD_clamp"] = LOAD
        self.send_command(cmd)


class PicoIMU(PicoDevice):
    """Specialized class for IMU devices (BNO08x UART RVC mode)."""

    pass


class PicoLidar(PicoDevice):
    """Specialized class for lidar distance sensor devices."""

    pass


class PicoPotentiometer(PicoDevice):
    """Potentiometer monitoring device with voltage-to-angle calibration."""

    def __init__(
        self,
        port,
        calibration_file=None,
        timeout=5.0,
        name=None,
        metadata_writer=None,
        usb_serial="",
    ):
        """
        Parameters
        ----------
        port : str
            Serial port device.
        calibration_file : str, optional
            Path to a JSON calibration file. If provided, calibration
            parameters are loaded at init.
        timeout : float
            Serial read timeout in seconds (default: 5.0).
        name : str, optional
        metadata_writer : eigsep_redis.MetadataWriter, optional
            Metadata bus writer. ``None`` disables Redis publication.
        usb_serial : str, optional
            USB serial number for port re-discovery.
        """
        self._cal = {"pot_el": None, "pot_az": None}
        super().__init__(
            port,
            timeout=timeout,
            name=name,
            metadata_writer=metadata_writer,
            usb_serial=usb_serial,
        )
        if calibration_file is not None:
            self.load_calibration(calibration_file)
        # Wrap the base redis handler to convert voltages to angles
        if self.redis_handler is not None:
            self._base_redis_handler = self.redis_handler
            self.redis_handler = self._pot_redis_handler

    def _pot_redis_handler(self, data):
        """Add per-component cal scalars and angle before uploading to Redis.

        Augments the raw voltage payload with the calibration slope and
        intercept (flattened into scalar fields per the
        :func:`redis_handler` scalar-only contract) and the derived
        angle. All added fields are ``None`` when the corresponding pot
        is uncalibrated, so the published shape is stable regardless of
        calibration state.
        """
        data = data.copy()
        for key in ("pot_el", "pot_az"):
            cal = self._cal[key]
            v = data.get(f"{key}_voltage")
            if cal is not None:
                m, b = cal
                data[f"{key}_cal_slope"] = float(m)
                data[f"{key}_cal_intercept"] = float(b)
                if v is not None:
                    data[f"{key}_angle"] = float(m * v + b)
                else:
                    data[f"{key}_angle"] = None
            else:
                data[f"{key}_cal_slope"] = None
                data[f"{key}_cal_intercept"] = None
                data[f"{key}_angle"] = None
        self._base_redis_handler(data)

    def set_calibration(self, pot_el_params=None, pot_az_params=None):
        """Set calibration parameters (m, b) for one or both pots.

        Parameters
        ----------
        pot_el_params : tuple of (float, float), optional
            (slope, intercept) such that angle = m * voltage + b.
        pot_az_params : tuple of (float, float), optional
        """
        if pot_el_params is not None:
            self._cal["pot_el"] = tuple(pot_el_params)
        if pot_az_params is not None:
            self._cal["pot_az"] = tuple(pot_az_params)

    def load_calibration(self, path):
        """Load calibration from a JSON file.

        Expected format: ``{"pot_el": [m, b], "pot_az": [m, b], ...}``
        """
        with open(path, "r") as f:
            cal_data = json.load(f)
        if "pot_el" in cal_data:
            self._cal["pot_el"] = tuple(cal_data["pot_el"])
        if "pot_az" in cal_data:
            self._cal["pot_az"] = tuple(cal_data["pot_az"])

    @property
    def is_calibrated(self):
        """True if both pots have calibration parameters."""
        return (
            self._cal["pot_el"] is not None and self._cal["pot_az"] is not None
        )

    def read_voltage(self):
        """Return the latest voltage readings.

        Returns
        -------
        dict
            ``{"pot_el_voltage": float, "pot_az_voltage": float}``
        """
        return {
            "pot_el_voltage": self.last_status.get("pot_el_voltage"),
            "pot_az_voltage": self.last_status.get("pot_az_voltage"),
        }

    def read_angle(self):
        """Convert current voltage readings to angles using calibration.

        Returns
        -------
        dict
            ``{"pot_el": float, "pot_az": float}`` in degrees.

        Raises
        ------
        RuntimeError
            If calibration has not been set or voltage data is missing.
        """
        result = {}
        for key in ("pot_el", "pot_az"):
            v = self.last_status.get(f"{key}_voltage")
            if v is None:
                raise RuntimeError(f"No voltage reading for {key}")
            cal = self._cal[key]
            if cal is None:
                raise RuntimeError(
                    f"No calibration for {key}. "
                    "Call load_calibration() or set_calibration() first."
                )
            m, b = cal
            result[key] = m * v + b
        return result
