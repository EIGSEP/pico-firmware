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
from . import imu_geometry as ig

logger = logging.getLogger(__name__)

# USB IDs for Raspberry Pi Pico
PICO_VID = 0x2E8A
PICO_PID_CDC = 0x0009  # CDC mode (serial)
PICO_PID_BOOTSEL = 0x000F  # RP2350 BOOTSEL mode (RP2040 was 0x0003)


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
    ``pot_az_cal_slope`` / ``pot_az_cal_intercept`` for a linear
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
        self._write_lock = threading.Lock()
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
        if not self._attempt_reopen():
            return False
        self._start_reader()
        return True

    def _attempt_reopen(self) -> bool:
        """
        Shared post-open sequence: rediscover the USB port (if tracked),
        reopen the serial handle, and fire ``on_reconnect()`` on success.

        Does not touch the reader-thread lifecycle — callers decide
        whether to start/stop the reader around this call. Both the
        public :meth:`reconnect` and the reader thread's in-thread
        self-heal route through here so they share one post-open
        contract.
        """
        if self.usb_serial:
            self._rediscover_port()
        if not self._open_serial():
            return False
        self.on_reconnect()
        return True

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
        Hook invoked after the serial port is reopened.

        Fires from both reconnect paths: the public :meth:`reconnect`
        (e.g. ``PicoManager._check_health``) and the reader thread's
        in-thread self-heal after a transient serial drop. Default is
        a no-op; subclasses override to re-apply firmware state that
        doesn't persist across a USB serial drop.
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
        payload = (json_str + "\n").encode("utf-8")
        try:
            with self._write_lock:
                self.ser.write(payload)
                self.ser.flush()
        except Exception as e:
            raise ConnectionError(f"{self.name} write failed: {e}") from e

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
                # Reopen in-thread — we can't call the public reconnect()
                # because it joins this thread and would deadlock.
                self.logger.info(f"Attempting to reconnect to {self.port}...")
                if self._attempt_reopen():
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
                        self.logger.debug(json.dumps(data, sort_keys=True))
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

    # Sentinel sw_state value the firmware emits while the RF switch is
    # settling. Mirrors SW_STATE_UNKNOWN in src/rfswitch.h.
    SW_STATE_UNKNOWN = -1
    SW_STATE_UNKNOWN_NAME = "UNKNOWN"

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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._name_by_state = {v: k for k, v in self.paths.items()}
        if self.redis_handler is not None:
            self._base_redis_handler = self.redis_handler
            self.redis_handler = self._rfswitch_redis_handler

    def _rfswitch_redis_handler(self, data):
        """Add the human-readable switch name before uploading to Redis.

        Firmware reports ``sw_state`` as a raw 8-bit integer, or
        :attr:`SW_STATE_UNKNOWN` (-1) while the physical switch is
        settling after a command. Downstream consumers see a named
        state (``"VNAO"``, ``"RFANT"``, ...) alongside the raw integer:

        * ``SW_STATE_UNKNOWN`` maps to ``"UNKNOWN"`` — switch is mid-transition.
        * A known state integer maps to its path name.
        * Any other integer maps to ``None`` (manual override, firmware bug).

        The published shape stays stable regardless.
        """
        data = data.copy()
        sw_state = data.get("sw_state")
        if sw_state == self.SW_STATE_UNKNOWN:
            data["sw_state_name"] = self.SW_STATE_UNKNOWN_NAME
        else:
            data["sw_state_name"] = self._name_by_state.get(sw_state)
        self._base_redis_handler(data)

    def switch(self, state: str) -> None:
        """
        Set RF switch state.

        The call returns as soon as the command has been delivered to
        the firmware. The firmware holds its reported ``sw_state`` at
        :attr:`SW_STATE_UNKNOWN` until the physical switch is trusted
        to have settled, so callers that need closed-loop confirmation
        should poll for the expected state name (e.g. via Redis) rather
        than time.sleep here.

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
        self.logger.info(f"Switched to {state}.")


class PicoPeltier(PicoDevice):
    """Specialized class for Peltier temperature control Pico devices.

    Sends periodic keepalive commands to prevent the firmware communication
    watchdog from tripping and disabling the peltiers. Caches the last
    config pushed by each setter and replays it in :meth:`on_reconnect`
    so a pico reboot (brownout, firmware watchdog, picotool re-flash)
    doesn't leave the firmware running on defaults.
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
        self._last_watchdog_timeout_ms = None
        self._last_clamp = {}
        self._last_cooling = {}
        self._last_gains = {}
        self._last_temperature = {}
        self._last_enable = None
        super().__init__(
            port,
            timeout=timeout,
            name=name,
            metadata_writer=metadata_writer,
            usb_serial=usb_serial,
            verbose=verbose,
        )
        if self.redis_handler is not None:
            self._base_redis_handler = self.redis_handler
            self.redis_handler = self._peltier_redis_handler
        self._start_keepalive()

    _PELTIER_CHANNEL_FIELDS = (
        "T_now",
        "voltage",
        "resistance",
        "timestamp",
        "T_target",
        "drive_level",
        "enabled",
        "active",
        "int_disabled",
        "stall_tripped",
        "cooling_enabled",
        "hysteresis",
        "clamp",
        "Kp",
        "Ki",
        "integral",
    )
    _PELTIER_STREAMS = (("LNA", "tempctrl_lna"), ("LOAD", "tempctrl_load"))

    def _peltier_redis_handler(self, data):
        """Fan out the combined tempctrl status dict into two Redis streams.

        The firmware emits one combined message per status tick with
        ``LNA_*`` / ``LOAD_*`` prefixed fields plus the device-wide
        watchdog state. We publish two streams (``tempctrl_lna``,
        ``tempctrl_load``), each matching the standard one-stream-per-
        sensor schema with a top-level ``status`` derived from the
        channel's ``LNA_status`` / ``LOAD_status``. The device-wide
        watchdog fields are duplicated into both streams; both come from
        the same firmware tick so a momentary tick-to-tick disagreement
        is harmless.
        """
        app_id = data.get("app_id")
        watchdog_tripped = data.get("watchdog_tripped")
        watchdog_timeout_ms = data.get("watchdog_timeout_ms")
        for prefix, stream in self._PELTIER_STREAMS:
            out = {
                "sensor_name": stream,
                "app_id": app_id,
                "status": data.get(f"{prefix}_status"),
                "watchdog_tripped": watchdog_tripped,
                "watchdog_timeout_ms": watchdog_timeout_ms,
            }
            for k in self._PELTIER_CHANNEL_FIELDS:
                out[k] = data.get(f"{prefix}_{k}")
            self._base_redis_handler(out)

    def _start_keepalive(self):
        """Start the background keepalive thread (idempotent)."""
        if self._keepalive_interval <= 0:
            return
        if (
            self._keepalive_thread is not None
            and self._keepalive_thread.is_alive()
        ):
            return
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
        """Replay the last-applied config, then restart keepalive.

        A serial-link recovery is the host's proxy for "the pico may
        have rebooted" — on EIGSEP hardware every firmware reset path
        (hard watchdog, brownout, picotool re-flash via BOOTSEL) drops
        USB CDC, so reader-thread reconnect coincides with the firmware
        coming up at defaults. Replay whatever the host most recently
        pushed in a safe order: watchdog → clamp → cooling_enabled →
        gains → temperature → enable. cooling_enabled lands between
        clamp and gains so the asymmetric-clamp safety setting is in
        place before any drive can result from the next setpoint. Gains
        land before temperature so the channel is fully tuned the
        instant it goes active. Keepalive starts last so the firmware
        watchdog is configured before we start pinging it.
        """
        if self._last_watchdog_timeout_ms is not None:
            self.send_command(
                {"watchdog_timeout_ms": self._last_watchdog_timeout_ms}
            )
        if self._last_clamp:
            self.send_command(dict(self._last_clamp))
        if self._last_cooling:
            self.send_command(dict(self._last_cooling))
        if self._last_gains:
            self.send_command(dict(self._last_gains))
        if self._last_temperature:
            self.send_command(dict(self._last_temperature))
        if self._last_enable is not None:
            self.send_command(dict(self._last_enable))
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
        timeout_ms = int(timeout_ms)
        self.send_command({"watchdog_timeout_ms": timeout_ms})
        self._last_watchdog_timeout_ms = timeout_ms

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
        if cmd:
            self.send_command(cmd)
            self._last_temperature.update(cmd)

    def set_enable(self, LNA=True, LOAD=True):
        """Enable temperature control."""
        cmd = {"LNA_enable": LNA, "LOAD_enable": LOAD}
        self.send_command(cmd)
        self._last_enable = cmd

    def set_clamp(self, LNA=None, LOAD=None):
        """Set maximum drive level [0.0, 1.0], 0.2 default."""
        cmd = {}
        if LNA is not None:
            cmd["LNA_clamp"] = LNA
        if LOAD is not None:
            cmd["LOAD_clamp"] = LOAD
        if cmd:
            self.send_command(cmd)
            self._last_clamp.update(cmd)

    def set_cooling_enabled(self, LNA=None, LOAD=None):
        """Allow/forbid negative (cooling) drive per channel.

        ``False`` clamps drive to ``[0, +clamp]`` instead of
        ``[-clamp, +clamp]`` firmware-side — the cooling-mode
        thermal-runaway guard. Firmware default after reboot is
        ``True`` (symmetric); deployments that cannot dissipate
        Peltier heat should explicitly set this ``False`` on the
        affected channel. Cached for replay on reconnect.
        """
        cmd = {}
        if LNA is not None:
            if not isinstance(LNA, bool):
                raise TypeError("LNA must be a bool or None")
            cmd["LNA_cooling_enabled"] = LNA
        if LOAD is not None:
            if not isinstance(LOAD, bool):
                raise TypeError("LOAD must be a bool or None")
            cmd["LOAD_cooling_enabled"] = LOAD
        if cmd:
            self.send_command(cmd)
            self._last_cooling.update(cmd)

    def set_gains(self, LNA_Kp=None, LNA_Ki=None, LOAD_Kp=None, LOAD_Ki=None):
        """Set PI gains per channel.

        Ki defaults to 0 in firmware, so the controller runs as pure
        proportional + deadband until a host opts in. Cached for replay
        on reconnect (firmware resets gains to defaults on reboot).
        """
        cmd = {}
        if LNA_Kp is not None:
            cmd["LNA_Kp"] = LNA_Kp
        if LNA_Ki is not None:
            cmd["LNA_Ki"] = LNA_Ki
        if LOAD_Kp is not None:
            cmd["LOAD_Kp"] = LOAD_Kp
        if LOAD_Ki is not None:
            cmd["LOAD_Ki"] = LOAD_Ki
        if cmd:
            self.send_command(cmd)
            self._last_gains.update(cmd)

    def reset_integral(self, LNA=False, LOAD=False):
        """Clear the PI integrator on the selected channel(s).

        One-shot — not cached for replay (firmware reset clears the
        integral implicitly).
        """
        cmd = {}
        if LNA:
            cmd["LNA_integral_reset"] = True
        if LOAD:
            cmd["LOAD_integral_reset"] = True
        if cmd:
            self.send_command(cmd)


class PicoIMU(PicoDevice):
    """IMU device (BNO08x UART RVC mode) with live az/el conversion.

    Loads its mount calibration from an :class:`ImuCalStore` section keyed
    by sensor name (``imu_el`` / ``imu_az``) and augments each status tick
    with derived angles. ``imu_az`` reports az + el; ``imu_el`` reports el
    only (it cannot observe azimuth). All derived fields are ``None`` when
    that IMU is uncalibrated, so the published shape is stable.

    Note on ``el_deg`` sign convention: ``imu_el.el_deg`` is SIGNED
    (``el_from_imu``, negative below horizontal), while ``imu_az.el_deg``
    is the unsigned magnitude |θ| (``el_abs_from_imu_az``, assumes θ≥0 for
    a single-tick az estimate). Downstream consumers must account for this
    difference.
    """

    def __init__(self, *args, imu_cal_store=None, **kwargs):
        # {"imu_el": {...}, "imu_az": {...}} — only loaded sections present.
        self._imu_cal = {}
        self._imu_derive_warned = set()
        super().__init__(*args, **kwargs)
        if imu_cal_store is not None:
            cal = imu_cal_store.get()
            if cal:
                for key in ("imu_el", "imu_az"):
                    if key in cal:
                        self._imu_cal[key] = cal[key]
        if self.redis_handler is not None:
            self._base_redis_handler = self.redis_handler
            self.redis_handler = self._imu_redis_handler

    def set_calibration(self, imu_el=None, imu_az=None):
        """Merge per-IMU calibration sections (live push from calibrate_imu)."""
        if imu_el is not None:
            self._imu_cal["imu_el"] = imu_el
        if imu_az is not None:
            self._imu_cal["imu_az"] = imu_az

    @staticmethod
    def _accel_unit(data, cal):
        a = [data.get("accel_x"), data.get("accel_y"), data.get("accel_z")]
        if any(v is None for v in a):
            return None
        return ig.precondition(a, cal["accel_bias"])

    def _imu_redis_handler(self, data):
        data = data.copy()
        name = data.get("sensor_name")
        cal = self._imu_cal.get(name)
        try:
            if name == "imu_el":
                data["el_deg"] = self._el_only(data, cal)
            elif name == "imu_az":
                data.update(self._az_and_el(data, cal))
        except Exception:
            # A malformed/partial cal section (or a bad sample) must never
            # suppress the raw firmware tick: publish raw with null derived
            # fields, keeping the published shape stable. Warn once per IMU.
            if name == "imu_el":
                data["el_deg"] = None
            elif name == "imu_az":
                data.update(self._az_null())
            if name not in self._imu_derive_warned:
                self._imu_derive_warned.add(name)
                self.logger.warning(
                    "IMU derivation failed for %s; publishing raw with null "
                    "derived fields — check the stored calibration section.",
                    name,
                )
        self._base_redis_handler(data)

    @staticmethod
    def _az_null():
        """Null-valued imu_az derived shape (stable keys, no calibration)."""
        return {
            "el_deg": None,
            "az_deg": None,
            "az_from_accel_deg": None,
            "az_from_yaw_deg": None,
            "az_blend_weight": None,
        }

    def _el_only(self, data, cal):
        if cal is None:
            return None
        a = self._accel_unit(data, cal)
        if a is None:
            return None
        return ig.el_from_imu(a, cal["M"])

    def _az_and_el(self, data, cal):
        out = self._az_null()
        if cal is None:
            return out
        a = self._accel_unit(data, cal)
        if a is None:
            return out
        M = cal["M"]
        el = ig.el_abs_from_imu_az(a, M)
        az_a = ig.az_from_accel(
            a, M, cal["az_sign"], cal["az_accel_offset_deg"]
        )
        out["el_deg"] = el
        out["az_from_accel_deg"] = az_a
        yaw = data.get("yaw")
        if yaw is not None:
            az_y = ig.az_from_yaw(
                yaw, cal["az_yaw_sign"], cal["az_yaw_offset_deg"]
            )
            az, w = ig.blend_az(az_a, az_y, el, cal["theta_cross_deg"])
            out["az_from_yaw_deg"] = az_y
            out["az_deg"] = az
            out["az_blend_weight"] = w
        else:
            out["az_deg"] = az_a
            out["az_blend_weight"] = 1.0
        return out


class PicoLidar(PicoDevice):
    """Lidar distance sensor; also hosts the whole-system current monitor.

    The lidar Pico carries an ACS724 current sensor on GP26/ADC0 (it uses no
    other ADC). The firmware merges the raw ``current_voltage`` into the lidar
    status line; this class fans that out into a separate
    ``metadata['system_current']`` entry so the user-facing key never names
    lidar. The publish stays additive (raw ``current_voltage`` + derived
    ``current_a``) and scalar-only.
    """

    # ACS724-10AB (bidirectional, Vcc/2 zero point, 200 mV/A) read through a
    # 3.32k (top) / 4.64k (bottom) divider into GP26 (DMM-measured values).
    # These constants give the nominal transfer function used until a measured
    # calibration is uploaded.
    _DIVIDER_RATIO = 4.64 / (3.32 + 4.64)  # = 0.5829 (measured)
    _SENSOR_VQ = 2.5  # volts at 0 A (Vcc/2)
    _SENSOR_SENSITIVITY = 0.2  # volts per amp

    def __init__(self, *args, current_cal_store=None, **kwargs):
        """
        Parameters
        ----------
        current_cal_store : picohost.buses.CurrentCalStore, optional
            Redis-backed two-point current calibration. When provided and
            the store holds a cal, ``(V0, slope)`` is applied before the
            first status tick, so a rebooted lidar Pico comes up calibrated
            from Redis. Falls back to the nominal transfer function when
            absent. Other args/kwargs pass through to :class:`PicoDevice`.
        """
        # (V0, slope) at the ADC pin: I = (V_adc - V0) / slope. None ⇒
        # nominal conversion. Set before super().__init__ so the handler
        # never sees an undefined attribute.
        self._current_cal = None
        # Warn-once latch: trips if a lidar status line ever arrives without
        # current_voltage (firmware field renamed/dropped → system_current
        # silently goes stale). Set before super().__init__ for the same reason.
        self._warned_no_current = False
        super().__init__(*args, **kwargs)
        if current_cal_store is not None:
            cal = current_cal_store.get()
            if cal is not None and "system_current" in cal:
                self._current_cal = tuple(cal["system_current"])
        if self.redis_handler is not None:
            self._base_redis_handler = self.redis_handler
            self.redis_handler = self._lidar_redis_handler

    @property
    def is_current_calibrated(self):
        """True when a measured two-point current cal is loaded."""
        return self._current_cal is not None

    def set_calibration(self, system_current_params=None):
        """Set the two-point current calibration.

        Parameters
        ----------
        system_current_params : sequence of (float, float), optional
            ``(V0, slope)`` such that ``I = (V_adc - V0) / slope``.
            ``calibrate-current`` pushes this to the running device so a new
            cal takes effect on the next status tick without a restart.
        """
        if system_current_params is not None:
            self._current_cal = tuple(system_current_params)

    def _v_to_current(self, v_adc):
        """Convert the raw ADC-pin voltage to amps.

        Uses the measured two-point cal ``(V0, slope)`` when one is loaded,
        otherwise the nominal ACS724 + divider transfer function.
        """
        if self._current_cal is not None:
            v0, slope = self._current_cal
            return (v_adc - v0) / slope
        v_sensor = v_adc / self._DIVIDER_RATIO
        return (v_sensor - self._SENSOR_VQ) / self._SENSOR_SENSITIVITY

    def _lidar_redis_handler(self, data):
        """Split the merged lidar line into two metadata keys.

        ``metadata['lidar']`` keeps the distance reading (current stripped);
        ``metadata['system_current']`` carries the current. The current
        entry's status is hard-set to ``"update"`` — the ADC read is
        independent of lidar's I2C result, so a lidar failure must not mark
        the current reading errored.
        """
        data = data.copy()
        v = data.pop("current_voltage", None)
        self._base_redis_handler(data)
        if v is not None:
            self._base_redis_handler(
                {
                    "sensor_name": "system_current",
                    "status": "update",
                    "current_voltage": v,
                    "current_a": self._v_to_current(v),
                }
            )
        elif (
            data.get("sensor_name") == "lidar" and not self._warned_no_current
        ):
            # The firmware↔host contract for current rides on this field name.
            # If it vanishes, system_current stops updating with no other
            # symptom — surface it once rather than failing silently.
            self._warned_no_current = True
            self.logger.warning(
                "lidar status line missing 'current_voltage'; system_current "
                "will go stale. Firmware field renamed or dropped?"
            )


class PicoPotentiometer(PicoDevice):
    """Potentiometer monitoring device with voltage-to-angle calibration."""

    def __init__(
        self,
        port,
        calibration_file=None,
        pot_cal_store=None,
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
            Path to a JSON calibration file. Used only when Redis has
            no calibration (or no ``pot_cal_store`` was provided).
        pot_cal_store : picohost.buses.PotCalStore, optional
            Redis-backed calibration store. When provided and the
            store holds a calibration, it takes precedence over
            ``calibration_file`` — the cal is applied before the
            first status tick, so a pot Pico that reboots picks its
            cal up from Redis without any on-disk file.
        timeout : float
            Serial read timeout in seconds (default: 5.0).
        name : str, optional
        metadata_writer : eigsep_redis.MetadataWriter, optional
            Metadata bus writer. ``None`` disables Redis publication.
        usb_serial : str, optional
            USB serial number for port re-discovery.
        """
        self._cal = {"pot_az": None}
        super().__init__(
            port,
            timeout=timeout,
            name=name,
            metadata_writer=metadata_writer,
            usb_serial=usb_serial,
        )
        # Cal source precedence: Redis wins, JSON fallback,
        # uncalibrated if neither (matches the project decision that
        # Redis is the canonical cal store).
        cal_from_redis = None
        if pot_cal_store is not None:
            cal_from_redis = pot_cal_store.get()
        if cal_from_redis is not None:
            if "pot_az" in cal_from_redis:
                self._cal["pot_az"] = tuple(cal_from_redis["pot_az"])
        elif calibration_file is not None:
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
        for key in ("pot_az",):
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

    def set_calibration(self, pot_az_params=None):
        """Set calibration parameters (m, b) for the az pot.

        Parameters
        ----------
        pot_az_params : tuple of (float, float), optional
            (slope, intercept) such that angle = m * voltage + b.
        """
        if pot_az_params is not None:
            self._cal["pot_az"] = tuple(pot_az_params)

    def load_calibration(self, path):
        """Load calibration from a JSON file.

        Expected format: ``{"pot_az": [m, b], ...}``
        """
        with open(path, "r") as f:
            cal_data = json.load(f)
        if "pot_az" in cal_data:
            self._cal["pot_az"] = tuple(cal_data["pot_az"])

    @property
    def is_calibrated(self):
        """True if the az pot has calibration parameters."""
        return self._cal["pot_az"] is not None

    def read_voltage(self):
        """Return the latest az voltage reading.

        Returns
        -------
        dict
            ``{"pot_az_voltage": float}``
        """
        return {
            "pot_az_voltage": self.last_status.get("pot_az_voltage"),
        }

    def read_angle(self):
        """Convert current az voltage reading to angle using calibration.

        Returns
        -------
        dict
            ``{"pot_az": float}`` in degrees.

        Raises
        ------
        RuntimeError
            If calibration has not been set or voltage data is missing.
        """
        result = {}
        for key in ("pot_az",):
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
