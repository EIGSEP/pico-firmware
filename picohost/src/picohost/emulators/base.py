import json
import math
import threading
import time
import logging

logger = logging.getLogger(__name__)


def _safe_int(val, default=0):
    """Convert to int, returning *default* on failure.

    Matches cJSON behaviour: ``valueint`` silently returns 0 for
    non-numeric JSON types (strings, arrays, objects).
    """
    if isinstance(val, (str, bytes, list, dict)):
        return default
    try:
        return int(val)
    except (ValueError, TypeError):
        return default


def _cjson_number(value):
    """Reshape one JSON value the way firmware cJSON prints it.

    cJSON's print_number emits a whole-valued double with no decimal
    point ("30" not "30.0") — via %d when the value fits in a C int and
    via %1.15g (which also drops the point for whole values) above that
    — so the host's json.loads yields int for KV_FLOAT fields whenever
    the reading lands on a whole value (issue #148). %1.15g switches to
    exponent notation at 1e15, where the text parses back as float.
    NaN/inf print as JSON null in cJSON; Python's json.dumps would emit
    the non-JSON tokens 'NaN'/'Infinity' instead, so map those to None.
    """
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        if value.is_integer() and abs(value) < 1e15:
            return int(value)
    return value


def _safe_float(val, default=0.0):
    """Convert to float, returning *default* on failure.

    Matches cJSON behaviour: ``valuedouble`` silently returns 0.0 for
    non-numeric JSON types (strings, arrays, objects).
    """
    if isinstance(val, (str, bytes, list, dict)):
        return default
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


class PicoEmulator:
    """Models the C firmware's four-phase execution loop.

    Each subclass implements init(), server(), op(), and get_status()
    to match the corresponding C firmware app.
    """

    def __init__(self, app_id=0, status_cadence_ms=200.0):
        self.app_id = app_id
        self.status_cadence_ms = status_cadence_ms
        self._peer = None
        self._running = False
        self._thread = None
        self._cmd_buffer = ""
        self.init()

    def attach(self, serial_peer):
        """Connect to the firmware side of a MockSerial pair."""
        self._peer = serial_peer

    def init(self):
        """One-time initialization. Override in subclasses."""
        pass

    def server(self, cmd):
        """Process a JSON command dict. Override in subclasses."""
        pass

    def op(self):
        """Advance simulation state. Override in subclasses."""
        pass

    def get_status(self):
        """Return status dict(s). Override in subclasses.

        May return a single dict or a list of dicts (for composite emulators
        like RFSwitch+IMU that send multiple status messages per cadence).
        """
        return {}

    def start(self):
        """Start the background emulator thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the background emulator thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run_loop(self):
        """Main emulator loop mirroring the C main() loop."""
        next_status = time.monotonic() + self.status_cadence_ms / 1000.0

        while self._running:
            # 1. Non-blocking read from peer serial (check for host commands)
            self._read_commands()

            # 2. Advance state
            self.op()

            # 3. Send status at cadence interval
            now = time.monotonic()
            if now >= next_status:
                self._send_status()
                next_status = now + self.status_cadence_ms / 1000.0

            time.sleep(0.001)  # yield

    def _read_commands(self):
        """Non-blocking read of commands from the peer serial."""
        if self._peer is None:
            return

        try:
            avail = self._peer.in_waiting
            if avail > 0:
                data = self._peer.read(avail)
                if data:
                    self._cmd_buffer += data.decode("utf-8", errors="ignore")
        except Exception:
            return

        # Process complete lines
        while "\n" in self._cmd_buffer:
            line, self._cmd_buffer = self._cmd_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                cmd = json.loads(line)
                self.server(cmd)
            except json.JSONDecodeError:
                pass

    def _send_status(self):
        """Write status JSON to the peer serial."""
        if self._peer is None:
            return

        status = self.get_status()

        if isinstance(status, list):
            for s in status:
                self._write_json(s)
        else:
            self._write_json(status)

    def _write_json(self, data):
        """Write a single JSON dict to the peer serial.

        Factored out of _send_status so composite emulators (e.g.
        RFSwitch+IMU) can send multiple status dicts per cadence.
        """
        if not data or self._peer is None:
            return
        try:
            # Serialize numbers the way firmware cJSON does, so tests
            # driving devices through emulators see the same shapes as
            # real hardware (whole-valued floats arrive as JSON ints).
            line = (
                json.dumps(
                    {k: _cjson_number(v) for k, v in data.items()},
                    separators=(",", ":"),
                )
                + "\n"
            )
            self._peer.write(line.encode("utf-8"))
        except Exception as e:
            logger.debug(f"Emulator write error: {e}")
