"""
PicoManager - standalone service that owns all pico serial connections.

Redis is the source of truth. ``flash-picos`` writes the discovered
device list into ``PicoConfigStore`` on the host side; the manager
boots, reads that list, instantiates the matching :class:`PicoDevice`
subclass per entry, and then exposes each device to the rest of the
system through four bus surfaces (all from ``eigsep_redis`` /
``picohost.buses``):

- :class:`eigsep_redis.MetadataWriter` — per-sensor firmware status
  (the 200 ms JSON packets) is republished onto ``stream:{sensor}``
  and the ``metadata`` snapshot hash, same as every other sensor in
  the system.
- :class:`eigsep_redis.HeartbeatWriter` — per-device liveness under
  ``heartbeat:pico:{name}`` with a TTL, so a consumer that loses its
  view of a pico (or of the manager itself) detects it within the
  TTL window via :class:`eigsep_redis.HeartbeatReader.check`.
- :class:`eigsep_redis.StatusWriter` — manager-level log events
  (discover, reconnect, stop) land on ``stream:status`` alongside
  every other service's events.
- :class:`picohost.buses.PicoCmdReader` /
  :class:`picohost.buses.PicoRespWriter` /
  :class:`picohost.buses.PicoClaimStore` — command/response stream
  with soft claim ownership, the one surface that is picohost-specific
  and has no eigsep_observing analogue.

Usage:
    python -m picohost.manager [--uf2 build/pico_multi.uf2] [--log-level INFO]
"""

import argparse
import json
import logging
import signal
import threading
import time
from pathlib import Path

from eigsep_redis import (
    HeartbeatWriter,
    MetadataWriter,
    StatusWriter,
    Transport,
)

from .base import (
    PicoDevice,
    PicoIMU,
    PicoLidar,
    PicoPeltier,
    PicoPotentiometer,
    PicoRFSwitch,
)
from .buses import (
    PicoClaimStore,
    PicoCmdReader,
    PicoConfigStore,
    PicoRespWriter,
    PotCalStore,
)
from .keys import PICO_CMD_STREAM, pico_heartbeat_name
from .motor import PicoMotor

logger = logging.getLogger(__name__)

# Map firmware app_id (from src/pico_multi.h) to a logical device name.
# Names are used as metadata/heartbeat keys and as the "target" field in
# command stream entries — they must stay stable across releases.
APP_NAMES = {
    0: "motor",
    1: "tempctrl",
    2: "potmon",
    3: "imu_el",
    4: "lidar",
    5: "rfswitch",
    6: "imu_az",
}

# Inverse mapping: name -> app_id
APP_IDS = {v: k for k, v in APP_NAMES.items()}

# Map device name to picohost class.
PICO_CLASSES = {
    "motor": PicoMotor,
    "tempctrl": PicoPeltier,
    "potmon": PicoPotentiometer,
    "imu_el": PicoIMU,
    "lidar": PicoLidar,
    "imu_az": PicoIMU,
    "rfswitch": PicoRFSwitch,
}

# Timing
HEALTH_CHECK_INTERVAL = 5.0  # seconds between health checks
HEALTH_TIMEOUT = 10.0  # seconds without status before unhealthy
# Heartbeat TTL is 4× the check interval so a single missed tick
# (or a reconnection storm that blocks one pass) doesn't expire the
# key before the next tick has a chance to re-assert liveness.
HEARTBEAT_TTL = int(HEALTH_CHECK_INTERVAL * 4)
CLAIM_TTL = 300  # default soft-claim TTL in seconds

# Methods that must not be invoked via the command stream. These are
# either local lifecycle calls (no firmware effect, dangerous to expose),
# or blocking helpers that would stall the cmd thread.
_BLOCKED_ACTIONS = frozenset(
    {
        "connect",
        "disconnect",
        "reconnect",
        "set_response_handler",
        "set_raw_handler",
        "wait_for_response",
        "wait_for_start",
        "wait_for_stop",
        "find_pico_ports",
        "read_line",
        "parse_response",
    }
)


class PicoManager:
    """
    Standalone service that owns all pico serial connections.

    Discovers devices from the Redis :class:`PicoConfigStore`,
    monitors their health via per-device heartbeats, and relays
    commands from :class:`PicoCmdReader` to the right
    :class:`PicoDevice`.
    """

    def __init__(
        self,
        transport,
        uf2_path="build/pico_multi.uf2",
    ):
        """
        Parameters
        ----------
        transport : eigsep_redis.Transport
            Shared transport used to construct every bus writer/reader
            this manager needs. The same instance underpins
            :class:`MetadataWriter` (passed to each ``PicoDevice``),
            per-device :class:`HeartbeatWriter`,
            :class:`StatusWriter`, :class:`PicoConfigStore`,
            :class:`PotCalStore` (passed to ``PicoPotentiometer``),
            :class:`PicoCmdReader`, :class:`PicoRespWriter`, and
            :class:`PicoClaimStore`.
        uf2_path : str or Path
            Path to the UF2 firmware file for auto-flashing when
            Redis is empty at boot.
        """
        self.transport = transport
        self.uf2_path = Path(uf2_path)
        self.picos = {}
        self._heartbeats = {}
        self._metadata_writer = MetadataWriter(transport)
        self._config_store = PicoConfigStore(transport)
        self._pot_cal_store = PotCalStore(transport)
        self._cmd_reader = PicoCmdReader(transport)
        self._resp_writer = PicoRespWriter(transport)
        self._claim_store = PicoClaimStore(transport)
        self._status_writer = StatusWriter(transport)
        self._running = False
        self._stop_event = threading.Event()
        self._health_thread = None
        self._cmd_thread = None
        # Serializes mutations of self.picos / self._heartbeats between
        # the health thread and the cmd thread (rediscover). Without it,
        # a rediscover teardown can run concurrently with an in-flight
        # reconnect on the same serial connection, and heartbeats popped
        # by rediscover can be reasserted alive by a lingering health
        # iteration.
        self._lock = threading.Lock()
        self.logger = logger

    @staticmethod
    def _decode(value):
        """Decode bytes to str if needed; tolerate non-bytes input."""
        if isinstance(value, bytes):
            return value.decode("utf-8")
        return str(value) if value is not None else ""

    def _status(self, msg, level=logging.INFO):
        """Log *msg* and mirror it onto the shared status stream."""
        self.logger.log(level, msg)
        try:
            self._status_writer.send(msg, level=level)
        except (
            Exception
        ) as e:  # pragma: no cover - don't mask bugs behind status
            self.logger.warning(f"Failed to publish status: {e}")

    # --- Discovery & Config ---

    def discover(self):
        """
        Resolve the device list and instantiate devices.

        1. Read :class:`PicoConfigStore` from Redis. If non-empty,
           register those devices.
        2. If empty, run :func:`flash_picos.flash_and_discover` and
           publish the result back into the config store.
        """
        devices = self._config_store.get()
        if devices:
            self._status(f"Loaded {len(devices)} device(s) from Redis")
            self._register_devices(devices)
            return

        self._try_flash_discover()
        if self.picos:
            self._config_store.upload(self._current_device_list())

    def _current_device_list(self):
        """Build a list of device dicts from currently registered picos."""
        devices = []
        for name, pico in self.picos.items():
            app_id = APP_IDS.get(name, -1)
            devices.append(
                {
                    "app_id": app_id,
                    "port": pico.port,
                    "usb_serial": getattr(pico, "usb_serial", ""),
                }
            )
        return devices

    def _register_devices(self, devices):
        """Instantiate each :class:`PicoDevice` and publish an initial
        heartbeat."""
        for dev_info in devices:
            app_id = dev_info.get("app_id")
            port = dev_info.get("port")
            usb_serial = dev_info.get("usb_serial", "")

            if app_id is None or port is None:
                self.logger.warning(
                    f"Skipping incomplete device entry: {dev_info}"
                )
                continue

            name = APP_NAMES.get(app_id)
            if name is None:
                self.logger.warning(f"Unknown app_id {app_id}, skipping")
                continue

            if name in self.picos:
                raise ValueError(f"Duplicate device name '{name}' in config")

            cls = PICO_CLASSES.get(name, PicoDevice)
            kwargs = {
                "metadata_writer": self._metadata_writer,
                "name": name,
                "usb_serial": usb_serial,
            }
            if cls is PicoPotentiometer:
                kwargs["pot_cal_store"] = self._pot_cal_store
            try:
                pico = cls(port, **kwargs)
                self.picos[name] = pico
                self._heartbeats[name] = HeartbeatWriter(
                    self.transport, name=pico_heartbeat_name(name)
                )
                self._heartbeats[name].set(ex=HEARTBEAT_TTL, alive=True)
                self._status(f"Discovered {name} (app_id={app_id}) on {port}")
            except Exception as e:
                self._status(
                    f"Failed to init {name} on {port}: {e}",
                    level=logging.ERROR,
                )

    def _try_flash_discover(self):
        """Attempt to flash attached Picos and discover devices."""
        from .flash_picos import flash_and_discover

        try:
            devices = flash_and_discover(uf2_path=self.uf2_path)
        except FileNotFoundError:
            self.logger.warning(
                f"UF2 file {self.uf2_path} not found, skipping flash"
            )
            return
        except Exception as e:
            self.logger.error(f"Flash-and-discover failed: {e}")
            return

        if not devices:
            self.logger.warning("Flash produced no devices")
            return

        self._register_devices(devices)

    # --- Health Monitoring ---

    def health_loop(self):
        """Periodic health check thread."""
        while self._running:
            try:
                self._check_health()
            except Exception as e:
                if self._running:
                    self.logger.error(f"health_loop error: {e}")
            if self._stop_event.wait(HEALTH_CHECK_INTERVAL):
                break

    def _check_health(self):
        """Run one iteration of health checks for all picos."""
        with self._lock:
            for name, pico in list(self.picos.items()):
                connected = pico.is_connected
                last_seen = pico.last_status_time or 0
                now = time.time()
                stale = (
                    (now - last_seen) > HEALTH_TIMEOUT if last_seen else True
                )
                healthy = connected and not stale

                if not healthy:
                    self._status(
                        f"{name}: unhealthy "
                        f"(connected={connected}, stale={stale})",
                        level=logging.WARNING,
                    )
                    try:
                        old_port = pico.port
                        if pico.reconnect():
                            self._status(f"{name}: reconnected")
                            connected = True
                            if pico.port != old_port:
                                self._config_store.upload(
                                    self._current_device_list()
                                )
                        else:
                            connected = False
                    except Exception as e:
                        self._status(
                            f"{name}: reconnect failed: {e}",
                            level=logging.ERROR,
                        )
                        connected = False

                hb = self._heartbeats.get(name)
                if hb is not None:
                    hb.set(ex=HEARTBEAT_TTL, alive=connected)

    # --- Command Relay ---

    def cmd_loop(self):
        """Listen for incoming pico commands on the Redis stream."""
        while self._running:
            try:
                messages = self._cmd_reader.read(timeout=1.0, count=10)
                for msg_id, fields in messages:
                    if not self._running:
                        return
                    self._process_command(msg_id, fields)
            except Exception as e:
                if self._running:
                    self.logger.error(f"cmd_loop error: {e}")
                    time.sleep(1)

    def _process_command(self, msg_id, fields):
        """Validate and dispatch a single command stream entry."""
        f = {self._decode(k): self._decode(v) for k, v in fields.items()}
        target = f.get("target", "")
        source = f.get("source", "unknown")
        request_id = f.get("request_id", "")
        cmd_raw = f.get("cmd", "{}")

        def _err(error_msg):
            self._resp_writer.send(
                target=target,
                source=source,
                request_id=request_id,
                status="error",
                data={"error": error_msg},
            )

        try:
            cmd = json.loads(cmd_raw)
        except json.JSONDecodeError:
            self.logger.error(f"Invalid JSON in command: {cmd_raw}")
            _err("invalid JSON")
            return
        if not isinstance(cmd, dict):
            self.logger.error(f"Command must be a JSON object: {cmd_raw}")
            _err("command must be a JSON object")
            return

        if target == "manager":
            self._handle_manager_cmd(source, cmd, request_id)
            return

        pico = self.picos.get(target)
        if pico is None:
            self.logger.error(f"Unknown target: {target}")
            _err(f"unknown target: {target}")
            return

        # Soft claims: warn (but allow) when a non-owner sends a command
        # to a claimed device. Claims are advisory, not enforced.
        warning = None
        current_owner = self._claim_store.get(target)
        if current_owner is not None and current_owner != source:
            warning = f"overriding claim by {current_owner}"
            self.logger.warning(
                f"{target}: {source} overrides claim by {current_owner}"
            )

        action = cmd.get("action")
        if action == "claim":
            ttl = cmd.get("ttl", CLAIM_TTL)
            try:
                ttl = int(ttl)
            except (ValueError, TypeError):
                _err(f"invalid ttl: {ttl!r}")
                return
            self._claim_store.set(target, source, ttl)
            self._resp_writer.send(
                target=target,
                source=source,
                request_id=request_id,
                status="ok",
                data={"claimed": target, "ttl": ttl},
                warning=warning,
            )
            return
        if action == "release":
            self._claim_store.delete(target)
            self._resp_writer.send(
                target=target,
                source=source,
                request_id=request_id,
                status="ok",
                data={"released": target},
                warning=warning,
            )
            return

        try:
            result = self._route_command(pico, target, cmd)
            self._resp_writer.send(
                target=target,
                source=source,
                request_id=request_id,
                status="ok",
                data=result if result is not None else {},
                warning=warning,
            )
        except Exception as e:
            self.logger.error(f"Command failed on {target}: {e}")
            self._resp_writer.send(
                target=target,
                source=source,
                request_id=request_id,
                status="error",
                data={"error": str(e)},
                warning=warning,
            )

    def _route_command(self, pico, target, cmd):
        """
        Route a parsed command dict to the right pico method.

        ``cmd["action"]`` must name a public method on the device class.
        The method is invoked with the remaining fields as kwargs
        (subject to the :data:`_BLOCKED_ACTIONS` deny-list).
        """
        action = cmd.pop("action", None)
        if action is None:
            raise ValueError("'action' is required")

        if action in _BLOCKED_ACTIONS or action.startswith("_"):
            raise ValueError(f"Action '{action}' is not allowed")

        method = getattr(pico, action, None)
        if method is None or not callable(method):
            raise ValueError(f"Unknown action '{action}' for {target}")

        result = method(**cmd)
        return {"action": action, "result": result}

    def _handle_manager_cmd(self, source, cmd, request_id=""):
        """Handle commands targeted at the manager itself."""
        action = cmd.get("action", "")
        if action == "rediscover":
            self._status(f"Rediscover requested by {source}")
            try:
                with self._lock:
                    for name, pico in list(self.picos.items()):
                        try:
                            pico.disconnect()
                        except Exception:
                            pass
                        hb = self._heartbeats.pop(name, None)
                        if hb is not None:
                            hb.set(ex=HEARTBEAT_TTL, alive=False)
                    self.picos.clear()
                    self.discover()
                    device_names = list(self.picos.keys())
                self._resp_writer.send(
                    target="manager",
                    source=source,
                    request_id=request_id,
                    status="ok",
                    data={
                        "devices": device_names,
                        "count": len(device_names),
                    },
                )
            except Exception as e:
                self.logger.error(f"Rediscover failed: {e}")
                self._resp_writer.send(
                    target="manager",
                    source=source,
                    request_id=request_id,
                    status="error",
                    data={"error": str(e)},
                )
        else:
            self._resp_writer.send(
                target="manager",
                source=source,
                request_id=request_id,
                status="error",
                data={"error": f"unknown manager action: {action}"},
            )

    # --- Lifecycle ---

    def start(self):
        """Start the health monitor and command relay threads."""
        self._running = True
        self._stop_event.clear()
        self._health_thread = threading.Thread(
            target=self.health_loop, daemon=True, name="health"
        )
        self._cmd_thread = threading.Thread(
            target=self.cmd_loop, daemon=True, name="cmd"
        )
        self._health_thread.start()
        self._cmd_thread.start()
        self._status("PicoManager started")

    def stop(self):
        """Graceful shutdown: stop threads, disconnect picos, mark dead."""
        self._status("PicoManager stopping...")
        self._running = False
        self._stop_event.set()
        # Wake cmd_loop's blocking xread so it observes _running = False
        # without waiting out the full block timeout.
        try:
            self.transport.r.xadd(PICO_CMD_STREAM, {"__shutdown__": "1"})
        except Exception:
            pass
        if self._health_thread:
            self._health_thread.join(timeout=HEALTH_CHECK_INTERVAL + 1)
        if self._cmd_thread:
            self._cmd_thread.join(timeout=2)

        for name, pico in self.picos.items():
            try:
                pico.disconnect()
            except Exception as e:
                self.logger.error(f"Error stopping {name}: {e}")
            hb = self._heartbeats.get(name)
            if hb is not None:
                try:
                    hb.set(ex=HEARTBEAT_TTL, alive=False)
                except Exception as e:
                    self.logger.error(
                        f"Failed to clear heartbeat for {name}: {e}"
                    )
        self.picos.clear()
        self._heartbeats.clear()
        self._status("PicoManager stopped")

    def run(self):
        """Discover, start, and block until interrupted."""
        signal.signal(signal.SIGTERM, lambda *_: self.stop())
        self.discover()
        if not self.picos:
            self.logger.warning("No picos discovered, running anyway")
        self.start()
        try:
            while self._running:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            if self._running:
                self.stop()


def main():
    """Console-script and ``python -m picohost.manager`` entry point."""
    parser = argparse.ArgumentParser(description="EIGSEP Pico Manager")
    parser.add_argument(
        "--uf2",
        default="build/pico_multi.uf2",
        help="Path to pico_multi.uf2 for auto-flashing when Redis is empty "
        "(default: build/pico_multi.uf2)",
    )
    parser.add_argument(
        "--redis-host",
        default="localhost",
        help="Redis host (default: localhost)",
    )
    parser.add_argument(
        "--redis-port",
        type=int,
        default=6379,
        help="Redis port (default: 6379)",
    )
    parser.add_argument(
        "--clear-config",
        action="store_true",
        help="Clear PicoConfigStore before discovering",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    transport = Transport(host=args.redis_host, port=args.redis_port)
    mgr = PicoManager(transport, uf2_path=args.uf2)
    if args.clear_config:
        mgr._config_store.clear()
    mgr.run()


if __name__ == "__main__":
    main()
