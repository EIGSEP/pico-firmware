"""
PicoManager - standalone service that owns all pico serial connections.

The manager discovers devices from a ``pico_config.json`` produced by
``flash-picos``, instantiates the matching :class:`PicoDevice` subclass
for each one, monitors device health (reconnecting on serial drops),
and relays commands from a Redis stream so that other processes can
talk to picos without holding the serial port themselves.

Usage:
    python -m picohost.manager [--config pico_config.json] [--log-level INFO]
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
from pathlib import Path

from .base import (
    PicoDevice,
    PicoIMU,
    PicoLidar,
    PicoPeltier,
    PicoPotentiometer,
    PicoRFSwitch,
)
from .motor import PicoMotor

logger = logging.getLogger(__name__)

# Map firmware app_id (from src/pico_multi.h) to a logical device name.
# Names are used as Redis keys and as the "target" field in command
# stream entries — they must stay stable across releases.
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

# Redis keys
PICOS_SET = "picos"
HEALTH_HASH = "pico_health"
CONFIG_HASH = "pico_config"
CMD_STREAM = "stream:pico_cmd"
RESP_STREAM = "stream:pico_resp"

# Timing
HEALTH_CHECK_INTERVAL = 5.0  # seconds between health checks
HEALTH_TIMEOUT = 10.0  # seconds without status before unhealthy
CLAIM_TTL = 300  # default soft-claim TTL in seconds

# Methods that must not be invoked via the command stream. These are
# either local lifecycle calls (no firmware effect, dangerous to expose),
# or blocking helpers that would stall the cmd thread.
_BLOCKED_ACTIONS = frozenset(
    {
        "connect",
        "disconnect",
        "reconnect",
        "start",
        "stop",
        "set_response_handler",
        "set_raw_handler",
        "wait_for_response",
        "wait_for_updates",
        "wait_for_start",
        "wait_for_stop",
        "find_pico_ports",
        "read_line",
        "parse_response",
        "update_status",
    }
)


class PicoManager:
    """
    Standalone service that owns all pico serial connections.

    Discovers devices from a config file, monitors their health, and
    relays commands from a Redis stream to the right device.
    """

    def __init__(
        self,
        eig_redis,
        config_file="pico_config.json",
        uf2_path="build/pico_multi.uf2",
        use_redis_config=True,
    ):
        """
        Parameters
        ----------
        eig_redis : EigsepRedis or redis.Redis
            Redis client used both as the source for incoming commands
            and as the publication target for status, health, and
            command responses. Either an :class:`EigsepRedis` (which
            exposes the underlying client as ``.r``) or a bare
            ``redis.Redis`` is accepted.
        config_file : str or Path
            Path to ``pico_config.json`` produced by ``flash-picos``.
        uf2_path : str or Path
            Path to the UF2 firmware file for auto-flashing.
        use_redis_config : bool
            If ``True`` (default), check Redis for existing config
            before falling back to the file/flash.
        """
        self.eig_redis = eig_redis
        self.config_file = Path(config_file)
        self.uf2_path = Path(uf2_path)
        self.use_redis_config = use_redis_config
        self.picos = {}
        self._running = False
        self._health_thread = None
        self._cmd_thread = None
        self.logger = logger

    def _redis(self):
        """Return the underlying ``redis.Redis`` client."""
        if hasattr(self.eig_redis, "r"):
            return self.eig_redis.r
        return self.eig_redis

    @staticmethod
    def _decode(value):
        """Decode bytes to str if needed; tolerate non-bytes input."""
        if isinstance(value, bytes):
            return value.decode("utf-8")
        return str(value) if value is not None else ""

    # --- Discovery & Config ---

    def discover(self):
        """
        Discover pico devices using a cascading config strategy:

        1. Redis ``CONFIG_HASH`` (previous run persisted config)
        2. JSON config file on disk
        3. Flash-and-discover (if UF2 exists)

        After any source succeeds the config is written back to both
        Redis (via :meth:`_register_devices`) and the JSON file (via
        :meth:`_save_config_to_file`).
        """
        devices = (
            self._load_config_from_redis() if self.use_redis_config else None
        )
        if devices:
            self.logger.info(f"Loaded {len(devices)} device(s) from Redis")
            self._register_devices(devices)
            self._save_config_to_file(devices)
            return

        devices = self._load_config_from_file()
        if devices:
            self.logger.info(
                f"Loaded {len(devices)} device(s) from config file"
            )
            self._register_devices(devices)
            self._save_config_to_file(devices)
            return

        self._try_flash_discover()
        if self.picos:
            devices = self._current_device_list()
            self._save_config_to_file(devices)

    def _load_config_from_redis(self):
        """
        Try to load device config from the Redis ``CONFIG_HASH``.

        Returns
        -------
        list[dict] or None
            Device config dicts if Redis has config, ``None`` otherwise.
        """
        r = self._redis()
        try:
            raw = r.hgetall(CONFIG_HASH)
        except Exception as e:
            self.logger.warning(f"Failed to read config from Redis: {e}")
            return None

        if not raw:
            return None

        devices = []
        for name, blob in raw.items():
            blob = self._decode(blob)
            try:
                devices.append(json.loads(blob))
            except json.JSONDecodeError:
                self.logger.warning(
                    f"Invalid JSON in Redis config for {self._decode(name)}"
                )
        return devices if devices else None

    def _load_config_from_file(self):
        """
        Read the device list from the JSON config file.

        Returns
        -------
        list[dict] or None
            Device config dicts if the file is valid, ``None`` otherwise.
        """
        if not self.config_file.exists():
            self.logger.warning(f"Config file {self.config_file} not found")
            return None

        with open(self.config_file) as f:
            try:
                devices = json.load(f)
            except json.JSONDecodeError as e:
                self.logger.error(f"Invalid JSON in config file: {e}")
                return None

        return devices if devices else None

    def _save_config_to_file(self, devices):
        """Persist the device list to the JSON config file."""
        try:
            with open(self.config_file, "w") as f:
                json.dump(devices, f, indent=2)
            self.logger.debug(
                f"Wrote {len(devices)} device(s) to {self.config_file}"
            )
        except OSError as e:
            self.logger.warning(
                f"Could not write config file {self.config_file}: {e}"
            )

    def _current_device_list(self):
        """Build a device list from currently registered picos."""
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
        """
        Instantiate the matching :class:`PicoDevice` subclass for each
        entry in *devices* and publish to Redis.
        """
        r = self._redis()
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
            try:
                pico = cls(
                    port,
                    eig_redis=self.eig_redis,
                    name=name,
                    usb_serial=usb_serial,
                )
                self.picos[name] = pico
                r.sadd(PICOS_SET, name)
                r.hset(
                    CONFIG_HASH,
                    name,
                    json.dumps(
                        {
                            "port": port,
                            "app_id": app_id,
                            "usb_serial": usb_serial,
                        }
                    ),
                )
                r.hset(
                    HEALTH_HASH,
                    name,
                    json.dumps(
                        {
                            "connected": True,
                            "last_seen": time.time(),
                            "app_id": app_id,
                        }
                    ),
                )
                self.logger.info(
                    f"Discovered {name} (app_id={app_id}) on {port}"
                )
            except Exception as e:
                self.logger.error(f"Failed to init {name} on {port}: {e}")

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
            self._check_health()
            time.sleep(HEALTH_CHECK_INTERVAL)

    def _check_health(self):
        """Run one iteration of health checks for all picos."""
        r = self._redis()
        for name, pico in list(self.picos.items()):
            connected = pico.is_connected
            last_seen = pico.last_status_time or 0
            now = time.time()
            stale = (now - last_seen) > HEALTH_TIMEOUT if last_seen else True
            healthy = connected and not stale

            if not healthy:
                self.logger.warning(
                    f"{name}: unhealthy (connected={connected}, stale={stale})"
                )
                try:
                    old_port = pico.port
                    if pico.reconnect():
                        self.logger.info(f"{name}: reconnected")
                        r.sadd(PICOS_SET, name)
                        connected = True
                        if pico.port != old_port:
                            app_id = APP_IDS.get(name, -1)
                            r.hset(
                                CONFIG_HASH,
                                name,
                                json.dumps(
                                    {
                                        "port": pico.port,
                                        "app_id": app_id,
                                        "usb_serial": pico.usb_serial,
                                    }
                                ),
                            )
                            self._save_config_to_file(
                                self._current_device_list()
                            )
                    else:
                        r.srem(PICOS_SET, name)
                        connected = False
                except Exception as e:
                    self.logger.error(f"{name}: reconnect failed: {e}")
                    r.srem(PICOS_SET, name)
                    connected = False

            app_id = APP_IDS.get(name, -1)
            r.hset(
                HEALTH_HASH,
                name,
                json.dumps(
                    {
                        "connected": connected,
                        "last_seen": pico.last_status_time or 0,
                        "app_id": app_id,
                    }
                ),
            )

    # --- Command Relay ---

    def cmd_loop(self):
        """Listen for incoming pico commands on the Redis stream."""
        r = self._redis()
        last_id = "$"  # only read new messages
        while self._running:
            try:
                result = r.xread({CMD_STREAM: last_id}, block=1000, count=10)
                if not result:
                    continue
                for _stream, messages in result:
                    for msg_id, fields in messages:
                        last_id = msg_id
                        self._process_command(r, msg_id, fields)
            except Exception as e:
                if self._running:
                    self.logger.error(f"cmd_loop error: {e}")
                    time.sleep(1)

    def _process_command(self, r, msg_id, fields):
        """Validate and dispatch a single command stream entry."""
        f = {self._decode(k): self._decode(v) for k, v in fields.items()}
        target = f.get("target", "")
        source = f.get("source", "unknown")
        cmd_raw = f.get("cmd", "{}")

        try:
            cmd = json.loads(cmd_raw)
        except json.JSONDecodeError:
            self.logger.error(f"Invalid JSON in command: {cmd_raw}")
            r.xadd(
                RESP_STREAM,
                {
                    "target": target,
                    "source": source,
                    "status": "error",
                    "data": json.dumps({"error": "invalid JSON"}),
                },
            )
            return
        if not isinstance(cmd, dict):
            self.logger.error(f"Command must be a JSON object: {cmd_raw}")
            r.xadd(
                RESP_STREAM,
                {
                    "target": target,
                    "source": source,
                    "status": "error",
                    "data": json.dumps(
                        {"error": "command must be a JSON object"}
                    ),
                },
            )
            return

        if target == "manager":
            self._handle_manager_cmd(r, source, cmd)
            return

        pico = self.picos.get(target)
        if pico is None:
            self.logger.error(f"Unknown target: {target}")
            r.xadd(
                RESP_STREAM,
                {
                    "target": target,
                    "source": source,
                    "status": "error",
                    "data": json.dumps({"error": f"unknown target: {target}"}),
                },
            )
            return

        # Soft claims: warn (but allow) when a non-owner sends a command
        # to a claimed device. Claims are advisory, not enforced.
        resp = {"target": target, "source": source}
        claim_key = f"pico_claim:{target}"
        current_owner = r.get(claim_key)
        if current_owner is not None:
            current_owner = self._decode(current_owner)
            if current_owner != source:
                warning = f"overriding claim by {current_owner}"
                self.logger.warning(
                    f"{target}: {source} overrides claim by {current_owner}"
                )
                resp["warning"] = warning

        action = cmd.get("action")
        if action == "claim":
            ttl = cmd.get("ttl", CLAIM_TTL)
            try:
                ttl = int(ttl)
            except (ValueError, TypeError):
                r.xadd(
                    RESP_STREAM,
                    {
                        "target": target,
                        "source": source,
                        "status": "error",
                        "data": json.dumps({"error": f"invalid ttl: {ttl!r}"}),
                    },
                )
                return
            r.set(claim_key, source, ex=ttl)
            resp.update(
                {
                    "status": "ok",
                    "data": json.dumps({"claimed": target, "ttl": ttl}),
                }
            )
            r.xadd(RESP_STREAM, resp)
            return
        if action == "release":
            r.delete(claim_key)
            resp.update(
                {
                    "status": "ok",
                    "data": json.dumps({"released": target}),
                }
            )
            r.xadd(RESP_STREAM, resp)
            return

        try:
            result = self._route_command(pico, target, cmd)
            resp.update(
                {
                    "status": "ok",
                    "data": json.dumps(result if result is not None else {}),
                }
            )
        except Exception as e:
            self.logger.error(f"Command failed on {target}: {e}")
            resp.update(
                {
                    "status": "error",
                    "data": json.dumps({"error": str(e)}),
                }
            )
        r.xadd(RESP_STREAM, resp)

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

    def _handle_manager_cmd(self, r, source, cmd):
        """Handle commands targeted at the manager itself."""
        action = cmd.get("action", "")
        resp = {"target": "manager", "source": source}

        if action == "rediscover":
            self.logger.info(f"Rediscover requested by {source}")
            try:
                for name, pico in list(self.picos.items()):
                    try:
                        pico.disconnect()
                    except Exception:
                        pass
                    r.srem(PICOS_SET, name)
                self.picos.clear()
                self.discover()
                device_names = list(self.picos.keys())
                resp.update(
                    {
                        "status": "ok",
                        "data": json.dumps(
                            {
                                "devices": device_names,
                                "count": len(device_names),
                            }
                        ),
                    }
                )
            except Exception as e:
                self.logger.error(f"Rediscover failed: {e}")
                resp.update(
                    {
                        "status": "error",
                        "data": json.dumps({"error": str(e)}),
                    }
                )
        else:
            resp.update(
                {
                    "status": "error",
                    "data": json.dumps(
                        {"error": f"unknown manager action: {action}"}
                    ),
                }
            )

        r.xadd(RESP_STREAM, resp)

    # --- Lifecycle ---

    def start(self):
        """Start the health monitor and command relay threads."""
        self._running = True
        self._health_thread = threading.Thread(
            target=self.health_loop, daemon=True, name="health"
        )
        self._cmd_thread = threading.Thread(
            target=self.cmd_loop, daemon=True, name="cmd"
        )
        self._health_thread.start()
        self._cmd_thread.start()
        self.logger.info("PicoManager started")

    def stop(self):
        """Graceful shutdown: stop threads, disconnect picos, clear Redis."""
        self.logger.info("PicoManager stopping...")
        self._running = False
        if self._health_thread:
            self._health_thread.join(timeout=HEALTH_CHECK_INTERVAL + 1)
        if self._cmd_thread:
            self._cmd_thread.join(timeout=2)

        r = self._redis()
        for name, pico in self.picos.items():
            try:
                pico.disconnect()
                r.srem(PICOS_SET, name)
                r.hset(
                    HEALTH_HASH,
                    name,
                    json.dumps(
                        {
                            "connected": False,
                            "last_seen": 0,
                            "app_id": APP_IDS.get(name, -1),
                        }
                    ),
                )
            except Exception as e:
                self.logger.error(f"Error stopping {name}: {e}")
        self.picos.clear()
        self.logger.info("PicoManager stopped")

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
        "--config",
        default="pico_config.json",
        help="Path to pico_config.json (default: pico_config.json)",
    )
    parser.add_argument(
        "--uf2",
        default="build/pico_multi.uf2",
        help="Path to pico_multi.uf2 for auto-flashing "
        "(default: build/pico_multi.uf2)",
    )
    parser.add_argument(
        "--no-redis-config",
        action="store_true",
        help="Skip Redis config lookup, re-discover from file/flash",
    )
    parser.add_argument(
        "--clear-config",
        action="store_true",
        help="Clear stored config from Redis before discovering",
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

    try:
        from eigsep_observing import EigsepRedis
    except ImportError:
        print(
            "eigsep_observing is required to run PicoManager.\n"
            "Install it with: pip install eigsep_observing",
            file=sys.stderr,
        )
        sys.exit(1)

    r = EigsepRedis()
    if args.clear_config:
        r.r.delete(CONFIG_HASH)
    mgr = PicoManager(
        r,
        config_file=args.config,
        uf2_path=args.uf2,
        use_redis_config=not args.no_redis_config,
    )
    mgr.run()


if __name__ == "__main__":
    main()
