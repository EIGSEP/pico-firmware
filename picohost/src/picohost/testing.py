import json
import logging

try:
    import mockserial
except ImportError:
    logging.warning("Mockserial not found, dummy devices will not work")

from .base import PicoDevice, PicoRFSwitch, PicoPeltier
from .motor import PicoMotor


class MockRedis:
    """Mock Redis client for testing purposes.

    Provides stubs for standard Redis operations used by PicoManager
    and PicoDevice.
    """

    def __init__(self):
        self._sets = {}
        self._hashes = {}
        self._keys = {}
        self._streams = {}

    def add_metadata(self, name, data):
        pass

    # -- Set operations --

    def sadd(self, key, *values):
        self._sets.setdefault(key, set()).update(values)

    def srem(self, key, *values):
        if key in self._sets:
            self._sets[key] -= set(values)

    def smembers(self, key):
        return self._sets.get(key, set())

    # -- Hash operations --

    def hset(self, name, key=None, value=None, mapping=None):
        self._hashes.setdefault(name, {})
        if key is not None:
            self._hashes[name][key] = value
        if mapping:
            self._hashes[name].update(mapping)

    def hget(self, name, key):
        return self._hashes.get(name, {}).get(key)

    def hgetall(self, name):
        return dict(self._hashes.get(name, {}))

    # -- Key operations --

    def set(self, key, value, ex=None):
        self._keys[key] = value

    def get(self, key):
        return self._keys.get(key)

    def delete(self, *keys):
        for key in keys:
            self._keys.pop(key, None)

    # -- Stream operations --

    def xadd(self, stream, fields, maxlen=None):
        self._streams.setdefault(stream, [])
        msg_id = f"{len(self._streams[stream])}"
        self._streams[stream].append((msg_id, fields))
        return msg_id

    def xread(self, streams, block=None, count=None):
        return []


def _get_mock_redis(eig_redis=None):
    """Helper function to get a mock Redis instance if none provided."""
    return eig_redis if eig_redis is not None else MockRedis()


class DummyPicoDevice(PicoDevice):

    def __init__(self, port, eig_redis=None, **kwargs):
        """
        Initialize dummy device with optional eig_redis.

        For testing, eig_redis can be None since we don't actually upload data.
        """
        super().__init__(port, _get_mock_redis(eig_redis), **kwargs)

    def connect(self):
        self.ser = mockserial.MockSerial()
        # MockSerial needs a peer to be considered "open"
        peer = mockserial.MockSerial()
        self.ser.add_peer(peer)
        peer.add_peer(self.ser)
        self.ser.reset_input_buffer()
        return True

    def start(self):
        """Override start to not create a reader thread for dummy devices."""
        # For dummy devices, we don't need a background thread
        self._running = True


class DummyPicoMotor(DummyPicoDevice, PicoMotor):
    def __init__(self, port, eig_redis=None, **kwargs):
        """Initialize dummy motor with optional eig_redis."""
        super().__init__(port, eig_redis, **kwargs)

    def wait_for_updates(self, timeout=10):
        """Override to provide immediate dummy status for tests."""
        self.status = {
            "az_pos": 0,
            "el_pos": 0,
            "az_target_pos": 0,
            "el_target_pos": 0,
            "az_speed": 100,
            "el_speed": 100,
        }


class DummyPicoRFSwitch(DummyPicoDevice, PicoRFSwitch):
    pass


class DummyPicoPeltier(DummyPicoDevice, PicoPeltier):

    def wait_for_updates(self, timeout=3):
        """Override to provide immediate dummy status for tests."""
        self.status = {
            "temperature": 25.0,
            "target_temperature": 25.0,
            "mode": "off",
            "power": 0.0,
        }


# --- PicoManager testing support ---

# Dummy class mapping (mirrors PICO_CLASSES in manager.py)
DUMMY_PICO_CLASSES = {
    "motor": DummyPicoMotor,
    "imu": DummyPicoDevice,
    "therm": DummyPicoDevice,
    "peltier": DummyPicoPeltier,
    "lidar": DummyPicoDevice,
    "switch": DummyPicoRFSwitch,
}


class DummyPicoManager:
    """PicoManager replacement using dummy devices for testing.

    Provides the same command-processing interface as PicoManager
    without real serial ports or Redis. Use add_dummy_device() to
    register devices manually.
    """

    def __init__(self, eig_redis=None):
        from .manager import PICOS_SET

        self.eig_redis = _get_mock_redis(eig_redis)
        self.picos = {}
        self._running = False
        self.logger = logging.getLogger(__name__)
        self._picos_set_key = PICOS_SET

    def _redis(self):
        if hasattr(self.eig_redis, "r"):
            return self.eig_redis.r
        return self.eig_redis

    @staticmethod
    def _decode(value):
        if isinstance(value, bytes):
            return value.decode("utf-8")
        return str(value) if value is not None else ""

    def add_dummy_device(self, name):
        """Add a dummy pico device by name.

        Parameters
        ----------
        name : str
            Device name (e.g. "switch", "motor", "peltier").

        Returns
        -------
        PicoDevice
            The instantiated dummy device.
        """
        cls = DUMMY_PICO_CLASSES.get(name, DummyPicoDevice)
        pico = cls(
            port=f"/dev/dummy_{name}",
            eig_redis=self.eig_redis,
            name=name,
        )
        self.picos[name] = pico
        r = self._redis()
        r.sadd(self._picos_set_key, name)
        return pico

    def discover(self):
        pass

    def _process_command(self, r, msg_id, fields):
        """Process a command (delegates to manager module logic)."""
        from .manager import (
            RESP_STREAM, CLAIM_TTL, _BLOCKED_ACTIONS,
        )
        f = {
            self._decode(k): self._decode(v)
            for k, v in fields.items()
        }
        target = f.get("target", "")
        source = f.get("source", "unknown")
        cmd_raw = f.get("cmd", "{}")

        try:
            cmd = json.loads(cmd_raw)
        except json.JSONDecodeError:
            r.xadd(RESP_STREAM, {
                "target": target,
                "status": "error",
                "data": json.dumps({"error": "invalid JSON"}),
            })
            return

        pico = self.picos.get(target)
        if pico is None:
            r.xadd(RESP_STREAM, {
                "target": target,
                "status": "error",
                "data": json.dumps(
                    {"error": f"unknown target: {target}"}
                ),
            })
            return

        resp = {"target": target, "source": source}
        claim_key = f"pico_claim:{target}"
        current_owner = r.get(claim_key)
        if current_owner is not None:
            current_owner = self._decode(current_owner)
            if current_owner != source:
                resp["warning"] = (
                    f"overriding claim by {current_owner}"
                )

        action = cmd.get("action")
        if action == "claim":
            ttl = cmd.get("ttl", CLAIM_TTL)
            r.set(claim_key, source, ex=int(ttl))
            resp.update({
                "status": "ok",
                "data": json.dumps(
                    {"claimed": target, "ttl": ttl}
                ),
            })
            r.xadd(RESP_STREAM, resp)
            return
        if action == "release":
            r.delete(claim_key)
            resp.update({
                "status": "ok",
                "data": json.dumps({"released": target}),
            })
            r.xadd(RESP_STREAM, resp)
            return

        try:
            result = self._route_command(pico, target, cmd)
            resp.update({
                "status": "ok",
                "data": json.dumps(
                    result if result is not None else {}
                ),
            })
        except Exception as e:
            resp.update({
                "status": "error",
                "data": json.dumps({"error": str(e)}),
            })
        r.xadd(RESP_STREAM, resp)

    def _route_command(self, pico, target, cmd):
        from .manager import _BLOCKED_ACTIONS

        action = cmd.pop("action", None)
        if action is None:
            success = pico.send_command(cmd)
            if not success:
                raise RuntimeError("send_command failed")
            return {"sent": True}

        if action in _BLOCKED_ACTIONS or action.startswith("_"):
            raise ValueError(f"Action '{action}' is not allowed")

        method = getattr(pico, action, None)
        if method is None or not callable(method):
            raise ValueError(
                f"Unknown action '{action}' for {target}"
            )
        result = method(**cmd)
        return {"action": action, "result": result}

    def start(self):
        self._running = True

    def stop(self):
        self._running = False
        for pico in self.picos.values():
            try:
                pico.disconnect()
            except Exception:
                pass
        self.picos.clear()
