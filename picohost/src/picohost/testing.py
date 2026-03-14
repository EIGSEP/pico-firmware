import logging

try:
    import mockserial
except ImportError:
    logging.warning("Mockserial not found, dummy devices will not work")

from .base import (
    PicoDevice, PicoRFSwitch, PicoPeltier, PicoTherm, PicoLidar,
)
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


class DummyPicoTherm(DummyPicoDevice, PicoTherm):

    def wait_for_updates(self, timeout=3):
        """Override to provide immediate dummy status for tests."""
        self.status = {"sensor_name": "therm"}


class DummyPicoLidar(DummyPicoDevice, PicoLidar):

    def wait_for_updates(self, timeout=3):
        """Override to provide immediate dummy status for tests."""
        self.status = {"sensor_name": "lidar"}


# --- PicoManager testing support ---

# Dummy class mapping (mirrors PICO_CLASSES in manager.py)
DUMMY_PICO_CLASSES = {
    "motor": DummyPicoMotor,
    "imu": DummyPicoDevice,
    "therm": DummyPicoTherm,
    "peltier": DummyPicoPeltier,
    "lidar": DummyPicoLidar,
    "switch": DummyPicoRFSwitch,
}


class DummyPicoManager:
    """PicoManager replacement using dummy devices for testing.

    Inherits command processing logic from PicoManager but does not
    start threads or read config files. Use add_dummy_device() to
    register devices manually.
    """

    def __init__(self, eig_redis=None):
        from .manager import PicoManager

        self._mgr = PicoManager.__new__(PicoManager)
        self._mgr.eig_redis = _get_mock_redis(eig_redis)
        self._mgr.picos = {}
        self._mgr._running = False
        self._mgr._health_thread = None
        self._mgr._cmd_thread = None
        self._mgr.logger = logging.getLogger(__name__)
        self._mgr.config_file = None

        # Expose same attributes for test access
        self.eig_redis = self._mgr.eig_redis
        self.picos = self._mgr.picos
        self._running = False
        self.logger = self._mgr.logger

    def _redis(self):
        return self._mgr._redis()

    @staticmethod
    def _decode(value):
        from .manager import PicoManager
        return PicoManager._decode(value)

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
        from .manager import PICOS_SET

        cls = DUMMY_PICO_CLASSES.get(name, DummyPicoDevice)
        pico = cls(
            port=f"/dev/dummy_{name}",
            eig_redis=self.eig_redis,
            name=name,
        )
        self.picos[name] = pico
        r = self._redis()
        r.sadd(PICOS_SET, name)
        return pico

    def discover(self):
        pass

    # Delegate command processing to real PicoManager methods
    def _process_command(self, r, msg_id, fields):
        return self._mgr._process_command(r, msg_id, fields)

    def _route_command(self, pico, target, cmd):
        return self._mgr._route_command(pico, target, cmd)

    def _check_health(self):
        return self._mgr._check_health()

    def start(self):
        self._running = True
        self._mgr._running = True

    def stop(self):
        self._running = False
        self._mgr._running = False
        for pico in self.picos.values():
            try:
                pico.disconnect()
            except Exception:
                pass
        self.picos.clear()
