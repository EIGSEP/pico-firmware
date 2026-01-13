import logging

try:
    import mockserial
except ImportError:
    logging.warning("Mockserial not found, dummy devices will not work")

from .base import PicoDevice, PicoRFSwitch, PicoPeltier
from .motor import PicoMotor


class DummyPicoDevice(PicoDevice):

    def __init__(self, port, eig_redis=None, **kwargs):
        """
        Initialize dummy device with optional eig_redis.
        
        For testing, eig_redis can be None since we don't actually upload data.
        """
        # Create a mock redis handler if none provided
        if eig_redis is None:
            # Create a no-op mock redis object
            class MockRedis:
                def add_metadata(self, name, data):
                    pass
            eig_redis = MockRedis()
        super().__init__(port, eig_redis, **kwargs)

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
        # Create a mock redis if none provided
        if eig_redis is None:
            class MockRedis:
                def add_metadata(self, name, data):
                    pass
            eig_redis = MockRedis()
        # Call PicoMotor's __init__ which will handle the rest
        PicoMotor.__init__(self, port, eig_redis, **kwargs)
    
    def wait_for_updates(self, timeout=10):
        """Override to provide immediate dummy status for tests."""
        self.status = {
            "az_pos": 0,
            "el_pos": 0,
            "az_target_pos": 0,
            "el_target_pos": 0,
            "az_speed": 100,
            "el_speed": 100
        }


class DummyPicoRFSwitch(DummyPicoDevice, PicoRFSwitch):
    def __init__(self, port, eig_redis=None, **kwargs):
        """Initialize dummy RF switch with optional eig_redis."""
        # Use DummyPicoDevice's __init__ which handles mock redis
        DummyPicoDevice.__init__(self, port, eig_redis, **kwargs)


class DummyPicoPeltier(DummyPicoDevice, PicoPeltier):
    def __init__(self, port, eig_redis=None, **kwargs):
        """Initialize dummy Peltier with optional eig_redis."""
        # Create a mock redis if none provided
        if eig_redis is None:
            class MockRedis:
                def add_metadata(self, name, data):
                    pass
            eig_redis = MockRedis()
        # Call PicoPeltier's parent (PicoStatus) __init__ which will handle the rest
        from .base import PicoStatus
        PicoStatus.__init__(self, port, eig_redis, **kwargs)
    
    def wait_for_updates(self, timeout=3):
        """Override to provide immediate dummy status for tests."""
        self.status = {
            "temperature": 25.0,
            "target_temperature": 25.0,
            "mode": "off",
            "power": 0.0
        }
