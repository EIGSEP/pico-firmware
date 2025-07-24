import logging

try:
    import mockserial
except ImportError:
    logging.warning("Mockserial not found, dummy devices will not work")

from .base import PicoDevice, PicoRFSwitch, PicoPeltier
from .motor import PicoMotor


class DummyPicoDevice(PicoDevice):

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
    pass


class DummyPicoPeltier(DummyPicoDevice, PicoPeltier):
    def wait_for_updates(self, timeout=3):
        """Override to provide immediate dummy status for tests."""
        self.status = {
            "temperature": 25.0,
            "target_temperature": 25.0,
            "mode": "off",
            "power": 0.0
        }
