import logging
try:
    import mockserial
except ImportError:
    logging.warning("Mockserial not found, dummy devices will not work")

from .base import PicoDevice, PicoMotor, PicoRFSwitch, PicoPeltier


class DummyPicoDevice(PicoDevice):

    def connect(self):
        self.ser = mockserial.MockSerial()
        # MockSerial needs a peer to be considered "open"
        peer = mockserial.MockSerial()
        self.ser.add_peer(peer)
        peer.add_peer(self.ser)
        self.ser.reset_input_buffer()
        return True


class DummyPicoMotor(DummyPicoDevice, PicoMotor):
    pass


class DummyPicoRFSwitch(DummyPicoDevice, PicoRFSwitch):
    pass


class DummyPicoPeltier(DummyPicoDevice, PicoPeltier):
    pass
