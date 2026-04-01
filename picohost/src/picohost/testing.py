import logging

try:
    import mockserial
except ImportError:
    logging.warning("Mockserial not found, dummy devices will not work")

from .base import PicoDevice, PicoRFSwitch, PicoPeltier, PicoIMU, PicoPotentiometer
from .motor import PicoMotor
from .emulators import (
    MotorEmulator,
    TempCtrlEmulator,
    TempMonEmulator,
    ImuEmulator,
    LidarEmulator,
    PotMonEmulator,
    RFSwitchEmulator,
)


class DummyPicoDevice(PicoDevice):
    EMULATOR_CLASS = None
    EMULATOR_CADENCE_MS = 200.0

    def connect(self):
        self.ser = mockserial.MockSerial(timeout=0.5)
        self._peer = mockserial.MockSerial(timeout=0.01)
        self.ser.add_peer(self._peer)
        self._peer.add_peer(self.ser)
        self.ser.reset_input_buffer()
        # Create and start emulator if a class is configured
        self._emulator = None
        if self.EMULATOR_CLASS is not None:
            self._emulator = self.EMULATOR_CLASS(
                status_cadence_ms=self.EMULATOR_CADENCE_MS
            )
            self._emulator.attach(self._peer)
            self._emulator.start()
        return True

    def disconnect(self):
        if hasattr(self, '_emulator') and self._emulator:
            self._emulator.stop()
            self._emulator = None
        # Stop the reader thread explicitly via PicoDevice.stop() to avoid
        # PicoMotor.stop() (which sends a halt command instead of stopping
        # the thread).
        PicoDevice.stop(self)
        if self.is_connected:
            self.ser.close()
            self.ser = None


class DummyPicoMotor(DummyPicoDevice, PicoMotor):
    EMULATOR_CLASS = MotorEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoRFSwitch(DummyPicoDevice, PicoRFSwitch):
    EMULATOR_CLASS = RFSwitchEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoPeltier(DummyPicoDevice, PicoPeltier):
    EMULATOR_CLASS = TempCtrlEmulator
    EMULATOR_CADENCE_MS = 50.0

    def disconnect(self):
        # PicoPeltier.stop() handles keepalive + reader thread cleanup
        PicoPeltier.stop(self)
        super().disconnect()


class DummyPicoIMU(DummyPicoDevice, PicoIMU):
    EMULATOR_CLASS = ImuEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoTempMon(DummyPicoDevice):
    EMULATOR_CLASS = TempMonEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoLidar(DummyPicoDevice):
    EMULATOR_CLASS = LidarEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoPotentiometer(DummyPicoDevice, PicoPotentiometer):
    EMULATOR_CLASS = PotMonEmulator
    EMULATOR_CADENCE_MS = 50.0
