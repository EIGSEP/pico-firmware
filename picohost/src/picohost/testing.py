import logging
import time

try:
    import mockserial
except ImportError:
    logging.warning("Mockserial not found, dummy devices will not work")

from .base import (
    PicoDevice,
    PicoRFSwitch,
    PicoPeltier,
    PicoIMU,
    PicoLidar,
    PicoPotentiometer,
)
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

    def _make_emulator(self):
        if self.EMULATOR_CLASS is None:
            return None
        return self.EMULATOR_CLASS(
            status_cadence_ms=self.EMULATOR_CADENCE_MS,
        )

    def connect(self):
        self.ser = mockserial.MockSerial(timeout=0.5)
        self._peer = mockserial.MockSerial(timeout=0.01)
        self.ser.add_peer(self._peer)
        self._peer.add_peer(self.ser)
        self.ser.reset_input_buffer()
        # Mirror PicoDevice._open_serial: stamp open time so the health
        # loop gives us a HEALTH_TIMEOUT grace window before declaring
        # the device stale and triggering a reconnect.
        self.last_status_time = time.time()
        self._emulator = self._make_emulator()
        if self._emulator is not None:
            self._emulator.attach(self._peer)
            self._emulator.start()
        self._start_reader()
        return True

    def disconnect(self):
        if hasattr(self, "_emulator") and self._emulator:
            self._emulator.stop()
            self._emulator = None
        super().disconnect()


class DummyPicoMotor(DummyPicoDevice, PicoMotor):
    EMULATOR_CLASS = MotorEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoRFSwitch(DummyPicoDevice, PicoRFSwitch):
    EMULATOR_CLASS = RFSwitchEmulator
    EMULATOR_CADENCE_MS = 50.0
    # Firmware default is 200 ms; tests use a short settle so integration
    # tests do not spend seconds waiting for the emulator to "settle."
    EMULATOR_SETTLE_MS = 20

    def _make_emulator(self):
        return RFSwitchEmulator(
            status_cadence_ms=self.EMULATOR_CADENCE_MS,
            settle_ms=self.EMULATOR_SETTLE_MS,
        )


class DummyPicoPeltier(DummyPicoDevice, PicoPeltier):
    EMULATOR_CLASS = TempCtrlEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoIMU(DummyPicoDevice, PicoIMU):
    EMULATOR_CLASS = ImuEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoTempMon(DummyPicoDevice):
    EMULATOR_CLASS = TempMonEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoLidar(DummyPicoDevice, PicoLidar):
    EMULATOR_CLASS = LidarEmulator
    EMULATOR_CADENCE_MS = 50.0


class DummyPicoPotentiometer(DummyPicoDevice, PicoPotentiometer):
    EMULATOR_CLASS = PotMonEmulator
    EMULATOR_CADENCE_MS = 50.0
