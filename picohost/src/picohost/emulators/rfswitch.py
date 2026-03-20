from .base import PicoEmulator
from .imu import ImuEmulator


class RFSwitchEmulator(PicoEmulator):
    """Emulates src/rfswitch.c firmware."""

    def __init__(self, app_id=5, **kwargs):
        self.sw_state = 0
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.sw_state = 0

    def server(self, cmd):
        if "sw_state" in cmd:
            self.sw_state = int(cmd["sw_state"])

    def op(self):
        pass  # GPIO writes in firmware, no-op in emulator

    def get_status(self):
        return {
            "sensor_name": "rfswitch",
            "status": "update",
            "app_id": self.app_id,
            "sw_state": self.sw_state,
        }


class RFSwitchWithImuEmulator(PicoEmulator):
    """Composite emulator for APP_RFSWITCH (5).

    In main.c, APP_RFSWITCH runs both rfswitch and imu functions,
    sending two status messages per cadence.
    """

    def __init__(self, app_id=5, **kwargs):
        self._rfswitch = RFSwitchEmulator.__new__(RFSwitchEmulator)
        self._rfswitch.sw_state = 0
        self._rfswitch.app_id = app_id
        self._imu = ImuEmulator.__new__(ImuEmulator)
        self._imu.q = [0.0, 0.0, 0.0, 1.0]
        self._imu.a = [0.0, 0.0, 9.81]
        self._imu.la = [0.0, 0.0, 0.0]
        self._imu.g = [0.0, 0.0, 0.0]
        self._imu.m = [0.0, 0.0, 0.0]
        self._imu.grav = [0.0, 0.0, 9.81]
        self._imu.accel_status = 3
        self._imu.mag_status = 3
        self._imu.do_calibration = False
        self._imu.is_initialized = True
        self._imu.name = "imu_antenna"
        self._imu.app_id = app_id
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self._rfswitch.init()

    def server(self, cmd):
        # main.c dispatches to both imu_server and rfswitch_server
        self._imu.server(cmd)
        self._rfswitch.server(cmd)

    def op(self):
        self._imu.op()
        self._rfswitch.op()

    def get_status(self):
        # main.c sends both imu_status and rfswitch_status per cadence
        return [self._imu.get_status(), self._rfswitch.get_status()]
