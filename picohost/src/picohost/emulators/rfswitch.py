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
        # Use regular constructors to avoid fragile manual attribute copying.
        # The sub-emulators' own threads/locks are unused since this composite
        # emulator drives them directly via server()/op()/get_status().
        self._rfswitch = RFSwitchEmulator(app_id=app_id)
        self._imu = ImuEmulator(app_id=app_id)
        self._imu.name = "imu_antenna"
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
