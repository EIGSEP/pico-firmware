from .base import PicoEmulator, _safe_int


class RFSwitchEmulator(PicoEmulator):
    """Emulates src/rfswitch.c firmware."""

    def __init__(self, app_id=5, **kwargs):
        self.sw_state = 0
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.sw_state = 0

    def server(self, cmd):
        if "sw_state" in cmd:
            self.sw_state = _safe_int(cmd["sw_state"], self.sw_state)

    def op(self):
        pass  # GPIO writes in firmware, no-op in emulator

    def get_status(self):
        return {
            "sensor_name": "rfswitch",
            "status": "update",
            "app_id": self.app_id,
            "sw_state": self.sw_state,
        }
