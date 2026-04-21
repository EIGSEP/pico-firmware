import math
import time

from .base import PicoEmulator


class RFSwitchEmulator(PicoEmulator):
    """Emulates src/rfswitch.c firmware.

    Mirrors the firmware's settle-timer behavior: after a commanded
    state change, ``sw_state`` is reported as
    :attr:`SW_STATE_UNKNOWN` (-1) until ``settle_ms`` has elapsed, at
    which point the new commanded state becomes the reported state.
    Boot also starts in a transition so the very first reported state
    is UNKNOWN until settle.

    Passing ``settle_ms=0`` disables the transition entirely (instant
    settle, no boot transition). Tests that do not care about the
    transition path use this to keep behavior as-if settled.
    """

    SW_STATE_UNKNOWN = -1
    DEFAULT_SETTLE_MS = 20

    def __init__(self, app_id=5, settle_ms=None, **kwargs):
        self.settle_ms = (
            self.DEFAULT_SETTLE_MS if settle_ms is None else settle_ms
        )
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.commanded_state = 0
        self.reported_state = 0
        if self.settle_ms > 0:
            self.in_transition = True
            self._transition_end = (
                time.monotonic() + self.settle_ms / 1000.0
            )
        else:
            self.in_transition = False
            self._transition_end = time.monotonic()

    def server(self, cmd):
        if "sw_state" not in cmd:
            return
        raw = cmd["sw_state"]
        # cJSON_IsNumber matches only real JSON numbers; bools parse as
        # cJSON_True/cJSON_False and must be rejected here too.
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return
        if not math.isfinite(raw) or raw != int(raw):
            return
        new_state = int(raw)
        if (
            new_state < 0
            or new_state > 255
            or new_state == self.SW_STATE_UNKNOWN
        ):
            return
        if new_state != self.commanded_state:
            self.commanded_state = new_state
            if self.settle_ms > 0:
                self._transition_end = (
                    time.monotonic() + self.settle_ms / 1000.0
                )
                self.in_transition = True
            else:
                self.reported_state = new_state

    def op(self):
        if self.in_transition and time.monotonic() >= self._transition_end:
            self.reported_state = self.commanded_state
            self.in_transition = False

    def get_status(self):
        sw_state = (
            self.SW_STATE_UNKNOWN
            if self.in_transition
            else self.reported_state
        )
        return {
            "sensor_name": "rfswitch",
            "status": "update",
            "app_id": self.app_id,
            "sw_state": sw_state,
        }
