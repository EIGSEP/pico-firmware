import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.01  # meters

# ACS724-10AB current monitor co-located on the lidar Pico (GP26/ADC0),
# read through a 3.32k/4.64k divider (DMM-measured). The firmware reports the
# raw ADC-pin voltage; emulate that for a representative whole-system draw.
CURRENT_VQ = 2.5  # sensor volts at 0 A (Vcc/2)
CURRENT_SENSITIVITY = 0.2  # sensor volts per amp
CURRENT_DIVIDER_RATIO = 4.64 / (3.32 + 4.64)  # = 0.5829 (measured)
CURRENT_NOISE_STDDEV = 0.002  # volts at the ADC pin
BASE_CURRENT_A = 2.0  # representative system draw


class LidarEmulator(PicoEmulator):
    """Emulates src/lidar.c firmware."""

    def __init__(self, app_id=4, **kwargs):
        self._base_distance = 100.0  # meters; noise mean, not initial value
        # Firmware static struct {0}: distance unread until first successful op.
        self.distance = 0.0
        self._sensor_failed = False  # set via simulate_sensor_failure()
        # RFI standby: disable GRF-250 laser firing (opcode 50). laser_firing
        # mirrors the value read back from opcode 50 for host confirmation.
        self._standby = False
        self.laser_firing = 1
        # Per-cycle freshness flag: True iff op() refreshed the
        # distance reading since the last get_status() call.
        self.last_op_ok = False
        # Raw ADC-pin voltage the firmware would report for BASE_CURRENT_A.
        self._base_current_v = (
            CURRENT_VQ + CURRENT_SENSITIVITY * BASE_CURRENT_A
        ) * CURRENT_DIVIDER_RATIO
        self.current_voltage = self._base_current_v
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        pass  # lidar_init() only sets up I2C; distance is not reset

    def server(self, cmd):
        # The only lidar commands are the universal RFI standby controls:
        # standby writes opcode 50 <- 0 (laser off), resume writes 50 <- 1.
        action = cmd.get("cmd")
        if action == "standby":
            self._standby = True
            self.laser_firing = 0
        elif action == "resume":
            self._standby = False
            self.laser_firing = 1

    def simulate_sensor_failure(self):
        """Simulate i2c read failure / TF-Luna not-ready loop."""
        self._sensor_failed = True

    def simulate_sensor_recovery(self):
        """Simulate sensor coming back after a failure."""
        self._sensor_failed = False

    def op(self):
        # currentmon_op() runs as its own dispatch call, independent of the
        # lidar I2C result — refresh current every cycle, even on failure.
        self.current_voltage = float(
            np.clip(
                self._base_current_v
                + np.random.normal(0, CURRENT_NOISE_STDDEV),
                0.0,
                3.3,
            )
        )
        if self._standby:
            # Laser off: skip the distance read entirely. currentmon above is
            # a separate dispatch and keeps sampling (mirrors lidar_op()'s
            # early return while currentmon_op() still runs).
            return
        if self._sensor_failed:
            # Matches firmware: i2c_read_timeout or dist_cm==0 → reset+return,
            # leaving previous distance unchanged and last_op_ok at False.
            return
        self.distance = self._base_distance + np.random.normal(0, NOISE_STDDEV)
        self.last_op_ok = True

    def get_status(self):
        if self._standby:
            # Commanded-off looks like an error tick (no valid distance) but
            # with standby=true so the host distinguishes it from a real fault.
            # laser_firing is the opcode-50 read-back (0 = confirmed off).
            return {
                "sensor_name": "lidar",
                "status": "error",
                "app_id": self.app_id,
                "current_voltage": self.current_voltage,
                "laser_firing": self.laser_firing,
                "standby": True,
            }
        status = "update" if self.last_op_ok else "error"
        self.last_op_ok = False
        return {
            "sensor_name": "lidar",
            "status": status,
            "app_id": self.app_id,
            "distance_m": self.distance,
            "current_voltage": self.current_voltage,
        }
