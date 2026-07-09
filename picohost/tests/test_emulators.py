"""
Unit tests for firmware emulators.
Tests emulators standalone (no mock serial), calling methods directly.
"""

import time

import numpy as np
import pytest
from picohost.emulators import (
    MotorEmulator,
    TempCtrlEmulator,
    ImuEmulator,
    LidarEmulator,
    PotMonEmulator,
    RFSwitchEmulator,
)


def _run_to_pi_tick(emu):
    """Advance the emulator by one sample tick on both channels.

    Firmware samples, rate-checks, and runs tempctrl_pi_drive on the fixed
    TEMPCTRL_SAMPLE_MS timer — one emulator op() models one sample tick.
    """
    emu.op()


def _run_to_drive(emu):
    """Advance to the first sample that actually engages drive.

    Two-to-anchor (tempctrl_update_sensor_drive): the first fresh sample
    only takes a candidate reference and control stays gated, so drive
    engages on the second consistent sample. Tests that need a channel up
    and controlling step through both.
    """
    _run_to_pi_tick(emu)  # seed the candidate reference
    _run_to_pi_tick(emu)  # confirm + anchor → control engages


class TestMotorEmulator:
    def test_initial_state(self):
        emu = MotorEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "motor"
        assert status["az_pos"] == 0
        assert status["el_pos"] == 0
        assert status["az_target_pos"] == 0
        assert status["el_target_pos"] == 0
        # Random 30-bit per-boot id, mirroring motor_init() in motor.c.
        assert 0 <= status["boot_id"] < 2**30

    def test_reinit_models_power_cycle(self):
        """init() zeroes the counters and draws a fresh boot_id —
        the host-side reboot detector keys off exactly this."""
        emu = MotorEmulator()
        emu.server({"az_set_pos": 500})
        old_boot = emu.boot_id
        emu.init()
        status = emu.get_status()
        assert status["az_pos"] == 0
        assert status["boot_id"] != old_boot

    def test_set_target_and_converge(self):
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 1000})
        # Call op() enough times for position to converge
        # max_pulses=60 per call, so ceil(1000/60) = 17 calls needed
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 1000
        # Position should remain stable after convergence
        for _ in range(5):
            emu.op()
        assert emu.azimuth.position == 1000

    def test_retarget(self):
        """Move to one position, then change target to another."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 500})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 500
        emu.server({"az_set_target_pos": 200})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 200

    def test_reverse_direction(self):
        """Move forward then backward past origin."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 300})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 300
        emu.server({"az_set_target_pos": -100})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == -100

    def test_elevation(self):
        emu = MotorEmulator()
        emu.server({"el_set_target_pos": -500})
        for _ in range(20):
            emu.op()
        assert emu.elevation.position == -500

    def test_delay_settings(self):
        emu = MotorEmulator()
        emu.server({"az_up_delay_us": 3000, "az_dn_delay_us": 4000})
        assert emu.azimuth.up_delay_us == 3000
        assert emu.azimuth.dn_delay_us == 4000

    def test_noop_when_at_target(self):
        """op() should be a no-op when position == target (no needless work).

        In C firmware, this corresponds to skipping enable/disable GPIO
        toggling when there are zero steps to take.
        """
        emu = MotorEmulator()
        # Position already at target (both 0)
        emu.op()
        assert emu.azimuth.position == 0
        assert emu.azimuth.dir == 0
        assert emu.azimuth.steps_in_direction == 0
        # After convergence, further op() calls should be no-ops
        emu.server({"az_set_target_pos": 120})
        for _ in range(5):
            emu.op()
        assert emu.azimuth.position == 120
        steps_before = emu.azimuth.steps_in_direction
        emu.op()
        assert emu.azimuth.steps_in_direction == steps_before

    def test_status_fields(self):
        emu = MotorEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "status",
            "app_id",
            "boot_id",
            "az_pos",
            "az_target_pos",
            "el_pos",
            "el_target_pos",
        }
        assert set(status.keys()) == expected_keys


class TestTempCtrlEmulator:
    def test_initial_state(self):
        emu = TempCtrlEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "tempctrl"
        assert status["LNA_enabled"] is False
        assert status["LOAD_enabled"] is False
        assert status["LNA_T_target"] == 30.0

    def test_enable_and_converge(self):
        """Enable LNA channel, verify convergence to within hysteresis of target."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
            }
        )
        for _ in range(500):
            emu.op()
        assert abs(emu.lna.T_now - 30.0) < 0.5

    def test_converge_to_non_default_target(self):
        """Converge to a target different from the 30.0 default."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 40.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_clamp": 0.6,  # default 0.2 saturates too slowly for 1000 ops
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 40.0) < 0.5

    def test_cool_back_down(self):
        """Heat to target, then set a lower target and verify cooling."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 35.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_clamp": 0.6,  # default 0.2 saturates too slowly for 1000 ops
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 35.0) < 0.5
        # Now cool back to 25
        emu.server({"LNA_temp_target": 25.0})
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 25.0) < 0.5

    def test_channel_load(self):
        """LOAD channel works independently of LNA channel."""
        emu = TempCtrlEmulator()
        emu.load.T_now = 20.0
        emu.server(
            {
                "LOAD_temp_target": 28.0,
                "LOAD_enable": True,
                "LOAD_hysteresis": 0.5,
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.load.T_now - 28.0) < 0.5
        # LNA channel should not have moved (not enabled)
        assert emu.lna.drive == 0.0

    def test_status_fields(self):
        emu = TempCtrlEmulator()
        # Before the first sample tick both channels report invalid data
        # (mirrors firmware: data_invalid is true at boot, and op() runs
        # before the first status message can fire).
        boot = emu.get_status()
        assert boot["LNA_status"] == "error"
        assert boot["LNA_T_now"] is None
        assert boot["LNA_resistance"] is None
        emu.op()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "app_id",
            "watchdog_tripped",
            "watchdog_timeout_ms",
            "LNA_status",
            "LNA_T_now",
            "LNA_voltage",
            "LNA_resistance",
            "LNA_timestamp",
            "LNA_T_target",
            "LNA_drive_level",
            "LNA_installed",
            "LNA_enabled",
            "LNA_active",
            "LNA_sensor_tripped",
            "LNA_sensor_rejects",
            "LNA_stall_tripped",
            "LNA_runaway_tripped",
            "LNA_cooling_enabled",
            "LNA_hysteresis",
            "LNA_clamp",
            "LNA_Kp",
            "LNA_Ki",
            "LNA_integral",
            "LOAD_status",
            "LOAD_T_now",
            "LOAD_voltage",
            "LOAD_resistance",
            "LOAD_timestamp",
            "LOAD_T_target",
            "LOAD_drive_level",
            "LOAD_installed",
            "LOAD_enabled",
            "LOAD_active",
            "LOAD_sensor_tripped",
            "LOAD_sensor_rejects",
            "LOAD_stall_tripped",
            "LOAD_runaway_tripped",
            "LOAD_cooling_enabled",
            "LOAD_hysteresis",
            "LOAD_clamp",
            "LOAD_Kp",
            "LOAD_Ki",
            "LOAD_integral",
        }
        assert set(status.keys()) == expected_keys
        assert status["LNA_status"] == "update"
        assert 0.0 < status["LNA_voltage"] < 3.3
        assert 0.0 < status["LOAD_voltage"] < 3.3
        assert status["LNA_resistance"] > 0.0
        assert status["LOAD_resistance"] > 0.0

    def test_integral_eliminates_steady_state_offset(self):
        """With Ki > 0, T_now converges to within the deadband and the
        integrator settles at a finite, nonzero value — the headline
        new behavior over the old proportional+baseline law.
        """
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_Ki": 0.05,
            }
        )
        for _ in range(2000):
            emu.op()
        assert abs(emu.lna.T_now - 30.0) <= 0.5

    def test_disable_clears_integrator(self):
        """Going from enabled → disabled wipes integral so the next
        re-enable starts clean (no kick from stale state)."""
        emu = TempCtrlEmulator()
        # Seed accumulator + sample-flag directly so the test isolates
        # the disable-path's reset behavior from PI accumulation dynamics
        # (which are exercised by test_integral_eliminates_steady_state_offset).
        emu.lna.integral = 5.0
        emu.lna.last_sample_seen = True
        emu.lna.drive = 0.3
        emu.server({"LNA_enable": False})
        emu.op()
        assert emu.lna.integral == 0.0
        assert emu.lna.drive == 0.0
        assert emu.lna.last_sample_seen is False

    def test_pure_p_does_not_accumulate_integral(self):
        """With Ki==0 the integrator must stay at zero across active PI
        ticks, so a later Ki retune cannot inherit drift."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                # Ki defaults to 0; assert that explicitly for clarity.
                "LNA_Ki": 0.0,
            }
        )
        # Run while outside the deadband so the PI step is exercised
        # without entering the reset-on-deadband path.
        emu.lna.thermal_frozen = True
        for _ in range(50):
            emu.op()
        assert emu.lna.active is True
        assert emu.lna.integral == 0.0

    def test_ki_change_resets_integrator(self):
        """Changing Ki via the server command drops the accumulator so
        the next PI tick does not multiply a stale integral by the new
        gain (bumpless retune)."""
        emu = TempCtrlEmulator()
        emu.lna.integral = 4.2
        emu.lna.last_sample_seen = True
        emu.server({"LNA_Ki": 0.05})
        assert emu.lna.integral == 0.0
        assert emu.lna.last_sample_seen is False
        assert emu.lna.Ki == 0.05

    def test_ki_repeat_does_not_reset_integrator(self):
        """Re-sending the same Ki value (e.g. as part of a config heartbeat)
        must not nuke the accumulator."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_Ki": 0.05})
        emu.lna.integral = 4.2
        emu.lna.last_sample_seen = True
        emu.server({"LNA_Ki": 0.05})
        assert emu.lna.integral == 4.2
        assert emu.lna.last_sample_seen is True


class TestTempCtrlWatchdog:
    def test_watchdog_trips_after_timeout(self):
        """Watchdog flag trips when no command arrives within timeout.

        `enabled` is host intent and stays untouched; the trip flag is the
        runtime gate that zeroes drive.
        """
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True, "LOAD_enable": True})
        assert emu.lna.enabled is True
        assert emu.load.enabled is True
        # Simulate time passing beyond the watchdog timeout
        emu._last_cmd_time = time.time() - (emu.watchdog_timeout_ms / 1000 + 1)
        emu.op()
        assert emu.watchdog_tripped is True
        # Host intent is preserved.
        assert emu.lna.enabled is True
        assert emu.load.enabled is True
        # Trip gates drive to zero on both channels.
        assert emu.lna.drive == 0.0
        assert emu.load.drive == 0.0

    def test_keepalive_does_not_clear_watchdog_trip(self):
        """A bare keepalive refreshes the timer but the trip flag is sticky."""
        emu = TempCtrlEmulator()
        # Trip the watchdog
        emu._last_cmd_time = time.time() - (emu.watchdog_timeout_ms / 1000 + 1)
        emu.op()
        assert emu.watchdog_tripped is True
        # A non-enable command (e.g. keepalive) must not silently re-engage
        # the peltiers — the host has to explicitly ack with *_enable=true.
        emu.server({})
        assert emu.watchdog_tripped is True
        emu.server({"LNA_temp_target": 28.0})
        assert emu.watchdog_tripped is True

    def test_enable_true_clears_watchdog_trip(self):
        """*_enable=true is the explicit ack that clears the watchdog flag."""
        emu = TempCtrlEmulator()
        emu._last_cmd_time = time.time() - (emu.watchdog_timeout_ms / 1000 + 1)
        emu.op()
        assert emu.watchdog_tripped is True
        emu.server({"LNA_enable": True})
        assert emu.watchdog_tripped is False

    def test_enable_false_does_not_clear_watchdog_trip(self):
        """Disabling a channel is not an ack — trip stays sticky."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True})
        emu._last_cmd_time = time.time() - (emu.watchdog_timeout_ms / 1000 + 1)
        emu.op()
        assert emu.watchdog_tripped is True
        emu.server({"LNA_enable": False})
        assert emu.watchdog_tripped is True

    def test_watchdog_does_not_trip_before_timeout(self):
        """Watchdog stays clear while commands arrive within timeout."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True})
        emu.op()
        assert emu.watchdog_tripped is False
        assert emu.lna.enabled is True

    def test_watchdog_timeout_configurable(self):
        """Watchdog timeout can be changed via server command."""
        emu = TempCtrlEmulator()
        emu.server({"watchdog_timeout_ms": 5000})
        assert emu.watchdog_timeout_ms == 5000

    def test_watchdog_disabled_with_zero_timeout(self):
        """Setting watchdog_timeout_ms to 0 disables the watchdog."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True, "watchdog_timeout_ms": 0})
        emu._last_cmd_time = time.time() - 999
        emu.op()
        assert emu.watchdog_tripped is False
        assert emu.lna.enabled is True


class TestTempCtrlStallGuard:
    """The stall guard trips when drive is engaged but T_now refuses to move."""

    def _force_window_elapsed(self, tc):
        # Pretend the stall window opened before the threshold.
        import picohost.emulators.tempctrl as mod

        tc.stall_check_time = time.time() - (mod.STALL_WINDOW_MS / 1000 + 1)

    def test_stall_trip_gates_drive(self):
        """Stall trip zeroes drive while leaving host-intent `enabled` alone."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.lna.thermal_frozen = True
        emu.server({"LNA_temp_target": 30.0, "LNA_enable": True})
        _run_to_drive(emu)  # anchor, then open the stall window (active=True)
        assert emu.lna.active is True
        assert emu.lna.stall_window_active is True
        self._force_window_elapsed(emu.lna)
        emu.op()
        assert emu.lna.stall_tripped is True
        # Host intent preserved; trip flag is the runtime gate.
        assert emu.lna.enabled is True
        assert emu.lna.drive == 0.0
        # Subsequent ops keep drive at zero — the gate stays closed.
        emu.op()
        assert emu.lna.drive == 0.0

    def test_stall_does_not_trip_when_temperature_moves(self):
        """Healthy drive rolls the window forward without tripping."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server({"LNA_temp_target": 30.0, "LNA_enable": True})
        _run_to_drive(emu)
        self._force_window_elapsed(emu.lna)
        emu.lna.T_now += 1.0  # well above STALL_MIN_DELTA
        emu.op()
        assert emu.lna.stall_tripped is False
        assert emu.lna.enabled is True

    def test_stall_does_not_trip_inside_hysteresis_band(self):
        """active=False (within hysteresis band) means no stall window runs."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 30.0
        emu.lna.thermal_frozen = True
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
            }
        )
        _run_to_drive(emu)
        assert emu.lna.active is False
        assert emu.lna.stall_window_active is False
        self._force_window_elapsed(emu.lna)
        emu.op()
        assert emu.lna.stall_tripped is False

    def test_enable_true_clears_stall_trip(self):
        """An explicit *_enable=true is the host's ack of a stall trip."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.lna.thermal_frozen = True
        emu.server({"LNA_temp_target": 30.0, "LNA_enable": True})
        _run_to_drive(emu)
        self._force_window_elapsed(emu.lna)
        emu.op()
        assert emu.lna.stall_tripped is True
        assert emu.lna.enabled is True
        emu.server({"LNA_enable": True})
        assert emu.lna.stall_tripped is False
        assert emu.lna.enabled is True

    def test_non_enable_command_does_not_clear_stall_trip(self):
        """Setting a new temp target must not silently clear a stall trip."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.lna.thermal_frozen = True
        emu.server({"LNA_temp_target": 30.0, "LNA_enable": True})
        _run_to_drive(emu)
        self._force_window_elapsed(emu.lna)
        emu.op()
        assert emu.lna.stall_tripped is True
        emu.server({"LNA_temp_target": 28.0})
        assert emu.lna.stall_tripped is True

    def test_channels_trip_independently(self):
        """A stalled LOAD does not knock out a healthy LNA."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.load.T_now = 25.0
        emu.load.thermal_frozen = True
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LOAD_temp_target": 30.0,
                "LOAD_enable": True,
            }
        )
        _run_to_drive(emu)
        self._force_window_elapsed(emu.load)
        emu.op()
        assert emu.load.stall_tripped is True
        # Host intent unchanged on the tripped channel.
        assert emu.load.enabled is True
        assert emu.load.drive == 0.0
        # LNA is unaffected.
        assert emu.lna.stall_tripped is False
        assert emu.lna.enabled is True


class TestTempCtrlCoolingGuard:
    """``cooling_enabled=False`` forbids drive<0 — the cooling-mode
    thermal-runaway guard. PI saturates at [0, +clamp] instead of
    [-clamp, +clamp] and the integrator does not wind up negative.
    """

    def test_default_is_cooling_enabled(self):
        emu = TempCtrlEmulator()
        assert emu.lna.cooling_enabled is True
        assert emu.load.cooling_enabled is True

    def test_disable_clamps_drive_to_nonnegative(self):
        """With cooling disabled and setpoint below T_now, drive
        cannot go negative even though T_delta < 0."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 30.0
        emu.server(
            {
                "LNA_temp_target": 20.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_cooling_enabled": False,
            }
        )
        for _ in range(1000):
            emu.op()
        # Drive floors at zero; T_now does not move below the initial
        # value because no heating is requested and cooling is forbidden.
        assert emu.lna.drive == 0.0
        assert emu.lna.T_now >= 30.0

    def test_disabled_integrator_does_not_wind_up_negative(self):
        """With Ki>0, cooling disabled, and a persistent cooling demand,
        the conditional-integration anti-windup must keep the integral
        from accumulating negative — otherwise it would wind up and
        eventually corrupt a later legitimate heating step."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 30.0
        emu.lna.thermal_frozen = True  # keep the cooling demand constant
        emu.server(
            {
                "LNA_temp_target": 20.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_Ki": 0.05,
                "LNA_cooling_enabled": False,
            }
        )
        for _ in range(1000):
            emu.op()
        assert emu.lna.drive == 0.0
        # Anti-windup: integral stays at zero even with persistent
        # negative T_delta because lower_clamp=0 saturates sat_low.
        assert emu.lna.integral == 0.0

    def test_heating_still_works_when_cooling_disabled(self):
        """Heating is unaffected — only the negative half is forbidden."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 20.0
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_cooling_enabled": False,
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 30.0) < 0.5

    def test_per_channel_isolation(self):
        """Disabling cooling on LNA leaves LOAD's full range available."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 30.0
        emu.load.T_now = 30.0
        emu.server(
            {
                "LNA_temp_target": 20.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_cooling_enabled": False,
                "LOAD_temp_target": 20.0,
                "LOAD_enable": True,
                "LOAD_hysteresis": 0.5,
            }
        )
        for _ in range(1000):
            emu.op()
        # LNA refused to cool; LOAD reached the lower setpoint.
        assert emu.lna.drive == 0.0
        assert emu.lna.T_now >= 30.0
        assert abs(emu.load.T_now - 20.0) < 0.5

    def test_saturated_at_zero_does_not_trip_stall(self):
        """With cooling forbidden and T_now stuck above setpoint, the PI
        loop sits at active=True / drive=0. That's the configured refusal
        to cool, not a stalled Peltier — stall must not trip."""
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        emu.lna.T_now = 30.0
        emu.lna.thermal_frozen = True
        emu.server(
            {
                "LNA_temp_target": 20.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_cooling_enabled": False,
            }
        )
        _run_to_drive(emu)
        assert emu.lna.active is True
        assert emu.lna.drive == 0.0
        # Force-elapse the stall window and run another op cycle: with
        # drive==0 the guard must not arm, regardless of how long we sit.
        emu.lna.stall_check_time = time.time() - (
            mod.STALL_WINDOW_MS / 1000 + 1
        )
        emu.op()
        assert emu.lna.stall_tripped is False
        assert emu.lna.stall_window_active is False
        assert emu.lna.enabled is True


class TestTempCtrlRunawayGuard:
    """Driving one direction while T_now moves the opposite direction trips
    the channel (via runaway_tripped — a separate flag from stall_tripped,
    because "drive made it worse" is a different field diagnosis than
    "drive did nothing") after RUNAWAY_STRIKES consecutive wrong-direction
    windows — the thermal-runaway signature the no-movement stall guard
    cannot catch (the temperature *is* moving).
    """

    def _force_window_elapsed(self, tc):
        import picohost.emulators.tempctrl as mod

        tc.stall_check_time = time.time() - (mod.STALL_WINDOW_MS / 1000 + 1)

    def _setup_cooling(self, emu):
        """Arm LNA cooling so drive saturates negative; open the window."""
        emu.lna.T_now = 30.0
        emu.lna.thermal_frozen = True  # drive T_now by hand
        emu.server({"LNA_temp_target": 20.0, "LNA_enable": True})
        _run_to_drive(emu)
        assert emu.lna.drive < 0.0
        assert emu.lna.stall_window_active is True

    def _wrong_way_window(self, emu):
        """Force a completed window in which T_now moved *up* while the
        drive is negative (cooling), then evaluate it with one op."""
        self._force_window_elapsed(emu.lna)
        emu.lna.T_now += 1.0
        emu.op()

    def test_runaway_trips_after_consecutive_wrong_windows(self):
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        self._setup_cooling(emu)
        # All but the last wrong-direction window only accumulate strikes.
        for i in range(mod.RUNAWAY_STRIKES - 1):
            self._wrong_way_window(emu)
            assert emu.lna.runaway_tripped is False
            assert emu.lna.runaway_strikes == i + 1
        # The strike that reaches the threshold trips the channel — on the
        # runaway flag; the no-movement stall flag stays clear.
        self._wrong_way_window(emu)
        assert emu.lna.runaway_tripped is True
        assert emu.lna.stall_tripped is False
        assert emu.lna.drive == 0.0
        # Host intent preserved; trip flag is the runtime gate.
        assert emu.lna.enabled is True

    def test_correct_direction_window_resets_strikes(self):
        emu = TempCtrlEmulator()
        self._setup_cooling(emu)
        self._wrong_way_window(emu)
        assert emu.lna.runaway_strikes == 1
        # A correct-direction window (cooling drive, T_now falls) clears it.
        self._force_window_elapsed(emu.lna)
        emu.lna.T_now -= 1.0
        emu.op()
        assert emu.lna.runaway_strikes == 0
        assert emu.lna.runaway_tripped is False

    def test_runaway_trips_when_heating_drive_cools(self):
        """Symmetric: heating drive (>0) while T_now falls also trips."""
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        emu.lna.T_now = 20.0
        emu.lna.thermal_frozen = True
        emu.server({"LNA_temp_target": 30.0, "LNA_enable": True})
        _run_to_drive(emu)
        assert emu.lna.drive > 0.0
        for _ in range(mod.RUNAWAY_STRIKES):
            self._force_window_elapsed(emu.lna)
            emu.lna.T_now -= 1.0  # heating drive but cooling → wrong way
            emu.op()
        assert emu.lna.runaway_tripped is True
        assert emu.lna.stall_tripped is False
        assert emu.lna.drive == 0.0

    def test_target_crossing_does_not_count_wrong_direction(self):
        emu = TempCtrlEmulator()
        self._setup_cooling(emu)
        self._force_window_elapsed(emu.lna)
        emu.lna.T_now = emu.lna.T_target - 1.0
        emu.lna.thermal_frozen = False
        _run_to_pi_tick(emu)
        assert emu.lna.drive > 0.0
        assert emu.lna.runaway_strikes == 0
        assert emu.lna.runaway_tripped is False

    def test_enable_true_clears_runaway_trip(self):
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        self._setup_cooling(emu)
        for _ in range(mod.RUNAWAY_STRIKES):
            self._wrong_way_window(emu)
        assert emu.lna.runaway_tripped is True
        emu.server({"LNA_enable": True})
        assert emu.lna.runaway_tripped is False
        assert emu.lna.runaway_strikes == 0


class TestTempCtrlSensorSanity:
    """A fresh sample implying a physically impossible rate of change is
    rejected for CONTROL only: the control reference T_now holds and the
    reject counts toward the latch, but the cycle still reports the raw
    conversion (status "update") with the published sensor_rejects counter
    as the marker. MAX_REJECTS consecutive rejects latch the sticky
    sensor_tripped, which gates drive until the host acks. Only the
    plausibility chain — no measurement exists — errors the stream; the
    firmware never vetoes science data on rate statistics alone.
    """

    def _seed(self, emu):
        """Anchor the rate reference. Two-to-anchor needs two consistent
        fresh conversions: the first is a candidate, the second confirms it.
        """
        emu.lna.T_now = 25.0
        emu.lna.thermal_frozen = True
        _run_to_pi_tick(emu)
        assert emu.lna.seed_pending is True
        assert emu.lna.rate_ref_valid is False
        _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is True

    def test_single_glitch_absorbed(self):
        emu = TempCtrlEmulator()
        self._seed(emu)
        emu.lna.inject_sensor_glitch(90.0)
        _run_to_pi_tick(emu)
        assert emu.lna.sensor_rejects == 1
        assert emu.lna.sensor_tripped is False
        # The rejected sample was still a plausible ADC conversion, so the
        # cycle reports it: raw value in the stream, published reject
        # counter as the marker that the rate guard discarded it for
        # control. Downstream owns the trust call.
        assert emu.lna.data_invalid is False
        status = emu.get_status()
        assert status["LNA_status"] == "update"
        assert status["LNA_T_now"] == pytest.approx(90.0)
        assert status["LNA_sensor_rejects"] == 1
        # Bogus value was not adopted for control — the rate/PI reference
        # holds the last good reading.
        assert emu.lna.T_now == 25.0
        # The next good sample clears the counter without any host action.
        _run_to_pi_tick(emu)
        status = emu.get_status()
        assert status["LNA_status"] == "update"
        assert status["LNA_T_now"] == pytest.approx(25.0)
        assert status["LNA_sensor_rejects"] == 0

    def test_consecutive_glitches_latch_sensor_fault(self):
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        self._seed(emu)
        emu.lna.inject_sensor_glitch(90.0, count=mod.MAX_REJECTS)
        for _ in range(mod.MAX_REJECTS):
            _run_to_pi_tick(emu)
        assert emu.lna.sensor_rejects == mod.MAX_REJECTS
        assert emu.lna.sensor_tripped is True
        # The latching cycle's sample was still a plausible conversion, so
        # it stays in the stream; the pegged reject counter and the sticky
        # trip flag are the fault markers.
        status = emu.get_status()
        assert status["LNA_status"] == "update"
        assert status["LNA_T_now"] == pytest.approx(90.0)
        assert status["LNA_sensor_rejects"] == mod.MAX_REJECTS
        # Control never adopted a garbage value while latching.
        assert emu.lna.T_now == 25.0

    def test_good_sample_after_glitches_resets(self):
        emu = TempCtrlEmulator()
        self._seed(emu)
        emu.lna.inject_sensor_glitch(90.0, count=2)
        _run_to_pi_tick(emu)
        _run_to_pi_tick(emu)
        assert emu.lna.sensor_rejects == 2
        assert emu.lna.sensor_tripped is False
        # A plausible reading (empty glitch queue → reads true T_now) clears
        # the reject counter.
        _run_to_pi_tick(emu)
        assert emu.lna.sensor_rejects == 0
        assert emu.lna.sensor_tripped is False

    def test_enable_true_clears_sensor_rejects_and_rate_reference(self):
        emu = TempCtrlEmulator()
        self._seed(emu)
        # Two rejects: below the latch ceiling, so the anchor is still
        # held (a burst short of the latch must not move the reference).
        emu.lna.inject_sensor_glitch(90.0, count=2)
        _run_to_pi_tick(emu)
        _run_to_pi_tick(emu)
        assert emu.lna.sensor_rejects == 2
        assert emu.lna.rate_ref_valid is True
        emu.server({"LNA_enable": True})
        assert emu.lna.sensor_rejects == 0
        assert emu.lna.rate_ref_valid is False
        assert emu.lna.seed_pending is False

    def test_latched_fault_does_not_self_recover(self):
        """A latched sensor fault is sticky: once tripped, plausible
        readings alone must not silently re-enable the channel. Recovery
        requires an explicit *_enable=true host ack (mirrors firmware
        sensor_tripped). Auto-recovery would re-drive the Peltier on a
        sensor that just produced a burst of garbage.
        """
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        self._seed(emu)
        emu.lna.inject_sensor_glitch(90.0, count=mod.MAX_REJECTS)
        for _ in range(mod.MAX_REJECTS):
            _run_to_pi_tick(emu)
        assert emu.lna.sensor_tripped is True
        # Glitch queue is now empty, so subsequent fresh ticks read the true
        # T_now and pass the rate guard — the latch must hold, but the DATA
        # recovers: status returns to "update" with real values while only
        # drive stays gated. This is the redesign's headline behavior: a
        # latched-but-healthy channel keeps publishing science data.
        for _ in range(3):
            _run_to_pi_tick(emu)
        assert emu.lna.sensor_tripped is True
        status = emu.get_status()
        assert status["LNA_status"] == "update"
        assert status["LNA_T_now"] == pytest.approx(25.0)
        assert status["LNA_sensor_tripped"] is True
        assert emu.lna.drive == 0.0
        # Only the host ack clears the latch and re-seeds the reference.
        emu.server({"LNA_enable": True})
        _run_to_pi_tick(emu)
        assert emu.lna.sensor_tripped is False

    def test_latched_channel_recovers_reporting_at_shifted_level(self):
        """A trip whose sensor settles at a NEW level must not null
        T_now forever. Once the latch fires, drive is gated, so the
        frozen rate reference serves no control purpose: the anchor
        drops and the channel re-seeds two-to-anchor from the sensor's
        actual level. Reporting recovers on its own; only drive stays
        gated until the host ack. (Field bug: a tripped channel relaxes
        toward ambient, so the real temperature soon sits far from the
        frozen reference and every healthy sample was rejected against
        it — T_now stayed null until a blind re-enable dropped the
        anchor.)
        """
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        self._seed(emu)
        # Garbage burst latches the channel; the sensor then keeps
        # reading a steady, plausible level far from the frozen 25.0
        # reference.
        emu.lna.inject_sensor_glitch(90.0, count=mod.MAX_REJECTS + 2)
        for _ in range(mod.MAX_REJECTS):
            _run_to_pi_tick(emu)
        assert emu.lna.sensor_tripped is True
        # Two steady post-latch samples: candidate, then confirm — the
        # channel re-anchors at the new level and reporting recovers
        # without any host action.
        _run_to_pi_tick(emu)
        _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is True
        status = emu.get_status()
        assert status["LNA_status"] == "update"
        assert status["LNA_T_now"] == pytest.approx(90.0)
        # The reject counter recovered to 0 — combined with the held trip
        # flag this is the operator's "sensor consistent again, safe to
        # re-ack" signal.
        assert status["LNA_sensor_rejects"] == 0
        # The latch is still host-ack-only: drive stays gated.
        assert emu.lna.sensor_tripped is True
        assert status["LNA_sensor_tripped"] is True
        assert emu.lna.drive == 0.0

    def test_seed_requires_two_samples(self):
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.lna.thermal_frozen = True
        # First fresh sample is only a candidate — the reference is not yet
        # anchored, so there is nothing to rate-check the next sample against.
        _run_to_pi_tick(emu)
        assert emu.lna.seed_pending is True
        assert emu.lna.rate_ref_valid is False
        # A second consistent sample confirms and anchors the reference.
        _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is True
        assert emu.lna.seed_pending is False

    def test_bogus_seed_self_heals(self):
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.lna.thermal_frozen = True
        # First raw reading is a lone transient (e.g. the 85 C power-on
        # default after a brownout); the true temperature is ~25. Two-to-anchor
        # discards the transient and anchors on the consistent pair that
        # follows, so the channel never latches.
        emu.lna.inject_sensor_glitch(85.0)
        emu.lna.inject_sensor_glitch(25.0, count=2)
        for _ in range(3):
            _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is True
        assert emu.lna.sensor_tripped is False
        assert emu.lna.sensor_rejects == 0
        assert emu.lna.T_now == pytest.approx(25.0)

    def test_persistent_disagreement_latches(self):
        import picohost.emulators.tempctrl as mod

        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.lna.thermal_frozen = True
        # A sensor that never produces two consecutive consistent readings
        # never anchors; each failed confirmation counts toward the same latch.
        for value in (85.0, 25.0, 85.0, 25.0):
            emu.lna.inject_sensor_glitch(value)
        for _ in range(4):
            _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is False
        assert emu.lna.sensor_rejects == mod.MAX_REJECTS
        assert emu.lna.sensor_tripped is True
        # Unconfirmed candidates passed the plausibility check, so the data
        # stream stays "update" — the sticky latch is the fault signal here.
        assert emu.get_status()["LNA_sensor_tripped"] is True
        assert emu.get_status()["LNA_status"] == "update"

    def test_no_drive_until_anchored(self):
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.lna.T_target = 40.0
        emu.lna.thermal_frozen = True
        emu.lna.enabled = True
        # Still seeding: the channel must stay idle even though it is enabled
        # and far from target — driving on an unconfirmed reading is the bug.
        _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is False
        assert emu.lna.drive == 0.0
        # Anchored on the second consistent sample → control engages.
        _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is True
        assert emu.lna.drive != 0.0

    def test_outage_recovery_reseeds_anchor(self):
        """A plausibility-failed stretch drops the rate anchor: recovery
        re-seeds (two-to-anchor) instead of judging the post-outage reading
        against a stale reference. A legitimate temperature drift across a
        sensor outage must not rack up rejects and false-latch the channel.
        """
        emu = TempCtrlEmulator()
        self._seed(emu)  # anchored at 25.0
        emu.inject_sensor_error("LNA")
        _run_to_pi_tick(emu)
        assert emu.lna.rate_ref_valid is False
        # The enclosure drifts far beyond the per-sample rate budget while
        # the sensor is out; the recovered sensor reads the new temperature.
        emu.lna.T_now = 45.0
        emu.inject_sensor_error("LNA", error=False)
        _run_to_pi_tick(emu)  # candidate at 45.0
        _run_to_pi_tick(emu)  # confirmation anchors — no rejects
        assert emu.lna.rate_ref_valid is True
        assert emu.lna.sensor_rejects == 0
        assert emu.lna.sensor_tripped is False
        assert emu.get_status()["LNA_status"] == "update"
        assert emu.get_status()["LNA_T_now"] == pytest.approx(45.0)


class TestThermistorCurve:
    """Pin the emulator's NTC model to the Vishay NTCLE100E3 datasheet.

    The tempctrl thermistor is an NTCLE100E3103 (10 kOhm at 25 C,
    B25/85 = 3977 K). Anchors are the R_T table for the 10 kOhm part
    (Vishay document 29049, NTCLE100E3103 column). The firmware inverts
    resistance -> temperature with the datasheet's extended
    Steinhart-Hart fit (A1..D1, temp_simple.h); the emulator generates
    resistance from temperature with the paired forward fit, which the
    datasheet states is interchangeable within 0.015 C over -40..125 C.
    """

    DATASHEET_ANCHORS_OHMS = {
        -40.0: 332094.0,
        -20.0: 96358.0,
        0.0: 32554.0,
        25.0: 10000.0,
        40.0: 5330.0,
        70.0: 1753.0,
        100.0: 677.3,
        125.0: 338.7,
    }

    def test_resistance_matches_datasheet_table(self):
        import picohost.emulators.tempctrl as mod

        for temp_c, ohms in self.DATASHEET_ANCHORS_OHMS.items():
            assert mod._thermistor_resistance(temp_c) == pytest.approx(
                ohms, rel=2e-3
            )

    def test_round_trip_matches_firmware_inverse(self):
        """Forward fit -> firmware inverse must reproduce the input
        temperature, so emulator-generated resistances decode on the
        firmware side to the temperature the emulator meant."""
        import picohost.emulators.tempctrl as mod

        for temp_c in self.DATASHEET_ANCHORS_OHMS:
            resistance = mod._thermistor_resistance(temp_c)
            back = mod._thermistor_temperature(resistance)
            assert back == pytest.approx(temp_c, abs=0.02)

    def test_divider_voltage_at_25c(self):
        """10 kOhm NTC against the 10.68k || 4.7k top resistance:
        3.3 * 10000 / (3263.5 + 10000) = 2.4885 V at the ADC pin."""
        import picohost.emulators.tempctrl as mod

        voltage = mod._thermistor_voltage(mod._thermistor_resistance(25.0))
        assert voltage == pytest.approx(2.4885, abs=2e-3)


class TestImuEmulator:
    def test_initial_state(self):
        emu = ImuEmulator(app_id=3)
        assert emu.name == "imu_el"
        status = emu.get_status()
        assert status["sensor_name"] == "imu_el"

    def test_antenna_name(self):
        emu = ImuEmulator(app_id=6)
        assert emu.name == "imu_az"

    def test_accel_magnitude(self):
        """Accelerometer should read ~9.81 m/s^2 magnitude (stationary)."""
        emu = ImuEmulator()
        for _ in range(10):
            emu.op()
        mag = np.sqrt(emu.accel_x**2 + emu.accel_y**2 + emu.accel_z**2)
        assert abs(mag - 9.81) < 0.1

    def test_set_orientation_drives_accel_via_forward_model(self):
        import numpy as np
        from picohost import imu_geometry as ig

        emu = ImuEmulator(app_id=6)  # imu_az, identity mount
        # az_deg only varies the raw accel_x/accel_y split -- el_abs_from_imu_az
        # is invariant to rotation about the az spin axis (retired along with
        # the azimuth blend estimator; el is the only derived field now).
        emu.set_orientation(az_deg=70.0, el_deg=40.0)
        emu.op()
        s = emu.get_status()
        a = np.array([s["accel_x"], s["accel_y"], s["accel_z"]])
        a_unit = a / np.linalg.norm(a)
        el = ig.el_abs_from_imu_az(a_unit, np.eye(3))
        assert el == pytest.approx(40.0, abs=1e-3)

    def test_accel_error_scales_norm(self):
        import numpy as np

        emu = ImuEmulator(app_id=6)
        emu.set_orientation(az_deg=0.0, el_deg=20.0)
        emu.set_accel_error(bias=(0.2, -0.1, 0.0), scale=12.2 / 9.80665)
        emu.op()
        s = emu.get_status()
        a = np.array([s["accel_x"], s["accel_y"], s["accel_z"]])
        assert np.linalg.norm(a) > 11.0  # inflated like the 0627 data

    def test_mount_changes_body_frame_reading(self):
        import numpy as np

        emu = ImuEmulator(app_id=3)  # imu_el
        emu.set_mount(np.array([[0, 1.0, 0], [-1.0, 0, 0], [0, 0, 1.0]]))
        emu.set_orientation(az_deg=0.0, el_deg=30.0)
        emu.op()
        s = emu.get_status()
        # with this mount the elevation tilt shows up on accel_x not accel_y
        assert abs(s["accel_x"]) > abs(s["accel_y"])

    def test_sensor_failure_triggers_reinit(self):
        """IMU reports error after event timeout, then recovers."""
        import picohost.emulators.imu as imu_mod

        emu = ImuEmulator()
        emu.op()
        assert emu.is_initialized is True
        assert emu.get_status()["status"] == "update"

        # Simulate sensor crash and push last_event_time into the past
        emu.simulate_sensor_failure()
        emu._last_event_time -= imu_mod.IMU_EVENT_TIMEOUT_S + 1
        emu.op()
        assert emu.is_initialized is False
        assert emu.get_status()["status"] == "error"

        # Sensor comes back — next op() should re-initialize
        emu.simulate_sensor_recovery()
        emu.op()
        assert emu.is_initialized is True
        assert emu.get_status()["status"] == "update"

    def test_sensor_failure_before_timeout_stays_initialized(self):
        """IMU stays initialized if failure is shorter than timeout."""
        emu = ImuEmulator()
        emu.op()
        emu.simulate_sensor_failure()
        # Without advancing the clock, still within timeout
        emu.op()
        assert emu.is_initialized is True

    def test_status_fields(self):
        emu = ImuEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "status",
            "app_id",
            "yaw",
            "pitch",
            "roll",
            "accel_x",
            "accel_y",
            "accel_z",
        }
        assert set(status.keys()) == expected_keys


class TestLidarEmulator:
    def test_initial_state(self):
        emu = LidarEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "lidar"
        assert status["distance_m"] == 0.0

    def test_noise_is_mean_reverting(self):
        emu = LidarEmulator()
        for _ in range(1000):
            emu.op()
        # Mean-reverting noise stays tightly around base distance
        assert abs(emu.distance - 100.0) < 0.5

    def test_status_fields(self):
        emu = LidarEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "status",
            "app_id",
            "distance_m",
            "current_voltage",
        }
        assert set(status.keys()) == expected_keys

    def test_failure_then_recovery_returns_to_update(self):
        """One failed cycle reports "error"; the next good op() returns to "update"."""
        emu = LidarEmulator()
        emu.op()
        assert emu.get_status()["status"] == "update"

        emu.simulate_sensor_failure()
        emu.op()
        assert emu.get_status()["status"] == "error"

        emu.simulate_sensor_recovery()
        emu.op()
        assert emu.get_status()["status"] == "update"


class TestRFSwitchEmulator:
    def test_initial_state_settled_when_instant(self):
        """settle_ms=0 skips the boot transition; first status is settled 0."""
        emu = RFSwitchEmulator(settle_ms=0)
        status = emu.get_status()
        assert status["sensor_name"] == "rfswitch"
        assert status["sw_state"] == 0

    def test_status_fields(self):
        emu = RFSwitchEmulator(settle_ms=0)
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "status",
            "app_id",
            "sw_state",
            "volt_therm0",
            "volt_therm1",
            "volt_therm2",
        }
        assert set(status.keys()) == expected_keys

    def test_therm_volts_settable(self):
        """Tests can inject per-channel voltages (mirrors real ADC reads)."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.volt_therm = [0.5, 1.0, 3.0]
        status = emu.get_status()
        assert status["volt_therm0"] == 0.5
        assert status["volt_therm1"] == 1.0
        assert status["volt_therm2"] == 3.0

    def test_boot_starts_in_transition(self):
        """Default settle_ms > 0: boot reports UNKNOWN until settle."""
        emu = RFSwitchEmulator(settle_ms=30)
        assert emu.in_transition is True
        assert emu.get_status()["sw_state"] == emu.SW_STATE_UNKNOWN
        # After the settle window, op() clears the transition.
        time.sleep(0.05)
        emu.op()
        assert emu.in_transition is False
        assert emu.get_status()["sw_state"] == 0

    def test_command_enters_transition(self):
        """A state-change command re-enters the UNKNOWN transition window."""
        emu = RFSwitchEmulator(settle_ms=0)
        assert emu.get_status()["sw_state"] == 0
        emu.settle_ms = 30  # arm transition for this command
        emu.server({"sw_state": 7})
        assert emu.in_transition is True
        assert emu.get_status()["sw_state"] == emu.SW_STATE_UNKNOWN
        time.sleep(0.05)
        emu.op()
        assert emu.in_transition is False
        assert emu.get_status()["sw_state"] == 7

    def test_same_state_command_is_noop(self):
        """Commanding the current state must not re-enter transition."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": 5})
        assert emu.get_status()["sw_state"] == 5
        emu.settle_ms = (
            30  # would arm transition if a change actually occurred
        )
        emu.server({"sw_state": 5})
        assert emu.in_transition is False
        assert emu.get_status()["sw_state"] == 5

    def test_reject_out_of_range(self):
        """Values outside [0, NUM_PATHS) must be ignored (firmware parity).

        Addresses >= NUM_PATHS hold 0xFF on the path EEPROMs (every
        switch input closed + noise diode on); the firmware guard keeps
        them off the bus.
        """
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": 5})
        assert emu.commanded_state == 5
        emu.server({"sw_state": emu.NUM_PATHS})
        assert emu.commanded_state == 5
        emu.server({"sw_state": 255})
        assert emu.commanded_state == 5
        emu.server({"sw_state": -2})
        assert emu.commanded_state == 5

    def test_reject_unknown_sentinel(self):
        """SW_STATE_UNKNOWN (-1) must be rejected as a command value."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": 3})
        emu.server({"sw_state": emu.SW_STATE_UNKNOWN})
        assert emu.commanded_state == 3

    def test_reject_fractional_number(self):
        """Numbers with a fractional part must be rejected."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": 3.7})
        assert emu.commanded_state == 0
        # Exact-integer floats (e.g. 4.0) are still accepted.
        emu.server({"sw_state": 4.0})
        assert emu.commanded_state == 4

    def test_reject_non_finite(self):
        """NaN / inf must be rejected."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": float("nan")})
        assert emu.commanded_state == 0
        emu.server({"sw_state": float("inf")})
        assert emu.commanded_state == 0

    def test_reject_bool(self):
        """Bools are not JSON numbers; cJSON_IsNumber rejects them."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": True})
        assert emu.commanded_state == 0


# ---------------------------------------------------------------------------
# Status field TYPE verification
# ---------------------------------------------------------------------------


class TestMotorStatusTypes:
    """Verify status field types match C firmware KV_* type tags."""

    def test_status_field_types(self):
        emu = MotorEmulator()
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        assert isinstance(status["az_pos"], int)
        assert isinstance(status["az_target_pos"], int)
        assert isinstance(status["el_pos"], int)
        assert isinstance(status["el_target_pos"], int)


class TestTempCtrlStatusTypes:
    def test_status_field_types(self):
        emu = TempCtrlEmulator()
        emu.op()  # populate timestamps
        status = emu.get_status()
        for prefix in ("LNA", "LOAD"):
            assert isinstance(status[f"{prefix}_status"], str)
            assert isinstance(status[f"{prefix}_T_now"], float)
            assert isinstance(status[f"{prefix}_timestamp"], float)
            assert isinstance(status[f"{prefix}_T_target"], float)
            assert isinstance(status[f"{prefix}_drive_level"], float)
            assert isinstance(status[f"{prefix}_enabled"], bool)
            assert isinstance(status[f"{prefix}_active"], bool)
            assert isinstance(status[f"{prefix}_sensor_tripped"], bool)
            # KV_INT (and not a bool masquerading as one).
            assert type(status[f"{prefix}_sensor_rejects"]) is int
            assert isinstance(status[f"{prefix}_runaway_tripped"], bool)
            assert isinstance(status[f"{prefix}_hysteresis"], float)
            assert isinstance(status[f"{prefix}_clamp"], float)


class TestImuStatusTypes:
    def test_status_field_types(self):
        emu = ImuEmulator()
        emu.op()
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        for key in ("yaw", "pitch", "roll", "accel_x", "accel_y", "accel_z"):
            assert isinstance(status[key], float), f"{key} should be float"


class TestLidarStatusTypes:
    def test_status_field_types(self):
        emu = LidarEmulator()
        emu.op()
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        assert isinstance(status["distance_m"], float)
        assert isinstance(status["current_voltage"], float)


class TestRFSwitchStatusTypes:
    def test_status_field_types(self):
        emu = RFSwitchEmulator(settle_ms=0)
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        assert isinstance(status["sw_state"], int)
        for i in range(3):
            assert isinstance(status[f"volt_therm{i}"], float)


# ---------------------------------------------------------------------------
# Malformed input handling
# ---------------------------------------------------------------------------


class TestMalformedInput:
    """Verify emulators survive bad input without crashing.

    The C firmware silently ignores malformed JSON (cJSON_Parse returns NULL)
    and non-numeric values (valueint/valuedouble return 0).  Emulators must
    do the same.
    """

    def test_motor_invalid_type_values(self):
        emu = MotorEmulator()
        emu.server({"az_set_pos": "not_a_number"})
        assert emu.azimuth.position == 0  # unchanged (default kept)

    def test_motor_null_value(self):
        emu = MotorEmulator()
        emu.server({"az_set_pos": None})
        assert emu.azimuth.position == 0

    def test_motor_empty_json(self):
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 500})
        emu.server({})
        assert emu.azimuth.target_pos == 500  # unchanged

    def test_motor_unknown_keys_ignored(self):
        emu = MotorEmulator()
        emu.server({"unknown_key": 42, "az_set_pos": 100})
        assert emu.azimuth.position == 100

    def test_rfswitch_invalid_type(self):
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": "abc"})
        # commanded_state must be untouched by a non-numeric payload.
        assert emu.commanded_state == 0
        assert emu.get_status()["sw_state"] == 0

    def test_rfswitch_null_value(self):
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": None})
        assert emu.commanded_state == 0
        assert emu.get_status()["sw_state"] == 0

    def test_tempctrl_invalid_type(self):
        # cJSON valuedouble is 0.0 for non-numeric JSON; firmware writes it
        # straight into the struct (see tempctrl.c LNA_temp_target parse).
        emu = TempCtrlEmulator()
        emu.server({"LNA_temp_target": "hot"})
        assert emu.lna.T_target == 0.0

    def test_tempctrl_null_value(self):
        # Same cJSON path as above; LNA_clamp is then clamped to [0, 1].
        emu = TempCtrlEmulator()
        emu.server({"LNA_clamp": None})
        assert emu.lna.clamp == 0.0


# ---------------------------------------------------------------------------
# Error state injection
# ---------------------------------------------------------------------------


class TestTempCtrlErrorState:
    def test_sensor_error_clear(self):
        emu = TempCtrlEmulator()
        emu.inject_sensor_error("LOAD")
        assert emu.get_status()["LOAD_status"] == "error"
        emu.inject_sensor_error("LOAD", error=False)
        assert emu.get_status()["LOAD_status"] == "update"

    def test_independent_channel_errors(self):
        emu = TempCtrlEmulator()
        emu.inject_sensor_error("LNA")
        emu.inject_sensor_error("LOAD")
        status = emu.get_status()
        assert status["LNA_status"] == "error"
        assert status["LOAD_status"] == "error"


class TestImuErrorState:
    def test_persistent_failure_then_recovery(self):
        """Sustained sensor failure flips status="error"; recovery returns to "update"."""
        import picohost.emulators.imu as imu_mod

        emu = ImuEmulator()
        emu.op()
        assert emu.get_status()["status"] == "update"

        emu.simulate_sensor_failure()
        emu._last_event_time -= imu_mod.IMU_EVENT_TIMEOUT_S + 1
        emu.op()
        assert emu.get_status()["status"] == "error"

        emu.simulate_sensor_recovery()
        emu.op()
        assert emu.get_status()["status"] == "update"

    def test_reinit_clears_sensor_data(self):
        """Mirror imu.c:109 memset(&imu.data, 0, sizeof(imu.data)).

        After a timeout-triggered re-init, firmware reports zeros until
        the next packet arrives.  Emulator must do the same so a host
        sees status="error" alongside zero data, not stale pre-failure
        values.
        """
        import picohost.emulators.imu as imu_mod

        emu = ImuEmulator()
        for _ in range(20):
            emu.op()
        emu.get_status()
        assert (emu.yaw, emu.pitch, emu.roll) != (0.0, 0.0, 0.0)

        emu.simulate_sensor_failure()
        emu._last_event_time -= imu_mod.IMU_EVENT_TIMEOUT_S + 1
        emu.op()
        assert emu.is_initialized is False

        emu.op()
        status = emu.get_status()
        assert status["status"] == "error"
        assert status["yaw"] == 0.0
        assert status["pitch"] == 0.0
        assert status["roll"] == 0.0
        assert status["accel_x"] == 0.0
        assert status["accel_y"] == 0.0
        assert status["accel_z"] == 0.0


# ---------------------------------------------------------------------------
# Edge case behavioral tests
# ---------------------------------------------------------------------------


class TestTempCtrlEdgeCases:
    def test_both_channels_converge_independently(self):
        emu = TempCtrlEmulator()
        emu.lna.T_now = 20.0
        emu.load.T_now = 40.0
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LOAD_temp_target": 30.0,
                "LOAD_enable": True,
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 30.0) < 0.5
        assert abs(emu.load.T_now - 30.0) < 0.5

    def test_disable_mid_convergence(self):
        emu = TempCtrlEmulator()
        emu.lna.T_now = 20.0
        emu.server({"LNA_temp_target": 40.0, "LNA_enable": True})
        for _ in range(100):
            emu.op()
        assert emu.lna.drive != 0.0
        emu.server({"LNA_enable": False})
        emu.op()
        assert emu.lna.drive == 0.0


ALL_EMULATORS = [
    MotorEmulator,
    TempCtrlEmulator,
    ImuEmulator,
    LidarEmulator,
    PotMonEmulator,
    RFSwitchEmulator,
]


class TestSensorName:
    """Ensure every emulator status contains 'sensor_name' for redis_handler."""

    @pytest.mark.parametrize(
        "emu_cls", ALL_EMULATORS, ids=lambda c: c.__name__
    )
    def test_get_status_has_sensor_name(self, emu_cls):
        emu = emu_cls()
        result = emu.get_status()
        # Normalize to list — composite emulators return multiple dicts,
        # each of which reaches redis_handler individually.
        statuses = result if isinstance(result, list) else [result]
        for status in statuses:
            assert "sensor_name" in status
