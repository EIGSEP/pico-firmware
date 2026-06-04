"""Tests for picohost.gpio — mass BOOTSEL/reset via the ``pinctrl`` CLI.

The bussed control lines use inverting drivers: ``pinctrl set <gpio> op
dh`` (drive-high) grounds the bussed pico line (assert), and ``dl``
(drive-low) releases it. These tests mock ``subprocess.run`` so no real
GPIO is ever touched, and assert the exact ``(gpio, level)`` sequence
plus that both lines are released even on error.
"""

import subprocess

import pytest

import picohost.gpio as gpio

ASSERT = gpio._ASSERT
RELEASE = gpio._RELEASE


@pytest.fixture
def pinctrl(monkeypatch):
    """Record pinctrl invocations as ``(gpio, level)`` tuples.

    Patches ``subprocess.run`` in :mod:`picohost.gpio` (so nothing
    touches real hardware) and no-ops ``time.sleep``. Returns the list
    of recorded calls in order; tests that need ``sleep`` to raise can
    re-monkeypatch it (their override wins over this fixture's).
    """
    calls = []

    def fake_run(cmd, **kwargs):
        assert cmd[:2] == ["pinctrl", "set"]
        assert cmd[3] == "op"
        calls.append((int(cmd[2]), cmd[4]))
        return subprocess.CompletedProcess(cmd, 0, "", "")

    monkeypatch.setattr(gpio.subprocess, "run", fake_run)
    monkeypatch.setattr(gpio.time, "sleep", lambda s: None)
    return calls


def _last_level_per_gpio(calls):
    last = {}
    for g, level in calls:
        last[g] = level
    return last


class TestPinctrl:
    def test_invokes_subprocess_with_check(self, monkeypatch):
        """`pinctrl set <gpio> op <level>`, with check=True so a
        nonzero exit raises rather than silently no-op'ing the line."""
        seen = {}

        def fake_run(cmd, **kwargs):
            seen["cmd"] = cmd
            seen["check"] = kwargs.get("check")
            return subprocess.CompletedProcess(cmd, 0, "", "")

        monkeypatch.setattr(gpio.subprocess, "run", fake_run)

        gpio._pinctrl(17, "dh")

        assert seen["cmd"] == ["pinctrl", "set", "17", "op", "dh"]
        assert seen["check"] is True

    def test_raises_on_nonzero_exit(self, monkeypatch):
        def fake_run(cmd, **kwargs):
            raise subprocess.CalledProcessError(1, cmd, "", "boom")

        monkeypatch.setattr(gpio.subprocess, "run", fake_run)

        with pytest.raises(subprocess.CalledProcessError):
            gpio._pinctrl(17, "dh")


class TestEnterBootsel:
    def test_sequence_order(self, pinctrl):
        """17 dh, 18 dh, 17 dl, 18 dl — the verified-reliable order.

        Asserting RUN before BOOTSEL keeps the shared QSPI-CS line
        pulled only while the picos are halted; releasing RUN before
        BOOTSEL lets the bootrom sample BOOTSEL as they exit reset.
        """
        gpio.enter_bootsel()

        assert pinctrl == [
            (gpio.RUN_GPIO, ASSERT),
            (gpio.BOOTSEL_GPIO, ASSERT),
            (gpio.RUN_GPIO, RELEASE),
            (gpio.BOOTSEL_GPIO, RELEASE),
        ]

    def test_exact_argv(self, monkeypatch):
        """Each step shells out as ``pinctrl set <gpio> op <dh|dl>``."""
        argvs = []

        def fake_run(cmd, **kwargs):
            argvs.append(cmd)
            return subprocess.CompletedProcess(cmd, 0, "", "")

        monkeypatch.setattr(gpio.subprocess, "run", fake_run)
        monkeypatch.setattr(gpio.time, "sleep", lambda s: None)

        gpio.enter_bootsel()

        assert argvs[:4] == [
            ["pinctrl", "set", "17", "op", "dh"],
            ["pinctrl", "set", "18", "op", "dh"],
            ["pinctrl", "set", "17", "op", "dl"],
            ["pinctrl", "set", "18", "op", "dl"],
        ]

    def test_lines_end_released(self, pinctrl):
        """Both lines finish released (dl) — the safe, runnable state."""
        gpio.enter_bootsel()

        last = _last_level_per_gpio(pinctrl)
        assert last[gpio.RUN_GPIO] == RELEASE
        assert last[gpio.BOOTSEL_GPIO] == RELEASE

    def test_releases_lines_on_exception(self, pinctrl, monkeypatch):
        """Both lines are released even if interrupted mid-sequence.

        A stuck-asserted BOOTSEL driver grounds the picos' shared QSPI
        flash CS and corrupts every running pico, so cleanup must
        release BOOTSEL (before RUN) no matter what.
        """
        calls = {"n": 0}

        def boom(seconds):
            calls["n"] += 1
            if calls["n"] == 3:  # the bootsel_sample hold
                raise RuntimeError("boom")

        monkeypatch.setattr(gpio.time, "sleep", boom)

        with pytest.raises(RuntimeError, match="boom"):
            gpio.enter_bootsel()

        # cleanup releases BOOTSEL before RUN
        assert pinctrl[-2:] == [
            (gpio.BOOTSEL_GPIO, RELEASE),
            (gpio.RUN_GPIO, RELEASE),
        ]
        last = _last_level_per_gpio(pinctrl)
        assert last[gpio.RUN_GPIO] == RELEASE
        assert last[gpio.BOOTSEL_GPIO] == RELEASE

    def test_cleanup_runs_even_if_a_release_fails(self, monkeypatch):
        """If releasing BOOTSEL fails, RUN is still released, and the
        original error propagates — not the cleanup failure."""
        seq = []

        def fake_run(cmd, **kwargs):
            seq.append((int(cmd[2]), cmd[4]))
            # Make every BOOTSEL release fail (incl. the cleanup one).
            if cmd[2] == str(gpio.BOOTSEL_GPIO) and cmd[4] == RELEASE:
                raise subprocess.CalledProcessError(1, cmd, "", "no perm")
            return subprocess.CompletedProcess(cmd, 0, "", "")

        sleeps = {"n": 0}

        def boom(seconds):
            sleeps["n"] += 1
            if sleeps["n"] == 2:  # interrupt at the run_pulse hold
                raise RuntimeError("boom")

        monkeypatch.setattr(gpio.subprocess, "run", fake_run)
        monkeypatch.setattr(gpio.time, "sleep", boom)

        with pytest.raises(RuntimeError, match="boom"):
            gpio.enter_bootsel()

        # RUN still released by cleanup despite the BOOTSEL release error
        assert (gpio.RUN_GPIO, RELEASE) in seq


class TestReset:
    def test_pulses_run_then_releases(self, pinctrl):
        gpio.reset()

        assert pinctrl == [
            (gpio.RUN_GPIO, ASSERT),
            (gpio.RUN_GPIO, RELEASE),
        ]

    def test_does_not_touch_bootsel(self, pinctrl):
        gpio.reset()

        assert all(g == gpio.RUN_GPIO for g, _ in pinctrl)

    def test_releases_on_exception(self, pinctrl, monkeypatch):
        def boom(seconds):
            raise RuntimeError("boom")

        monkeypatch.setattr(gpio.time, "sleep", boom)

        with pytest.raises(RuntimeError, match="boom"):
            gpio.reset()

        assert pinctrl[-1] == (gpio.RUN_GPIO, RELEASE)
        assert (gpio.BOOTSEL_GPIO, ASSERT) not in pinctrl


class TestConstants:
    def test_wiring_assignment(self):
        """BCM 18 = BOOTSEL bus, BCM 17 = RUN bus (hardware wiring).

        Swapping these would pulse BOOTSEL as if it were RUN — guard
        the assignment explicitly.
        """
        assert gpio.BOOTSEL_GPIO == 18
        assert gpio.RUN_GPIO == 17

    def test_drive_levels(self):
        """Inverting drivers: dh asserts (grounds the line), dl
        releases. These pinctrl tokens are the verified ones."""
        assert gpio._ASSERT == "dh"
        assert gpio._RELEASE == "dl"


class TestAvailable:
    def test_true_when_pinctrl_present(self, monkeypatch):
        monkeypatch.setattr(
            gpio.shutil, "which", lambda name: "/usr/bin/pinctrl"
        )
        assert gpio.available() is True

    def test_false_when_pinctrl_absent(self, monkeypatch):
        monkeypatch.setattr(gpio.shutil, "which", lambda name: None)
        assert gpio.available() is False


class TestMain:
    def test_bootsel_subcommand_invokes_enter_bootsel(self, monkeypatch):
        calls = []
        monkeypatch.setattr(
            gpio, "enter_bootsel", lambda: calls.append("bootsel")
        )
        monkeypatch.setattr(gpio, "reset", lambda: calls.append("reset"))
        gpio.main(["bootsel"])
        assert calls == ["bootsel"]

    def test_reset_subcommand_invokes_reset(self, monkeypatch):
        calls = []
        monkeypatch.setattr(
            gpio, "enter_bootsel", lambda: calls.append("bootsel")
        )
        monkeypatch.setattr(gpio, "reset", lambda: calls.append("reset"))
        gpio.main(["reset"])
        assert calls == ["reset"]

    def test_subcommand_required(self):
        with pytest.raises(SystemExit):
            gpio.main([])
