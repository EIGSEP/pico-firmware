"""Tests for picohost.gpio — mass BOOTSEL/reset via bussed GPIO lines.

All assertions run against gpiozero's mock pin factory (installed by
the ``mock_pins`` conftest fixture). The wiring uses inverting
drivers: driving a Pi pin HIGH pulls the bussed pico line to ground
(assert); driving it LOW releases the line. The ordering tests record
``OutputDevice.on``/``off`` and ``_set_function`` events to prove the
sequence and that every line is released even on error.
"""

import pytest
from gpiozero import OutputDevice
from gpiozero.pins.mock import MockPin

import picohost.gpio as gpio


@pytest.fixture
def events(monkeypatch, mock_pins):
    """Record OutputDevice.on/off calls and pin function changes.

    Entries are ``("on", pin)``, ``("off", pin)`` and
    ``("func", pin, value)`` tuples; pins compare by identity, and the
    mock factory caches instances, so they can be matched against
    ``mock_pins.pin(n)``.
    """
    log = []

    orig_on = OutputDevice.on

    def spy_on(self):
        log.append(("on", self.pin))
        orig_on(self)

    monkeypatch.setattr(OutputDevice, "on", spy_on)

    orig_off = OutputDevice.off

    def spy_off(self):
        log.append(("off", self.pin))
        orig_off(self)

    monkeypatch.setattr(OutputDevice, "off", spy_off)

    orig_set = MockPin._set_function

    def spy_set(self, value):
        log.append(("func", self, value))
        orig_set(self, value)

    monkeypatch.setattr(MockPin, "_set_function", spy_set)
    return log


class TestEnterBootsel:
    def test_sequence_order(self, events, mock_pins, monkeypatch):
        """Reset first, BOOTSEL while held in reset, release in order.

        Asserting RESET before BOOTSEL means the shared QSPI-CS line is
        only pulled low while the picos are already halted; the bootrom
        samples BOOTSEL as RUN is released, so RUN must be released
        before BOOTSEL.
        """
        monkeypatch.setattr(gpio.time, "sleep", lambda s: None)

        gpio.enter_bootsel()

        bootsel = mock_pins.pin(gpio.BOOTSEL_GPIO)
        run = mock_pins.pin(gpio.RUN_GPIO)
        # list.index finds the first occurrence of each event.
        assert (
            events.index(("on", run))
            < events.index(("on", bootsel))
            < events.index(("off", run))
            < events.index(("off", bootsel))
        )

    def test_pins_end_released(self, mock_pins, monkeypatch):
        # Both Pi pins end LOW (drivers off) and re-muxed to input,
        # where the Pi's default pull-downs keep the drivers off.
        monkeypatch.setattr(gpio.time, "sleep", lambda s: None)

        gpio.enter_bootsel()

        for n in (gpio.BOOTSEL_GPIO, gpio.RUN_GPIO):
            assert mock_pins.pin(n).state is False
            assert mock_pins.pin(n).function == "input"

    def test_releases_pins_on_exception(self, mock_pins, monkeypatch):
        """Both drivers switch off even if interrupted mid-sequence.

        A stuck-asserted BOOTSEL driver grounds the picos' shared QSPI
        flash CS and corrupts every running pico.
        """
        calls = {"n": 0}

        def boom(seconds):
            calls["n"] += 1
            if calls["n"] == 3:  # the bootsel_sample hold
                raise RuntimeError("boom")

        monkeypatch.setattr(gpio.time, "sleep", boom)

        with pytest.raises(RuntimeError, match="boom"):
            gpio.enter_bootsel()

        for n in (gpio.BOOTSEL_GPIO, gpio.RUN_GPIO):
            assert mock_pins.pin(n).state is False
            assert mock_pins.pin(n).function == "input"


class TestReset:
    def test_pulses_run_then_releases(self, events, mock_pins,
                                      monkeypatch):
        monkeypatch.setattr(gpio.time, "sleep", lambda s: None)

        gpio.reset()

        run = mock_pins.pin(gpio.RUN_GPIO)
        assert events.index(("on", run)) < events.index(("off", run))
        assert run.state is False
        assert run.function == "input"

    def test_does_not_touch_bootsel(self, events, mock_pins, monkeypatch):
        monkeypatch.setattr(gpio.time, "sleep", lambda s: None)

        gpio.reset()

        run = mock_pins.pin(gpio.RUN_GPIO)
        assert [e for e in events if e[0] == "on"] == [("on", run)]

    def test_releases_on_exception(self, mock_pins, monkeypatch):
        def boom(seconds):
            raise RuntimeError("boom")

        monkeypatch.setattr(gpio.time, "sleep", boom)

        with pytest.raises(RuntimeError, match="boom"):
            gpio.reset()

        run = mock_pins.pin(gpio.RUN_GPIO)
        assert run.state is False
        assert run.function == "input"


class TestLineDriver:
    def test_construction_starts_released(self, mock_pins):
        """Opening a line must start with the driver off (line floats
        high via the picos' pull-ups), never asserted."""
        pin = mock_pins.pin(gpio.BOOTSEL_GPIO)

        with gpio._line_driver(gpio.BOOTSEL_GPIO):
            assert pin.function == "output"
            assert pin.state is False  # driver off

        assert pin.function == "input"

    def test_drive_semantics(self, mock_pins):
        """on() drives the Pi pin HIGH — the inverting driver grounds
        the bussed line (assert); off() releases it."""
        with gpio._line_driver(gpio.RUN_GPIO) as dev:
            pin = mock_pins.pin(gpio.RUN_GPIO)
            dev.on()
            assert pin.state is True
            dev.off()
            assert pin.state is False


class TestConstants:
    def test_wiring_assignment(self):
        """BCM 18 = BOOTSEL bus, BCM 17 = RUN bus (hardware wiring).

        Swapping these would pulse BOOTSEL as if it were RUN — guard
        the assignment explicitly.
        """
        assert gpio.BOOTSEL_GPIO == 18
        assert gpio.RUN_GPIO == 17


class TestAvailable:
    def test_true_under_mock_factory(self, mock_pins):
        assert gpio.available() is True
