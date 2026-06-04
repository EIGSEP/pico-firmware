"""Mass BOOTSEL entry and reset for the bussed observatory picos.

All picos on the observatory hub share two control lines driven from
Pi GPIOs (BCM numbering) through inverting drivers: every BOOTSEL pad
is bussed to a driver on GPIO 18 and every RUN/RESET pad to one on
GPIO 17. Driving a Pi pin HIGH pulls its bussed line to ground
(assert); driving it LOW switches the driver off and the line floats
high via the picos' pull-ups (release). One :func:`enter_bootsel`
call therefore puts the whole fleet into BOOTSEL at once, with no
dependency on working CDC firmware (it recovers wedged or bricked
picos too), and one :func:`reset` pulse boots them all.

BOOTSEL doubles as each pico's QSPI flash chip-select, so its line is
only ever asserted while the picos are held in reset, and every
operation switches the drivers off again — including on error —
before releasing the pins to input (where the Pi's default pull-downs
on BCM 17/18 keep the drivers off).

The pin factory backend (lgpio on the field image) is supplied by the
deployment; tests run against ``GPIOZERO_PIN_FACTORY=mock``.
"""

import logging
import time
from contextlib import contextmanager

from gpiozero import Device, OutputDevice

logger = logging.getLogger(__name__)

BOOTSEL_GPIO = 18  # BCM; drives all picos' BOOTSEL/QSPI-CS pads low
RUN_GPIO = 17  # BCM; drives all picos' RUN/RESET pads low

_SETTLE_S = 0.05  # RUN asserted before BOOTSEL joins it
_RUN_PULSE_S = 0.1  # RUN+BOOTSEL held together before RUN releases
_BOOTSEL_SAMPLE_S = 0.4  # BOOTSEL held after RUN release (bootrom samples)


@contextmanager
def _line_driver(gpio):
    """Yield an :class:`OutputDevice` driving the bussed line on *gpio*.

    The hardware inverts: Pi pin HIGH grounds the line (``on()`` =
    assert), Pi pin LOW releases it. Construction starts released
    (``initial_value=False`` drives LOW), and the ``finally`` always
    switches the driver off and re-muxes the pin to input before
    close — on the kernel cdev backend a released output pin keeps
    driving its last level, and as an input the Pi's default
    pull-down holds the driver off.
    """
    dev = OutputDevice(gpio, active_high=True, initial_value=False)
    try:
        yield dev
    finally:
        try:
            dev.off()
            if dev.pin is not None:
                dev.pin.function = "input"
        finally:
            dev.close()


def enter_bootsel(
    settle=_SETTLE_S,
    run_pulse=_RUN_PULSE_S,
    bootsel_sample=_BOOTSEL_SAMPLE_S,
):
    """Put ALL bussed picos into BOOTSEL via the shared GPIO lines.

    Sequence (inverted drivers; assert = Pi pin HIGH grounds the line):

    1. assert RUN — hold every pico in reset
    2. wait *settle*, then assert BOOTSEL (the shared QSPI-CS line is
       only pulled low while the picos are halted)
    3. hold both *run_pulse*, then release RUN — the picos exit reset
       and the bootrom samples BOOTSEL
    4. hold BOOTSEL low *bootsel_sample* while sampling completes
    5. release BOOTSEL

    Lines are always released, even on exception. After this returns,
    every pico re-enumerates as a ``2e8a:000f`` mass-storage device.
    """
    logger.info(
        "Mass BOOTSEL entry: BOOTSEL=GPIO%d RUN=GPIO%d",
        BOOTSEL_GPIO,
        RUN_GPIO,
    )
    with _line_driver(RUN_GPIO) as run:
        run.on()
        time.sleep(settle)
        with _line_driver(BOOTSEL_GPIO) as bootsel:
            bootsel.on()
            time.sleep(run_pulse)
            run.off()  # picos boot, bootrom samples BOOTSEL
            time.sleep(bootsel_sample)
            bootsel.off()


def reset(run_pulse=_RUN_PULSE_S):
    """Pulse the shared RUN line low, then release it.

    All bussed picos reset and boot their current firmware
    simultaneously.
    """
    logger.info("Mass reset: RUN=GPIO%d", RUN_GPIO)
    with _line_driver(RUN_GPIO) as run:
        run.on()
        time.sleep(run_pulse)
        run.off()


def available():
    """Return True if gpiozero can construct a usable pin factory.

    False on hosts without GPIO hardware/backends (gpiozero raises
    ``BadPinFactory`` when no factory loads) — callers should fall back
    to the USB flash path or tell the user to pass ``--no-gpio``.
    """
    try:
        Device.ensure_pin_factory()
    except Exception:
        return False
    return True
