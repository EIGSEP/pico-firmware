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
operation releases both drivers again — including on error, releasing
BOOTSEL before RUN — so a running pico never has its flash CS yanked
low.

The lines are driven through the ``pinctrl`` CLI that ships with
Raspberry Pi OS (``pinctrl set <gpio> op dh|dl`` sets a pin to output
and drives it high/low in one call) — no GPIO library or pin-factory
backend to install. This is the exact mechanism verified reliable on
the hub.
"""

import argparse
import logging
import shutil
import subprocess
import time

logger = logging.getLogger(__name__)

BOOTSEL_GPIO = 18  # BCM; drives all picos' BOOTSEL/QSPI-CS pads low
RUN_GPIO = 17  # BCM; drives all picos' RUN/RESET pads low

_SETTLE_S = 0.05  # RUN asserted before BOOTSEL joins it
_RUN_PULSE_S = 0.1  # RUN+BOOTSEL held together before RUN releases
_BOOTSEL_SAMPLE_S = 0.4  # BOOTSEL held after RUN release (bootrom samples)

# Inverting drivers: a Pi pin driven HIGH grounds its bussed line
# (assert); driven LOW switches the driver off and the line floats high
# via the picos' pull-ups (release). These are the pinctrl drive tokens.
_ASSERT = "dh"  # pinctrl drive-high
_RELEASE = "dl"  # pinctrl drive-low


def _pinctrl(gpio, level):
    """Drive *gpio* (BCM) to *level* (``_ASSERT``/``_RELEASE``).

    Shells out to ``pinctrl set <gpio> op <dh|dl>``, which configures
    the pin as an output and sets its level in one call. Raises
    ``subprocess.CalledProcessError`` if pinctrl exits non-zero (e.g.
    not installed, or insufficient permission) so a failed transition
    is never silently ignored.
    """
    subprocess.run(
        ["pinctrl", "set", str(gpio), "op", level],
        check=True,
        capture_output=True,
        text=True,
    )


def _release(gpio):
    """Best-effort release of *gpio*; log and swallow failures.

    Used on the cleanup path so a release error cannot mask the
    original exception, and so one stuck line still lets the others be
    released.
    """
    try:
        _pinctrl(gpio, _RELEASE)
    except Exception:
        logger.warning("failed to release GPIO%d via pinctrl", gpio,
                       exc_info=True)


def _release_lines():
    """Release BOOTSEL then RUN (best effort).

    BOOTSEL (the shared QSPI flash CS) is released first so the picos
    never exit reset with their flash CS still grounded.
    """
    _release(BOOTSEL_GPIO)
    _release(RUN_GPIO)


def enter_bootsel(
    settle=_SETTLE_S,
    run_pulse=_RUN_PULSE_S,
    bootsel_sample=_BOOTSEL_SAMPLE_S,
):
    """Put ALL bussed picos into BOOTSEL via the shared GPIO lines.

    Issues the verified-reliable ``pinctrl`` ordering
    (``17 dh, 18 dh, 17 dl, 18 dl``); with the inverting drivers that
    is:

    1. assert RUN — hold every pico in reset
    2. wait *settle*, then assert BOOTSEL (the shared QSPI-CS line is
       only pulled low while the picos are halted)
    3. hold both *run_pulse*, then release RUN — the picos exit reset
       and the bootrom samples BOOTSEL
    4. hold BOOTSEL low *bootsel_sample* while sampling completes
    5. release BOOTSEL

    Both lines are always released, even on exception (BOOTSEL before
    RUN). After this returns, every pico re-enumerates as a
    ``2e8a:000f`` mass-storage device.
    """
    logger.info(
        "Mass BOOTSEL entry: BOOTSEL=GPIO%d RUN=GPIO%d",
        BOOTSEL_GPIO,
        RUN_GPIO,
    )
    try:
        _pinctrl(RUN_GPIO, _ASSERT)
        time.sleep(settle)
        _pinctrl(BOOTSEL_GPIO, _ASSERT)
        time.sleep(run_pulse)
        _pinctrl(RUN_GPIO, _RELEASE)  # picos boot, bootrom samples BOOTSEL
        time.sleep(bootsel_sample)
        _pinctrl(BOOTSEL_GPIO, _RELEASE)
    except BaseException:
        _release_lines()
        raise


def reset(run_pulse=_RUN_PULSE_S):
    """Pulse the shared RUN line low, then release it.

    All bussed picos reset and boot their current firmware
    simultaneously. BOOTSEL is never touched, and RUN is released even
    if interrupted mid-pulse.
    """
    logger.info("Mass reset: RUN=GPIO%d", RUN_GPIO)
    try:
        _pinctrl(RUN_GPIO, _ASSERT)
        time.sleep(run_pulse)
        _pinctrl(RUN_GPIO, _RELEASE)
    except BaseException:
        _release(RUN_GPIO)
        raise


def available():
    """Return True if the ``pinctrl`` CLI is on PATH.

    False on hosts without it (e.g. a dev laptop) — callers should fall
    back to the USB flash path or tell the user to pass ``--no-gpio``.
    """
    return shutil.which("pinctrl") is not None


def main(argv=None):
    p = argparse.ArgumentParser(
        description=(
            "Drive the bussed Pico BOOTSEL/RUN GPIO lines on the "
            "observatory hub."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser(
        "bootsel",
        help="Put ALL bussed Picos into BOOTSEL (2e8a:000f).",
    )
    sub.add_parser(
        "reset",
        help="Reset (reboot) ALL bussed Picos into their firmware.",
    )
    args = p.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    if args.cmd == "bootsel":
        enter_bootsel()
    elif args.cmd == "reset":
        reset()


if __name__ == "__main__":
    main()
