"""
Picohost - Python library for communicating with Raspberry Pi Pico devices.
"""

from .base import PicoDevice, PicoRFSwitch, PicoPeltier, PicoIMU, PicoPotentiometer
from importlib.metadata import version

__version__ = version("picohost")

from .motor import PicoMotor
from . import testing
from . import emulators

__all__ = [
    "PicoDevice",
    "PicoMotor",
    "PicoRFSwitch",
    "PicoPeltier",
    "PicoIMU",
    "PicoPotentiometer",
    "testing",
    "emulators",
]
