"""
Picohost - Python library for communicating with Raspberry Pi Pico devices.
"""

from importlib.metadata import version

__version__ = version("picohost")

from .base import (
    PicoDevice,
    PicoRFSwitch,
    PicoPeltier,
    PicoIMU,
    PicoLidar,
    PicoPotentiometer,
)
from .motor import PicoMotor
from .manager import PicoManager
from .proxy import PicoProxy, RFSwitchProxy
from . import testing
from . import emulators

__all__ = [
    "PicoDevice",
    "PicoMotor",
    "PicoRFSwitch",
    "PicoPeltier",
    "PicoIMU",
    "PicoLidar",
    "PicoPotentiometer",
    "PicoManager",
    "PicoProxy",
    "RFSwitchProxy",
    "testing",
    "emulators",
]
