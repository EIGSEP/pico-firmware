__author__ = "Charlie Tolley"
__version__ = "0.0.1"

try:
    from .switch import SwitchNetwork
    from . import testing
except ImportError:
    print("Running on Pico, skipping SwitchNetwork import")
finally:
    from . import pico_utils
