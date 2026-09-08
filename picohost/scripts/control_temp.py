from argparse import ArgumentParser
import time
from picohost import PicoTempCtrl

parser = ArgumentParser(description="Record temperature from Pico device")
parser.add_argument(
    "-p",
    "--port",
    type=str,
    default="/dev/ttyACM0",
    help="Serial port for Pico device (default: /dev/ttyACM0)",
)
args = parser.parse_args()

temp_data = []


t = PicoTempCtrl(args.port, verbose=True)  # Control mode for LOAD heater
t.set_temperature(T_LOAD=25)
t.set_enable(LOAD=True)
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    t.set_enable(LOAD=False)
