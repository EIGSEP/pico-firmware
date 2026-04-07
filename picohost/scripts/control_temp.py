from argparse import ArgumentParser
import time
from picohost import PicoPeltier

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


t = PicoPeltier(args.port, verbose=True)  # Control mode for Peltier
t.set_temperature(T_LNA=25, T_LOAD=25)
t.set_enable(LNA=True, LOAD=True)
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    t.set_enable(LNA=False, LOAD=False)
