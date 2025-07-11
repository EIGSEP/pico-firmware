from argparse import ArgumentParser
import json
import sys
from threading import Thread
import time
import queue
from picohost import PicoDevice, PicoPeltier

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


t = PicoPeltier(args.port, verbose=True) # Control mode for Peltier
#t = PicoStatus(args.port, verbose=True) # temperature monitor

while True:
    temp_data.append(t.status.copy())
    time.sleep(0.1)
