from argparse import ArgumentParser
import json
import sys
from threading import Thread
import time
import queue
from eigsep_observing import EigsepRedis
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


t = PicoPeltier(args.port, EigsepRedis(), verbose=True) # Control mode for Peltier
t.set_temperature(T_A=25, T_B=25)
t.set_enable(A=True, B=True)
try:
    while True:
        time.sleep(0.1)
except(KeyboardInterrupt):
    t.set_enable(A=False, B=False)
