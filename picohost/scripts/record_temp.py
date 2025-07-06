from argparse import ArgumentParser
import time

import numpy as np
from picohost import PicoDevice

parser = ArgumentParser(description="Record temperature from Pico device")
parser.add_argument( "-p", "--port", type=str, default="/dev/ttyACM0")
args = parser.parse_args()

temp_data = []
def add_temp_data(json, temp_data=temp_data):
    """Callback function to add temperature data."""
    temp = json.get("temperature1")
    temp_data.append(temp)

t = PicoDevice(args.port)
t.set_response_handler(add_temp_data)
with t:
    print("Recording temperature data. Press Ctrl+C to stop.")
    try:
        while True:
            if temp_data:
                print(f"temp = {temp_data[-1]}")
            time.sleep(1)  # Adjust the sleep time as needed
    except KeyboardInterrupt:
        print("Recording stopped.")

np.save("temp_data.npy", temp_data)
