from argparse import ArgumentParser
import sys
from threading import Thread
import time
import queue
import numpy as np
from picohost import PicoDevice, PicoPeltier

parser = ArgumentParser(description="Record temperature from Pico device")
parser.add_argument("-p", "--port", type=str, default="/dev/ttyACM0")
parser.add_argument(
    "--ctrl", action="store_true", help="Use control mode for Peltier device"
)
parser.add_argument(
    "--print", action="store_true", help="Print temperature data to console"
)
args = parser.parse_args()

input_queue = queue.Queue()
data_queue = queue.Queue()
temp_data = []


def add_temp_data(json):
    """Callback function to add temperature data."""
    data_queue.put(json)


def stdin_reader():
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        input_queue.put(line.strip())


def handle_commands(cmd, peltier):
    if not cmd:
        return
    parts = cmd.split()
    cmd = parts[0].lower()
    ch = 1  # XXX can update to allow channel selection

    if cmd == "temp" and len(parts) >= 2:
        temp = float(parts[1])
        if peltier.set_temperature(temp, ch):
            print(f"Temperature set to {temp} °C on channel {ch}.")

    elif cmd == "hyst" and len(parts) >= 2:
        hyst = float(parts[1])
        if peltier.set_hysteresis(hyst, ch):
            print(f"Hysteresis set to {hyst} °C on channel {ch}.")

    elif cmd == "enable":
        if peltier.enable_channel(ch):
            print(f"Channel {ch} enabled.")

    elif cmd == "disable":
        if peltier.disable_channel(ch):
            print(f"Channel {ch} disabled.")

    else:
        print(
            f"Unknown command: {cmd}. Available commands: temp, hyst, "
            "enable, disable, exit."
        )


if args.ctrl:
    print("Using control mode for Peltier device.")
    t = PicoPeltier(args.port)
else:
    print("Using standard temperature recording mode.")
    t = PicoDevice(args.port)

t.set_response_handler(add_temp_data)
read_thread = Thread(target=stdin_reader, daemon=True)
read_thread.start()
t0 = time.time()
print_time = 0
print_cadence = 5  # seconds
with t:
    print("Recording temperature data. Press Ctrl+C to stop.")
    if args.ctrl:
        print("You can enter commands to control the Peltier device.")
        print(
            "Available commands: temp <value>, hyst <value>, enable, "
            "disable, exit."
        )
    while time.time() - t0 < 30 * 60:
        try:
            try:
                json = data_queue.get_nowait()
            except queue.Empty:
                json = None
            if json:
                temp_data.append(json)
            if args.print:
                now = time.time()
                if now - print_time < print_cadence:
                    continue
                print(json.dums(json.loads(json), indent=2))
                print_time = now
            if args.ctrl:
                try:
                    cmd = input_queue.get_nowait()
                except queue.Empty:
                    cmd = None
                handle_commands(cmd, t)
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("Recording stopped.")
            break

np.save("temp_data.npy", temp_data)
