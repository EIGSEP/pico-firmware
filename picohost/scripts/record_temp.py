from argparse import ArgumentParser

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

temp_data = []


def add_temp_data(json, temp_data=temp_data, print=args.print):
    """Callback function to add temperature data."""
    if print:
        print(json)
    temp = json.get("temperature1")
    temp_data.append(temp)


def handle_commands(peltier):
    while True:
        cmd = input("Enter command (or 'exit' to quit): ").strip()
        if not cmd:
            continue
        parts = cmd.split()
        cmd = parts[0].lower()
        if cmd in ("quit", "exit", "q"):
            break

        elif cmd == "temp" and len(parts) >= 2:
            temp = float(parts[1])
            ch = 1  # XXX can update to allow channel selection
            if peltier.set_temperature(temp, ch):
                print(f"Temperature set to {temp} °C on channel {ch}.")

        elif cmd == "hyst" and len(parts) >= 2:
            hyst = float(parts[1])
            ch = 1
            if peltier.set_hysteresis(hyst, ch):
                print(f"Hysteresis set to {hyst} °C on channel {ch}.")

        elif cmd == "enable":
            ch = 1
            if peltier.enable_channel(ch):
                print(f"Channel {ch} enabled.")

        elif cmd == "disable":
            ch = 1
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
with t:
    print("Recording temperature data. Press Ctrl+C to stop.")
    try:
        handle_commands()
    except KeyboardInterrupt:
        print("Recording stopped.")

np.save("temp_data.npy", temp_data)
