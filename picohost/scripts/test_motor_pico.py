import time
import sys
import json
from serial import Serial
import threading


class MotorSerial(Serial):

    def __init__(self, port, baud=115200, timeout=1.0):
        Serial.__init__(self, port, baud, timeout=timeout)
        self._running = True
        self._process_status_thread = None

    def process_status(self):
        while self._running:
            line = self.readline().decode("utf-8", errors="ignore").strip()
            if len(line) == 0:
                continue
            status = json.loads(line)
            print(status)

    def command(self, val_dict):
        json_str = json.dumps(val_dict, separators=(",", ":")).encode("utf-8")
        self.write(json_str + b"\n")  # send raw bytes
        self.flush()

    def start(self):
        self._running = True
        self._process_status_thread = threading.Thread(
            target=self.process_status, daemon=True
        )
        self._process_status_thread.start()

    def stop(self):
        self._running = False
        if self._process_status_thread is not None:
            self._process_status_thread.join()


def main():
    """
    Open the serial port, read until a valid JSON line appears or timeout.
    """
    ms = MotorSerial(sys.argv[-1])
    input("GO?")
    ms.start()
    payload = {
        "pulses_az": 0,  # signed or unsigned int
        "pulses_el": 0,  # signed or unsigned int
        "delay_us_az": 600,  # microseconds
        "delay_us_el": 600,  # microseconds
    }
    try:
        while True:
            for val in (1620, 0, -1620):
                print(f"Sending {val} pulses")
                payload["pulses_el"] = val
                payload["pulses_az"] = val
                ms.command(payload)
                time.sleep(3)
    finally:
        ms.stop()


if __name__ == "__main__":
    main()
