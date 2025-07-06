#!/usr/bin/env python3
import sys
import json
from serial import Serial
import threading


class RFSwitchSerial(Serial):
    """ """

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
    ms = RFSwitchSerial(sys.argv[-1])
    ms.start()
    payload = {"sw_state": 1}
    try:
        while True:
            for i in range(8):
                sw_state = input("Enter sw_state:")
                payload["sw_state"] = int(sw_state)
                ms.command(payload)
            # payload['sw_state'] = 255
            # ms.command(payload)
            # time.sleep(2)
            # payload['sw_state'] = 0
            # ms.command(payload)
            # time.sleep(2)
    finally:
        ms.stop()


if __name__ == "__main__":
    main()
