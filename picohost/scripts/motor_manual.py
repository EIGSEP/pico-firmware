"""
Interactive motor zeroing script.

Use arrow keys (u/d/l/r) to jog the motors into the desired home
position, then press Enter to zero the step counters.  After zeroing,
scan() will treat the current physical position as (0, 0).

Controls:
    u / d  -  jog elevation up / down
    l / r  -  jog azimuth left / right
    + / -  -  increase / decrease jog step size
    Enter  -  zero step counters and exit
    q      -  quit without zeroing
"""

import json
import curses
import logging

from picohost import PicoMotor

logger = logging.getLogger(__name__)


def main(screen):
    curses.noecho()
    screen.nodelay(False)
    import argparse

    parser = argparse.ArgumentParser(
        description="Jog motors to home position and zero step counters"
    )
    parser.add_argument(
        "-c",
        "--pico_config",
        type=str,
        default="pico_config.json",
        help="Output of flash_picos (pico_config.json)",
    )
    parser.add_argument(
        "--deg",
        type=float,
        default=1.0,
        help="Initial jog step size in degrees (default: 1.0)",
    )

    args = parser.parse_args()
    port = None
    with open(args.pico_config, "r", encoding="utf-8") as f:
        records = json.load(f)
        for config in records:
            if config["app_id"] == 0:
                port = config["port"]
                break
    assert port is not None, "didn't find app_id 0 in pico_config.json"

    c = PicoMotor(port, verbose=False)
    c.set_delay(
        az_up_delay_us=2400,
        az_dn_delay_us=300,
        el_up_delay_us=2400,
        el_dn_delay_us=600,
    )

    deg = args.deg
    zeroed = False

    def refresh_status():
        screen.clear()
        screen.addstr(0, 0, "=== Motor Zeroing ===")
        screen.addstr(2, 0, f"Jog step: {deg:.1f} deg")
        if c.is_connected:
            screen.addstr(3, 0, f"AZ pos: {c.last_status.get('az_pos', '?')}")
            screen.addstr(4, 0, f"EL pos: {c.last_status.get('el_pos', '?')}")
        else:
            screen.addstr(3, 0, "AZ pos: DISCONNECTED (waiting for reconnect)")
            screen.addstr(4, 0, "EL pos: ---")
        screen.addstr(6, 0, "u/d = jog EL | l/r = jog AZ")
        screen.addstr(7, 0, "+/- = change step size")
        screen.addstr(8, 0, "Enter = zero and exit | q = quit")
        screen.refresh()

    try:
        while True:
            refresh_status()
            ch = screen.getch()
            if ch == -1:
                continue
            if ch == ord("\n"):
                if not c.is_connected:
                    continue
                c.halt()
                c.reset_step_position(az_step=0, el_step=0)
                zeroed = True
                break
            if 0 <= ch < 256:
                key = chr(ch).lower()
                if key == "q":
                    break
                elif key == "+":
                    deg += 1
                elif key == "-":
                    deg = max(0.1, deg - 1)
                elif key in ("u", "d", "l", "r"):
                    if not c.is_connected:
                        continue
                    if key == "u":
                        c.el_move_deg(deg, wait_for_stop=True)
                    elif key == "d":
                        c.el_move_deg(-deg, wait_for_stop=True)
                    elif key == "l":
                        c.az_move_deg(deg, wait_for_stop=True)
                    elif key == "r":
                        c.az_move_deg(-deg, wait_for_stop=True)
    except KeyboardInterrupt:
        pass
    finally:
        c.halt()

    if zeroed:
        logger.info("Step counters zeroed. Motors are at home (0, 0).")
    else:
        logger.info("Exited without zeroing.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    curses.wrapper(main)
