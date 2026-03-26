"""
Manual potentiometer calibration script for lab use.

Connects to a potentiometer Pico, prompts the user to set min and max
positions, collects voltage samples, computes a linear fit, and saves
calibration to a JSON file that PicoPotentiometer.load_calibration() can read.

Two modes:
  --mode minmax   : collect only at min and max (2-point fit, default)
  --mode turns    : collect at every full turn from min to max (least-squares)

Usage:
    python calibrate_pot.py -p /dev/ttyACM0 -o pot_calibration.json
    python calibrate_pot.py -p /dev/ttyACM0 --mode turns
"""

from argparse import ArgumentParser
import json
import time
from datetime import datetime, timezone

import numpy as np

from picohost import PicoPotentiometer


def collect_samples(pot, n, interval=0.25):
    """Collect *n* voltage samples and return averages for both pots."""
    samples_0, samples_1 = [], []
    for _ in range(n):
        v = pot.read_voltage()
        samples_0.append(v["pot0_voltage"])
        samples_1.append(v["pot1_voltage"])
        time.sleep(interval)
    return np.mean(samples_0), np.mean(samples_1)


def compute_linear_fit(voltages, angles):
    """Compute (m, b) such that angle = m * voltage + b via least-squares.

    Returns None if the voltage range is too small to calibrate.
    """
    voltages = np.asarray(voltages)
    if np.abs(voltages[-1] - voltages[0]) < 1e-6:
        print(
            f"  ERROR: min and max voltages are identical "
            f"({voltages[0]:.4f} V). Cannot calibrate."
        )
        return None
    m, b = np.polyfit(voltages, angles, 1)
    return (float(m), float(b))


def collect_minmax(pot, n_samples, total_degrees):
    """Collect at min and max only (2-point calibration)."""
    input(
        "\nSet both potentiometers to MINIMUM position, "
        "then press Enter."
    )
    v_min_0, v_min_1 = collect_samples(pot, n_samples)
    print(f"  pot0 min voltage: {v_min_0:.4f} V")
    print(f"  pot1 min voltage: {v_min_1:.4f} V")

    input(
        "\nSet both potentiometers to MAXIMUM position, "
        "then press Enter."
    )
    v_max_0, v_max_1 = collect_samples(pot, n_samples)
    print(f"  pot0 max voltage: {v_max_0:.4f} V")
    print(f"  pot1 max voltage: {v_max_1:.4f} V")

    voltages_0 = [v_min_0, v_max_0]
    voltages_1 = [v_min_1, v_max_1]
    angles = [0.0, total_degrees]
    return voltages_0, voltages_1, angles


def collect_per_turn(pot, n_samples, turns):
    """Collect at every full turn from min to max."""
    input(
        "\nSet both potentiometers to MINIMUM position (turn 0), "
        "then press Enter."
    )
    v0, v1 = collect_samples(pot, n_samples)
    print(f"  turn  0: pot0={v0:.4f} V, pot1={v1:.4f} V")
    voltages_0 = [v0]
    voltages_1 = [v1]
    angles = [0.0]

    for turn in range(1, turns + 1):
        input(f"\nAdvance both pots 1 full turn (to turn {turn}), "
              "then press Enter.")
        v0, v1 = collect_samples(pot, n_samples)
        print(f"  turn {turn:2d}: pot0={v0:.4f} V, pot1={v1:.4f} V")
        voltages_0.append(v0)
        voltages_1.append(v1)
        angles.append(turn * 360.0)

    return voltages_0, voltages_1, angles


def main():
    parser = ArgumentParser(
        description="Calibrate potentiometer voltage-to-angle mapping."
    )
    parser.add_argument(
        "-p", "--port", type=str, required=True,
        help="Serial port for the potentiometer Pico",
    )
    parser.add_argument(
        "-o", "--output", type=str, default="pot_calibration.json",
        help="Output calibration file (default: pot_calibration.json)",
    )
    parser.add_argument(
        "-t", "--turns", type=int, default=10,
        help="Number of full turns from min to max (default: 10)",
    )
    parser.add_argument(
        "-n", "--n-samples", type=int, default=10,
        help="Number of voltage samples to average at each position (default: 10)",
    )
    parser.add_argument(
        "-m", "--mode", type=str, choices=["minmax", "turns"],
        default="minmax",
        help="Calibration mode: 'minmax' for 2-point, 'turns' for per-turn "
             "least-squares (default: minmax)",
    )
    args = parser.parse_args()

    total_degrees = 360.0 * args.turns

    print(f"Connecting to {args.port}...")
    print(f"Mode: {args.mode} ({args.turns} turns, {args.n_samples} "
          f"samples/position)")

    with PicoPotentiometer(args.port) as pot:
        if args.mode == "minmax":
            voltages_0, voltages_1, angles = collect_minmax(
                pot, args.n_samples, total_degrees,
            )
        else:
            voltages_0, voltages_1, angles = collect_per_turn(
                pot, args.n_samples, args.turns,
            )

    cal0 = compute_linear_fit(voltages_0, angles)
    cal1 = compute_linear_fit(voltages_1, angles)

    if cal0 is None or cal1 is None:
        print("\nCalibration failed. Exiting.")
        raise SystemExit(1)

    cal_data = {
        "pot0": list(cal0),
        "pot1": list(cal1),
        "metadata": {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "port": args.port,
            "turns": args.turns,
            "total_degrees": total_degrees,
            "mode": args.mode,
            "n_points": len(angles),
            "pot0_voltages": [float(v) for v in voltages_0],
            "pot1_voltages": [float(v) for v in voltages_1],
            "angles": [float(a) for a in angles],
            "n_samples": args.n_samples,
        },
    }

    with open(args.output, "w") as f:
        json.dump(cal_data, f, indent=2)
    print(f"\nCalibration saved to {args.output}")
    print(f"  pot0: angle = {cal0[0]:.4f} * V + {cal0[1]:.4f}")
    print(f"  pot1: angle = {cal1[0]:.4f} * V + {cal1[1]:.4f}")
    if args.mode == "turns":
        print(f"  ({len(angles)} points used for least-squares fit)")


if __name__ == "__main__":
    main()
