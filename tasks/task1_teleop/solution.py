"""
Task 1 Solution: Keyboard Teleoperation with Speed Ramping Filter.
Maps keyboard inputs (W/A/S/D) to linear (v) and angular (w) velocities.
Applies continuous acceleration/deceleration ramping to ensure smooth motion.
"""

import sys
import time
import logging
from typing import Tuple, Dict

from api.rover_client import RoverClient

logger = logging.getLogger("task1_teleop")


def process_key_input(keys_pressed: Dict[str, bool]) -> Tuple[float, float]:
    """
    Map active keypresses to raw target linear and angular velocities.

    :param keys_pressed: Dict mapping key strings ('w', 'a', 's', 'd', 'space') to boolean status.
    :return: Tuple of (target_linear_v, target_angular_w).
    """
    target_v = 0.0
    target_w = 0.0

    max_v = 0.5   # m/s
    max_w = 1.0   # rad/s

    if keys_pressed.get("space") or keys_pressed.get("x"):
        return 0.0, 0.0

    if keys_pressed.get("w"):
        target_v += max_v
    if keys_pressed.get("s"):
        target_v -= max_v

    if keys_pressed.get("a"):
        target_w += max_w  # Left rotation positive angular velocity
    if keys_pressed.get("d"):
        target_w -= max_w  # Right rotation negative angular velocity

    return target_v, target_w


def apply_speed_ramp(current: float, target: float, max_accel: float, dt: float) -> float:
    """
    Apply linear acceleration limit to smooth velocity transitions.

    :param current: Current velocity magnitude.
    :param target: Target requested velocity magnitude.
    :param max_accel: Maximum allowed acceleration magnitude per second.
    :param dt: Elapsed time interval in seconds.
    :return: Ramped velocity.
    """
    max_change = max_accel * dt
    diff = target - current
    if abs(diff) <= max_change:
        return target
    elif diff > 0:
        return current + max_change
    else:
        return current - max_change


def run_teleop_loop(client: RoverClient, max_runtime_sec: float = 10.0) -> None:
    """
    Main teleop loop updating commands at 20 Hz.

    :param client: Instance of RoverClient.
    :param max_runtime_sec: Maximum runtime duration for automated demonstration.
    """
    curr_v = 0.0
    curr_w = 0.0
    max_accel_v = 1.0  # m/s^2
    max_accel_w = 2.0  # rad/s^2

    dt = 0.05  # 20 Hz loop (50 ms)
    start_time = time.time()

    # Simulated sequence of keypresses for demonstration / test execution
    key_schedule = [
        (0.0, {"w": True}),
        (2.0, {"w": True, "d": True}),
        (4.0, {"s": True}),
        (6.0, {"a": True}),
        (8.0, {"space": True}),
    ]

    print("Starting Teleoperation Control Loop...")
    try:
        while time.time() - start_time < max_runtime_sec:
            elapsed = time.time() - start_time
            # Determine active keys for current time step
            active_keys = {}
            for timestamp, keys in key_schedule:
                if elapsed >= timestamp:
                    active_keys = keys

            target_v, target_w = process_key_input(active_keys)

            # Apply acceleration ramps
            curr_v = apply_speed_ramp(curr_v, target_v, max_accel_v, dt)
            curr_w = apply_speed_ramp(curr_w, target_w, max_accel_w, dt)

            # Dispatch command to rover
            client.drive(curr_v, curr_w)
            time.sleep(dt)
    finally:
        client.stop()
        print("Teleoperation loop finished. Rover stopped safely.")


if __name__ == "__main__":
    client = RoverClient(mock=True)
    run_teleop_loop(client, max_runtime_sec=5.0)
