"""
Task 2 Solution: LiDAR Obstacle Avoidance.
Processes 360-degree range scans to detect obstacles within 0.4m safety threshold
and computes reactive avoidance velocity vectors.
"""

import time
import math
import logging
from typing import List, Tuple

from api.rover_client import RoverClient

logger = logging.getLogger("task2_obstacle_avoidance")


def find_closest_obstacle(scan: List[float]) -> Tuple[float, int]:
    """
    Locate the minimum distance measurement and its corresponding angle index in degrees.

    :param scan: 360-element range array in meters.
    :return: Tuple of (min_distance_m, angle_degrees).
    """
    min_dist = float("inf")
    min_angle = 0

    for i in range(len(scan)):
        d = scan[i]
        # Ignore zero, negative, or invalid infinite readings
        if 0.05 < d < min_dist:
            min_dist = d
            min_angle = i

    return min_dist, min_angle


def compute_avoidance_vector(scan: List[float], safety_threshold_m: float = 0.4) -> Tuple[float, float]:
    """
    Calculate (linear_v, angular_w) drive command based on front sector LiDAR scan.

    :param scan: 360-element range array in meters.
    :param safety_threshold_m: Distance threshold below which avoidance triggers (0.4m).
    :return: Tuple of (linear_velocity, angular_velocity).
    """
    # Evaluate front sector: 315 deg to 359 deg (-45 to 0) and 0 deg to 45 deg (+45)
    front_indices = list(range(315, 360)) + list(range(0, 46))
    front_distances = [scan[i] for i in front_indices if 0.05 < scan[i] < 10.0]

    if not front_distances:
        min_front_dist = 10.0
    else:
        min_front_dist = min(front_distances)

    # Evaluate Left (270 deg to 315 deg) vs Right (45 deg to 90 deg) sectors for steering direction
    left_sector = [scan[i] for i in range(270, 315) if 0.05 < scan[i] < 10.0]
    right_sector = [scan[i] for i in range(45, 90) if 0.05 < scan[i] < 10.0]

    avg_left = sum(left_sector) / len(left_sector) if left_sector else 10.0
    avg_right = sum(right_sector) / len(right_sector) if right_sector else 10.0

    target_v = 0.4  # Default cruising speed (m/s)
    target_w = 0.0  # Straight ahead

    # Trigger avoidance if an obstacle is detected within safety threshold
    if min_front_dist < safety_threshold_m:
        # Scale linear speed down proportionally to proximity
        speed_factor = max(0.0, (min_front_dist - 0.1) / (safety_threshold_m - 0.1))
        target_v = 0.3 * speed_factor

        # Steer toward the sector with greater clearance
        if avg_left > avg_right:
            target_w = 0.8  # Steer left (positive angular w)
        else:
            target_w = -0.8  # Steer right (negative angular w)
    elif min_front_dist < 0.8:
        # Warning zone: reduce speed slightly
        target_v = 0.35

    return target_v, target_w


def run_avoidance_loop(client: RoverClient, max_runtime_sec: float = 10.0) -> None:
    """
    Run active obstacle avoidance loop.

    :param client: RoverClient instance.
    :param max_runtime_sec: Runtime duration for demo loop.
    """
    print("Starting Autonomous Obstacle Avoidance Loop...")
    start_time = time.time()
    try:
        while time.time() - start_time < max_runtime_sec:
            scan = client.get_lidar_scan()
            min_dist, min_angle = find_closest_obstacle(scan)
            v, w = compute_avoidance_vector(scan, safety_threshold_m=0.4)

            logger.info(f"Closest Obstacle: {min_dist:.2f}m @ {min_angle}° -> Drive Cmd (v={v:.2f}, w={w:.2f})")
            client.drive(v, w)
            time.sleep(0.05)  # 20 Hz
    finally:
        client.stop()
        print("Obstacle avoidance loop finished.")


if __name__ == "__main__":
    client = RoverClient(mock=True)
    run_avoidance_loop(client, max_runtime_sec=5.0)
