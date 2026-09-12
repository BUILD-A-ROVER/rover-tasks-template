"""
Mock Rover Simulator for Offline Development and Pytest Suite.
Simulates synthetic LiDAR 360 scans with configurable walls and obstacles.
Simulates OpenCV camera frames with visual tracking targets (black line / AprilTag marker).
"""

import math
import time
import logging
from typing import List, Tuple, Dict, Any, Optional

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    OPENCV_AVAILABLE = False

logger = logging.getLogger("mock_rover")


class MockRover:
    """
    In-memory virtual rover simulating sensors and kinematics.
    """

    def __init__(self, line_offset_px: int = 0):
        """
        Initialize Mock Rover.

        :param line_offset_px: Simulated line centroid offset from center pixel (320).
        """
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0
        self.line_offset_px = line_offset_px
        self.obstacles: List[Dict[str, Any]] = []

        # Default synthetic environment: Obstacle ahead at 0.35m (0 deg), open right side
        self.add_obstacle(angle_deg=0.0, distance_m=0.35, width_deg=20.0)
        self.add_obstacle(angle_deg=90.0, distance_m=2.5, width_deg=15.0)

    def add_obstacle(self, angle_deg: float, distance_m: float, width_deg: float = 10.0) -> None:
        """Add a synthetic virtual obstacle to the simulated environment."""
        self.obstacles.append({
            "angle": float(angle_deg) % 360.0,
            "distance": float(distance_m),
            "width": float(width_deg),
        })

    def clear_obstacles(self) -> None:
        """Clear all virtual obstacles."""
        self.obstacles.clear()

    def set_drive_cmd(self, linear: float, angular: float) -> None:
        """Receive drive command and update state."""
        self.last_linear_cmd = float(linear)
        self.last_angular_cmd = float(angular)

    def get_lidar_scan(self) -> List[float]:
        """
        Generate a 360-element range array (meters).

        :return: List of 360 float range values.
        """
        # Default open space distance
        scan = [10.0] * 360

        for obs in self.obstacles:
            center_angle = obs["angle"]
            dist = obs["distance"]
            half_w = obs["width"] / 2.0

            for i in range(360):
                angle_diff = min(abs(i - center_angle), 360.0 - abs(i - center_angle))
                if angle_diff <= half_w:
                    # Synthetic parabolic range response across width
                    scan[i] = min(scan[i], dist + (angle_diff / half_w) * 0.05)

        return scan

    def get_camera_frame(self) -> Optional[Any]:
        """
        Generate a synthetic BGR image frame (640x480) with a simulated track line.

        :return: OpenCV BGR image numpy array.
        """
        if not OPENCV_AVAILABLE or np is None:
            return None

        width, height = 640, 480
        # White background canvas
        img = np.full((height, width, 3), 255, dtype=np.uint8)

        # Draw black track line centered at 320 + line_offset_px
        center_x = 320 + self.line_offset_px
        line_thickness = 40

        cv2.rectangle(
            img,
            (center_x - line_thickness // 2, 0),
            (center_x + line_thickness // 2, height),
            (0, 0, 0),
            -1
        )

        # Draw AprilTag simulated marker in top right
        cv2.rectangle(img, (500, 40), (580, 120), (0, 0, 0), -1)
        cv2.rectangle(img, (520, 60), (560, 100), (255, 255, 255), -1)
        cv2.circle(img, (540, 80), 10, (0, 0, 0), -1)

        # Add simulated text overlay
        cv2.putText(img, "MOCK CAMERA FRAME", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)

        return img
