"""
Task 3 Solution: OpenCV Line / AprilTag Visual Centroid Tracking.
Computes target centroid (Cx) using OpenCV image moments and applies proportional (Kp) steering control.
"""

import time
import logging
from typing import Tuple, Optional, Any

from api.rover_client import RoverClient

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    OPENCV_AVAILABLE = False

logger = logging.getLogger("task3_line_follow")


def track_line_centroid(frame: Any) -> Tuple[Optional[int], Optional[int]]:
    """
    Extract the horizontal (Cx) and vertical (Cy) centroid of a tracked black line in frame.

    :param frame: BGR OpenCV image array (640x480).
    :return: Tuple of (cx, cy) or (None, None) if line is not detected.
    """
    if not OPENCV_AVAILABLE or frame is None:
        return None, None

    height, width = frame.shape[:2]
    # Crop Region of Interest (ROI) to lower 30% of the camera view
    roi_top = int(height * 0.7)
    roi = frame[roi_top:height, 0:width]

    # Convert ROI to grayscale
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur and binary inverse threshold to isolate dark line
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

    # Compute spatial image moments
    moments = cv2.moments(thresh)

    if moments["m00"] > 1000:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"]) + roi_top
        return cx, cy

    return None, None


def compute_steering_control(cx: Optional[int], frame_width: int = 640, kp: float = 0.004) -> Tuple[float, float]:
    """
    Compute linear (v) and angular (w) drive velocity given tracked line centroid Cx.

    :param cx: Detected centroid X coordinate in pixels.
    :param frame_width: Width of image in pixels (default: 640).
    :param kp: Proportional gain constant.
    :return: Tuple of (linear_v, angular_w).
    """
    if cx is None:
        # Search maneuver when line is lost: slow creep forward with gentle search turn
        return 0.1, 0.3

    center_x = frame_width // 2
    error_px = center_x - cx

    # Proportional angular velocity control:
    # If line is to the left (cx < center_x), error > 0 -> steer left (w > 0)
    angular_w = kp * error_px
    angular_w = max(-1.0, min(1.0, angular_w))

    # Reduce linear speed when error is large to prevent overshooting curves
    linear_v = 0.35 * (1.0 - min(1.0, abs(error_px) / (frame_width / 2)))
    linear_v = max(0.1, linear_v)

    return linear_v, angular_w


def run_line_follow_loop(client: RoverClient, max_runtime_sec: float = 10.0) -> None:
    """
    Execute main visual tracking control loop.

    :param client: RoverClient instance.
    :param max_runtime_sec: Maximum runtime duration in seconds.
    """
    print("Starting Visual Line / Marker Following Control Loop...")
    start_time = time.time()
    try:
        while time.time() - start_time < max_runtime_sec:
            frame = client.get_camera_frame()
            cx, cy = track_line_centroid(frame)
            v, w = compute_steering_control(cx, frame_width=640, kp=0.004)

            logger.info(f"Visual Tracker -> Cx: {cx} | Target Velocities: (v={v:.2f}, w={w:.2f})")
            client.drive(v, w)
            time.sleep(0.05)  # 20 Hz loop
    finally:
        client.stop()
        print("Visual tracking loop finished.")


if __name__ == "__main__":
    client = RoverClient(mock=True)
    run_line_follow_loop(client, max_runtime_sec=5.0)
