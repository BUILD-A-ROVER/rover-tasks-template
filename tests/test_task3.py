"""
Test Suite for Task 3: Visual Tracking & Centroid Control.
Verifies line centroid extraction and proportional steering calculations.
"""

import pytest
from api.mock_rover import MockRover
from tasks.task3_line_or_tag_follow.solution import track_line_centroid, compute_steering_control


def test_compute_steering_control_centered():
    v, w = compute_steering_control(cx=320, frame_width=640, kp=0.004)
    assert v > 0.0
    assert pytest.approx(w, 0.001) == 0.0


def test_compute_steering_control_off_center_left():
    # Line is to the left (cx = 200 vs center = 320 -> error = +120)
    v, w = compute_steering_control(cx=200, frame_width=640, kp=0.004)
    assert v > 0.0
    assert w > 0.0  # Steer left to center target


def test_compute_steering_control_off_center_right():
    # Line is to the right (cx = 440 vs center = 320 -> error = -120)
    v, w = compute_steering_control(cx=440, frame_width=640, kp=0.004)
    assert v > 0.0
    assert w < 0.0  # Steer right to center target


def test_track_line_centroid_with_mock_frame():
    mock = MockRover(line_offset_px=50)  # Line centered at 320 + 50 = 370
    frame = mock.get_camera_frame()

    if frame is not None:
        cx, cy = track_line_centroid(frame)
        assert cx is not None
        # Verify detected centroid is close to expected offset
        assert abs(cx - 370) < 20
