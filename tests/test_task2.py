"""
Test Suite for Task 2: LiDAR Obstacle Avoidance.
Verifies closest obstacle detection and reactive avoidance vector calculations.
"""

import pytest
from tasks.task2_obstacle_avoidance.solution import find_closest_obstacle, compute_avoidance_vector


def test_find_closest_obstacle():
    scan = [10.0] * 360
    scan[45] = 0.35  # Inject obstacle at 45 degrees, 0.35m

    min_dist, angle = find_closest_obstacle(scan)
    assert pytest.approx(min_dist, 0.001) == 0.35
    assert angle == 45


def test_compute_avoidance_vector_clear_path():
    scan = [10.0] * 360
    v, w = compute_avoidance_vector(scan, safety_threshold_m=0.4)
    assert v >= 0.3
    assert w == 0.0


def test_compute_avoidance_vector_obstacle_ahead():
    scan = [10.0] * 360
    # Inject front obstacle at 0 degrees (0.25m distance)
    scan[0] = 0.25
    scan[1] = 0.25
    scan[359] = 0.25

    # Inject right side wall to force left turn
    scan[45] = 0.3

    v, w = compute_avoidance_vector(scan, safety_threshold_m=0.4)

    # Linear speed should slow down dramatically
    assert v < 0.3
    # Angular velocity should trigger steering maneuver away from obstacle
    assert w != 0.0
