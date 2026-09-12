"""
Test Suite for Task 1: Keyboard Teleoperation & Speed Ramping.
Verifies key-to-velocity mapping, input clamping, and continuous speed ramps.
"""

import pytest
from tasks.task1_teleop.solution import process_key_input, apply_speed_ramp


def test_process_key_input_forward():
    keys = {"w": True}
    v, w = process_key_input(keys)
    assert v > 0.0
    assert w == 0.0


def test_process_key_input_turn_left():
    keys = {"w": True, "a": True}
    v, w = process_key_input(keys)
    assert v > 0.0
    assert w > 0.0  # Left rotation is positive angular velocity


def test_process_key_input_emergency_stop():
    keys = {"w": True, "space": True}
    v, w = process_key_input(keys)
    assert v == 0.0
    assert w == 0.0


def test_speed_ramp_up():
    current = 0.0
    target = 0.5
    max_accel = 1.0  # m/s^2
    dt = 0.1         # sec -> max change = 0.1

    ramped = apply_speed_ramp(current, target, max_accel, dt)
    assert pytest.approx(ramped, 0.001) == 0.1

    ramped2 = apply_speed_ramp(ramped, target, max_accel, dt)
    assert pytest.approx(ramped2, 0.001) == 0.2


def test_speed_ramp_down():
    current = 0.5
    target = 0.0
    max_accel = 1.0  # m/s^2
    dt = 0.1         # max change = 0.1

    ramped = apply_speed_ramp(current, target, max_accel, dt)
    assert pytest.approx(ramped, 0.001) == 0.4
