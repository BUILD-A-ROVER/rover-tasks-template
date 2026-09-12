# Task 1: Keyboard Teleoperation & Speed Ramping

## Workshop Objective
Implement real-time keyboard control ($W/A/S/D$) for the rover. Your control loop must normalize user keypresses into valid linear ($v$) and angular ($\omega$) velocity commands, while applying a smooth speed ramping filter to avoid mechanical jerk and wheel slippage.

---

## Instructions
1. Edit `tasks/task1_teleop/solution.py`.
2. Implement key mapping logic:
   - `W`: Move Forward ($+v$)
   - `S`: Move Reverse ($-v$)
   - `A`: Turn Left ($+\omega$)
   - `D`: Turn Right ($-\omega$)
   - `SPACE` / `X`: Emergency Stop ($v=0, \omega=0$)
3. Implement `apply_speed_ramp(current_v, target_v, max_acceleration, dt)` to smooth out rapid velocity transitions.

---

## Verification
Run tests to verify your implementation:
```bash
pytest tests/test_task1.py
```
