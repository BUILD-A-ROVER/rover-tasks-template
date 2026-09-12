# Task 2: LiDAR Obstacle Avoidance & Reactive Steering

## Workshop Objective
Develop an autonomous reactive obstacle avoidance algorithm using 360-degree LiDAR range data.
When an obstacle enters the safety threshold ($d < 0.4\text{ m}$), the rover must automatically reduce linear forward speed and steer towards clear open space.

---

## Instructions
1. Edit `tasks/task2_obstacle_avoidance/solution.py`.
2. Implement `find_closest_obstacle(scan)`:
   - Evaluates a 360-float scan array (in meters).
   - Filters out invalid sensor values ($d \le 0$ or infinite).
   - Returns tuple `(min_distance, angle_deg)`.
3. Implement `compute_avoidance_vector(scan)`:
   - Examines front arc ($\pm 45^\circ$).
   - If `min_distance < 0.4 m`, calculates corrective angular steering command away from the obstacle.

---

## Verification
Run automated unit tests:
```bash
pytest tests/test_task2.py
```
