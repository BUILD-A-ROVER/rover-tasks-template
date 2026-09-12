# Task 3: Visual Tracking (Line / AprilTag Centroid Control)

## Workshop Objective
Implement an OpenCV-based visual controller that tracks a ground line or marker, computes its horizontal centroid offset from image center (320px), and applies proportional steering control to center the target in the camera frame.

---

## Instructions
1. Edit `tasks/task3_line_or_tag_follow/solution.py`.
2. Implement `track_line_centroid(frame)`:
   - Convert frame to grayscale / binary threshold.
   - Extract region of interest (ROI) at bottom of image.
   - Compute image moments (`cv2.moments`) to locate centroid $(C_x, C_y)$.
3. Implement `compute_steering_control(cx, image_width=640, kp=0.003)`:
   - Compute error: $e = \text{center\_x} - C_x$.
   - Calculate angular velocity: $\omega = K_p \cdot e$.

---

## Verification
Run unit tests:
```bash
pytest tests/test_task3.py
```
