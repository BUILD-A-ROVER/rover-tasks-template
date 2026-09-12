# BUILD-A-ROVER Student Starter Template

Welcome to the hands-on **BUILD-A-ROVER Workshop**! This repository is your starter workspace containing mock sensor simulation environments, task boilerplates, and automated pytest harnesses.

---

## Directory Structure

```
rover-tasks-template/
├── .devcontainer/
│   └── devcontainer.json           # VS Code Dev Container configuration
├── .github/workflows/
│   └── test_submission.yml         # Automated CI/CD test workflow
├── api/
│   ├── rover_client.py             # Main RoverClient API (supports hardware & mock)
│   └── mock_rover.py               # Simulated 360 LiDAR & camera sensor environment
├── tasks/
│   ├── task1_teleop/
│   │   ├── README.md               # Task 1 instructions
│   │   └── solution.py             # Keyboard teleoperation & acceleration ramping
│   ├── task2_obstacle_avoidance/
│   │   ├── README.md               # Task 2 instructions
│   │   └── solution.py             # LiDAR scan processing & reactive steering
│   └── task3_line_or_tag_follow/
│       ├── README.md               # Task 3 instructions
│       └── solution.py             # OpenCV centroid extraction & proportional control
├── tests/
│   ├── conftest.py                 # Pytest fixture definitions
│   ├── test_task1.py               # Teleoperation unit tests
│   ├── test_task2.py               # Obstacle avoidance unit tests
│   └── test_task3.py               # Visual line tracking unit tests
├── requirements.txt                # Python package requirements
└── README.md                       # Workshop overview & getting started guide
```

---

## Getting Started

### Option A: VS Code Dev Containers (Recommended)
1. Open this repository in VS Code.
2. Click **Reopen in Container** when prompted (or run `Dev Containers: Reopen in Container` from Command Palette).
3. All dependencies (Python 3.10, OpenCV, Pytest) will install automatically.

### Option B: Local Environment
```bash
pip install -r requirements.txt
```

---

## Workshop Tasks Overview

### Task 1: Keyboard Teleoperation & Speed Ramping
- File: [`tasks/task1_teleop/solution.py`](tasks/task1_teleop/solution.py)
- Objective: Convert keyboard commands ($W/A/S/D$) into normalized velocity requests and apply smooth acceleration filters.

### Task 2: LiDAR Reactive Obstacle Avoidance
- File: [`tasks/task2_obstacle_avoidance/solution.py`](tasks/task2_obstacle_avoidance/solution.py)
- Objective: Read 360-degree LiDAR range vectors, detect obstacles under $0.4\text{ m}$, and compute steering avoidance vectors.

### Task 3: Visual Tracking (Line / AprilTag Follower)
- File: [`tasks/task3_line_or_tag_follow/solution.py`](tasks/task3_line_or_tag_follow/solution.py)
- Objective: Extract target centroid ($C_x$) from OpenCV camera frames using spatial moments and apply proportional steering control.

---

## Running Verification Tests

Run all unit tests locally or inside the dev container:

```bash
pytest tests/ -v
```
