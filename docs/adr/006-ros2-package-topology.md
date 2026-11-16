---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 10, 2026 at 18:57
---

# ADR-006: ROS 2 Package & Node Topology

- **Status**: Accepted
- **Date**: 2026-11-08

## Context
The ROS 2 codebase must be easy to build, debug, and launch inside the course's standard Docker (`ros2_course` / ROS 2 Jazzy) + Host Webots R2025a architecture, while maintaining clear separation between the three CPS layers.

## Decision
We will organize the implementation as a **Single ROS 2 Python Package (`isoroba_cleaner_pkg`)** with modular node scripts launched via a single `cleaner.launch.py`:
- `youbot_driver.py`: `webots_ros2_driver` plugin bridging the KUKA YouBot 4-wheel Mecanum base, 5-DOF arm, parallel gripper, dual LiDARs, Camera `Recognition`, and IMU/InertialUnit.
- `perception_node.py`: Layer 1 — 2D LiDAR sector processor and Camera trash target tracker.
- `behavior_tree_node.py`: Layer 2 - `py_trees` reactive/deliberative tree; selects the base mode (`/youbot/base_mode`) and triggers/aborts the arm FSM.
- `base_controller_node.py`: Layer 3 Base Control - Holonomic Base PID; the **sole `/cmd_vel` publisher** (acts as the velocity arbiter based on the BT-selected mode).
- `arm_fsm_node.py`: Layer 3 Manipulation — 5-state Pick-and-Place Sub-FSM controlling `arm1`–`arm5` and gripper fingers.

Telemetry will be logged directly to console/log output during execution, with an optional standalone plotter script only if visual PID tuning is needed later.

Custom driver code is required (not provided by `webots_ros2_driver`): 4-wheel Mecanum drive (`wheel1`–`wheel4`) and wheel odometry, conversion of camera `Recognition` objects to `/camera/recognized_trash`, and `/arm/joint_N/cmd` + `/arm/gripper/cmd` handling.

## Consequences
- **Pros**:
  - Matches the exact `colcon build --symlink-install` workflow used in Assignments 1–4.
  - Each CPS architectural layer maps cleanly to its own Python file for easy explanation in the final presentation and video report.
- **Cons**:
  - All nodes share a single `package.xml` and `setup.py`.

## Sources
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- Course Assignments 1–4 Docker & Launch Setups
