---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 8, 2026 at 11:38
---

# ADR-006: ROS 2 Package & Node Topology

- **Status**: Accepted
- **Date**: 2026-11-08

## Context
The ROS 2 codebase must be easy to build, debug, and launch inside the course's standard Docker (`ros2_course` / ROS 2 Jazzy) + Host Webots R2025a architecture, while maintaining clear separation between the three CPS layers.

## Decision
We will organize the implementation as a **Single ROS 2 Python Package (`roadside_rover_pkg`)** with modular node scripts launched via a single `rover.launch.py`:
- `rover_driver.py`: `webots_ros2_driver` plugin bridging the Koala 6WD base, 5-DOF arm, gripper, LiDAR, Camera `Recognition`, and IMU.
- `perception_node.py`: Layer 1 — Complementary Filter (IMU + 6-wheel encoders), LiDAR sector processor, and trash target tracker.
- `behavior_tree_node.py`: Layer 2 & Base Control — `py_trees` reactive/deliberative tree + Terrain-Adaptive Base PID controller.
- `arm_fsm_node.py`: Layer 3 Manipulation — 5-state Pick-and-Place Sub-FSM controlling `arm1`–`arm5` and gripper fingers.

Telemetry will be logged directly to console/log output during execution, with an optional standalone plotter script only if visual PID tuning is needed later.

## Consequences
- **Pros**:
  - Matches the exact `colcon build --symlink-install` workflow used in Assignments 1–4.
  - Each CPS architectural layer maps cleanly to its own Python file for easy explanation in the final presentation and video report.
- **Cons**:
  - All nodes share a single `package.xml` and `setup.py`.

## Sources
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- Course Assignments 1–4 Docker & Launch Setups
