---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 8, 2026 at 11:38
---

# ADR-005: Terrain-Adaptive Control & Grasping Reliability

- **Status**: Accepted
- **Date**: 2026-11-08

## Context
Layer 3 (Act & Control) requires closed-loop control for both the 6WD mobile base and the 5-DOF manipulator arm. Driving at full speed over rough slopes risks tipping or overshooting the target pick position, and physical finger grasping in Webots can occasionally slip when the robot is pitched on uneven ground.

## Decision
1. **Terrain-Adaptive Base PID Controller (`base_controller_node.py`)**:
   - Uses dual PID loops on target bearing error ($e_\theta$) and standoff distance error ($e_d = d - d_{\text{pick}}$) to smoothly align and park the rover directly in front of detected trash.
   - Dynamically scales maximum linear velocity inversely with IMU tilt magnitude ($\gamma = \sqrt{\text{roll}^2 + \text{pitch}^2}$), slowing the rover down on slopes and bumps for stability.
2. **5-DOF Arm Control**:
   - Uses physical parallel-gripper contact (`finger::left`, `finger::right`) around the `Box` trash item.
   - A Supervisor-based grasp-assist (weld/attach) fallback is **not** part of the baseline; it can be added later only if ODE contact physics slip proves to be a problem.

## Consequences
- **Pros**:
  - Demonstrates genuine cyber-physical feedback between IMU terrain sensing and base actuation speed.
  - Grasping is fully physical, so the arm kinematics and contact results are honest.
- **Cons**:
  - Requires tuning Base PID gains and standoff distance ($d_{\text{pick}}$) to match the arm's front reach envelope on the Koala chassis.
  - Grasps may slip on slopes; a grasp-assist fallback would be needed if so.

## Sources
- Course Assignment 4 (`task2_pid_joint.py`, `task3_pick_place.py`)
- [Webots Supervisor Node Documentation](https://cyberbotics.com/doc/reference/supervisor)
