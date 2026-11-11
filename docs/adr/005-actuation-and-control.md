---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:38
last_updated: November 11, 2026 at 16:15
---

# ADR-005: Holonomic Base PID & Manipulation Control

- **Status**: Accepted (revised 2026-11-11: updated for YouBot Mecanum holonomic control)
- **Date**: 2026-11-08

## Context
Layer 3 (Act & Control) requires closed-loop control for both the mobile base and the 5-DOF manipulator arm. On a pedestrian street, the robot must align cleanly with small litter items on the ground without awkward multi-point skid-steer maneuvers, and the arm must reliably grasp objects and swing them backward into the onboard bin.

## Decision
1. **Holonomic Base PID Controller (`base_controller_node.py`)**:
   - Takes full advantage of the YouBot's 4-wheel Mecanum drive.
   - Controls 3 degrees of freedom independently: forward/backward velocity $v_x$, lateral strafe velocity $v_y$, and angular yaw velocity $\omega_z$.
   - Simultaneously minimizes lateral alignment error ($e_y$) and standoff distance error ($e_x = d - d_{\text{pick}}$) to bring the target item directly into the 5-DOF arm's frontal pick zone.
2. **5-DOF Arm Trajectory Control**:
   - Positions `arm1` through `arm5` through pre-calculated, reachable joint angles on the YouBot chassis.
   - Physical contact grasping using parallel gripper fingers on flat pavement.

## Consequences
- **Pros**:
  - Mecanum strafing allows lateral adjustment without changing heading, resulting in significantly faster and more accurate parking than differential/skid-steer bases.
  - Arm joint trajectories follow course Assignment 4 conventions (`arm1`–`arm5`), simplifying tuning and validation.
- **Cons**:
  - Forward reach is limited to ~0.35 m from the front bumper, requiring precise base stopping tolerance ($\pm 2\text{ cm}$).

## Sources
- Course Assignment 4 (`task2_pid_joint.py`, `task3_pick_place.py`, `youbot_arm_driver.py`)
