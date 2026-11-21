---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 21, 2026 at 15:05
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
   - Simultaneously minimizes longitudinal standoff distance error ($e_x = x - 0.45\text{ m}$), lateral alignment error ($e_y = y$), and corridor-aligned heading error ($e_\theta$ relative to $0$ or $\pi\text{ rad}$ via IMU-fused `/wheel_odom_raw`) to bring rectangular litter directly into the 5-DOF arm's frontal pick zone.
2. **5-DOF Arm Trajectory Control (`arm_fsm_node.py`)**:
   - Positions `arm1` through `arm5` through calibrated joint waypoints on the YouBot chassis (`HOME`, `PRE_GRASP`, `GRASP`/`GRASP_TALL`, `LIFT`, `DROP_IN_BIN`).
   - Physical contact grasping using parallel gripper fingers on flat pavement with closed-loop `/joint_states` verification.

## Consequences
- **Pros**:
  - Mecanum strafing allows lateral adjustment while keeping the chassis aligned with the street corridor, resulting in fast, square parking across the narrow `4.2 cm` width of `BiscuitBox` litter.
  - Arm joint trajectories follow course Assignment 4 conventions (`arm1`–`arm5`), simplifying tuning and validation.
- **Cons**:
  - Forward reach is limited to `0.45 m` from `base_link` (~0.17 m from the front camera), requiring tight stopping tolerances (`±1.2 cm` longitudinal, `±5.0 cm` lateral, `±0.065 rad` heading held for 5 cycles).

## Sources
- Course Assignment 4 (`task2_pid_joint.py`, `task3_pick_place.py`, `youbot_arm_driver.py`)
