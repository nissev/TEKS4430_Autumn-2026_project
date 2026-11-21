---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 21, 2026 at 15:05
---

# ADR-004: Hybrid Behavior Tree & Arm Sub-FSM Architecture

- **Status**: Accepted (revised 2026-11-11: updated safety leaves for pedestrian street obstacles)
- **Date**: 2026-11-08

## Context
Layer 2 of the CPS architecture requires reasoning, planning, and decision-making capable of reacting immediately to unexpected physical obstacles in a pedestrian walkway while executing multi-step mission tasks (patrolling the promenade, approaching litter, picking it up with the 5-DOF arm, and stowing it in the rear bin).

## Decision
We implement a **Hybrid `py_trees` Behavior Tree + Arm Pick-and-Place Sub-FSM**:
1. **High-Level `py_trees` Behavior Tree (`behavior_tree_node.py`)**:
   - **Reactive Safety Branch (Priority 1)**: Evaluates `IsObstacleTooClose` via front/rear 2D LiDAR clearance cones (with `0.40 m` near-field arm self-occlusion filtering during active picks and `0.75 m` clear hysteresis) $\rightarrow$ executes two-stage evasive turning/strafing and reversing (`AvoidObstacle`) around cafe fences, benches, or pedestrians.
   - **Deliberative Mission Branch (Priority 2)**: Executes `LitterCollection` (`DetectAndApproachTrash` $\rightarrow$ `ExecuteArmSubFSM`) and `PatrolStreet` (IMU-stabilized corridor sweeping), with an optional `DepotReturn` stretch branch when bin count $\ge N$.
2. **Dedicated Arm Pick-and-Place Sub-FSM (`arm_fsm_node.py`)**:
   - Triggered once the YouBot is parked at the `0.45 m` pick standoff distance facing the target litter.
   - Steps through 8 deterministic FSM states (`IDLE` $\rightarrow$ `PRE_GRASP` $\rightarrow$ `GRASP` $\rightarrow$ `CLOSE_GRIPPER` $\rightarrow$ `LIFT` $\rightarrow$ `DROP_IN_BIN` $\rightarrow$ `RELEASE_GRIPPER` $\rightarrow$ `RETURN_HOME`) covering the 5 kinematic arm stages (`HOME`, `PRE_GRASP`, `GRASP`/`GRASP_TALL`, `LIFT`, `DROP_IN_BIN`).
   - Verifies parallel gripper finger settling (`finger::left`, `finger::right`) and detects empty grasps (`< 0.002 m`) in `CLOSE_GRIPPER`.
   - **Abort protocol**: If a safety condition triggers during manipulation, the BT publishes `/arm/abort`, causing the FSM to immediately stow the arm back to `HOME` (`RETURN_HOME`) and abort the pick.

## Consequences
- **Pros**:
  - Leverages `py_trees` preemption (Assignment 3) for reactive safety while keeping sequential arm trajectories modular and testable (Assignment 4 Task 3).
  - Clean separation between base navigation and arm joint sequencing.
- **Cons**:
  - Requires clean inter-node status handshake (`/arm/pick_trigger`, `/arm/abort`, `/arm/pick_status`).

## Sources
- [py_trees Documentation](https://py-trees.readthedocs.io/en/devel/)
- Course Assignments 3 & 4
