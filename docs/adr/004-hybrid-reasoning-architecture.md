---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 8, 2026 at 11:38
---

# ADR-004: Hybrid Behavior Tree & Arm Sub-FSM Architecture

- **Status**: Accepted
- **Date**: 2026-11-08

## Context
Layer 2 of the CPS architecture requires reasoning, planning, and decision-making that can immediately react to physical hazards (excessive terrain tilt or close obstacles) while pursuing multi-step mission goals (patrolling, approaching trash, picking it up with a 5-DOF arm, and returning to a depot).

## Decision
We will implement a **Hybrid `py_trees` Behavior Tree + 5-State Arm Pick-and-Place Sub-FSM**:
1. **High-Level `py_trees` Behavior Tree (`behavior_tree_node.py`)**:
   - **Reactive Safety Branch (Priority 1)**: Checks `IsTiltCritical` (IMU pitch/roll threshold exceeded $\rightarrow$ stop/reverse off steep slope) and `IsObstacleTooClose` ($360^\circ$ LiDAR sectors $\rightarrow$ steer clear or reverse).
   - **Deliberative Mission Branch (Priority 2)**: Executes `DepotReturn` (stretch goal when bin count $\ge N$), `TrashCollection` (`ApproachTrashPID` $\rightarrow$ `ExecuteArmSubFSM`), and `PatrolRoadside`.
2. **Dedicated Arm Pick-and-Place Sub-FSM (`arm_fsm_node.py`)**:
   - Triggered by the Behavior Tree once the rover is parked at the pick standoff distance in front of a trash item.
   - Steps deterministically through 5 states (`HOME` $\rightarrow$ `PRE_GRASP` $\rightarrow$ `GRASP` $\rightarrow$ `LIFT` $\rightarrow$ `DROP_IN_BIN`) and reports `SUCCESS`/`FAILED` back to the Behavior Tree.
   - Gripper open/close is handled inside the states: open in `PRE_GRASP`, close in `GRASP`, release and return to `HOME` in `DROP_IN_BIN`.
   - **Abort/stow protocol**: the Reactive Safety branch outranks the mission branch, so when it preempts `ExecuteArmSubFSM` the BT publishes `/arm/abort`; the FSM stows the arm to `HOME` and reports `ABORTED`. The FSM also enforces a pick timeout (reports `FAILED`).

## Consequences
- **Pros**:
  - Combines the modularity and preemption of `py_trees` (Assignment 3) with the clean sequential joint trajectory control of an FSM (Assignment 4 Task 3).
  - Keeps `behavior_tree_node.py` clean and decoupled from low-level joint timing.
- **Cons**:
  - Requires a lightweight handoff protocol (`/arm/pick_trigger`, `/arm/abort` and `/arm/pick_status` topics) between the BT node and the Arm FSM node.

## Sources
- [py_trees Documentation](https://py-trees.readthedocs.io/en/devel/)
- Course Assignments 3 & 4 (`Autumn-2026_Assignment_3`, `Autumn-2026_Assignment_4`)
