---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:38
last_updated: November 11, 2026 at 16:15
---

# ADR-004: Hybrid Behavior Tree & Arm Sub-FSM Architecture

- **Status**: Accepted (revised 2026-11-11: updated safety leaves for pedestrian street obstacles)
- **Date**: 2026-11-08

## Context
Layer 2 of the CPS architecture requires reasoning, planning, and decision-making capable of reacting immediately to unexpected physical obstacles in a pedestrian walkway while executing multi-step mission tasks (patrolling the promenade, approaching litter, picking it up with the 5-DOF arm, and stowing it in the rear bin).

## Decision
We implement a **Hybrid `py_trees` Behavior Tree + 5-State Arm Pick-and-Place Sub-FSM**:
1. **High-Level `py_trees` Behavior Tree (`behavior_tree_node.py`)**:
   - **Reactive Safety Branch (Priority 1)**: Evaluates `IsObstacleTooClose` via 2D LiDAR sectors $\rightarrow$ stops and executes evasive steering/reversing around cafe fences, benches, or pedestrians.
   - **Deliberative Mission Branch (Priority 2)**: Executes `DepotReturn` (stretch goal when bin count $\ge N$), `LitterCollection` (`ApproachLitterPID` $\rightarrow$ `ExecuteArmSubFSM`), and `PatrolStreet` (corridor sweeping).
2. **Dedicated Arm Pick-and-Place Sub-FSM (`arm_fsm_node.py`)**:
   - Triggered once the YouBot is parked at pick standoff distance facing the target litter.
   - Steps through 5 deterministic states: `HOME` $\rightarrow$ `PRE_GRASP` $\rightarrow$ `GRASP` $\rightarrow$ `LIFT` $\rightarrow$ `DROP_IN_BIN`.
   - Open/close parallel gripper fingers (`finger::left`, `finger::right`) within `PRE_GRASP`, `GRASP`, and `DROP_IN_BIN`.
   - **Abort protocol**: If a safety condition triggers during manipulation, the BT publishes `/arm/abort`, causing the FSM to immediately stow the arm back to `HOME` and abort the pick.

## Consequences
- **Pros**:
  - Leverages `py_trees` preemption (Assignment 3) for reactive safety while keeping sequential arm trajectories modular and testable (Assignment 4 Task 3).
  - Clean separation between base navigation and arm joint sequencing.
- **Cons**:
  - Requires clean inter-node status handshake (`/arm/pick_trigger`, `/arm/abort`, `/arm/pick_status`).

## Sources
- [py_trees Documentation](https://py-trees.readthedocs.io/en/devel/)
- Course Assignments 3 & 4
