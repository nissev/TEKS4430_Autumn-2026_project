---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:37
last_updated: November 22, 2026 at 19:15
---

# Project Task List & Milestones (`TASKS.md`)

**Project**: Iso-Roba Urban Pedestrian Street Clean-Up Robot (KUKA YouBot)  
**Setting**: Finnish pedestrian walking district (inspired by *Iso Roobertinkatu*, Helsinki)  
**Course**: TEKS4430 – Autonomous Cyber-Physical Systems (Autumn 2026)  
**Target Scope**: Focused Proof-of-Concept (PoC) — Solo Project  

---

## Scope Tiers

- **Core Baseline (Must-Have)**:
  - Patrol flat, paved urban pedestrian street (Iso-Roba style: stone pavement flanked by building facades).
  - Reactive safety via 2D LiDAR obstacle avoidance (outdoor cafe barriers, benches, bike racks, trash cans).
  - Detect sidewalk litter (coffee cups, cans, boxes) via Webots Camera `Recognition`, approach and align using Holonomic (Mecanum) Base PID.
  - Execute 5-state Arm Pick-and-Place Sub-FSM (`arm1`–`arm5` + parallel gripper) to pick up trash from the pavement and drop it into an onboard rear collection bin.
- **Stretch Goal (Full Loop)**:
  - Track onboard bin capacity (e.g., 2–3 trash items) and navigate back to a designated municipal collection/depot zone when full.
- **Scoped Out (Future Architecture)**:
  - Real-time dynamic pedestrian trajectory prediction.
  - Pedestrian street boundary & crossing traffic exclusion (documented as a known safety gap in [ADR-007](docs/adr/007-road-exclusion-not-implemented.md)).

---

## Vertical Slice Milestones

### Milestone 1 (M1): Architecture, Docker Setup & Lightweight Webots World
- [x] **M1.1**: Document system architecture ([docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)) and Architecture Decision Records (`docs/adr/001`–`007`) updated for the Iso-Roba YouBot concept.
- [x] **M1.2**: Scaffold Docker & ROS 2 Jazzy workspace (`docker-compose.yml`, `Dockerfile`, `.env.example`, and `src/isoroba_cleaner_pkg`).
- [x] **M1.3**: Build lightweight Finnish pedestrian street Webots world (`src/worlds/isoroba_arena.wbt`): flat paved pedestrian corridor flanked by building facades, benches, bike racks, terrace fences, and `Recognition`-tagged litter items (see ADR-002).
- [x] **M1.4**: Configure native **KUKA `Youbot`** (`Youbot.proto`, `name "youbot"`) with its 5-DOF arm (`arm1`–`arm5`), 2-finger parallel gripper (`finger::left`, `finger::right`), onboard rear collection bin, dual 2D LiDARs (`lidar_front`, `lidar_rear`), Camera (with `Recognition`), and wheel encoders. Verify arm ground reach and stow/bin drop.
- [x] **M1.5**: Implement `youbot_driver.py` (`webots_ros2_driver` plugin):
  - [x] **M1.5a**: 4-wheel Mecanum holonomic drive + `InertialUnit` IMU-fused odometry (`/cmd_vel` linear $x, y$ + angular $z$, `/wheel_odom_raw`).
  - [x] **M1.5b**: Dual 2D LiDAR and IMU topics (`/scan_front`, `/scan_rear`).
  - [x] **M1.5c**: Camera `Recognition` conversion to `/camera/recognized_trash` and `/camera/recognized_trash_models`.
  - [x] **M1.5d**: Arm joint and gripper command interfaces (`/arm/joint_N/cmd`, `/arm/gripper/cmd`); verify manual teleop/arm movement.

### Milestone 2 (M2): Layer 1 — Sensing & Perception
- [x] **M2.1**: Process 2D LiDAR scans into directional clearance cones and avoidance turn bias (`/youbot/clearance/front`, `/youbot/clearance/rear`, `/youbot/clearance/recommended_turn`) for reactive obstacle avoidance around street furniture and terrace barriers.
- [x] **M2.2**: Process Webots Camera `Recognition` detections from `/camera/recognized_trash` and `/camera/recognized_trash_models` into relative target bearing and distance (`/youbot/trash_target`), applying PoC semantic filtering (`biscuit box`), ground-plane height ($z \approx 0$), and manipulator reachability filtering.

### Milestone 3 (M3): Layer 3 — Holonomic Base PID & Arm Pick-and-Place FSM
- [x] **M3.1**: Implement **Holonomic Base PID Controller** in `base_controller_node.py` (sole `/cmd_vel` publisher; mode selected by BT via `/youbot/base_mode`; regulates lateral error $e_y$, corridor-aligned heading error $e_\theta$ using `/wheel_odom_raw`, and standoff distance $e_x$ to park cleanly before trash using omnidirectional strafing).
- [x] **M3.2**: Calibrate 5-DOF arm joint waypoints (`HOME`, `PRE_GRASP`, `GRASP`, `GRASP_TALL`, `LIFT`, `DROP_IN_BIN`) for the YouBot chassis and rear bin geometry.
- [x] **M3.3**: Implement `arm_fsm_node.py` (5-state Pick-and-Place FSM triggered via `/arm/pick_trigger` once parked; supports `/arm/abort` with stow to `HOME` and a pick timeout; gripper open/close handled inside states).
- [x] **M3.4**: Test physical finger grasping on flat cobblestone/pavement.

### Milestone 4 (M4): Layer 2 — Hybrid Behavior Tree (`py_trees`) Integration
- [x] **M4.1**: Implement reactive safety leaves in `behavior_tree_node.py`:
  - `IsObstacleTooClose` + `AvoidObstacle` (2D LiDAR steering / backing up).
  - Preemption: publish `/arm/abort` if obstacle hazard interrupts picking.
- [x] **M4.2**: Implement deliberative mission leaves in `behavior_tree_node.py`:
  - `DetectAndApproachTrash` (locks onto `/youbot/trash_target` and drives via Holonomic PID to pick standoff).
  - `ExecuteArmSubFSM` (waits for `arm_fsm_node` completion).
  - `PatrolStreet` (default sweeping corridor navigation along Iso-Roba).
- [ ] **M4.3** *(Stretch Goal)*: Add `IsBinFull` + `ReturnToDepot` branch to return to the depot coordinates after collecting $N$ items.

### Milestone 5 (M5): Verification, Presentation & Final Submission
- [x] **M5.1**: End-to-end mission verification in Webots + telemetry log capture.
- [x] **M5.2**: Create presentation slides ([slides/index.html](slides/index.html)) covering architecture, ADRs (including the architectural pivot from rough terrain to Iso-Roba pedestrian YouBot), verification, and Part B self-reflection.
- [x] **M5.3**: Record video presentation demonstrating the system and explaining Part A & Part B.
- [x] **M5.4**: Part B deliverables: AI-tool usage reporting and self-reflection on architectural decision-making and learning (documented in `slides/index.html` Dias 10–13).

---

## Sources
- [TEKS4430 Course Project Specification](ASSIGNMENT.md)
- Course Assignments 1–4 (`Autumn-2026_Assignment_1` to `Autumn-2026_Assignment_4`)
