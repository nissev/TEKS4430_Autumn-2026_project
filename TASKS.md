---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:37
last_updated: November 8, 2026 at 11:37
---

# Project Task List & Milestones (`TASKS.md`)

**Project**: Autonomous Roadside Rough-Terrain Trash Collection Rover  
**Course**: TEKS4430 – Autonomous Cyber-Physical Systems (Autumn 2026)  
**Target Scope**: Focused Proof-of-Concept (PoC) — Solo Project  

---

## Scope Tiers

- **Core Baseline (Must-Have)**:
  - Patrol rough roadside terrain (low-poly slopes, bumps, and rocks).
  - Reactive safety via 360° 2D LiDAR obstacle avoidance (forward/reverse) and IMU pitch/roll tilt protection.
  - Detect trash via Webots Camera `Recognition`, approach and park at pick standoff via Terrain-Adaptive Base PID.
  - Execute 5-state Arm Pick-and-Place Sub-FSM (`arm1`–`arm5` + gripper) to pick up trash in front of the Koala 6WD rover and drop it into the rear onboard bin.
- **Stretch Goal (Full Loop)**:
  - Track onboard bin capacity (e.g., 2–3 trash items) and navigate back to a designated depot/dumpster zone when full.
- **Scoped Out (Future Architecture)**:
  - Long-term outdoor GPS waypoint navigation (documented in architecture for completeness).

---

## Vertical Slice Milestones

### Milestone 1 (M1): Architecture, Docker Setup & Lightweight Webots World
- [x] **M1.1**: Document system architecture ([docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)) and Architecture Decision Records (`docs/adr/001`–`006`).
- [ ] **M1.2**: Scaffold Docker & ROS 2 Jazzy workspace (`docker-compose.yml`, `Dockerfile`, `.env.example`, and `src/roadside_rover_pkg`).
- [ ] **M1.3**: Build lightweight low-poly Webots world (`src/worlds/roadside_arena.wbt`) using primitive shapes (`Plane`, tilted `Box` ditches/ramps, low-poly `Cylinder`/`Box` rocks, and `Recognition`-tagged trash blocks) for high laptop FPS.
- [ ] **M1.4**: Assemble the custom **K-Team `Koala` 6WD Rover** in `roadside_arena.wbt` with `extensionSlot` devices:
  - Extracted `Youbot` 5-DOF Arm (`arm1`–`arm5`, `finger::left`, `finger::right`) mounted at the front.
  - Onboard hollow 4-wall rear trash collection bin.
  - `RobotisLds01` 360° 2D LiDAR, front `Camera` (with `Recognition`), `Gyro`, `Accelerometer` / `InertialUnit`.
- [ ] **M1.5**: Implement `rover_driver.py` bridging Webots motors/sensors to ROS 2 topics and verify manual teleop/arm movement.

### Milestone 2 (M2): Layer 1 — Sensing, Perception & State Estimation
- [ ] **M2.1**: Implement 6-wheel encoder odometry + IMU **Complementary Filter** in `perception_node.py` to publish slip-compensated `/rover/odom_filtered` and `/rover/tilt_state` (roll, pitch, tilt magnitude, safety flag).
- [ ] **M2.2**: Process 360° 2D LiDAR `/scan` into directional clearance sectors (front, front-left, front-right, rear) for forward and reverse obstacle avoidance.
- [ ] **M2.3**: Process Webots Camera `Recognition` detections into relative target bearing and distance (`/rover/trash_target`).
- [ ] **M2.4**: Verify telemetry output in console/logs while driving over tilted ramps and past trash items.

### Milestone 3 (M3): Layer 3 — Terrain-Adaptive Base PID & Arm Pick-and-Place FSM
- [ ] **M3.1**: Implement **Terrain-Adaptive Base PID Controller** (regulates angular heading error and linear standoff distance to park accurately in front of trash, scaling max linear speed inversely with IMU tilt angle).
- [ ] **M3.2**: Calibrate 5-DOF arm joint waypoints (`HOME`, `PRE_GRASP`, `GRASP`, `LIFT`, `DROP_IN_REAR_BIN`) for the Koala chassis geometry.
- [ ] **M3.3**: Implement `arm_fsm_node.py` (5-state Pick-and-Place FSM triggered via ROS 2 topic/action once parked).
- [ ] **M3.4**: Test physical finger grasping on flat and tilted ground; implement optional Supervisor grasp-assist/weld fallback in `rover_driver.py` if physics slip occurs on slopes.

### Milestone 4 (M4): Layer 2 — Hybrid Behavior Tree (`py_trees`) Integration
- [ ] **M4.1**: Implement reactive safety leaves in `behavior_tree_node.py`:
  - `IsTiltCritical` + `RecoverFromTilt` (back off / throttle speed when pitch/roll exceeds threshold).
  - `IsObstacleClose` + `AvoidObstacle` (360° LiDAR steering / reversing).
- [ ] **M4.2**: Implement deliberative mission leaves in `behavior_tree_node.py`:
  - `DetectAndApproachTrash` (locks onto `/rover/trash_target` and drives via Base PID to pick standoff).
  - `TriggerArmPickAndPlace` (waits for `arm_fsm_node` completion and increments cargo counter).
  - `PatrolRoadside` (default exploration behavior).
- [ ] **M4.3** *(Stretch Goal)*: Add `IsBinFull` + `ReturnToDepot` branch to return to the depot coordinates after collecting $N$ items.

### Milestone 5 (M5): Verification, Presentation & Final Submission
- [ ] **M5.1**: End-to-end mission verification in Webots + telemetry log capture (optional Matplotlib PID plot if needed for tuning).
- [ ] **M5.2**: Decide final presentation slide format (see Open Decisions below) and create presentation slides covering architecture, ADRs, verification, and Part B self-reflection.
- [ ] **M5.3**: Record video presentation demonstrating the system and explaining Part A & Part B.

---

## Open / TBD Decisions

| ID | Topic | Options Under Consideration | Status |
| :--- | :--- | :--- | :---: |
| **TBD-01** | **Presentation Slide Format** | 1. **Reveal.js HTML/JS Deck** (`slides/index.html` embedding Markdown/ADRs & Mermaid diagrams, exportable to PDF)<br>2. **PowerPoint / Google Slides** (standard slide deck importing diagrams & screenshots)<br>3. **Marp Markdown Slides** | **TBD** (Decide in M5) |
| **TBD-02** | **PID Plotter Script** | Console/log telemetry is baseline; add `plotter.py` only if visual PID step-response tuning is needed in M3. | **Optional** |
| **TBD-03** | **Depot Return Stretch Goal** | Include if M1–M4 complete smoothly with time to spare. | **Optional** |

---

## Sources
- [TEKS4430 Course Project Specification](ASSIGNMENT.md) (`https://github.com/TEKS4430/Autumn-2026_project`)
- Course Assignments 1–4 (`Autumn-2026_Assignment_1` to `Autumn-2026_Assignment_4`)
