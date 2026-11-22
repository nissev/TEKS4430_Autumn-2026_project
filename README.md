---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:55
last_updated: November 22, 2026 at 19:15
---

# Iso-Roba Urban Pedestrian Clean-Up Robot (KUKA YouBot)

**Course**: TEKS4430 – Autonomous Cyber-Physical Systems (Autumn 2026)  
**Setting**: Finnish pedestrian street (inspired by *Iso Roobertinkatu*, Helsinki)  
**Tech Stack**: ROS 2 Jazzy (Docker) + Webots R2025a (Host)

---

## 1. Project Concept

An autonomous mobile manipulation clean-up robot based on the **KUKA `Youbot`** (`Youbot.proto`, `name "youbot"`), equipped with a 4-wheel Mecanum omnidirectional base, a **5-DOF manipulator arm** (`arm1`–`arm5`), a 2-finger parallel gripper, dual 2D LiDARs (`lidar_front`, `lidar_rear`), an IMU (`InertialUnit`), and an **onboard rear platform collection bin**. The robot is designed to patrol flat paved pedestrian zones (such as Helsinki's *Iso Roobertinkatu*), detect litter (`BiscuitBox` packaging), navigate around street obstacles (benches, cafe terrace fences, public bins, static pedestrians), pick up litter with its arm, and deposit it into the onboard bin.

The system implements the three core CPS architectural layers:
1. **Sensing & Perception (`perception_node.py`)**: Processes **dual 2D LiDAR** front/rear collision cones (`[-30°, +30°]`) and lateral hemisphere steering bias (`[0°, +90°]` vs. `[-90°, 0°]`), and filters **Webots Camera `Recognition`** objects into 3D relative positions of sidewalk litter in `base_link`.
2. **Reasoning, Planning & Decision-Making (`behavior_tree_node.py`)**: A hybrid reactive–deliberative **Behavior Tree (`py_trees`)** that prioritizes immediate collision avoidance (with arm self-occlusion filtering and pick preemption) over deliberative corridor patrolling, litter approach, and arm pick-and-place execution.
3. **Act & Control (`base_controller_node.py` & `arm_fsm_node.py`)**: A **Holonomic Base PID Controller** that leverages omnidirectional strafing ($v_x, v_y, \omega_z$) and IMU-fused corridor heading to align at the `0.45 m` pick standoff, coupled with an **Arm Pick-and-Place Sub-FSM** (8 FSM states stepping through 5 kinematic stages: `HOME` $\rightarrow$ `PRE_GRASP` $\rightarrow$ `GRASP` $\rightarrow$ `LIFT` $\rightarrow$ `DROP_IN_BIN`).

---

## 2. Repository Documentation & Links

- **[Course Assignment Specification (`ASSIGNMENT.md`)](ASSIGNMENT.md)**
- **[Project Task List & Milestones (`TASKS.md`)](TASKS.md)**
- **[System Architecture & Diagrams (`docs/ARCHITECTURE.md`)](docs/ARCHITECTURE.md)**
- **[Presentation Slides (`slides/index.html`)](slides/index.html)**: Interactive Reveal.js presentation covering CPS architecture, ADRs, Part B AI reporting & reflection, and 1-click PDF export.
- **Architecture Decision Records (`docs/adr/`)**:
  - [ADR-001: Robot Platform & Manipulation Hardware (KUKA YouBot)](docs/adr/001-robot-platform-and-arm.md)
  - [ADR-002: Lightweight Simulation World (Iso-Roba Pedestrian Street)](docs/adr/002-lightweight-simulation-world.md)
  - [ADR-003: Sensing & Perception Pipeline](docs/adr/003-sensing-and-perception.md)
  - [ADR-004: Hybrid Behavior Tree & Arm Sub-FSM Architecture](docs/adr/004-hybrid-reasoning-architecture.md)
  - [ADR-005: Holonomic Base PID & Manipulation Control](docs/adr/005-actuation-and-control.md)
  - [ADR-006: ROS 2 Package & Node Topology](docs/adr/006-ros2-package-topology.md)
  - [ADR-007: Pedestrian Zone & Street Crossing Exclusion (Known Safety Gap)](docs/adr/007-road-exclusion-not-implemented.md)

---

## 3. Environment Setup & Running Instructions

### Prerequisites
- **Webots R2025a** installed natively on the host OS (Windows / macOS / Linux).
- **Docker Desktop** running on the host OS.

### Step 1: Docker Configuration
```bash
cp .env.example .env
# Windows / macOS: leave defaults (host.docker.internal, ROBOT_NAME=youbot)
# Linux: uncomment NETWORK_MODE=host and WEBOTS_HOST=localhost

docker compose build
```

### Step 2: Launch Webots World
1. Open **Webots R2025a**.
2. Open `src/worlds/isoroba_arena.wbt`.
3. Press **Play (▶)** in Webots before starting the ROS 2 driver.

### Step 3: Build & Launch ROS 2 Container
```bash
docker compose run --rm ros2 bash
colcon build --symlink-install --packages-select isoroba_cleaner_pkg
source install/setup.bash
ros2 launch isoroba_cleaner_pkg cleaner.launch.py
```

---

## 4. Milestone Status Summary

| Milestone | Description | Status |
| :---: | :--- | :---: |
| **M1** | Architecture, Docker Setup & Iso-Roba World Setup | ✅ Done (`M1.1`–`M1.5`) |
| **M2** | Layer 1: Sensing, Perception & State Estimation | ✅ Done (`M2.1`–`M2.2`) |
| **M3** | Layer 3: Holonomic Base PID & Arm Pick-and-Place FSM | ✅ Done (`M3.1`–`M3.4`) |
| **M4** | Layer 2: Hybrid Behavior Tree (`py_trees`) | ✅ Core Done (`M4.1`–`M4.2`; `M4.3` Stretch Pending) |
| **M5** | Verification, Presentation & Part B Reflection | ✅ Done (`M5.1`–`M5.4`) |
