---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:55
last_updated: November 8, 2026 at 11:55
---

# Autonomous Roadside Rough-Terrain Trash Collection Rover

**Course**: TEKS4430 – Autonomous Cyber-Physical Systems (Autumn 2026)  
**Tech Stack**: ROS 2 Jazzy (Docker) + Webots R2025a (Host)

---

## 1. Project Concept

An autonomous 6-wheel-drive (6WD) outdoor rover (**K-Team `Koala`**) equipped with a **5-DOF manipulator arm** (`Youbot` arm extraction) and an **onboard rear collection bin**, designed to locate, pick up, and store roadside litter across uneven, obstacle-strewn terrain.

The system implements the three core CPS architectural layers:
1. **Sensing & Perception (`perception_node.py`)**: Fuses 6-wheel encoders with IMU gyro/accelerometer data via a **Complementary Filter** to compensate for wheel slip while reading roll/pitch tilt directly from the `InertialUnit`, while processing **360° 2D LiDAR** (`RobotisLds01`, front + rear units) clearance sectors and **Webots Camera `Recognition`** 3D trash detections.
2. **Reasoning, Planning & Decision-Making (`behavior_tree_node.py`)**: A hybrid reactive–deliberative **Behavior Tree (`py_trees`)** that prioritizes IMU rollover/tilt recovery and 360° obstacle avoidance (forward & reverse) over deliberative patrolling, approaching trash, and triggering the arm pick-and-place sequence.
3. **Act & Control (`base_controller_node.py` & `arm_fsm_node.py`)**: A **Terrain-Adaptive Base PID Controller** that aligns and parks the rover at the grasp standoff distance while dynamically scaling speed inversely with terrain tilt, paired with a **5-State Arm Pick-and-Place Sub-FSM** (`HOME` $\rightarrow$ `PRE_GRASP` $\rightarrow$ `GRASP` $\rightarrow$ `LIFT` $\rightarrow$ `DROP_IN_BIN`).

---

## 2. Repository Documentation & Links

- **[Course Assignment Specification (`ASSIGNMENT.md`)](ASSIGNMENT.md)**
- **[Project Task List & Milestones (`TASKS.md`)](TASKS.md)**
- **[System Architecture & Diagrams (`docs/ARCHITECTURE.md`)](docs/ARCHITECTURE.md)**
- **Architecture Decision Records (`docs/adr/`)**:
  - [ADR-001: Robot Platform & Manipulation Hardware](docs/adr/001-robot-platform-and-arm.md)
  - [ADR-002: Lightweight Simulation Environment Design](docs/adr/002-lightweight-simulation-world.md)
  - [ADR-003: Sensing, Perception & State Estimation Pipeline](docs/adr/003-sensing-and-perception.md)
  - [ADR-004: Hybrid Behavior Tree & Arm Sub-FSM Architecture](docs/adr/004-hybrid-reasoning-architecture.md)
  - [ADR-005: Terrain-Adaptive Control & Grasping Reliability](docs/adr/005-actuation-and-control.md)
  - [ADR-006: ROS 2 Package & Node Topology](docs/adr/006-ros2-package-topology.md)
  - [ADR-007: Road/Hazard-Zone Exclusion Not Implemented (Known Safety Gap)](docs/adr/007-road-exclusion-not-implemented.md)

---

## 3. Environment Setup & Running Instructions

> **Note**: Commands below will be updated and expanded as each milestone (`M1`–`M5`) in [`TASKS.md`](TASKS.md) is implemented.

### Prerequisites
- **Webots R2025a** installed natively on the host OS (Windows / macOS / Linux).
- **Docker Desktop** running on the host OS.

### Step 1: First-Time Docker Setup
```bash
cp .env.example .env
# Linux: uncomment NETWORK_MODE=host and WEBOTS_HOST=localhost
# Windows / macOS: leave defaults (host.docker.internal)

docker compose build
```

### Step 2: Launch Webots World
1. Open **Webots R2025a**.
2. Go to **File $\rightarrow$ Open World...** and select `src/worlds/roadside_arena.wbt`.
3. Press **Play (▶)** in Webots before starting the ROS 2 driver.

### Step 3: Build & Run in ROS 2 Container *(Upcoming in M1.4–M4)*

**Terminal 1 — Start Container, Build & Launch Driver:**
```bash
docker compose run --rm ros2 bash
colcon build --symlink-install --packages-select roadside_rover_pkg
source install/setup.bash
ros2 launch roadside_rover_pkg rover.launch.py
```

**Terminal 2 — Attach to Running Container (Inspection & Individual Nodes):**
```bash
docker exec -it <container_name> bash
source /ros2_ws/install/setup.bash

# Inspect active topics
ros2 topic list

# Monitor slip-compensated pose & IMU tilt state (M2)
ros2 topic echo /rover/odom_filtered
ros2 topic echo /rover/tilt_state

# Monitor detected trash target (M2)
ros2 topic echo /rover/trash_target

# Manually trigger 5-DOF arm pick-and-place cycle (M3)
ros2 topic pub --once /arm/pick_trigger std_msgs/msg/Bool "{data: true}"
```

---

## 4. Milestone Status Summary

| Milestone | Description | Status |
| :---: | :--- | :---: |
| **M1** | Architecture, Docker Setup & Lightweight Webots World | 🔄 In Progress (`M1.1`–`M1.3` Done) |
| **M2** | Layer 1: Sensing, Perception & Complementary Filter | ⏳ Pending |
| **M3** | Layer 3: Terrain-Adaptive Base PID & 5-DOF Arm Sub-FSM | ⏳ Pending |
| **M4** | Layer 2: Hybrid Behavior Tree (`py_trees`) Integration | ⏳ Pending |
| **M5** | Verification, Presentation Slides & Video Submission | ⏳ Pending |

---

## Sources
- [Course Project Instructions (`ASSIGNMENT.md`)](ASSIGNMENT.md)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- [Webots R2025a User Guide](https://cyberbotics.com/doc/guide/index)
