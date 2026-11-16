---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:37
last_updated: November 12, 2026 at 18:40
---

# System Architecture: Iso-Roba Pedestrian Street Clean-Up Robot

**Course**: TEKS4430 – Autonomous Cyber-Physical Systems  
**Setting**: Finnish pedestrian walking district (inspired by *Iso Roobertinkatu*, Helsinki)  
**Robot Platform**: KUKA `Youbot` (4-wheel Mecanum base + 5-DOF arm + parallel gripper)  
**Frameworks**: ROS 2 Jazzy (Docker) + Webots R2025a (Host)

---

## 1. Architectural Overview

The **Iso-Roba Clean-Up Robot** is an Autonomous Cyber-Physical System (CPS) built around a standard **KUKA `Youbot`** (`Youbot.proto`, `name "youbot"`). It operates on a flat, stone-paved pedestrian street flanked by building facades and urban obstacles (benches, cafe terraces, bike racks). It uses omnidirectional mobility to align with litter items, an onboard 5-DOF arm to grasp and deposit trash into a rear-deck bin, and a 2D LiDAR to maintain clearance from obstacles.

```mermaid
flowchart TD
    subgraph Webots["Webots R2025a Simulator (Iso-Roba Pedestrian Arena)"]
        Sensors["Sensors: 4x Mecanum Encoders, IMU (Gyro/Accelerometer),\n2D LiDAR (HokuyoURG04LX), Camera (with Recognition)"]
        Actuators["Actuators: 4x Mecanum Wheel Motors (wheel1..wheel4),\n5-DOF Arm (arm1..arm5), Gripper (finger::left, finger::right)"]
    end

    subgraph ROS2["ROS 2 Jazzy (isoroba_cleaner_pkg)"]
        Driver["youbot_driver.py\n(Webots ROS 2 Bridge)"]

        subgraph Layer1["Layer 1: Sensing & Perception (perception_node.py)"]
            LidarProc["2D LiDAR Sector Analyzer\n(Front / Front-Left / Front-Right / Rear)"]
            CamRec["Litter Target Tracker\n(Camera Recognition -> Relative Bearing & Distance)"]
        end

        subgraph Layer2["Layer 2: Reasoning & Planning (behavior_tree_node.py)"]
            BT["Hybrid Behavior Tree (py_trees)\nReactive Safety (LiDAR Obstacle Avoidance & Emergency Stop)\n+ Deliberative Mission (Patrol Street -> Approach Litter -> Pick & Bin -> Depot)"]
        end

        subgraph Layer3["Layer 3: Act & Control"]
            BasePID["Holonomic Base PID (base_controller_node.py)\n(vx, vy, omega_z error regulation using Mecanum strafing)"]
            ArmFSM["5-State Arm Pick-and-Place Sub-FSM (arm_fsm_node.py)\n(HOME -> PRE_GRASP -> GRASP -> LIFT -> DROP_IN_BIN)"]
        end
    end

    Sensors --> Driver
    Driver --> LidarProc
    Driver --> CamRec

    LidarProc -->|"/youbot/clearance/front, rear, recommended_turn"| BT
    CamRec -->|"/youbot/trash_target"| BT

    BT -->|"Mode & Target"| BasePID
    BT -->|"Trigger Pick (/arm/pick_trigger)"| ArmFSM
    ArmFSM -->|"Pick Status (/arm/pick_status)"| BT

    BasePID -->|"/cmd_vel (vx, vy, omega)"| Driver
    ArmFSM -->|"/arm/joint_1..5/cmd\n/arm/gripper/cmd"| Driver
    Driver --> Actuators
```

---

## 2. Three-Layer CPS Mapping

| CPS Layer | ROS 2 Node(s) | Responsibilities | Course Baseline Reference |
| :--- | :--- | :--- | :--- |
| **Layer 1: Sensing & Perception** | `perception_node.py` | • Splits 2D LiDAR into clearance sectors (`front`, `front_left`, `front_right`, `rear`).<br>• Extracts litter 3D relative bearing and distance from Camera `Recognition`. | Assignments 2 & 3 |
| **Layer 2: Reasoning & Decision-Making** | `behavior_tree_node.py` | • High-priority reactive safety: obstacle avoidance around street furniture.<br>• Deliberative mission: sweeping patrol, target lock, triggering manipulation FSM.<br>• Abort handling and state transitions. | Assignment 3 (`py_trees`) |
| **Layer 3: Act & Control** | `base_controller_node.py`<br>`arm_fsm_node.py` | • **Base**: Holonomic PID regulating $(v_x, v_y, \omega_z)$ to park at pick standoff distance.<br>• **Arm**: Deterministic 5-state trajectory (`HOME` $\rightarrow$ `PRE_GRASP` $\rightarrow$ `GRASP` $\rightarrow$ `LIFT` $\rightarrow$ `DROP_IN_BIN`). | Assignment 4 (Tasks 2 & 3) |

---

## 3. Communication Topic Interfaces

| Topic Name | Message Type | Publisher Node | Subscriber Node(s) | Role |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `base_controller_node` | `youbot_driver` | Holonomic base velocity commands ($v_x, v_y, \omega_z$). |
| `/scan_front`, `/scan_rear` | `sensor_msgs/msg/LaserScan` | `youbot_driver` | `perception_node` | Raw 2D LiDAR distances. |
| `/camera/recognized_trash` | `geometry_msgs/msg/PoseArray` | `youbot_driver` | `perception_node` | Raw recognized object 3D poses in `camera_link` (unfiltered hardware bridge). |
| `/youbot/clearance/front`, `/youbot/clearance/rear` | `std_msgs/msg/Float32` | `perception_node` | `behavior_tree_node` | Minimum clearance distance in front and rear collision cones (inf if clear). |
| `/youbot/clearance/recommended_turn` | `std_msgs/msg/Float32` | `perception_node` | `behavior_tree_node` | Recommended avoidance turn bias (+1.0 left, -1.0 right). |
| `/youbot/trash_target` | `geometry_msgs/msg/Point` | `perception_node` | `behavior_tree_node`, `base_controller_node` | Locked litter target $(x, y)$ in robot frame. |
| `/youbot/base_mode` | `std_msgs/msg/String` | `behavior_tree_node` | `base_controller_node` | Selected base control mode (`IDLE`, `PATROL`, `APPROACH`). |
| `/arm/pick_trigger` | `std_msgs/msg/Bool` | `behavior_tree_node` | `arm_fsm_node` | Starts the pick-and-place cycle. |
| `/arm/pick_status` | `std_msgs/msg/String` | `arm_fsm_node` | `behavior_tree_node` | Feedback (`BUSY`, `SUCCESS`, `FAILED`). |
| `/arm/abort` | `std_msgs/msg/Bool` | `behavior_tree_node` | `arm_fsm_node` | Preempts pick action back to `HOME`. |
