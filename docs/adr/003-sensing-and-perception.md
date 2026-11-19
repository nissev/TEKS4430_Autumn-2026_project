---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:38
last_updated: November 19, 2026 at 19:30
---

# ADR-003: Sensing & Perception Pipeline

- **Status**: Accepted (revised 2026-11-19: added Mecanum + IMU yaw fusion on `/wheel_odom_raw` and PoC semantic model filtering in `perception_node`)
- **Date**: 2026-11-08

## Context
Layer 1 of the CPS architecture requires observing the urban pedestrian environment and interpreting sensor data to support collision avoidance and litter manipulation:
1. The robot must detect street obstacles (benches, terrace railings, lamp posts, bike racks) with 2D LiDAR to prevent collisions while navigating the corridor.
2. It must identify litter items on the pavement and calculate their 3D relative bearing and distance without high CPU overhead.
3. Wheel odometry drifts over time (especially on Mecanum rollers during lateral strafing), requiring yaw drift stabilization.

## Decision
We implement `perception_node.py` and IMU-fused odometry combining three active sensing channels:
1. **2D LiDAR Collision Cones & Avoidance Bias Processing**:
   - Rather than publishing a raw multi-element array requiring index unpacking downstream, `perception_node.py` processes dual 2D LiDAR scans (`/scan_front`, `/scan_rear`) into dedicated, decoupled topics (`std_msgs/msg/Float32`):
     - `/youbot/clearance/front`: Minimum obstacle distance in the forward collision cone ($[-30^\circ, +30^\circ]$). Unconstrained free space is published as `float('inf')`.
     - `/youbot/clearance/rear`: Minimum obstacle distance in the rear collision cone ($[-30^\circ, +30^\circ]$ relative to the rear sensor frame).
     - `/youbot/clearance/recommended_turn`: Lateral flank comparison evaluating left hemisphere clearance ($[0^\circ, +90^\circ]$) versus right hemisphere clearance ($[-90^\circ, 0^\circ]$). Publishes `+1.0` (turn/strafe left into open space) or `-1.0` (turn/strafe right into open space) to provide immediate steering bias for reactive avoidance.
   - **Trigger**: Fully event-driven callbacks triggered immediately on scan arrival for minimum latency.
2. **Webots Camera with Native `Recognition` & Target Processing**:
   - **Driver Hardware Bridge Contract**: `youbot_driver.py` acts strictly as a sensor driver. It publishes all raw camera recognition objects as `geometry_msgs/msg/PoseArray` on `/camera/recognized_trash` in `camera_link` coordinates alongside raw object model strings on `/camera/recognized_trash_models` (`std_msgs/msg/String`). The driver does NOT perform model or semantic filtering.
   - **Perception Node Filtering & Locking**: `perception_node.py` subscribes to `/camera/recognized_trash` and `/camera/recognized_trash_models`. It applies a PoC mock semantic filter (`TARGET_TRASH_MODEL_NAME = "biscuit box"`, standing in for a trained vision semantic classifier in real-world deployment), transforms from `camera_link` $\to$ `base_link` (`[0.280083, -0.005, -0.03]` m, pitch `0.0` rad), filters by ground-plane height (`z_ground = z_base + 0.095` in `[-0.05, 0.20]` m) and approach window (`x` in `[0.15, 4.0]` m, `|y| <= 2.0` m), and publishes the closest candidate (by 2D Euclidean distance `hypot(x, y)`) to `/youbot/trash_target` (`geometry_msgs/msg/Point`).
3. **Mecanum Wheel Odometry + IMU (`InertialUnit`) Yaw Fusion (`/wheel_odom_raw`)**:
   - Mecanum roller slippage causes rotational drift in pure wheel-encoder dead-reckoning. `youbot_driver.py` fuses the 4 wheel encoder displacements (`delta_x`, `delta_y`) with the drift-free yaw $\psi$ from the onboard Webots `InertialUnit` (`inertial_unit`) when integrating global `(x, y, \psi)` on `/wheel_odom_raw`.
   - `behavior_tree_node.py` (`PatrolStreet`) uses fused `(odom_y, \psi)` to regulate corridor heading ($0$ or $\pi$ rad) and gently strafe back toward the street centerline (`odom_y = 0.0 m`) after obstacle avoidance.
   - `base_controller_node.py` (`APPROACH` mode) uses $\psi$ to align the robot parallel to the corridor ($0$ or $\pi$ rad) while Mecanum-strafing `target_y -> 0`, ensuring rectangular `BiscuitBox` litter is always grasped squarely across its narrow `4.2 cm` width.

## Consequences
- **Pros**:
   - Direct alignment with concepts practiced in Assignment 2 (odometry & IMU fusion) and Assignment 3 (LiDAR sector processing & reactive avoidance).
   - Dedicated float topics simplify Behavior Tree condition evaluation: condition leaves can directly compare `/youbot/clearance/front` against a distance threshold without array indexing errors.
   - Providing `/youbot/clearance/recommended_turn` over `[0°, +90°]` vs. `[-90°, 0°]` ensures the robot steers away from obstacles inside the forward cone.
   - IMU-fused odometry prevents the robot from wandering diagonally into side-wall benches after an avoidance maneuver and aligns the base squarely with rectangular litter items.
- **Cons**:
   - Mecanum wheel roller slippage still causes minor longitudinal/lateral position drift over very long runs, while yaw remains drift-free via `InertialUnit`.
   - The PoC semantic filter uses Webots `Recognition` model labels (`"biscuit box"`) and would be replaced by a real vision semantic detector (e.g., `vision_msgs`) in physical deployment.

## Sources
- [Webots R2025a Camera Recognition Documentation](https://cyberbotics.com/doc/reference/camera#recognition)
- Course Assignment 2 (`Autumn-2026_Assignment_2/README.md`)
- Course Assignment 3 (`Autumn-2026_Assignment_3/src/hybrid_assignment/hybrid_assignment/task1_behaviours.py`)
