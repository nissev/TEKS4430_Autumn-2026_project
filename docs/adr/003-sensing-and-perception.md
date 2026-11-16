---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:38
last_updated: November 16, 2026 at 20:12
---

# ADR-003: Sensing & Perception Pipeline

- **Status**: Accepted (revised 2026-11-16: refined LiDAR collision cones and locked M2.2 geometric camera filtering bounds)
- **Date**: 2026-11-08

## Context
Layer 1 of the CPS architecture requires observing the urban pedestrian environment and interpreting sensor data to support collision avoidance and litter manipulation:
1. The robot must detect street obstacles (benches, terrace railings, lamp posts, bike racks) with 2D LiDAR to prevent collisions while navigating the corridor.
2. It must identify litter items on the pavement and calculate their 3D relative bearing and distance without high CPU overhead.
3. Wheel odometry drifts over time (especially on Mecanum rollers during lateral strafing), requiring yaw drift stabilization.

## Decision
We implement `perception_node.py` combining two active perception channels:
1. **2D LiDAR Collision Cones & Avoidance Bias Processing**:
   - Rather than publishing a raw multi-element array requiring index unpacking downstream, `perception_node.py` processes dual 2D LiDAR scans (`/scan_front`, `/scan_rear`) into dedicated, decoupled topics (`std_msgs/msg/Float32`):
     - `/youbot/clearance/front`: Minimum obstacle distance in the forward collision cone ($[-30^\circ, +30^\circ]$). Unconstrained free space is published as `float('inf')`.
     - `/youbot/clearance/rear`: Minimum obstacle distance in the rear collision cone ($[-30^\circ, +30^\circ]$ relative to the rear sensor frame).
     - `/youbot/clearance/recommended_turn`: Lateral flank comparison evaluating left side clearance ($[+30^\circ, +90^\circ]$) versus right side clearance ($[-90^\circ, -30^\circ]$). Publishes `+1.0` (turn left into open space) or `-1.0` (turn right into open space) to provide immediate steering bias for reactive avoidance.
   - **Trigger**: Fully event-driven callbacks triggered immediately on scan arrival for minimum latency.
2. **Webots Camera with Native `Recognition` & Target Processing**:
   - **Driver Hardware Bridge Contract**: `youbot_driver.py` acts strictly as a sensor driver. It publishes all raw camera recognition objects as `geometry_msgs/msg/PoseArray` on `/camera/recognized_trash` in `camera_link` coordinates, with orientation converted from Webots axis-angle to valid unit quaternions. The driver does NOT perform model or semantic filtering.
   - **Perception Node Filtering & Locking**: `perception_node.py` subscribes to `/camera/recognized_trash`. It applies the static `camera_link` $\to$ `base_link` transform (`[0.280083, -0.005, -0.03]` m, pitch `0.0` rad), filters by ground-plane height (`z_ground = z_base + 0.095` in `[-0.05, 0.20]` m) and approach window (`x` in `[0.15, 4.0]` m, `|y| <= 2.0` m), and publishes the closest candidate (by 2D Euclidean distance `hypot(x, y)`) to `/youbot/trash_target` (`geometry_msgs/msg/Point`) only when a valid target exists (downstream nodes handle staleness/timeout).

*(Note: An earlier design included a wheel odometry + IMU gyro complementary filter. This was pruned after architectural review because the cleaner robot operates purely via closed-loop visual servoing and reactive LiDAR sectors; dead-reckoning odometry had no downstream subscribers).*

## Consequences
- **Pros**:
   - Direct alignment with concepts practiced in Assignment 2 (odometry & drift) and Assignment 3 (LiDAR sector processing & reactive avoidance).
   - Dedicated float topics simplify Behavior Tree condition evaluation: condition leaves can directly compare `/youbot/clearance/front` against a distance threshold without array indexing errors.
   - Providing `/youbot/clearance/recommended_turn` eliminates blind avoidance turning, ensuring the robot turns into the more open corridor flank.
   - Near-zero latency target tracking via Camera `Recognition`.
- **Cons**:
   - Mecanum wheel roller slippage causes minor longitudinal and lateral odometry drift; closed-loop camera tracking compensates for this during target approach.
   - Because `/camera/recognized_trash` uses `PoseArray`, litter classification is purely geometric (height and approach-window bounds) with no semantic/color label; any `Recognition`-enabled Webots object whose center falls in the ground-plane window would be treated as litter. This is sufficient for the controlled Iso-Roba PoC world, though a denser environment with low `Recognition`-enabled props would require semantic labeling (e.g., `vision_msgs`).

## Sources
- [Webots R2025a Camera Recognition Documentation](https://cyberbotics.com/doc/reference/camera#recognition)
- Course Assignment 2 (`Autumn-2026_Assignment_2/README.md`)
- Course Assignment 3 (`Autumn-2026_Assignment_3/src/hybrid_assignment/hybrid_assignment/task1_behaviours.py`)
