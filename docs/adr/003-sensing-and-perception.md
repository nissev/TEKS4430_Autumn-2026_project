---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:38
last_updated: November 11, 2026 at 16:15
---

# ADR-003: Sensing & Perception Pipeline

- **Status**: Accepted (revised 2026-11-11: updated for flat urban sidewalk & YouBot sensors)
- **Date**: 2026-11-08

## Context
Layer 1 of the CPS architecture requires observing the urban pedestrian environment and interpreting sensor data to support collision avoidance and litter manipulation:
1. The robot must detect street obstacles (benches, terrace railings, lamp posts, bike racks) with 2D LiDAR to prevent collisions while navigating the corridor.
2. It must identify litter items on the pavement and calculate their 3D relative bearing and distance without high CPU overhead.
3. Wheel odometry drifts over time (especially on Mecanum rollers during lateral strafing), requiring yaw drift stabilization.

## Decision
We implement `perception_node.py` combining three perception channels:
1. **2D LiDAR Sector Processing**: The YouBot's front 2D LiDAR (`HokuyoURG04LX`) scan is partitioned into directional clearance sectors (`front`, `front_left`, `front_right`) to guide obstacle avoidance and docking clearance.
2. **Webots Camera with Native `Recognition`**: Uses Webots' built-in object recognition to extract 3D positions of litter targets in the camera frame, translating them into target coordinates (`/rover/trash_target`).
3. **Mecanum Odometry + Gyro Fusion**: Fuses 4-wheel Mecanum kinematic odometry with high-frequency Gyro yaw to maintain heading stability on the flat pavement (`/rover/odom_filtered`).
   - *Note*: Pitch/roll tilt monitoring is removed because the paved pedestrian street is flat.

## Consequences
- **Pros**:
  - Direct alignment with concepts practiced in Assignment 2 (odometry & drift) and Assignment 3 (LiDAR sector processing).
  - Near-zero latency target tracking via Camera `Recognition`.
- **Cons**:
  - Mecanum wheel roller slippage causes minor longitudinal and lateral odometry drift; closed-loop camera tracking compensates for this during target approach.

## Sources
- [Webots R2025a Camera Recognition Documentation](https://cyberbotics.com/doc/reference/camera#recognition)
- Course Assignment 2 (`Autumn-2026_Assignment_2/README.md`)
