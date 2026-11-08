---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 8, 2026 at 11:38
---

# ADR-003: Sensing, Perception & State Estimation Pipeline

- **Status**: Accepted
- **Date**: 2026-11-08

## Context
Layer 1 of the CPS architecture requires observing the environment and interpreting noisy/drifting sensor data on rough terrain. Specifically:
1. The rover needs $360^\circ$ obstacle awareness to steer around rocks and safely reverse out of steep ditches/dead-ends.
2. It must identify collectible trash items and estimate their 3D relative position without lagging the laptop CPU.
3. Wheel encoders alone suffer from heading drift on slopes and bumps, requiring sensor fusion with an IMU.

## Decision
We will implement a unified `perception_node.py` combining three perception channels:
1. **360° 2D LiDAR (`RobotisLds01`) x2 (front + rear units)**: Each unit masks its self-occluded angles (arm, bin, chassis) and is partitioned into directional clearance sectors (`front`, `front_left`, `front_right`, `rear`) to support both forward obstacle avoidance and safe reversing.
2. **Webots Camera with Built-in `Recognition`**: Uses Webots' native object recognition node (`recognitionColors`) to extract 3D relative positions of trash items in the camera frame with near-zero CPU overhead, converting them into target distance and bearing (`/rover/trash_target`).
3. **IMU + 6-Wheel Encoder Complementary Filter**: Fuses high-frequency gyroscope yaw rate and wheel odometry (building on Assignment 2) to publish slip-compensated pose (`/rover/odom_filtered`), while the Webots `InertialUnit` provides roll and pitch angles directly (the filter is only needed for yaw) (`/rover/tilt_state`).
   - *Note*: Long-term outdoor **GPS localization** is documented in the system architecture as a future extension but scoped out of the local PoC arena.

## Consequences
- **Pros**:
  - Directly builds on course concepts from Assignment 2 (Complementary Filter & IMU/odom drift) and Assignment 3 (LiDAR sector perception).
  - Native Webots `Recognition` eliminates heavy OpenCV image processing loops while providing accurate 3D target coordinates for visual servoing.
- **Cons**:
  - Camera `Recognition` relies on simulator metadata rather than raw pixel segmentation.

## Sources
- [Webots R2025a Camera Recognition Documentation](https://cyberbotics.com/doc/reference/camera#recognition)
- Course Assignment 2 (`Autumn-2026_Assignment_2/README.md`)
