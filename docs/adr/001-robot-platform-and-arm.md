---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:38
last_updated: November 11, 2026 at 16:15
---

# ADR-001: Robot Platform & Manipulation Hardware (KUKA YouBot)

- **Status**: Accepted (revised 2026-11-11: pivoted from custom 4WD off-road rover to native KUKA `Youbot.proto` for urban pedestrian environment)
- **Date**: 2026-11-08

## Context
Initial design iterations focused on a rough roadside environment with steep ditch slopes, prompting the evaluation of rugged outdoor platforms (such as `Koala`, `Pioneer 3-AT`, and `SummitXlSteel`). However, composite robot assembly (mounting an arm and custom bin inside extension slots) and off-road wheel friction/tipping physics introduced high simulation complexity that detracted from core Autonomous CPS coursework goals.

Refocusing the scenario on an urban Finnish walking district (inspired by *Iso Roobertinkatu*, Helsinki) eliminates off-road terrain traversal requirements while introducing realistic municipal service robotics challenges (maneuvering through pedestrian zones, picking up litter near benches and cafe terraces).

## Decision
We adopt the native **KUKA `Youbot`** (`Youbot.proto`, `name "youbot"`):
1. **Holonomic Base**: 4-wheel Mecanum drive allowing direct sideways translation ($v_y$) and rotation ($\omega_z$) for precise alignment with street litter.
2. **Integrated 5-DOF Arm**: Built-in `arm1` through `arm5` with parallel gripper (`finger::left`, `finger::right`).
3. **Rear Platform Bin**: Lightweight collection bin mounted directly on the YouBot's flat rear utility platform.
4. **Sensor Suite**: Front 2D LiDAR (`HokuyoURG04LX`), front-facing Camera with built-in `Recognition`, 4 Mecanum wheel encoders, and IMU.

## Consequences
- **Pros**:
  - Official, pre-validated Webots PROTO: no manual hierarchy composition or joint instability issues.
  - Direct 1:1 reuse of course Assignment 4 joint names (`arm1`–`arm5`), kinematic conventions, and gripper control.
  - Omnidirectional Mecanum drive makes parking and grasping alignment fast and accurate without complex skid-steer scrubbing.
- **Cons**:
  - Mecanum rollers require flat pavement; they cannot climb tall sidewalk curbs (the operational domain is strictly flat pedestrian walkways).
  - 5-DOF arm has shorter reach (~0.55 m) than large industrial arms, requiring the mobile base to park within ~0.25–0.35 m of target trash.

## Sources
- [Webots R2025a KUKA YouBot Documentation](https://cyberbotics.com/doc/guide/youbot)
- Course Assignment 4 (`youbot_arm_driver.py`)
