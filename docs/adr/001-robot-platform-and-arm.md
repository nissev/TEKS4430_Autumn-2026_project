---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 10, 2026 at 18:57
---

# ADR-001: Robot Platform & Manipulation Hardware

- **Status**: Accepted (revised 2026-11-10: `Koala.proto` replaced by `Pioneer3at.proto` after early M1.4 PROTO inspection)
- **Date**: 2026-11-08

## Context
The project requires a robot platform capable of navigating rough roadside terrain (slopes, bumps, ditches, and scattered rocks) while physically collecting trash items and storing them in an onboard bin.

Indoor platforms like `TurtleBot3Burger` (tiny caster wheels) and standard `Youbot` (small Mecanum rollers) easily high-center or wedge on uneven outdoor geometry in Webots. We initially selected K-Team's 6-wheeled `Koala` (`Koala.proto`), but during the M1.4 "test early" verification we discovered that Webots R2025a's `Koala.proto` is an older **kinematic-only** PROTO: neither its chassis nor its 6 wheels contain `Physics` nodes (4 of the 6 wheels are static `Solid`s without joints). Without ODE physics on the base and wheels, the robot cannot follow 3D `ElevationGrid` slopes under gravity (so `InertialUnit` roll/pitch stays zero) and cannot stably support the dynamic 5.8 kg `Youbot` arm. We also evaluated `Clearpath Moose` (`Moose.proto`), which was rejected because its 1.14 m deck height is far too tall for the 0.46 m `Youbot` arm to reach the ground.

## Decision
We will use the **Adept `Pioneer 3-AT` 4-Wheel-Drive (4WD) All-Terrain Rover** (`Pioneer3at.proto`, $0.508 \times 0.497 \times 0.277\text{ m}$, $12\text{ kg}$ chassis, $r = 0.11\text{ m}$ outdoor tires) as the mobile base (`name "pioneer_rover"`) and extend its `extensionSlot` with:
1. **Extracted KUKA `Youbot` 5-DOF Manipulator Arm** (`arm1`–`arm5` + parallel gripper `finger::left`, `finger::right`) mounted on a **low front-bumper bracket** (`translation 0.24 0 -0.05` relative to `extensionSlot`, placing the arm base ~0.08 m above ground so the 0.46 m arm easily reaches both the ground in front and the onboard bin behind it).
2. **Onboard Front-Top Deck Trash Collection Bin** constructed from lightweight `Solid` wall primitives (`translation 0.02 0 0.17`, ~0.22 m behind the arm base, well within the arm's ~0.31 m backward reach when `arm1` rotates to its limit, while keeping the rear deck clear for the rear 2D LiDAR).
3. **Two `RobotisLds01` 360° 2D LiDARs** (`lds_front` and `lds_rear`), front **`Camera` with `Recognition`**, and **`InertialUnit` + `Gyro` + `Accelerometer`**.

## Consequences
- **Pros**:
  - Native ODE physics on the 12 kg chassis and all 4 independently motorized rugged outdoor wheels (`front left wheel`, `back left wheel`, `front right wheel`, `back right wheel` + position sensors), handling the roadside `ElevationGrid` slopes and bumps realistically without tipping under the arm's weight.
  - 100% compatibility with Assignment 4's 5-DOF arm joint naming (`arm1`–`arm5`), kinematics, and PID/FSM control logic, and Assignments 2–3's `RobotisLds01` 2D LiDAR perception.
  - Low front-bumper arm bracket + front-top deck bin guarantees reliable reach to both ground-level trash and the onboard collection bin.
- **Cons**:
  - Requires assembling the `Youbot` arm node hierarchy and front-bumper mounting bracket inside `Pioneer3at.extensionSlot` in the `.wbt` world file.

## Sources
- [Webots R2025a Adept Pioneer 3-AT Documentation](https://cyberbotics.com/doc/guide/pioneer-3at)
- [Webots R2025a KUKA YouBot Documentation](https://cyberbotics.com/doc/guide/youbot)
- Course Assignment 4 (`youbot_arm_driver.py`)
