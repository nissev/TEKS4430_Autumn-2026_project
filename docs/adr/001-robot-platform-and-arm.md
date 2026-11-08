---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 8, 2026 at 11:38
---

# ADR-001: Robot Platform & Manipulation Hardware

- **Status**: Accepted
- **Date**: 2026-11-08

## Context
The project requires a robot platform capable of navigating rough roadside terrain (slopes, bumps, ditches, and scattered rocks) while physically collecting trash items and storing them in an onboard bin. Indoor platforms like `TurtleBot3Burger` (tiny caster wheels) and standard `Youbot` (small Mecanum rollers) easily high-center or wedge on uneven outdoor geometry in Webots.

## Decision
We will use the **K-Team `Koala` 6-Wheel-Drive (6WD) Rover** (`Koala.proto`) as the mobile base and extend its `extensionSlot` with:
1. **Extracted KUKA `Youbot` 5-DOF Manipulator Arm** (`arm1`–`arm5` + parallel gripper `finger::left`, `finger::right`) mounted on the front deck.
2. **Onboard Rear Trash Collection Bin** constructed from lightweight `Solid` wall primitives on the rear deck.

## Consequences
- **Pros**:
  - 6 rugged wheels handle roadside ramps, bumps, and ditches smoothly without high-centering.
  - 100% compatibility with Assignment 4's 5-DOF arm joint naming (`arm1`–`arm5`), kinematics, and PID control logic.
  - Self-contained trash collection: the arm picks up trash in front of the rover and swings $180^\circ$ to drop it directly into the rear onboard bin.
- **Cons**:
  - Requires assembling the `Youbot` arm node hierarchy inside `Koala.extensionSlot` in the `.wbt` world file.

## Sources
- [Webots R2025a K-Team Koala Documentation](https://cyberbotics.com/doc/guide/koala)
- [Webots R2025a KUKA YouBot Documentation](https://cyberbotics.com/doc/guide/youbot)
- Course Assignment 4 (`youbot_arm_driver.py`)
