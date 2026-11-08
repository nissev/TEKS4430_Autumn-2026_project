---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 8, 2026 at 11:38
---

# ADR-002: Lightweight Simulation Environment Design

- **Status**: Accepted
- **Date**: 2026-11-08

## Context
The simulation must represent a rough roadside environment (slopes, ditches, bumps, rocks, and trash items) to trigger genuine physical disturbances (IMU pitch/roll tilt and wheel-slip odometry drift). However, the development machine is a laptop with limited CPU/GPU resources, and Webots' procedural `UnevenTerrain` (`ElevationGrid` triangle mesh) causes heavy ODE physics collision overhead when interacting with 6 wheels and dynamic objects.

## Decision
We will construct a **Lightweight Primitive Roadside Arena** (`src/worlds/roadside_arena.wbt`) using fewer than 20 primitive collision geometries:
- A flat base `Plane` / `RectangleArena` with matte `PBRAppearance` (no heavy image textures or complex shadow maps).
- Tilted `Box` primitives forming roadside ditches, shoulder slopes ($10^\circ$–$22^\circ$), and speed-bump ridges.
- Low-poly `Cylinder` and `Box` primitives representing impassable rocks/boulders.
- Colored `Box` primitives with `recognitionColors` representing collectible trash items.

## Consequences
- **Pros**:
  - Runs at high, stable FPS on integrated laptop graphics and CPU.
  - Provides deterministic, reproducible slopes for testing IMU rollover protection and terrain-adaptive speed scaling.
  - Flat/planar local contact surfaces make 5-DOF arm grasping predictable.
- **Cons**:
  - Visually simpler than a high-polygon heightmap mesh.

## Sources
- [Webots R2025a Speeding Up the Simulation Guide](https://cyberbotics.com/doc/guide/speeding-up-the-simulation)
