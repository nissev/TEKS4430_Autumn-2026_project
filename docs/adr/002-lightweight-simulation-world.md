---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 9, 2026 at 17:59
---

# ADR-002: Lightweight Simulation Environment Design

- **Status**: Accepted (revised 2026-11-09: primitive-box arena replaced by a coarse `ElevationGrid` roadside)
- **Date**: 2026-11-08

## Context
The simulation must represent a Finnish-style roadside (road, shoulder, grass slope, ditch, back slope, forest edge) with rocks and trash, so that real disturbances occur (IMU pitch/roll tilt and wheel-slip odometry drift). The development machine is a laptop with limited CPU/GPU resources. Webots' procedural `UnevenTerrain` (fine `ElevationGrid`) was judged too heavy for 6 wheels and dynamic objects, and an initial primitive-box ramp arena looked unlike a real roadside.

## Decision
The world `src/worlds/roadside_arena.wbt` is a hand-editable file; only its terrain height list is produced by the small script `src/worlds/make_terrain.py` (a road cross-section profile repeated along the road):
- **Terrain**: one **coarse `ElevationGrid`** (30 x 17 m, 0.5 m cells, about 2000 cells), static, with a cross-section profile: raised road bed, gravel shoulder, about 18° grass slope, ditch floor (floor about 1.0 m below the road surface), about 15° back slope, forest floor. The slopes provide the tilt and slip disturbances.
- **Road**: 7.5 m, two lanes with edge/centre lines, as visual-only boxes (no collision).
- **Obstacles**: low-poly `Box`/`Cylinder` rocks, one static parked car on the road, 12 visual-only low-poly spruce trees (no collision).
- **Trash**: small coloured `Box` solids with `recognitionColors` (`Recognition` target of the front camera).
- **Boundary**: four perimeter walls (forest, road barrier, two ends).
- **Depot**: flat visual patch in the ditch (stretch goal).
- Matte `PBRAppearance` only, no textures, shadows disabled. Cell size can be coarsened (edit `CELL`/`xSpacing` etc.) if FPS drops.

## Consequences
- **Pros**:
  - Recognisable roadside scene and deterministic, reproducible slopes for IMU rollover protection and terrain-adaptive speed scaling.
  - Height-field collision at about 2000 cells ran acceptably on the target laptop (visually verified by the author).
  - Slope angles are tuned by editing six numbers in `PROFILE`.
- **Cons**:
  - Slopes are faceted at 0.5 m cells.
  - The road is visual-only: the terrain beneath is grass-coloured and has the same friction.
  - Trees and road markings carry no semantics, so the rover cannot tell road from grass (see [ARCHITECTURE.md, Known Limitations](../ARCHITECTURE.md)).

## Sources
- [Webots R2025a Speeding Up the Simulation Guide](https://cyberbotics.com/doc/guide/speeding-up-the-simulation)
- [Webots ElevationGrid reference](https://cyberbotics.com/doc/reference/elevationgrid)
