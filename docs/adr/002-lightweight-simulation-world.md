---
author: Nisse + Antigravity
model: Gemini 4 Argon (Medium)
created: November 8, 2026 at 11:38
last_updated: November 21, 2026 at 15:05
---

# ADR-002: Lightweight Simulation World (Iso-Roba Pedestrian Street)

- **Status**: Accepted (revised 2026-11-11: replaced `ElevationGrid` roadside ditch with flat pedestrian street inspired by Iso Roobertinkatu)
- **Date**: 2026-11-08

## Context
The initial simulation world modeled an off-road Finnish rural roadside with an uneven `ElevationGrid` (slopes, mounds, and ditches). This required heavy ODE mesh contact physics and caused 2D LiDAR to produce false obstacle readings when the robot pitched on slopes.

In urban service robotics, pedestrian precincts like *Iso Roobertinkatu* ("Iso-Roba") in Helsinki are paved, flat walking streets flanked by building facades and furnished with outdoor cafes, benches, and trash bins.

## Decision
The world `src/worlds/isoroba_arena.wbt` models an **Iso-Roba style pedestrian walking district**:
- **Ground**: Flat paved stone/cobblestone surface (`20.4 m x 7 m` `Box` with `Pavement` appearance, eliminating `ElevationGrid` computation).
- **Boundaries**: Continuous multi-story building facades (`SimpleBuilding`) along the north and south edges plus east/west boundary walls.
- **Urban Furniture & Obstacles**:
  - Street park benches (`Bench`: `bench_north`, `bench_south_1`, `bench_south_2`).
  - Outdoor cafe terrace barriers (`UrbanFence`: `cafe_fence_west`, `cafe_fence_front_1`, `cafe_fence_front_2`, `cafe_fence_east`).
  - Public street litter bin (`PublicBin`) and static pedestrians (`Pedestrian`: `pedestrian_1`, `pedestrian_2`).
- **Litter Items**: Three `BiscuitBox` objects (`litter_box_1`..`3`, `size 0.075 x 0.042 x 0.05 m`, `mass 0.1 kg`) with native Webots `Recognition` tags placed on the pavement.
- **Depot**: Marked green visual zone (`depot_zone` at `x = -8.48 m, y = 0.0 m`) where the robot starts and can return to empty its onboard bin.

## Consequences
- **Pros**:
  - Near-zero physics engine overhead: perfectly smooth ODE wheel contact with zero mesh faceting or glitching.
  - Clean 2D LiDAR readings: beam reflections come strictly from actual obstacles (walls, benches, terrace railings), avoiding ground-slope false alarms.
  - Highly realistic municipal CPS use-case with distinct, authentic Finnish cultural context.
- **Cons**:
  - Pedestrian obstacles are modeled as static objects for this baseline PoC (dynamic crowd simulation is scoped out).

## Sources
- Helsinki Urban Planning & Pedestrian Zones (*Iso Roobertinkatu*, Punavuori)
- [Webots R2025a Speeding Up the Simulation Guide](https://cyberbotics.com/doc/guide/speeding-up-the-simulation)
