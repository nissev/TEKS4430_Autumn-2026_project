---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 8, 2026 at 11:38
last_updated: November 11, 2026 at 16:15
---

# ADR-002: Lightweight Simulation World (Iso-Roba Pedestrian Street)

- **Status**: Accepted (revised 2026-11-11: replaced `ElevationGrid` roadside ditch with flat pedestrian street inspired by Iso Roobertinkatu)
- **Date**: 2026-11-08

## Context
The initial simulation world modeled an off-road Finnish rural roadside with an uneven `ElevationGrid` (slopes, mounds, and ditches). This required heavy ODE mesh contact physics and caused 2D LiDAR to produce false obstacle readings when the robot pitched on slopes.

In urban service robotics, pedestrian precincts like *Iso Roobertinkatu* ("Iso-Roba") in Helsinki are paved, flat walking streets flanked by building facades and furnished with outdoor cafes, benches, bicycle racks, and trash bins.

## Decision
The world `src/worlds/isoroba_arena.wbt` models an **Iso-Roba style pedestrian walking district**:
- **Ground**: Flat paved stone/cobblestone surface (single flat `Plane` or low-poly mesh, eliminating `ElevationGrid` computation).
- **Boundaries**: Continuous multi-story building facade boxes on both sides representing shopfronts and apartment buildings.
- **Urban Furniture & Obstacles**:
  - Street park benches (metal/wood benches).
  - Bicycle racks with parked bicycles.
  - Outdoor cafe terrace boundaries (low railings, tables, and chairs).
  - Street lamp posts and public litter bins.
- **Litter Items**: Small colored `Box`/`Cylinder` objects with `recognitionColors` (representing discarded coffee cups, soda cans, and fast-food containers) placed on the pavement.
- **Depot**: Marked visual zone along the corridor where the robot docks to empty its onboard bin.

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
