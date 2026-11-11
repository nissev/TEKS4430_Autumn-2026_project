---
author: Nisse + Antigravity
model: Gemini 3.8 Flash (Medium)
created: November 9, 2026 at 18:05
last_updated: November 11, 2026 at 16:15
---

# ADR-007: Pedestrian Zone & Street Crossing Exclusion (Known Safety Gap)

- **Status**: Accepted (deliberate scope exclusion)
- **Date**: 2026-11-09

## Context
In a pedestrian walking district such as *Iso Roobertinkatu*, pedestrian streets intersect with perpendicular vehicular traffic streets (e.g., *Annankatu* or *Fredrikinkatu*). Additionally, outdoor cafe seating areas and private shop entrances represent zones the clean-up robot should not enter.

The baseline sensing stack (2D LiDAR, wheel encoders, camera `Recognition`) detects geometry and labeled litter items, but lacks semantic segmentation to detect street crossing signs, pedestrian zone limits, or curb boundaries.

## Decision
**This PoC does not implement dynamic street-crossing or geofenced pedestrian zone exclusion.** The robot operates within the bounded simulation corridor of the pedestrian street.

Real-world deployment would require:
1. **Geofenced keep-in/keep-out polygons**: Restricting navigation to authorized pedestrian pavement zones using high-accuracy RTK-GPS or semantic visual SLAM.
2. **Pedestrian flow awareness**: Distinguishing between static street furniture and walking pedestrians to yield right-of-way.

## Consequences
- **Pros**: Keeps the PoC focused on the three core CPS architectural layers (LiDAR perception, Behavior Tree planning, and Holonomic PID + Arm manipulation).
- **Cons / residual risk**: In a physical deployment without physical perimeter barriers, the robot would risk wandering into vehicular intersections.

## Sources
- [ISO 13849-1: Safety of machinery – Safety-related parts of control systems](https://www.iso.org/standard/69883.html)
- City of Helsinki Pedestrian Street Regulations (*Kävelykadut ja aukiot*)
