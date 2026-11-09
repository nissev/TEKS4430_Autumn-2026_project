---
author: Nisse + Antigravity
model: Claude Sonnet 5.5 (Medium)
created: November 9, 2026 at 18:05
last_updated: November 9, 2026 at 18:05
---

# ADR-007: Road/Hazard-Zone Exclusion Not Implemented (Known Safety Gap)

- **Status**: Accepted (deliberate scope exclusion)
- **Date**: 2026-11-09

## Context
The rover operates on a roadside verge next to a live road (see [ADR-002](002-lightweight-simulation-world.md)). A real unit that wandered onto the carriageway would endanger itself and traffic, so keeping off the road is a **critical safety function**.

The current sensing stack (2D LiDAR, IMU, wheel odometry, `Recognition` camera) sees only *geometry* and trash objects. It has no **semantics**: it cannot tell road from grass, and the simulated road is visual-only (same friction and collision as grass). The only boundary in the PoC is the perimeter wall of the Webots world, which is a test-bench artefact, not a safety mechanism.

## Decision
**This PoC does not implement road/hazard-zone exclusion.** Patrol, trash approach and obstacle avoidance are unconstrained with respect to the road. The PoC runs only in simulation under supervision, and results must not be read as safe for real roadside operation.

Reason: a reliable implementation needs semantic perception or georeferenced localisation, both outside the PoC scope (the GPS waypoint navigation is already scoped out in [`TASKS.md`](../../TASKS.md)).

## Options for a Future Implementation
| Option | Idea | Trade-off |
| :--- | :--- | :--- |
| **A. Geofence polygon** | Keep-out polygon from a georeferenced map; enforced against GPS/RTK + odometry pose. | Needs accurate absolute localisation (the scoped-out GPS item); odometry alone drifts. |
| **B. Semantic road detection** | Camera segmentation/classifier labels road vs. verge; project onto the ground plane. | Needs training data/compute; lighting and weather robustness. |
| **C. Geometric cues** | Detect kerb/ditch/shoulder edges from LiDAR and tilt profile. | Unreliable; verges differ widely and cues may be absent. |
| **D. Physical/infrastructure cue** | Fence, reflective markers or beacon line along the work zone. | Deployment effort; still needs detection logic. |

A realistic design combines A (primary) with B (cross-check).

## Future Architecture Sketch
- **Perception (Layer 1):** a `hazard_zone_node` publishing a hazard signed distance / flag (e.g. `/rover/hazard_zone`, distance to nearest road boundary and bearing).
- **Reasoning (Layer 2):** highest-priority reactive leaves in the BT, above `IsTiltCritical` and `IsObstacleTooClose`: `IsNearRoad` + `RetreatFromRoad`. Patrol and trash-target selection reject goals inside the hazard zone plus a margin (so trash on the carriageway is never approached).
- **Control (Layer 3):** `base_controller_node` clamps velocity commands that would cross the boundary.
- **Independent monitor:** a simple, separate node that stops the base if the pose is inside the hazard zone regardless of BT state (the failure fallback; ISO 13849-1 style diverse channel).

## Consequences
- **Pros:** keeps the PoC focused on the three CPS layers, terrain-adaptive control and the arm pick-and-place.
- **Cons / residual risk:** the rover can drive onto the road, and trash lying near or on the road would be pursued. Real deployment would require this layer, validated against the applicable machine-safety standards, before any field use.

## Sources
- [ISO 13849-1: Safety of machinery – Safety-related parts of control systems](https://www.iso.org/standard/69883.html) (general guidance for safety functions; applicability to this rover not assessed)
- [ISO 18497: Agricultural machinery and tractors – Safety of highly automated machinery](https://www.iso.org/standard/70089.html) (closest analogue for outdoor autonomous machinery; not assessed for this PoC)
