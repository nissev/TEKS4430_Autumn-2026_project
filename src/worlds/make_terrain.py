"""Rewrite DEF TERRAIN ElevationGrid in roadside_arena.wbt (everything else in the .wbt is edited by hand).
Terrain = the road cross-section PROFILE (y, height) repeated along the road (x) + seeded random bumps. Run: python make_terrain.py"""
import math, pathlib, random, re

xDimension, yDimension = 61, 35               # ElevationGrid: number of grid points along x / y
xSpacing = ySpacing = 0.5                     # ElevationGrid: distance between grid points (m)
translation_x, translation_y = -15.0, -11.0   # terrain Solid "translation" (x, y of first grid corner)
PROFILE = [(-11, 0.6), (-3, 0.6),             # road + shoulder
           (0, -0.4), (1, -0.4),              # ~18 deg slope down to ditch floor
           (4, 0.4), (6, 0.4)]                # ~15 deg back slope, forest floor
SHOULDER_Y, FADE_WIDTH = PROFILE[1][0], 0.5

random.seed(4)                                # change the seed for a different random terrain
WAVES = [(random.uniform(0.02, 0.05), random.uniform(0.4, 1.5), random.uniform(0.4, 1.5), random.uniform(0, 6.3)) for _ in range(6)]
MOUNDS = [(random.uniform(-14, 11), random.uniform(-2, 3.5), random.uniform(0.08, 0.2), random.uniform(0.4, 0.7)) for _ in range(12)]  # x, y, height, radius

def bumps(x, y):
    """Smooth random roughness on the grass (not on the road), fading in over FADE_WIDTH from the shoulder."""
    z = sum(a * math.sin(kx * x + phase) * math.sin(ky * y + phase) for a, kx, ky, phase in WAVES)
    z += sum(h * math.exp(-((x - mx) ** 2 + (y - my) ** 2) / (2 * r * r)) for mx, my, h, r in MOUNDS)
    return z * min(1.0, max(0.0, (y - SHOULDER_Y) / FADE_WIDTH))

def profile_height(y):
    if y <= PROFILE[0][0]: return PROFILE[0][1]
    for (y1, z1), (y2, z2) in zip(PROFILE, PROFILE[1:]):
        if y1 <= y <= y2:
            return z1 + (z2 - z1) * (y - y1) / (y2 - y1)
    return PROFILE[-1][1]

height = [f"{profile_height(translation_y + j * ySpacing) + bumps(translation_x + i * xSpacing, translation_y + j * ySpacing):.3f}"
          for j in range(yDimension) for i in range(xDimension)]
grid = (f"DEF TERRAIN ElevationGrid {{\n        xDimension {xDimension}\n        xSpacing {xSpacing}\n"
        f"        yDimension {yDimension}\n        ySpacing {ySpacing}\n        height [ {' '.join(height)} ]\n      }}")
wbt = pathlib.Path(__file__).with_name("roadside_arena.wbt")
wbt.write_text(re.sub(r"DEF TERRAIN ElevationGrid \{[^}]*\}", grid, wbt.read_text(), count=1), newline="\n")
