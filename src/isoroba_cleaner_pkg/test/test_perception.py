"""
test_perception.py — Unit tests for perception sector extraction logic.
Can be executed with pytest without requiring ROS 2 middleware or Webots.
"""

import math
from types import SimpleNamespace
from isoroba_cleaner_pkg.perception_node import (
    extract_minimum_range_in_angular_window,
    FORWARD_CONE_HALF_ANGLE_RADIANS,
    SIDE_SECTOR_OUTER_ANGLE_RADIANS,
)


def create_synthetic_laserscan(total_rays: int = 240, field_of_view_deg: float = 240.0, default_range: float = 5.0):
    """Generates a synthetic LaserScan-like structure matching Hokuyo URG-04LX."""
    fov_radians = math.radians(field_of_view_deg)
    angle_min = -fov_radians / 2.0
    angle_max = fov_radians / 2.0
    angle_increment = fov_radians / (total_rays - 1)

    return SimpleNamespace(
        angle_min=angle_min,
        angle_max=angle_max,
        angle_increment=angle_increment,
        range_min=0.06,
        range_max=5.6,
        ranges=[default_range] * total_rays,
    )


def test_free_space_returns_infinity():
    """Verify that when all readings exceed range_max or are infinite, float('inf') is returned."""
    scan = create_synthetic_laserscan(default_range=float("inf"))
    clearance = extract_minimum_range_in_angular_window(
        scan,
        -FORWARD_CONE_HALF_ANGLE_RADIANS,
        FORWARD_CONE_HALF_ANGLE_RADIANS,
    )
    assert clearance == float("inf"), f"Expected inf, got {clearance}"


def test_forward_obstacle_detected():
    """Verify that an obstacle placed directly forward (0 rad) is reported."""
    scan = create_synthetic_laserscan(default_range=5.0)
    # Middle index corresponds to angle = 0.0 rad
    center_index = len(scan.ranges) // 2
    scan.ranges[center_index] = 1.25

    clearance = extract_minimum_range_in_angular_window(
        scan,
        -FORWARD_CONE_HALF_ANGLE_RADIANS,
        FORWARD_CONE_HALF_ANGLE_RADIANS,
    )
    assert math.isclose(clearance, 1.25, abs_tol=1e-3), f"Expected 1.25, got {clearance}"


def test_obstacle_outside_forward_cone_ignored():
    """Verify that an obstacle at 60 deg is ignored by the forward cone."""
    scan = create_synthetic_laserscan(default_range=5.0)
    # Place obstacle at ~ +60 degrees
    angle_target = math.radians(60.0)
    target_index = int(round((angle_target - scan.angle_min) / scan.angle_increment))
    scan.ranges[target_index] = 0.50

    forward_clearance = extract_minimum_range_in_angular_window(
        scan,
        -FORWARD_CONE_HALF_ANGLE_RADIANS,
        FORWARD_CONE_HALF_ANGLE_RADIANS,
    )
    # Forward cone should remain unobstructed
    assert math.isclose(forward_clearance, 5.0, abs_tol=1e-3)

    # Side sector [+30°, +90°] should detect it
    side_clearance = extract_minimum_range_in_angular_window(
        scan,
        FORWARD_CONE_HALF_ANGLE_RADIANS,
        SIDE_SECTOR_OUTER_ANGLE_RADIANS,
    )
    assert math.isclose(side_clearance, 0.50, abs_tol=1e-3)


if __name__ == "__main__":
    test_free_space_returns_infinity()
    test_forward_obstacle_detected()
    test_obstacle_outside_forward_cone_ignored()
    print("All unit tests passed successfully!")
