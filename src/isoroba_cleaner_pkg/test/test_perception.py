"""
test_perception.py — Unit tests for perception sector extraction logic.
Can be executed with pytest without requiring ROS 2 middleware or Webots.
"""

import math
from types import SimpleNamespace
from isoroba_cleaner_pkg.perception_node import (
    extract_minimum_range_in_angular_window,
    transform_camera_point_to_base_link,
    select_closest_ground_trash_target,
    FORWARD_CONE_HALF_ANGLE_RADIANS,
    SIDE_SECTOR_OUTER_ANGLE_RADIANS,
    CAMERA_OFFSET_X_METERS,
    CAMERA_OFFSET_Y_METERS,
    CAMERA_OFFSET_Z_METERS,
    BASE_LINK_HEIGHT_ABOVE_GROUND_METERS,
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


def make_camera_pose(x_meters: float, y_meters: float, z_meters: float):
    """Generates a synthetic Pose-like object in camera_link."""
    return SimpleNamespace(position=SimpleNamespace(x=x_meters, y=y_meters, z=z_meters))


def camera_z_for_ground_height(ground_height_meters: float) -> float:
    """Computes camera_link z corresponding to a given height above pavement (assuming pitch=0)."""
    base_z = ground_height_meters - BASE_LINK_HEIGHT_ABOVE_GROUND_METERS
    return base_z - CAMERA_OFFSET_Z_METERS


def test_transform_camera_point_to_base_link():
    """Verify static offset and pitch transform from camera_link to base_link."""
    base_x, base_y, base_z = transform_camera_point_to_base_link(1.0, 0.2, 0.0, pitch_radians=0.0)
    assert math.isclose(base_x, 1.0 + CAMERA_OFFSET_X_METERS, abs_tol=1e-6)
    assert math.isclose(base_y, 0.2 + CAMERA_OFFSET_Y_METERS, abs_tol=1e-6)
    assert math.isclose(base_z, CAMERA_OFFSET_Z_METERS, abs_tol=1e-6)


def test_trash_selection_returns_none_when_empty():
    """Verify None is returned when no camera recognition poses are present."""
    assert select_closest_ground_trash_target([]) is None


def test_trash_selection_filters_elevated_and_out_of_range_objects():
    """Verify elevated objects, behind-bumper objects, far objects, and NaNs are rejected."""
    valid_ground_z = camera_z_for_ground_height(0.05)
    elevated_z = camera_z_for_ground_height(0.55)  # e.g., PublicBin or Pedestrian

    poses = [
        make_camera_pose(1.0, 0.0, elevated_z),            # too high
        make_camera_pose(-0.25, 0.0, valid_ground_z),      # behind min forward range (base_x < 0.15)
        make_camera_pose(4.5, 0.0, valid_ground_z),        # beyond max forward range (base_x > 4.0)
        make_camera_pose(1.5, 2.5, valid_ground_z),        # beyond max lateral offset (|base_y| > 2.0)
        make_camera_pose(float("nan"), 0.0, valid_ground_z),
    ]
    assert select_closest_ground_trash_target(poses) is None


def test_trash_selection_picks_closest_valid_ground_target():
    """Verify the closest valid ground-plane litter target is selected by planar distance."""
    can_z = camera_z_for_ground_height(0.061)
    apple_z = camera_z_for_ground_height(0.030)
    elevated_z = camera_z_for_ground_height(0.80)

    poses = [
        make_camera_pose(0.5, 0.0, elevated_z),   # closer, but elevated -> ignored
        make_camera_pose(2.5, 0.4, can_z),        # valid, farther
        make_camera_pose(1.2, -0.3, apple_z),     # valid, closest
    ]
    target = select_closest_ground_trash_target(poses)
    assert target is not None
    assert math.isclose(target[0], 1.2 + CAMERA_OFFSET_X_METERS, abs_tol=1e-6)
    assert math.isclose(target[1], -0.3 + CAMERA_OFFSET_Y_METERS, abs_tol=1e-6)
    assert math.isclose(target[2] + BASE_LINK_HEIGHT_ABOVE_GROUND_METERS, 0.030, abs_tol=1e-6)


def test_trash_selection_filters_by_mock_semantic_model_name():
    """Verify only 'biscuit box' detections are accepted when recognized_models is provided."""
    ground_z = camera_z_for_ground_height(0.030)
    poses = [
        make_camera_pose(0.8, 0.1, ground_z),  # closer, but pedestrian foot -> rejected
        make_camera_pose(1.5, -0.2, ground_z), # biscuit box -> accepted
    ]
    models = ("pedestrian", "biscuit box")
    target = select_closest_ground_trash_target(poses, recognized_models=models)
    assert target is not None
    assert math.isclose(target[0], 1.5 + CAMERA_OFFSET_X_METERS, abs_tol=1e-6)


if __name__ == "__main__":
    test_free_space_returns_infinity()
    test_forward_obstacle_detected()
    test_obstacle_outside_forward_cone_ignored()
    test_transform_camera_point_to_base_link()
    test_trash_selection_returns_none_when_empty()
    test_trash_selection_filters_elevated_and_out_of_range_objects()
    test_trash_selection_picks_closest_valid_ground_target()
    print("All unit tests passed successfully!")
