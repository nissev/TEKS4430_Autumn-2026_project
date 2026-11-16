"""
perception_node.py — Layer 1 Sensing & Perception Node for KUKA YouBot.

Implements M2.1 (2D LiDAR Clearance Sectors):
  1. Front collision cone clearance ([-30°, +30°]) -> /youbot/clearance/front
  2. Rear collision cone clearance ([-30°, +30°])  -> /youbot/clearance/rear
  3. Lateral steering recommendation (+1.0 left, -1.0 right) -> /youbot/clearance/recommended_turn

Implements M2.2 (Camera Recognition & Target Tracking):
  - Subscribes to /camera/recognized_trash (PoseArray in camera_link)
  - Transforms to base_link, filters by ground-plane height and approach reachability
  - Publishes closest valid target (x, y, z) on /youbot/trash_target (Point)
"""

import math
from typing import Iterable, Optional, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


# Sector angular thresholds (in radians)
FORWARD_CONE_HALF_ANGLE_RADIANS = math.radians(30.0)
SIDE_SECTOR_OUTER_ANGLE_RADIANS = math.radians(90.0)
REAR_CONE_HALF_ANGLE_RADIANS = math.radians(30.0)

# Camera mounting in base_link (matching isoroba_arena.wbt)
CAMERA_OFFSET_X_METERS = 0.280083
CAMERA_OFFSET_Y_METERS = -0.005
CAMERA_OFFSET_Z_METERS = -0.03
CAMERA_PITCH_RADIANS = 0.0
BASE_LINK_HEIGHT_ABOVE_GROUND_METERS = 0.095

# Ground-plane & approach reachability filter bounds (in base_link / ground-relative)
MIN_GROUND_HEIGHT_METERS = -0.05
MAX_GROUND_HEIGHT_METERS = 0.20
MIN_FORWARD_RANGE_METERS = 0.15
MAX_FORWARD_RANGE_METERS = 4.0
MAX_LATERAL_OFFSET_METERS = 2.0


def extract_minimum_range_in_angular_window(
    scan_message: LaserScan,
    minimum_angle_radians: float,
    maximum_angle_radians: float,
) -> float:
    """
    Computes the minimum valid range within an angular window [minimum_angle, maximum_angle].
    Returns float('inf') if no valid obstacles are detected within the window.
    """
    minimum_distance = float("inf")

    for ray_index, range_reading in enumerate(scan_message.ranges):
        # Calculate ray angle wrapped to [-pi, +pi]
        raw_ray_angle = scan_message.angle_min + (ray_index * scan_message.angle_increment)
        ray_angle_radians = math.atan2(math.sin(raw_ray_angle), math.cos(raw_ray_angle))

        if not (minimum_angle_radians <= ray_angle_radians <= maximum_angle_radians):
            continue

        if math.isnan(range_reading) or math.isinf(range_reading):
            continue

        if range_reading < scan_message.range_min or range_reading > scan_message.range_max:
            continue

        if range_reading < minimum_distance:
            minimum_distance = range_reading

    return minimum_distance


def transform_camera_point_to_base_link(
    camera_x_meters: float,
    camera_y_meters: float,
    camera_z_meters: float,
    offset_x_meters: float = CAMERA_OFFSET_X_METERS,
    offset_y_meters: float = CAMERA_OFFSET_Y_METERS,
    offset_z_meters: float = CAMERA_OFFSET_Z_METERS,
    pitch_radians: float = CAMERA_PITCH_RADIANS,
) -> Tuple[float, float, float]:
    """Transforms a 3D point from camera_link into base_link using static mounting offsets and pitch."""
    cosine_pitch = math.cos(pitch_radians)
    sine_pitch = math.sin(pitch_radians)
    base_x_meters = (camera_x_meters * cosine_pitch) + (camera_z_meters * sine_pitch) + offset_x_meters
    base_y_meters = camera_y_meters + offset_y_meters
    base_z_meters = (-camera_x_meters * sine_pitch) + (camera_z_meters * cosine_pitch) + offset_z_meters
    return base_x_meters, base_y_meters, base_z_meters


def select_closest_ground_trash_target(
    recognized_poses: Iterable,
    offset_x_meters: float = CAMERA_OFFSET_X_METERS,
    offset_y_meters: float = CAMERA_OFFSET_Y_METERS,
    offset_z_meters: float = CAMERA_OFFSET_Z_METERS,
    pitch_radians: float = CAMERA_PITCH_RADIANS,
    base_height_meters: float = BASE_LINK_HEIGHT_ABOVE_GROUND_METERS,
    min_ground_height_meters: float = MIN_GROUND_HEIGHT_METERS,
    max_ground_height_meters: float = MAX_GROUND_HEIGHT_METERS,
    min_forward_range_meters: float = MIN_FORWARD_RANGE_METERS,
    max_forward_range_meters: float = MAX_FORWARD_RANGE_METERS,
    max_lateral_offset_meters: float = MAX_LATERAL_OFFSET_METERS,
) -> Optional[Tuple[float, float, float]]:
    """
    Transforms camera recognition poses to base_link, filters for ground-level litter
    within approach reach, and returns the closest (x, y, z) in base_link or None.
    """
    closest_target: Optional[Tuple[float, float, float]] = None
    closest_distance_meters = float("inf")

    for pose in recognized_poses:
        position = pose.position
        if not (
            math.isfinite(position.x)
            and math.isfinite(position.y)
            and math.isfinite(position.z)
        ):
            continue

        base_x, base_y, base_z = transform_camera_point_to_base_link(
            position.x,
            position.y,
            position.z,
            offset_x_meters=offset_x_meters,
            offset_y_meters=offset_y_meters,
            offset_z_meters=offset_z_meters,
            pitch_radians=pitch_radians,
        )

        ground_height = base_z + base_height_meters
        if not (min_ground_height_meters <= ground_height <= max_ground_height_meters):
            continue
        if not (min_forward_range_meters <= base_x <= max_forward_range_meters):
            continue
        if abs(base_y) > max_lateral_offset_meters:
            continue

        planar_distance = math.hypot(base_x, base_y)
        if planar_distance < closest_distance_meters:
            closest_distance_meters = planar_distance
            closest_target = (base_x, base_y, base_z)

    return closest_target


class PerceptionNode(Node):
    """
    Processes front and rear 2D LiDAR scans into focused clearance cones
    and lateral steering bias for reactive collision avoidance.
    """

    def __init__(self) -> None:
        super().__init__("perception_node")

        # ── Publishers ──────────────────────────────────────────────
        self.front_clearance_publisher = self.create_publisher(
            Float32, "/youbot/clearance/front", 10
        )
        self.rear_clearance_publisher = self.create_publisher(
            Float32, "/youbot/clearance/rear", 10
        )
        self.recommended_turn_publisher = self.create_publisher(
            Float32, "/youbot/clearance/recommended_turn", 10
        )
        self.trash_target_publisher = self.create_publisher(
            Point, "/youbot/trash_target", 10
        )

        # ── Subscriptions ───────────────────────────────────────────
        self.front_scan_subscription = self.create_subscription(
            LaserScan, "/scan_front", self._front_scan_callback, 10
        )
        self.rear_scan_subscription = self.create_subscription(
            LaserScan, "/scan_rear", self._rear_scan_callback, 10
        )
        self.recognized_trash_subscription = self.create_subscription(
            PoseArray, "/camera/recognized_trash", self._recognized_trash_callback, 10
        )

        self.get_logger().info(
            "PerceptionNode initialized. Monitoring /scan_front, /scan_rear, and /camera/recognized_trash."
        )

    def _front_scan_callback(self, scan_message: LaserScan) -> None:
        """
        Evaluates forward collision cone ([-30°, +30°]) and compares lateral flank clearances
        ([+30°, +90°] vs [-90°, -30°]) to recommend avoidance turn direction.
        """
        # 1. Forward cone clearance
        minimum_front_distance = extract_minimum_range_in_angular_window(
            scan_message,
            minimum_angle_radians=-FORWARD_CONE_HALF_ANGLE_RADIANS,
            maximum_angle_radians=FORWARD_CONE_HALF_ANGLE_RADIANS,
        )

        front_message = Float32()
        front_message.data = minimum_front_distance
        self.front_clearance_publisher.publish(front_message)

        # 2. Side flank comparison for recommended avoidance steering
        # Left side (+30° to +90°) vs Right side (-90° to -30°)
        left_side_clearance = extract_minimum_range_in_angular_window(
            scan_message,
            minimum_angle_radians=FORWARD_CONE_HALF_ANGLE_RADIANS,
            maximum_angle_radians=SIDE_SECTOR_OUTER_ANGLE_RADIANS,
        )
        right_side_clearance = extract_minimum_range_in_angular_window(
            scan_message,
            minimum_angle_radians=-SIDE_SECTOR_OUTER_ANGLE_RADIANS,
            maximum_angle_radians=-FORWARD_CONE_HALF_ANGLE_RADIANS,
        )

        # Positive angular z turn (+1.0) is counter-clockwise (turn left into open space).
        # Negative angular z turn (-1.0) is clockwise (turn right into open space).
        recommended_turn_direction = 1.0 if left_side_clearance >= right_side_clearance else -1.0

        recommended_turn_message = Float32()
        recommended_turn_message.data = recommended_turn_direction
        self.recommended_turn_publisher.publish(recommended_turn_message)

    def _rear_scan_callback(self, scan_message: LaserScan) -> None:
        """
        Evaluates rear collision cone ([-30°, +30°] relative to rear LiDAR frame).
        """
        minimum_rear_distance = extract_minimum_range_in_angular_window(
            scan_message,
            minimum_angle_radians=-REAR_CONE_HALF_ANGLE_RADIANS,
            maximum_angle_radians=REAR_CONE_HALF_ANGLE_RADIANS,
        )

        rear_message = Float32()
        rear_message.data = minimum_rear_distance
        self.rear_clearance_publisher.publish(rear_message)

    def _recognized_trash_callback(self, poses_message: PoseArray) -> None:
        """
        Transforms camera detections to base_link, filters by ground-plane height and
        approach reachability, and publishes the closest target on /youbot/trash_target.
        """
        closest_target = select_closest_ground_trash_target(poses_message.poses)
        if closest_target is None:
            return

        target_message = Point()
        target_message.x, target_message.y, target_message.z = closest_target
        self.trash_target_publisher.publish(target_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
