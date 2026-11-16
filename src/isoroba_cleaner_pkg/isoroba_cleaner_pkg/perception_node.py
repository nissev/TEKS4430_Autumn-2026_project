"""
perception_node.py — Layer 1 Sensing & Perception Node for KUKA YouBot.

Implements M2.1 (2D LiDAR Clearance Sectors):
  1. Front collision cone clearance ([-30°, +30°]) -> /youbot/clearance/front
  2. Rear collision cone clearance ([-30°, +30°])  -> /youbot/clearance/rear
  3. Lateral steering recommendation (+1.0 left, -1.0 right) -> /youbot/clearance/recommended_turn

Prepares architecture for M2.2 (Camera Recognition & Target Tracking).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


# Sector angular thresholds (in radians)
FORWARD_CONE_HALF_ANGLE_RADIANS = math.radians(30.0)
SIDE_SECTOR_OUTER_ANGLE_RADIANS = math.radians(90.0)
REAR_CONE_HALF_ANGLE_RADIANS = math.radians(30.0)


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

        # ── Subscriptions ───────────────────────────────────────────
        self.front_scan_subscription = self.create_subscription(
            LaserScan, "/scan_front", self._front_scan_callback, 10
        )
        self.rear_scan_subscription = self.create_subscription(
            LaserScan, "/scan_rear", self._rear_scan_callback, 10
        )

        self.get_logger().info(
            "PerceptionNode initialized. Monitoring /scan_front and /scan_rear."
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
