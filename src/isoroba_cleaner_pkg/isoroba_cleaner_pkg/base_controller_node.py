"""
base_controller_node.py — Layer 3 Holonomic Base PID Controller for KUKA YouBot.

Implements M3.1:
  - Sole publisher to /cmd_vel (geometry_msgs/msg/Twist).
  - Subscribes to /youbot/base_mode (IDLE, APPROACH, PATROL, AVOID).
  - In APPROACH mode, runs a 3-axis Holonomic PID (linear_x, linear_y, angular_z)
    on /youbot/trash_target (base_link frame) to park at pick standoff distance
    and publishes /youbot/base_parked (std_msgs/msg/Bool).
  - In PATROL and AVOID modes, forwards /youbot/bt_cmd_vel (with timeout safety).
"""

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


VALID_BASE_MODES = frozenset({"IDLE", "APPROACH", "PATROL", "AVOID"})

# Control timing & timeouts
CONTROL_PERIOD_SECONDS = 0.05  # 20 Hz
COMMAND_TIMEOUT_SECONDS = 0.5

# Standoff geometry & parking tolerances in base_link
TARGET_STANDOFF_X_METERS = 0.45
TOLERANCE_X_METERS = 0.02
TOLERANCE_Y_METERS = 0.02
TOLERANCE_THETA_RADIANS = 0.05
PARKED_CYCLES_REQUIRED = 5

# Holonomic PID gains (Kp, Ki, Kd), integral clamp, and output velocity limits
LINEAR_X_GAINS = (0.8, 0.05, 0.15)
LINEAR_Y_GAINS = (0.9, 0.05, 0.15)
ANGULAR_Z_GAINS = (1.2, 0.0, 0.15)
INTEGRAL_CLAMP = 0.5
MAX_LINEAR_X_MPS = 0.25
MAX_LINEAR_Y_MPS = 0.20
MAX_ANGULAR_Z_RADPS = 0.60


def clamp_value(value: float, minimum_value: float, maximum_value: float) -> float:
    """Clamps value into [minimum_value, maximum_value]."""
    return max(minimum_value, min(maximum_value, value))


class PIDAxis:
    """Single-axis PID controller with integral anti-windup and output clamping."""

    def __init__(
        self,
        gains: Tuple[float, float, float],
        output_limit: float,
        integral_limit: float = INTEGRAL_CLAMP,
    ) -> None:
        self.proportional_gain, self.integral_gain, self.derivative_gain = gains
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.integral_error = 0.0
        self.previous_error: Optional[float] = None

    def reset(self) -> None:
        """Clears accumulated integral and previous derivative state."""
        self.integral_error = 0.0
        self.previous_error = None

    def compute(self, error: float, delta_time_seconds: float) -> float:
        """Computes clamped PID output for the given error and time step."""
        if delta_time_seconds <= 0.0 or not math.isfinite(error):
            return 0.0

        self.integral_error = clamp_value(
            self.integral_error + (error * delta_time_seconds),
            -self.integral_limit,
            self.integral_limit,
        )
        derivative_error = (
            0.0
            if self.previous_error is None
            else (error - self.previous_error) / delta_time_seconds
        )
        self.previous_error = error

        raw_output = (
            (self.proportional_gain * error)
            + (self.integral_gain * self.integral_error)
            + (self.derivative_gain * derivative_error)
        )
        return clamp_value(raw_output, -self.output_limit, self.output_limit)


def compute_approach_errors(
    target_x_meters: float,
    target_y_meters: float,
    standoff_x_meters: float = TARGET_STANDOFF_X_METERS,
) -> Tuple[float, float, float]:
    """Computes (error_x, error_y, error_theta) in base_link for a ground target."""
    error_x_meters = target_x_meters - standoff_x_meters
    error_y_meters = target_y_meters
    error_theta_radians = math.atan2(target_y_meters, max(target_x_meters, 0.05))
    return error_x_meters, error_y_meters, error_theta_radians


class BaseControllerNode(Node):
    """ROS 2 node managing YouBot base velocity commands across mission modes."""

    def __init__(self) -> None:
        super().__init__("base_controller_node")

        self.linear_x_pid = PIDAxis(LINEAR_X_GAINS, MAX_LINEAR_X_MPS)
        self.linear_y_pid = PIDAxis(LINEAR_Y_GAINS, MAX_LINEAR_Y_MPS)
        self.angular_z_pid = PIDAxis(ANGULAR_Z_GAINS, MAX_ANGULAR_Z_RADPS)

        self.current_mode = "IDLE"
        self.consecutive_in_tolerance_cycles = 0
        self.latest_target: Optional[Point] = None
        self.latest_target_time_seconds: Optional[float] = None
        self.latest_bt_command: Optional[Twist] = None
        self.latest_bt_command_time_seconds: Optional[float] = None

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.base_parked_publisher = self.create_publisher(Bool, "/youbot/base_parked", 10)

        self.create_subscription(String, "/youbot/base_mode", self._base_mode_callback, 10)
        self.create_subscription(Point, "/youbot/trash_target", self._trash_target_callback, 10)
        self.create_subscription(Twist, "/youbot/bt_cmd_vel", self._bt_cmd_vel_callback, 10)

        self.create_timer(CONTROL_PERIOD_SECONDS, self._control_loop_callback)
        self.get_logger().info("base_controller_node initialized in IDLE mode.")

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _reset_pid_state(self) -> None:
        self.linear_x_pid.reset()
        self.linear_y_pid.reset()
        self.angular_z_pid.reset()
        self.consecutive_in_tolerance_cycles = 0

    def _base_mode_callback(self, mode_message: String) -> None:
        requested_mode = mode_message.data.strip().upper()
        if requested_mode not in VALID_BASE_MODES:
            self.get_logger().warn(f"Ignoring unknown base_mode: '{mode_message.data}'")
            return
        if requested_mode != self.current_mode:
            self.current_mode = requested_mode
            self._reset_pid_state()

    def _trash_target_callback(self, target_message: Point) -> None:
        self.latest_target = target_message
        self.latest_target_time_seconds = self._now_seconds()

    def _bt_cmd_vel_callback(self, twist_message: Twist) -> None:
        self.latest_bt_command = twist_message
        self.latest_bt_command_time_seconds = self._now_seconds()

    def _publish(
        self,
        velocity_x: float,
        velocity_y: float,
        angular_velocity_z: float,
        is_parked: bool,
    ) -> None:
        twist_message = Twist()
        twist_message.linear.x = float(velocity_x)
        twist_message.linear.y = float(velocity_y)
        twist_message.angular.z = float(angular_velocity_z)
        self.cmd_vel_publisher.publish(twist_message)

        parked_message = Bool()
        parked_message.data = bool(is_parked)
        self.base_parked_publisher.publish(parked_message)

    def _control_loop_callback(self) -> None:
        now_seconds = self._now_seconds()

        if self.current_mode == "IDLE":
            self._reset_pid_state()
            self._publish(0.0, 0.0, 0.0, False)
            return

        if self.current_mode == "APPROACH":
            target_fresh = (
                self.latest_target is not None
                and self.latest_target_time_seconds is not None
                and (now_seconds - self.latest_target_time_seconds) <= COMMAND_TIMEOUT_SECONDS
                and math.isfinite(self.latest_target.x)
                and math.isfinite(self.latest_target.y)
            )
            if not target_fresh:
                self._reset_pid_state()
                self._publish(0.0, 0.0, 0.0, False)
                return

            error_x, error_y, error_theta = compute_approach_errors(
                self.latest_target.x, self.latest_target.y
            )
            within_tolerance = (
                abs(error_x) <= TOLERANCE_X_METERS
                and abs(error_y) <= TOLERANCE_Y_METERS
                and abs(error_theta) <= TOLERANCE_THETA_RADIANS
            )
            self.consecutive_in_tolerance_cycles = (
                self.consecutive_in_tolerance_cycles + 1 if within_tolerance else 0
            )
            if self.consecutive_in_tolerance_cycles >= PARKED_CYCLES_REQUIRED:
                self.linear_x_pid.reset()
                self.linear_y_pid.reset()
                self.angular_z_pid.reset()
                self._publish(0.0, 0.0, 0.0, True)
                return

            self._publish(
                self.linear_x_pid.compute(error_x, CONTROL_PERIOD_SECONDS),
                self.linear_y_pid.compute(error_y, CONTROL_PERIOD_SECONDS),
                self.angular_z_pid.compute(error_theta, CONTROL_PERIOD_SECONDS),
                False,
            )
            return

        # PATROL or AVOID: forward /youbot/bt_cmd_vel if fresh, otherwise stop safely.
        self._reset_pid_state()
        bt_fresh = (
            self.latest_bt_command is not None
            and self.latest_bt_command_time_seconds is not None
            and (now_seconds - self.latest_bt_command_time_seconds) <= COMMAND_TIMEOUT_SECONDS
        )
        if not bt_fresh:
            self._publish(0.0, 0.0, 0.0, False)
            return

        self._publish(
            self.latest_bt_command.linear.x,
            self.latest_bt_command.linear.y,
            self.latest_bt_command.angular.z,
            False,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    base_controller_node = BaseControllerNode()
    try:
        rclpy.spin(base_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        base_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
