"""
behavior_tree_node.py — Layer 2 Hybrid Behavior Tree (py_trees) for KUKA YouBot.

Implements M4.1 (Reactive Safety Leaves & Preemption):
  - Centralized RobotStateInterface & BehaviorTreeNode (10 Hz tick) managing ROS 2
    subscriptions (/youbot/clearance/front, /youbot/clearance/rear,
    /youbot/clearance/recommended_turn, /youbot/trash_target, /youbot/base_parked,
    /arm/pick_status, /wheel_odom_raw) and publishers (/youbot/base_mode,
    /youbot/bt_cmd_vel, /arm/pick_trigger, /arm/abort).
  - IsObstacleTooClose: checks front clearance (< 0.55 m, with 0.75 m exit
    hysteresis); while /arm/pick_status is BUSY, filters near-field self-arm
    reflections (< 0.40 m) while reacting to external hazards in [0.40 m, 0.55 m)
    or rear clearance (< 0.35 m).
  - AvoidObstacle: publishes /youbot/base_mode="AVOID" and /arm/abort=True;
    executes two-stage evasion (turns at 0.60 rad/s and Mecanum-strafes at
    0.14 m/s via recommended_turn when front < 0.75 m; reverses slowly at
    -0.12 m/s while turning and strafing when critically close, front < 0.35 m,
    and rear > 0.40 m); publishes /arm/abort=False on terminate() to unlatch
    arm_fsm_node.

Implements M4.2 (Deliberative Mission Leaves):
  - DetectAndApproachTrash: verifies fresh /youbot/trash_target (<= 0.5 s) and
    no active post-failure cooldown; drives via Holonomic Base PID (base_mode="APPROACH")
    until /youbot/base_parked is True (or fails on target loss / 60.0 s timeout).
  - ExecuteArmSubFSM: holds base in IDLE for 0.6 s settle dwell, triggers
    /arm/pick_trigger=True, performs a two-phase handshake waiting for
    /arm/pick_status=="BUSY" (up to 1.0 s) and then completion ("SUCCESS" or
    "FAILED" with a 3.0 s retry cooldown).
  - PatrolStreet: default sweeping corridor navigation (base_mode="PATROL",
    velocity_x=0.22 m/s with IMU-fused corridor heading and centerline regulation).
"""

from dataclasses import dataclass
import math
from typing import Callable, Optional, Tuple

import py_trees
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


# Behavior Tree tick frequency
TICK_RATE_HZ = 10.0
TICK_PERIOD_SECONDS = 1.0 / TICK_RATE_HZ

# M4.1 Reactive safety distance thresholds (meters)
FRONT_OBSTACLE_THRESHOLD_METERS = 0.55
FRONT_CLEAR_HYSTERESIS_METERS = 0.75
CRITICAL_FRONT_THRESHOLD_METERS = 0.35
ARM_SELF_OCCLUSION_DEADBAND_METERS = 0.40
REAR_SAFE_REVERSE_CLEARANCE_METERS = 0.40
REAR_BUSY_HAZARD_THRESHOLD_METERS = 0.35

# M4.1 Evasive velocity setpoints
AVOID_TURN_ANGULAR_VELOCITY_RADPS = 0.60
AVOID_STRAFE_VELOCITY_Y_MPS = 0.14
AVOID_REVERSE_VELOCITY_X_MPS = -0.12

# M4.2 Deliberative mission timing & velocity constants
TARGET_FRESHNESS_TIMEOUT_SECONDS = 0.5
MAX_APPROACH_LOCK_DISTANCE_METERS = 7.5
APPROACH_TIMEOUT_SECONDS = 60.0
PRE_PICK_SETTLE_DWELL_SECONDS = 0.6
ARM_TRIGGER_HANDSHAKE_TIMEOUT_SECONDS = 1.0
PICK_FAILURE_COOLDOWN_SECONDS = 3.0
PATROL_FORWARD_VELOCITY_X_MPS = 0.22
PATROL_HEADING_KP = 0.80
PATROL_MAX_ANGULAR_VELOCITY_RADPS = 0.35
PATROL_CENTERLINE_KY = 0.25
PATROL_MAX_CENTERLINE_VELOCITY_Y_MPS = 0.10
PATROL_CENTERLINE_ALIGN_THRESHOLD_RAD = 0.35


@dataclass
class RobotStateInterface:
    """
    Shared sensor state and actuator command callbacks used by py_trees leaves.
    Decouples leaf logic from a live ROS 2 executor for deterministic unit testing.
    """

    current_time_seconds: float = 0.0
    front_clearance_meters: Optional[float] = None
    rear_clearance_meters: Optional[float] = None
    recommended_turn_direction: float = 1.0
    trash_target: Optional[Tuple[float, float, float]] = None
    trash_target_time_seconds: Optional[float] = None
    base_parked: bool = False
    arm_pick_status: str = "IDLE"
    pick_cooldown_until_seconds: Optional[float] = None
    avoidance_active: bool = False
    latched_avoid_turn_direction: Optional[float] = None
    odom_x_meters: Optional[float] = None
    odom_y_meters: Optional[float] = None
    current_yaw_radians: Optional[float] = None

    publish_base_mode: Callable[[str], None] = lambda mode: None
    publish_bt_cmd_vel: Callable[[float, float, float], None] = (
        lambda velocity_x, velocity_y, angular_velocity_z: None
    )
    publish_arm_pick_trigger: Callable[[bool], None] = lambda trigger: None
    publish_arm_abort: Callable[[bool], None] = lambda abort: None


def is_trash_target_fresh(
    robot_state: RobotStateInterface,
    freshness_timeout_seconds: float = TARGET_FRESHNESS_TIMEOUT_SECONDS,
) -> bool:
    """Returns True when a finite trash target was received within freshness_timeout_seconds."""
    if (
        robot_state.trash_target is None
        or robot_state.trash_target_time_seconds is None
    ):
        return False
    target_x, target_y, target_z = robot_state.trash_target
    if not (
        math.isfinite(target_x)
        and math.isfinite(target_y)
        and math.isfinite(target_z)
    ):
        return False
    age_seconds = (
        robot_state.current_time_seconds - robot_state.trash_target_time_seconds
    )
    return 0.0 <= age_seconds <= freshness_timeout_seconds


def is_pick_cooldown_active(robot_state: RobotStateInterface) -> bool:
    """Returns True while the post-failed-pick retry cooldown is still active."""
    if robot_state.pick_cooldown_until_seconds is None:
        return False
    return robot_state.current_time_seconds < robot_state.pick_cooldown_until_seconds


def is_obstacle_hazard_detected(
    front_clearance_meters: Optional[float],
    rear_clearance_meters: Optional[float],
    arm_pick_status: str,
    avoidance_active: bool = False,
) -> bool:
    """
    Evaluates whether an obstacle hazard requires reactive preemption.
    - When arm_pick_status == "BUSY", ignores front readings < 0.40 m (self-arm
      lowering through lidar_front scan plane) while triggering on external
      front hazards in [0.40 m, 0.55 m) or rear hazards < 0.35 m.
    - When not BUSY, triggers on front < 0.55 m and applies hysteresis (0.75 m)
      while avoidance_active is True so the robot clears the obstacle cleanly.
    """
    is_arm_busy = arm_pick_status.strip().upper() == "BUSY"

    if is_arm_busy:
        front_hazard = (
            front_clearance_meters is not None
            and not math.isnan(front_clearance_meters)
            and ARM_SELF_OCCLUSION_DEADBAND_METERS
            <= front_clearance_meters
            < FRONT_OBSTACLE_THRESHOLD_METERS
        )
        rear_hazard = (
            rear_clearance_meters is not None
            and not math.isnan(rear_clearance_meters)
            and 0.0 < rear_clearance_meters < REAR_BUSY_HAZARD_THRESHOLD_METERS
        )
        return front_hazard or rear_hazard

    if front_clearance_meters is None or math.isnan(front_clearance_meters):
        return False

    threshold = (
        FRONT_CLEAR_HYSTERESIS_METERS
        if avoidance_active
        else FRONT_OBSTACLE_THRESHOLD_METERS
    )
    return 0.0 < front_clearance_meters < threshold


def compute_avoidance_velocity(
    front_clearance_meters: Optional[float],
    rear_clearance_meters: Optional[float],
    recommended_turn_direction: float,
) -> Tuple[float, float, float]:
    """
    Computes two-stage evasive (velocity_x, velocity_y, angular_velocity_z):
      - Stage 1 (0.35 m <= front < 0.75 m): turn and Mecanum-strafe into the open flank.
      - Stage 2 (front < 0.35 m): reverse at -0.12 m/s while turning and strafing
        if rear clearance > 0.40 m (or None/inf); otherwise turn + strafe only.
    """
    turn_sign = -1.0 if recommended_turn_direction < 0.0 else 1.0
    angular_velocity_z = turn_sign * AVOID_TURN_ANGULAR_VELOCITY_RADPS
    velocity_y = turn_sign * AVOID_STRAFE_VELOCITY_Y_MPS

    is_critically_close = (
        front_clearance_meters is not None
        and not math.isnan(front_clearance_meters)
        and front_clearance_meters < CRITICAL_FRONT_THRESHOLD_METERS
    )
    is_rear_clear_for_reverse = (
        rear_clearance_meters is None
        or math.isnan(rear_clearance_meters)
        or rear_clearance_meters > REAR_SAFE_REVERSE_CLEARANCE_METERS
    )

    velocity_x = (
        AVOID_REVERSE_VELOCITY_X_MPS
        if (is_critically_close and is_rear_clear_for_reverse)
        else 0.0
    )
    return velocity_x, velocity_y, angular_velocity_z


class IsObstacleTooClose(py_trees.behaviour.Behaviour):
    """
    M4.1 Reactive Condition Leaf:
    Returns SUCCESS when LiDAR clearance indicates an obstacle hazard,
    activating the high-priority reactive safety sequence.
    """

    def __init__(
        self,
        robot_state: RobotStateInterface,
        name: str = "IsObstacleTooClose",
    ) -> None:
        super().__init__(name=name)
        self.robot_state = robot_state

    def update(self) -> py_trees.common.Status:
        if is_obstacle_hazard_detected(
            self.robot_state.front_clearance_meters,
            self.robot_state.rear_clearance_meters,
            self.robot_state.arm_pick_status,
            self.robot_state.avoidance_active,
        ):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class AvoidObstacle(py_trees.behaviour.Behaviour):
    """
    M4.1 Reactive Action Leaf:
    Preempts any active pick via /arm/abort=True, switches base to AVOID mode,
    latches turn direction across the maneuver, commands two-stage evasive
    velocity, and unlatches /arm/abort=False on exit.
    """

    def __init__(
        self,
        robot_state: RobotStateInterface,
        name: str = "AvoidObstacle",
    ) -> None:
        super().__init__(name=name)
        self.robot_state = robot_state

    def initialise(self) -> None:
        self.robot_state.avoidance_active = True
        self.robot_state.latched_avoid_turn_direction = (
            self.robot_state.recommended_turn_direction
        )

    def update(self) -> py_trees.common.Status:
        self.robot_state.avoidance_active = True
        turn_direction = (
            self.robot_state.latched_avoid_turn_direction
            if self.robot_state.latched_avoid_turn_direction is not None
            else self.robot_state.recommended_turn_direction
        )
        self.robot_state.publish_arm_abort(True)
        self.robot_state.publish_base_mode("AVOID")
        velocity_x, velocity_y, angular_velocity_z = compute_avoidance_velocity(
            self.robot_state.front_clearance_meters,
            self.robot_state.rear_clearance_meters,
            turn_direction,
        )
        self.robot_state.publish_bt_cmd_vel(velocity_x, velocity_y, angular_velocity_z)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.robot_state.avoidance_active = False
        self.robot_state.latched_avoid_turn_direction = None
        self.robot_state.publish_arm_abort(False)


class DetectAndApproachTrash(py_trees.behaviour.Behaviour):
    """
    M4.2 Deliberative Action Leaf:
    Locks onto a fresh /youbot/trash_target and commands base_controller_node
    into APPROACH mode until /youbot/base_parked becomes True.
    """

    def __init__(
        self,
        robot_state: RobotStateInterface,
        name: str = "DetectAndApproachTrash",
    ) -> None:
        super().__init__(name=name)
        self.robot_state = robot_state
        self.approach_start_time_seconds: Optional[float] = None
        self.has_commanded_approach: bool = False

    def initialise(self) -> None:
        self.approach_start_time_seconds = self.robot_state.current_time_seconds
        self.has_commanded_approach = False
        self.robot_state.base_parked = False

    def update(self) -> py_trees.common.Status:
        if is_pick_cooldown_active(self.robot_state):
            return py_trees.common.Status.FAILURE

        if not is_trash_target_fresh(self.robot_state):
            return py_trees.common.Status.FAILURE

        target_x, target_y, _ = self.robot_state.trash_target
        target_distance = math.hypot(target_x, target_y)
        if not self.has_commanded_approach and target_distance > MAX_APPROACH_LOCK_DISTANCE_METERS:
            return py_trees.common.Status.FAILURE

        # Do not lock onto distant trash behind a closer forward obstacle (e.g. pedestrian)
        front_clearance = self.robot_state.front_clearance_meters
        if (
            front_clearance is not None
            and math.isfinite(front_clearance)
            and front_clearance < 1.10
            and front_clearance < (target_x - 0.15)
        ):
            return py_trees.common.Status.FAILURE

        if self.approach_start_time_seconds is None:
            self.approach_start_time_seconds = self.robot_state.current_time_seconds

        elapsed_seconds = (
            self.robot_state.current_time_seconds - self.approach_start_time_seconds
        )
        if elapsed_seconds > APPROACH_TIMEOUT_SECONDS:
            return py_trees.common.Status.FAILURE

        if self.has_commanded_approach and self.robot_state.base_parked:
            self.robot_state.publish_base_mode("IDLE")
            self.robot_state.publish_bt_cmd_vel(0.0, 0.0, 0.0)
            return py_trees.common.Status.SUCCESS

        self.has_commanded_approach = True
        self.robot_state.publish_base_mode("APPROACH")
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.approach_start_time_seconds = None
        self.has_commanded_approach = False


class ExecuteArmSubFSM(py_trees.behaviour.Behaviour):
    """
    M4.2 Deliberative Action Leaf:
    Holds the base in IDLE for PRE_PICK_SETTLE_DWELL_SECONDS so chassis motion
    fully stops, then triggers arm_fsm_node via /arm/pick_trigger=True,
    waits for the two-phase BUSY -> SUCCESS/FAILED handshake, and applies a
    retry cooldown on FAILED so the robot patrols/re-approaches.
    """

    def __init__(
        self,
        robot_state: RobotStateInterface,
        name: str = "ExecuteArmSubFSM",
    ) -> None:
        super().__init__(name=name)
        self.robot_state = robot_state
        self.enter_time_seconds: Optional[float] = None
        self.trigger_start_time_seconds: Optional[float] = None
        self.has_seen_busy: bool = False

    def initialise(self) -> None:
        self.enter_time_seconds = self.robot_state.current_time_seconds
        self.trigger_start_time_seconds = None
        self.has_seen_busy = False
        self.robot_state.publish_base_mode("IDLE")
        self.robot_state.publish_bt_cmd_vel(0.0, 0.0, 0.0)

    def update(self) -> py_trees.common.Status:
        self.robot_state.publish_base_mode("IDLE")
        self.robot_state.publish_bt_cmd_vel(0.0, 0.0, 0.0)

        if self.enter_time_seconds is None:
            self.enter_time_seconds = self.robot_state.current_time_seconds

        settle_elapsed = (
            self.robot_state.current_time_seconds - self.enter_time_seconds
        )
        if settle_elapsed < PRE_PICK_SETTLE_DWELL_SECONDS:
            return py_trees.common.Status.RUNNING

        status = self.robot_state.arm_pick_status.strip().upper()
        if status == "BUSY":
            self.has_seen_busy = True
            return py_trees.common.Status.RUNNING

        if not self.has_seen_busy:
            if self.trigger_start_time_seconds is None:
                self.trigger_start_time_seconds = self.robot_state.current_time_seconds
            handshake_elapsed = (
                self.robot_state.current_time_seconds - self.trigger_start_time_seconds
            )
            if handshake_elapsed > ARM_TRIGGER_HANDSHAKE_TIMEOUT_SECONDS:
                self.robot_state.pick_cooldown_until_seconds = (
                    self.robot_state.current_time_seconds + PICK_FAILURE_COOLDOWN_SECONDS
                )
                return py_trees.common.Status.FAILURE
            self.robot_state.publish_arm_pick_trigger(True)
            return py_trees.common.Status.RUNNING

        if status == "SUCCESS":
            return py_trees.common.Status.SUCCESS

        # Terminal FAILED (or unexpected IDLE reset after BUSY): apply retry cooldown
        self.robot_state.pick_cooldown_until_seconds = (
            self.robot_state.current_time_seconds + PICK_FAILURE_COOLDOWN_SECONDS
        )
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.enter_time_seconds = None
        self.trigger_start_time_seconds = None
        self.has_seen_busy = False


def compute_patrol_velocity(
    odom_y_meters: Optional[float],
    current_yaw_radians: Optional[float],
) -> Tuple[float, float, float]:
    """
    Computes (velocity_x, velocity_y, angular_velocity_z) for PatrolStreet:
      - Cruises forward at PATROL_FORWARD_VELOCITY_X_MPS (0.22 m/s).
      - Regulates heading back to the street corridor axis (0 or pi rad) using
        IMU-fused yaw from /wheel_odom_raw.
      - When aligned with the corridor, gently strafes back toward the street
        centerline (odom_y = 0.0 m) to keep clear of side walls and benches.
    """
    if current_yaw_radians is None or not math.isfinite(current_yaw_radians):
        return PATROL_FORWARD_VELOCITY_X_MPS, 0.0, 0.0

    corridor_yaw_error = 0.5 * math.atan2(
        math.sin(-2.0 * current_yaw_radians),
        math.cos(-2.0 * current_yaw_radians),
    )
    angular_velocity_z = max(
        -PATROL_MAX_ANGULAR_VELOCITY_RADPS,
        min(
            PATROL_MAX_ANGULAR_VELOCITY_RADPS,
            PATROL_HEADING_KP * corridor_yaw_error,
        ),
    )

    velocity_y = 0.0
    if (
        odom_y_meters is not None
        and math.isfinite(odom_y_meters)
        and abs(corridor_yaw_error) <= PATROL_CENTERLINE_ALIGN_THRESHOLD_RAD
    ):
        heading_sign = 1.0 if math.cos(current_yaw_radians) >= 0.0 else -1.0
        raw_velocity_y = -PATROL_CENTERLINE_KY * odom_y_meters * heading_sign
        velocity_y = max(
            -PATROL_MAX_CENTERLINE_VELOCITY_Y_MPS,
            min(PATROL_MAX_CENTERLINE_VELOCITY_Y_MPS, raw_velocity_y),
        )

    return PATROL_FORWARD_VELOCITY_X_MPS, velocity_y, angular_velocity_z


class PatrolStreet(py_trees.behaviour.Behaviour):
    """
    M4.2 Deliberative Fallback Action Leaf:
    Drives forward along the Iso-Roba pedestrian street in PATROL mode when
    no trash target is actively being approached or picked, using IMU-fused
    odometry to maintain corridor heading and centerline clearance.
    """

    def __init__(
        self,
        robot_state: RobotStateInterface,
        name: str = "PatrolStreet",
    ) -> None:
        super().__init__(name=name)
        self.robot_state = robot_state

    def update(self) -> py_trees.common.Status:
        self.robot_state.publish_base_mode("PATROL")
        velocity_x, velocity_y, angular_velocity_z = compute_patrol_velocity(
            self.robot_state.odom_y_meters,
            self.robot_state.current_yaw_radians,
        )
        self.robot_state.publish_bt_cmd_vel(
            velocity_x, velocity_y, angular_velocity_z
        )
        return py_trees.common.Status.RUNNING


def create_behavior_tree_root(
    robot_state: RobotStateInterface,
) -> py_trees.behaviour.Behaviour:
    """
    Constructs the Hybrid Behavior Tree root Selector (memory=False):
      1. ReactiveSafety Sequence (memory=False: IsObstacleTooClose -> AvoidObstacle)
      2. LitterCollection Sequence (memory=True: DetectAndApproachTrash -> ExecuteArmSubFSM)
      3. PatrolStreet Fallback Action
    """
    reactive_safety_sequence = py_trees.composites.Sequence(
        name="ReactiveSafety",
        memory=False,
        children=[
            IsObstacleTooClose(robot_state),
            AvoidObstacle(robot_state),
        ],
    )
    litter_collection_sequence = py_trees.composites.Sequence(
        name="LitterCollection",
        memory=True,
        children=[
            DetectAndApproachTrash(robot_state),
            ExecuteArmSubFSM(robot_state),
        ],
    )
    return py_trees.composites.Selector(
        name="Root",
        memory=False,
        children=[
            reactive_safety_sequence,
            litter_collection_sequence,
            PatrolStreet(robot_state),
        ],
    )


class BehaviorTreeNode(Node):
    """ROS 2 node hosting and ticking the Layer 2 py_trees Hybrid Behavior Tree."""

    def __init__(self) -> None:
        super().__init__("behavior_tree_node")

        self.robot_state = RobotStateInterface(
            current_time_seconds=self._now_seconds(),
            publish_base_mode=self._publish_base_mode,
            publish_bt_cmd_vel=self._publish_bt_cmd_vel,
            publish_arm_pick_trigger=self._publish_arm_pick_trigger,
            publish_arm_abort=self._publish_arm_abort,
        )

        # Publishers
        self.base_mode_publisher = self.create_publisher(
            String, "/youbot/base_mode", 10
        )
        self.bt_cmd_vel_publisher = self.create_publisher(
            Twist, "/youbot/bt_cmd_vel", 10
        )
        self.arm_pick_trigger_publisher = self.create_publisher(
            Bool, "/arm/pick_trigger", 10
        )
        self.arm_abort_publisher = self.create_publisher(Bool, "/arm/abort", 10)

        # Subscriptions
        self.create_subscription(
            Float32, "/youbot/clearance/front", self._front_clearance_callback, 10
        )
        self.create_subscription(
            Float32, "/youbot/clearance/rear", self._rear_clearance_callback, 10
        )
        self.create_subscription(
            Float32,
            "/youbot/clearance/recommended_turn",
            self._recommended_turn_callback,
            10,
        )
        self.create_subscription(
            Point, "/youbot/trash_target", self._trash_target_callback, 10
        )
        self.create_subscription(
            Bool, "/youbot/base_parked", self._base_parked_callback, 10
        )
        self.create_subscription(
            String, "/arm/pick_status", self._arm_pick_status_callback, 10
        )
        self.create_subscription(
            Odometry, "/wheel_odom_raw", self._odom_callback, 10
        )

        root = create_behavior_tree_root(self.robot_state)
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(timeout=15)
        self._last_active_tip_name: Optional[str] = None

        self.create_timer(TICK_PERIOD_SECONDS, self._tick_tree)
        self.get_logger().info(
            "BehaviorTreeNode initialized at 10 Hz:\n"
            + py_trees.display.unicode_tree(root=self.tree.root, show_status=False)
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _front_clearance_callback(self, message: Float32) -> None:
        self.robot_state.front_clearance_meters = float(message.data)

    def _rear_clearance_callback(self, message: Float32) -> None:
        self.robot_state.rear_clearance_meters = float(message.data)

    def _recommended_turn_callback(self, message: Float32) -> None:
        if math.isfinite(message.data) and message.data != 0.0:
            self.robot_state.recommended_turn_direction = (
                1.0 if message.data > 0.0 else -1.0
            )

    def _trash_target_callback(self, message: Point) -> None:
        self.robot_state.trash_target = (
            float(message.x),
            float(message.y),
            float(message.z),
        )
        self.robot_state.trash_target_time_seconds = self._now_seconds()

    def _base_parked_callback(self, message: Bool) -> None:
        self.robot_state.base_parked = bool(message.data)

    def _arm_pick_status_callback(self, message: String) -> None:
        self.robot_state.arm_pick_status = message.data.strip().upper()

    def _odom_callback(self, message: Odometry) -> None:
        self.robot_state.odom_x_meters = float(message.pose.pose.position.x)
        self.robot_state.odom_y_meters = float(message.pose.pose.position.y)
        orientation = message.pose.pose.orientation
        self.robot_state.current_yaw_radians = 2.0 * math.atan2(
            orientation.z, orientation.w
        )

    def _publish_base_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self.base_mode_publisher.publish(message)

    def _publish_bt_cmd_vel(
        self,
        velocity_x: float,
        velocity_y: float,
        angular_velocity_z: float,
    ) -> None:
        twist = Twist()
        twist.linear.x = float(velocity_x)
        twist.linear.y = float(velocity_y)
        twist.angular.z = float(angular_velocity_z)
        self.bt_cmd_vel_publisher.publish(twist)

    def _publish_arm_pick_trigger(self, trigger: bool) -> None:
        message = Bool()
        message.data = bool(trigger)
        self.arm_pick_trigger_publisher.publish(message)

    def _publish_arm_abort(self, abort: bool) -> None:
        message = Bool()
        message.data = bool(abort)
        self.arm_abort_publisher.publish(message)

    def _tick_tree(self) -> None:
        self.robot_state.current_time_seconds = self._now_seconds()
        self.tree.tick()
        tip = self.tree.tip()
        tip_name = tip.name if tip is not None else "None"
        if tip_name != self._last_active_tip_name:
            self._last_active_tip_name = tip_name
            self.get_logger().info(f"Behavior Tree active leaf -> {tip_name}")


def main(args=None) -> None:
    rclpy.init(args=args)
    behavior_tree_node = BehaviorTreeNode()
    try:
        rclpy.spin(behavior_tree_node)
    except KeyboardInterrupt:
        pass
    finally:
        behavior_tree_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
