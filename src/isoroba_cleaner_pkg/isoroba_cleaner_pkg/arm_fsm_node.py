"""
arm_fsm_node.py — Layer 3 Arm Pick-and-Place Sub-FSM for KUKA YouBot.

Implements M3.2 + M3.3:
  - Calibrated 5-DOF arm waypoints (HOME, PRE_GRASP, GRASP, GRASP_TALL, LIFT, DROP_IN_BIN)
    and gripper targets (GRIPPER_OPEN, GRIPPER_CLOSED) targeting 0.45 m standoff
    and the rear collection_bin (x = -0.09 m, y = 0.0 m).
  - Deterministic 8-state FSM (IDLE, PRE_GRASP, GRASP, CLOSE_GRIPPER, LIFT,
    DROP_IN_BIN, RELEASE_GRIPPER, RETURN_HOME) stepping through the 5 pick-and-place
    kinematic stages with closed-loop joint tolerance and gripper stall checks.
  - Subscribes to /arm/pick_trigger (Bool), /arm/abort (Bool), /joint_states (JointState),
    and /youbot/trash_target (Point).
  - Publishes /arm/joint_1..5/cmd (Float64), /arm/gripper/cmd (Float64),
    and /arm/pick_status (String: IDLE, BUSY, SUCCESS, FAILED) at 10 Hz.
"""

from enum import Enum
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


# Control loop timing & tolerances
CONTROL_PERIOD_SECONDS = 0.1  # 10 Hz
JOINT_POSITION_TOLERANCE_RAD = 0.04
WAYPOINT_DWELL_SECONDS = 1.0
GRIPPER_SETTLE_EPSILON_M = 0.0005
GRIPPER_SETTLE_CYCLES_REQUIRED = 4
EMPTY_GRASP_THRESHOLD_M = 0.002

# Safety timeouts
STATE_TIMEOUT_SECONDS = 6.0
OVERALL_PICK_TIMEOUT_SECONDS = 25.0

# Gripper limits (meters per finger, matching youbot_driver.py)
GRIPPER_OPEN_METERS = 0.025
GRIPPER_CLOSED_METERS = 0.0

ARM_JOINT_NAMES = ("arm1", "arm2", "arm3", "arm4", "arm5")
GRIPPER_JOINT_NAMES = ("finger::left", "finger::right")

# Analytical FK waypoint estimates for KUKA YouBot (arm1..arm5 in radians).
# Targeted at 0.45 m base_link standoff and rear collection_bin (x=-0.09, y=0.0).
# GRASP_LOW is calibrated for ~0.05 m tall litter (z <= -0.065 m in base_link);
# GRASP_TALL is ~2.4 cm higher for ~0.08 m tall litter (z > -0.065 m).
TALL_TARGET_Z_THRESHOLD_M = -0.065

ARM_WAYPOINTS: Dict[str, Tuple[float, float, float, float, float]] = {
    "HOME": (0.0, 0.80, -1.40, 1.00, 0.0),
    "PRE_GRASP": (0.0, -0.82, -1.08, -1.24, 0.0),
    "GRASP": (0.0, -1.12, -1.12, -0.90, 0.0),
    "GRASP_TALL": (0.0, -0.96, -1.10, -1.08, 0.0),
    "LIFT": (0.0, -0.20, -0.60, -1.20, 0.0),
    "DROP_IN_BIN": (0.0, 0.15, 0.65, 1.65, 0.0),
}


class ArmFSMState(str, Enum):
    """Deterministic states of the 5-state pick-and-place sub-FSM."""

    IDLE = "IDLE"
    PRE_GRASP = "PRE_GRASP"
    GRASP = "GRASP"
    CLOSE_GRIPPER = "CLOSE_GRIPPER"
    LIFT = "LIFT"
    DROP_IN_BIN = "DROP_IN_BIN"
    RELEASE_GRIPPER = "RELEASE_GRIPPER"
    RETURN_HOME = "RETURN_HOME"


class PickStatus(str, Enum):
    """High-level status published on /arm/pick_status for Behavior Tree."""

    IDLE = "IDLE"
    BUSY = "BUSY"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


def all_arm_joints_reached(
    current_positions: Sequence[float],
    target_positions: Sequence[float],
    tolerance_rad: float = JOINT_POSITION_TOLERANCE_RAD,
) -> bool:
    """Returns True when all 5 arm joints are within tolerance of target."""
    if len(current_positions) != 5 or len(target_positions) != 5:
        return False
    return all(
        abs(current - target) <= tolerance_rad
        for current, target in zip(current_positions, target_positions)
    )


class ArmPickFSM:
    """Pure-Python 5-state pick-and-place state machine (testable without ROS 2)."""

    def __init__(self) -> None:
        self.state: ArmFSMState = ArmFSMState.IDLE
        self.status: PickStatus = PickStatus.IDLE
        self.abort_active: bool = False
        self.state_elapsed_seconds: float = 0.0
        self.cycle_elapsed_seconds: float = 0.0
        self.waypoint_dwell_elapsed_seconds: float = 0.0
        self.previous_gripper_average: Optional[float] = None
        self.gripper_stall_cycles: int = 0
        self._pending_outcome: Optional[PickStatus] = None
        self.active_grasp_waypoint_name: str = "GRASP"

        self.commanded_arm: Tuple[float, float, float, float, float] = ARM_WAYPOINTS["HOME"]
        self.commanded_gripper: float = GRIPPER_OPEN_METERS
        self.command_dirty: bool = True

    def _enter_state(
        self,
        new_state: ArmFSMState,
        waypoint_name: Optional[str] = None,
        gripper_target: Optional[float] = None,
    ) -> None:
        """Transitions to new_state and updates commanded targets if provided."""
        self.state = new_state
        self.state_elapsed_seconds = 0.0
        self.waypoint_dwell_elapsed_seconds = 0.0
        self.previous_gripper_average = None
        self.gripper_stall_cycles = 0
        if waypoint_name is not None:
            self.commanded_arm = ARM_WAYPOINTS[waypoint_name]
            self.command_dirty = True
        if gripper_target is not None:
            self.commanded_gripper = gripper_target
            self.command_dirty = True

    def _abort_to_home(self, outcome: PickStatus = PickStatus.FAILED) -> None:
        """Immediately stows arm to HOME, opens gripper, and schedules outcome."""
        self._pending_outcome = outcome
        self._enter_state(ArmFSMState.RETURN_HOME, "HOME", GRIPPER_OPEN_METERS)

    def set_abort(self, abort_requested: bool) -> None:
        """Handles /arm/abort updates; preempts active pick back to HOME."""
        self.abort_active = abort_requested
        if abort_requested and self.state != ArmFSMState.IDLE:
            if self.state != ArmFSMState.RETURN_HOME:
                self._abort_to_home(PickStatus.FAILED)
            else:
                self._pending_outcome = PickStatus.FAILED

    def trigger_pick(self, target_z_meters: Optional[float] = None) -> bool:
        """Starts a pick cycle if IDLE and not aborted. Selects GRASP height from target_z."""
        if self.abort_active or self.state != ArmFSMState.IDLE:
            return False
        self.active_grasp_waypoint_name = (
            "GRASP_TALL"
            if target_z_meters is not None and target_z_meters > TALL_TARGET_Z_THRESHOLD_M
            else "GRASP"
        )
        self.status = PickStatus.BUSY
        self.cycle_elapsed_seconds = 0.0
        self._pending_outcome = PickStatus.SUCCESS
        self._enter_state(ArmFSMState.PRE_GRASP, "PRE_GRASP", GRIPPER_OPEN_METERS)
        return True

    def _gripper_settled(self, gripper_positions: Sequence[float]) -> bool:
        """Detects gripper completion when finger motion stalls across cycles."""
        if len(gripper_positions) < 2:
            return False
        current_average = 0.5 * (gripper_positions[0] + gripper_positions[1])
        if self.previous_gripper_average is not None:
            delta = abs(current_average - self.previous_gripper_average)
            if delta <= GRIPPER_SETTLE_EPSILON_M:
                self.gripper_stall_cycles += 1
            else:
                self.gripper_stall_cycles = 0
        self.previous_gripper_average = current_average
        return self.gripper_stall_cycles >= GRIPPER_SETTLE_CYCLES_REQUIRED

    def _waypoint_settled(
        self,
        arm_positions: Sequence[float],
        target_positions: Sequence[float],
        delta_time_seconds: float,
        dwell_seconds: float = WAYPOINT_DWELL_SECONDS,
    ) -> bool:
        """Requires arm joints to stay within tolerance for dwell_seconds."""
        if all_arm_joints_reached(arm_positions, target_positions):
            self.waypoint_dwell_elapsed_seconds += delta_time_seconds
            return self.waypoint_dwell_elapsed_seconds >= dwell_seconds
        self.waypoint_dwell_elapsed_seconds = 0.0
        return False

    def step(
        self,
        delta_time_seconds: float,
        arm_positions: Optional[Sequence[float]],
        gripper_positions: Optional[Sequence[float]],
    ) -> None:
        """Advances the FSM by one control tick using latest /joint_states."""
        if self.state == ArmFSMState.IDLE:
            return

        self.state_elapsed_seconds += delta_time_seconds
        self.cycle_elapsed_seconds += delta_time_seconds

        if self.state != ArmFSMState.RETURN_HOME and (
            self.cycle_elapsed_seconds >= OVERALL_PICK_TIMEOUT_SECONDS
            or self.state_elapsed_seconds >= STATE_TIMEOUT_SECONDS
        ):
            self._abort_to_home(PickStatus.FAILED)
            return

        if arm_positions is None or gripper_positions is None:
            return

        if self.state == ArmFSMState.PRE_GRASP:
            if self._waypoint_settled(
                arm_positions, ARM_WAYPOINTS["PRE_GRASP"], delta_time_seconds
            ):
                self._enter_state(ArmFSMState.GRASP, self.active_grasp_waypoint_name)
        elif self.state == ArmFSMState.GRASP:
            if self._waypoint_settled(
                arm_positions,
                ARM_WAYPOINTS[self.active_grasp_waypoint_name],
                delta_time_seconds,
            ):
                self._enter_state(
                    ArmFSMState.CLOSE_GRIPPER, gripper_target=GRIPPER_CLOSED_METERS
                )
        elif self.state == ArmFSMState.CLOSE_GRIPPER:
            if self._gripper_settled(gripper_positions):
                gripper_average = 0.5 * (gripper_positions[0] + gripper_positions[1])
                if gripper_average < EMPTY_GRASP_THRESHOLD_M:
                    self._abort_to_home(PickStatus.FAILED)
                else:
                    self._enter_state(ArmFSMState.LIFT, "LIFT")
        elif self.state == ArmFSMState.LIFT:
            if all_arm_joints_reached(arm_positions, ARM_WAYPOINTS["LIFT"]):
                self._enter_state(ArmFSMState.DROP_IN_BIN, "DROP_IN_BIN")
        elif self.state == ArmFSMState.DROP_IN_BIN:
            if all_arm_joints_reached(arm_positions, ARM_WAYPOINTS["DROP_IN_BIN"]):
                self._enter_state(
                    ArmFSMState.RELEASE_GRIPPER, gripper_target=GRIPPER_OPEN_METERS
                )
        elif self.state == ArmFSMState.RELEASE_GRIPPER:
            if self._gripper_settled(gripper_positions):
                self._pending_outcome = PickStatus.SUCCESS
                self._enter_state(ArmFSMState.RETURN_HOME, "HOME", GRIPPER_OPEN_METERS)
        elif self.state == ArmFSMState.RETURN_HOME:
            reached = all_arm_joints_reached(arm_positions, ARM_WAYPOINTS["HOME"])
            if reached or self.state_elapsed_seconds >= STATE_TIMEOUT_SECONDS:
                self.status = self._pending_outcome or PickStatus.FAILED
                self.state = ArmFSMState.IDLE
                self.state_elapsed_seconds = 0.0


class ArmFSMNode(Node):
    """ROS 2 node wrapping ArmPickFSM for KUKA YouBot manipulation."""

    def __init__(self) -> None:
        super().__init__("arm_fsm_node")
        self.fsm = ArmPickFSM()
        self.latest_arm_positions: Optional[Tuple[float, ...]] = None
        self.latest_gripper_positions: Optional[Tuple[float, ...]] = None
        self.latest_target_z_meters: Optional[float] = None

        self.arm_command_pubs = [
            self.create_publisher(Float64, f"/arm/joint_{index}/cmd", 10)
            for index in range(1, 6)
        ]
        self.gripper_command_pub = self.create_publisher(Float64, "/arm/gripper/cmd", 10)
        self.pick_status_pub = self.create_publisher(String, "/arm/pick_status", 10)

        self.create_subscription(JointState, "/joint_states", self._joint_states_callback, 10)
        self.create_subscription(Point, "/youbot/trash_target", self._trash_target_callback, 10)
        self.create_subscription(Bool, "/arm/pick_trigger", self._pick_trigger_callback, 10)
        self.create_subscription(Bool, "/arm/abort", self._abort_callback, 10)

        self._publish_commands_if_dirty()
        self.create_timer(CONTROL_PERIOD_SECONDS, self._control_loop)
        self.get_logger().info("ArmFSMNode initialized at HOME; waiting for /arm/pick_trigger.")

    def _joint_states_callback(self, msg: JointState) -> None:
        name_to_position = dict(zip(msg.name, msg.position))
        if all(name in name_to_position for name in ARM_JOINT_NAMES):
            self.latest_arm_positions = tuple(
                name_to_position[name] for name in ARM_JOINT_NAMES
            )
        if all(name in name_to_position for name in GRIPPER_JOINT_NAMES):
            self.latest_gripper_positions = tuple(
                name_to_position[name] for name in GRIPPER_JOINT_NAMES
            )

    def _trash_target_callback(self, msg: Point) -> None:
        self.latest_target_z_meters = msg.z

    def _pick_trigger_callback(self, msg: Bool) -> None:
        if msg.data:
            if self.fsm.trigger_pick(self.latest_target_z_meters):
                self.get_logger().info(
                    f"Pick cycle triggered -> PRE_GRASP ({self.fsm.active_grasp_waypoint_name})."
                )

    def _abort_callback(self, msg: Bool) -> None:
        was_aborted = self.fsm.abort_active
        self.fsm.set_abort(msg.data)
        if msg.data and not was_aborted:
            self.get_logger().warn("Arm pick aborted via /arm/abort; stowing to HOME.")

    def _publish_commands_if_dirty(self) -> None:
        if not self.fsm.command_dirty:
            return
        for publisher, angle in zip(self.arm_command_pubs, self.fsm.commanded_arm):
            cmd = Float64()
            cmd.data = float(angle)
            publisher.publish(cmd)
        gripper_cmd = Float64()
        gripper_cmd.data = float(self.fsm.commanded_gripper)
        self.gripper_command_pub.publish(gripper_cmd)
        self.fsm.command_dirty = False

    def _control_loop(self) -> None:
        if (
            self.fsm.state == ArmFSMState.IDLE
            and self.latest_arm_positions is not None
            and not all_arm_joints_reached(
                self.latest_arm_positions, self.fsm.commanded_arm
            )
        ):
            self.fsm.command_dirty = True

        previous_state = self.fsm.state
        self.fsm.step(
            CONTROL_PERIOD_SECONDS,
            self.latest_arm_positions,
            self.latest_gripper_positions,
        )
        self._publish_commands_if_dirty()

        if self.fsm.state != previous_state:
            self.get_logger().info(
                f"Arm FSM transition: {previous_state.value} -> {self.fsm.state.value} "
                f"(status={self.fsm.status.value})"
            )

        status_msg = String()
        status_msg.data = self.fsm.status.value
        self.pick_status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
