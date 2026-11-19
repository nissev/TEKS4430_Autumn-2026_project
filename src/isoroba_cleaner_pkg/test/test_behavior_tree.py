"""
test_behavior_tree.py — Unit tests for M4.1 Reactive Safety and M4.2 Deliberative Mission Leaves.
Can be executed with pytest without requiring ROS 2 middleware or Webots.
"""

import math
import py_trees

from isoroba_cleaner_pkg.behavior_tree_node import (
    APPROACH_TIMEOUT_SECONDS,
    ARM_SELF_OCCLUSION_DEADBAND_METERS,
    AVOID_REVERSE_VELOCITY_X_MPS,
    AVOID_STRAFE_VELOCITY_Y_MPS,
    AVOID_TURN_ANGULAR_VELOCITY_RADPS,
    CRITICAL_FRONT_THRESHOLD_METERS,
    FRONT_CLEAR_HYSTERESIS_METERS,
    FRONT_OBSTACLE_THRESHOLD_METERS,
    PATROL_FORWARD_VELOCITY_X_MPS,
    PICK_FAILURE_COOLDOWN_SECONDS,
    PRE_PICK_SETTLE_DWELL_SECONDS,
    REAR_BUSY_HAZARD_THRESHOLD_METERS,
    REAR_SAFE_REVERSE_CLEARANCE_METERS,
    TARGET_FRESHNESS_TIMEOUT_SECONDS,
    RobotStateInterface,
    create_behavior_tree_root,
)


def _build_test_harness():
    """Creates a RobotStateInterface with command recorders and a py_trees BehaviourTree."""
    published_modes = []
    published_twists = []
    published_aborts = []
    published_pick_triggers = []

    robot_state = RobotStateInterface(
        current_time_seconds=10.0,
        publish_base_mode=lambda mode: published_modes.append(mode),
        publish_bt_cmd_vel=lambda vx, vy, wz: published_twists.append((vx, vy, wz)),
        publish_arm_pick_trigger=lambda trigger: published_pick_triggers.append(trigger),
        publish_arm_abort=lambda abort: published_aborts.append(abort),
    )
    root = create_behavior_tree_root(robot_state)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)
    return (
        robot_state,
        tree,
        published_modes,
        published_twists,
        published_aborts,
        published_pick_triggers,
    )


def test_startup_without_scan_or_trash_runs_patrol_street():
    robot_state, tree, published_modes, published_twists, published_aborts, _ = (
        _build_test_harness()
    )
    assert robot_state.front_clearance_meters is None

    tree.tick()
    assert tree.tip().name == "PatrolStreet"
    assert published_modes[-1] == "PATROL"
    assert published_twists[-1] == (PATROL_FORWARD_VELOCITY_X_MPS, 0.0, 0.0)
    assert published_aborts == []


def test_stage_one_avoidance_turns_in_place_using_recommended_turn():
    robot_state, tree, published_modes, published_twists, published_aborts, _ = (
        _build_test_harness()
    )
    # Front within [0.35 m, 0.55 m) -> turn in place without reversing
    robot_state.front_clearance_meters = 0.45
    robot_state.rear_clearance_meters = 2.0
    robot_state.recommended_turn_direction = -1.0

    tree.tick()
    assert tree.tip().name == "AvoidObstacle"
    assert published_modes[-1] == "AVOID"
    assert published_aborts[-1] is True
    velocity_x, velocity_y, angular_velocity_z = published_twists[-1]
    assert math.isclose(velocity_x, 0.0, abs_tol=1e-9)
    assert math.isclose(
        velocity_y, -AVOID_STRAFE_VELOCITY_Y_MPS, rel_tol=1e-6
    )
    assert math.isclose(
        angular_velocity_z, -AVOID_TURN_ANGULAR_VELOCITY_RADPS, rel_tol=1e-6
    )


def test_stage_two_critical_front_reverses_when_rear_clear_and_stops_reverse_when_rear_blocked():
    robot_state, tree, _, published_twists, _, _ = _build_test_harness()
    # Critically close front (< 0.35 m) and clear rear (> 0.40 m) -> reverse + turn
    robot_state.front_clearance_meters = CRITICAL_FRONT_THRESHOLD_METERS - 0.05
    robot_state.rear_clearance_meters = REAR_SAFE_REVERSE_CLEARANCE_METERS + 0.20
    robot_state.recommended_turn_direction = 1.0

    tree.tick()
    assert tree.tip().name == "AvoidObstacle"
    velocity_x, _, angular_velocity_z = published_twists[-1]
    assert math.isclose(velocity_x, AVOID_REVERSE_VELOCITY_X_MPS, rel_tol=1e-6)
    assert math.isclose(
        angular_velocity_z, AVOID_TURN_ANGULAR_VELOCITY_RADPS, rel_tol=1e-6
    )

    # Rear becomes blocked (<= 0.40 m) -> stops reversing, turns in place only
    robot_state.rear_clearance_meters = REAR_SAFE_REVERSE_CLEARANCE_METERS - 0.05
    tree.tick()
    velocity_x, _, angular_velocity_z = published_twists[-1]
    assert math.isclose(velocity_x, 0.0, abs_tol=1e-9)
    assert math.isclose(
        angular_velocity_z, AVOID_TURN_ANGULAR_VELOCITY_RADPS, rel_tol=1e-6
    )


def test_arm_busy_self_occlusion_deadband_and_external_hazard_preemption():
    robot_state, tree, published_modes, _, published_aborts, _ = (
        _build_test_harness()
    )
    robot_state.arm_pick_status = "BUSY"
    robot_state.rear_clearance_meters = float("inf")

    # 1. Near-field arm reflection (< 0.40 m) while BUSY must NOT self-abort the pick
    robot_state.front_clearance_meters = ARM_SELF_OCCLUSION_DEADBAND_METERS - 0.08
    tree.tick()
    assert tree.tip().name == "PatrolStreet"
    assert published_aborts == []

    # 2. External front obstacle in [0.40 m, 0.55 m) while BUSY MUST preempt and abort pick
    robot_state.front_clearance_meters = 0.45
    tree.tick()
    assert tree.tip().name == "AvoidObstacle"
    assert published_aborts[-1] is True
    assert published_modes[-1] == "AVOID"

    # 3. Obstacle clears past hysteresis threshold -> AvoidObstacle terminates and unlatches /arm/abort=False
    robot_state.front_clearance_meters = FRONT_CLEAR_HYSTERESIS_METERS + 0.10
    tree.tick()
    assert tree.tip().name == "PatrolStreet"
    assert published_aborts[-1] is False


def test_arm_busy_rear_hazard_preempts_pick():
    robot_state, tree, _, _, published_aborts, _ = _build_test_harness()
    robot_state.arm_pick_status = "BUSY"
    robot_state.front_clearance_meters = 0.18  # self-arm reflection in front
    robot_state.rear_clearance_meters = REAR_BUSY_HAZARD_THRESHOLD_METERS - 0.05

    tree.tick()
    assert tree.tip().name == "AvoidObstacle"
    assert published_aborts[-1] is True


def test_full_litter_collection_sequence_approach_park_and_two_phase_pick_handshake():
    (
        robot_state,
        tree,
        published_modes,
        published_twists,
        _,
        published_pick_triggers,
    ) = _build_test_harness()
    robot_state.front_clearance_meters = 2.0
    # Simulate stale previous pick status still showing "SUCCESS" on /arm/pick_status
    robot_state.arm_pick_status = "SUCCESS"

    # 1. Fresh trash target detected -> DetectAndApproachTrash runs in APPROACH mode
    robot_state.trash_target = (0.85, 0.12, -0.075)
    robot_state.trash_target_time_seconds = robot_state.current_time_seconds
    tree.tick()
    assert tree.tip().name == "DetectAndApproachTrash"
    assert published_modes[-1] == "APPROACH"

    # 2. Base controller finishes parking -> DetectAndApproachTrash succeeds and
    #    Sequence(memory=True) immediately enters ExecuteArmSubFSM in the same tick.
    #    During PRE_PICK_SETTLE_DWELL_SECONDS, base is held in IDLE without triggering pick yet.
    robot_state.base_parked = True
    tree.tick()
    assert tree.tip().name == "ExecuteArmSubFSM"
    assert published_modes[-1] == "IDLE"
    assert published_twists[-1] == (0.0, 0.0, 0.0)
    assert published_pick_triggers == []

    # 2b. Advance past PRE_PICK_SETTLE_DWELL_SECONDS -> pick_trigger=True is published,
    #     and two-phase handshake waits for "BUSY" first even though arm_pick_status is "SUCCESS".
    robot_state.current_time_seconds += PRE_PICK_SETTLE_DWELL_SECONDS + 0.05
    tree.tick()
    assert tree.tip().name == "ExecuteArmSubFSM"
    assert published_pick_triggers[-1] is True

    # 3. Arm FSM enters BUSY; even if trash_target becomes stale during manipulation,
    #    Sequence(memory=True) keeps ExecuteArmSubFSM running.
    robot_state.current_time_seconds += 1.0
    robot_state.arm_pick_status = "BUSY"
    tree.tick()
    assert tree.tip().name == "ExecuteArmSubFSM"

    # 4. Arm FSM completes pick -> status becomes SUCCESS -> LitterCollection succeeds
    robot_state.current_time_seconds += 5.0
    robot_state.arm_pick_status = "SUCCESS"
    tree.tick()
    assert tree.root.status == py_trees.common.Status.SUCCESS

    # 5. Next tick with no fresh target falls through to PatrolStreet
    robot_state.base_parked = False
    tree.tick()
    assert tree.tip().name == "PatrolStreet"
    assert published_modes[-1] == "PATROL"


def test_failed_pick_activates_cooldown_and_resumes_patrol():
    (
        robot_state,
        tree,
        published_modes,
        _,
        _,
        _,
    ) = _build_test_harness()
    robot_state.front_clearance_meters = 2.0
    robot_state.trash_target = (0.45, 0.0, -0.075)
    robot_state.trash_target_time_seconds = robot_state.current_time_seconds

    # First tick enters DetectAndApproachTrash (resets stale base_parked and commands APPROACH)
    tree.tick()
    assert tree.tip().name == "DetectAndApproachTrash"

    # Base reports parked -> transitions to ExecuteArmSubFSM
    robot_state.base_parked = True
    tree.tick()
    assert tree.tip().name == "ExecuteArmSubFSM"

    # Advance past pre-pick settle dwell and transition through BUSY -> FAILED
    robot_state.current_time_seconds += PRE_PICK_SETTLE_DWELL_SECONDS + 0.05
    tree.tick()
    robot_state.arm_pick_status = "BUSY"
    tree.tick()
    assert tree.tip().name == "ExecuteArmSubFSM"

    # Missed grasp -> FAILED -> ExecuteArmSubFSM fails, sets cooldown, and Selector
    # falls through to PatrolStreet in the same tick!
    robot_state.arm_pick_status = "FAILED"
    tree.tick()
    assert tree.tip().name == "PatrolStreet"
    assert published_modes[-1] == "PATROL"

    # While within PICK_FAILURE_COOLDOWN_SECONDS, DetectAndApproachTrash returns FAILURE
    # even if trash_target is refreshed, preventing an infinite re-trigger loop in place
    robot_state.current_time_seconds += PICK_FAILURE_COOLDOWN_SECONDS - 0.5
    robot_state.trash_target_time_seconds = robot_state.current_time_seconds
    tree.tick()
    assert tree.tip().name == "PatrolStreet"

    # Once cooldown expires, DetectAndApproachTrash locks onto fresh target again
    robot_state.current_time_seconds += 1.0
    robot_state.trash_target_time_seconds = robot_state.current_time_seconds
    robot_state.base_parked = False
    tree.tick()
    assert tree.tip().name == "DetectAndApproachTrash"
    assert published_modes[-1] == "APPROACH"


def test_stale_target_or_approach_timeout_falls_back_to_patrol():
    robot_state, tree, published_modes, _, _, _ = _build_test_harness()
    robot_state.front_clearance_meters = 2.0
    robot_state.trash_target = (1.2, 0.1, -0.075)
    robot_state.trash_target_time_seconds = robot_state.current_time_seconds

    tree.tick()
    assert tree.tip().name == "DetectAndApproachTrash"

    # Target becomes stale (> 0.5 s old) -> falls back to PatrolStreet
    robot_state.current_time_seconds += TARGET_FRESHNESS_TIMEOUT_SECONDS + 0.1
    tree.tick()
    assert tree.tip().name == "PatrolStreet"
    assert published_modes[-1] == "PATROL"

    # Re-acquire target, then exceed APPROACH_TIMEOUT_SECONDS without parking
    robot_state.trash_target_time_seconds = robot_state.current_time_seconds
    tree.tick()
    assert tree.tip().name == "DetectAndApproachTrash"

    robot_state.current_time_seconds += APPROACH_TIMEOUT_SECONDS + 0.5
    robot_state.trash_target_time_seconds = robot_state.current_time_seconds
    tree.tick()
    assert tree.tip().name == "PatrolStreet"


def test_patrol_street_uses_imu_fused_odometry_for_corridor_heading_and_centerline():
    robot_state, tree, published_modes, published_twists, _, _ = _build_test_harness()
    # Robot displaced south of centerline (odom_y = -0.40 m) and angled right (yaw = -0.20 rad)
    robot_state.odom_y_meters = -0.40
    robot_state.current_yaw_radians = -0.20

    tree.tick()
    assert tree.tip().name == "PatrolStreet"
    assert published_modes[-1] == "PATROL"
    velocity_x, velocity_y, angular_velocity_z = published_twists[-1]
    assert math.isclose(velocity_x, PATROL_FORWARD_VELOCITY_X_MPS, rel_tol=1e-6)
    # Should strafe left (positive velocity_y) back toward odom_y = 0.0 and steer left (positive angular_z) back to yaw = 0.0
    assert velocity_y > 0.0
    assert angular_velocity_z > 0.0
