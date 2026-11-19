"""
test_arm_fsm.py — Unit tests for M3.2 waypoints and M3.3 ArmPickFSM.
Can be executed with pytest without requiring ROS 2 middleware or Webots.
"""

from isoroba_cleaner_pkg.arm_fsm_node import (
    ARM_WAYPOINTS,
    GRIPPER_CLOSED_METERS,
    GRIPPER_OPEN_METERS,
    GRIPPER_SETTLE_CYCLES_REQUIRED,
    OVERALL_PICK_TIMEOUT_SECONDS,
    STATE_TIMEOUT_SECONDS,
    WAYPOINT_DWELL_SECONDS,
    ArmFSMState,
    ArmPickFSM,
    PickStatus,
)
from isoroba_cleaner_pkg.youbot_driver import ARM_LIMITS


def _settle_gripper(fsm: ArmPickFSM, finger_position_m: float) -> None:
    """Advances enough ticks with constant finger_position_m to settle gripper."""
    for _ in range(GRIPPER_SETTLE_CYCLES_REQUIRED + 1):
        fsm.step(0.1, fsm.commanded_arm, (finger_position_m, finger_position_m))


def test_waypoints_within_youbot_joint_limits():
    for name, waypoint in ARM_WAYPOINTS.items():
        assert len(waypoint) == 5, f"{name} must have 5 joints"
        for angle, (lower, upper) in zip(waypoint, ARM_LIMITS):
            assert lower <= angle <= upper, f"{name} angle {angle} outside [{lower}, {upper}]"


def test_full_happy_path_pick_and_place_cycle():
    fsm = ArmPickFSM()
    assert fsm.state == ArmFSMState.IDLE
    assert fsm.status == PickStatus.IDLE
    assert fsm.commanded_arm == ARM_WAYPOINTS["HOME"]
    assert fsm.commanded_gripper == GRIPPER_OPEN_METERS

    assert fsm.trigger_pick() is True
    assert fsm.state == ArmFSMState.PRE_GRASP
    assert fsm.status == PickStatus.BUSY

    # PRE_GRASP -> GRASP (with WAYPOINT_DWELL_SECONDS dwell)
    fsm.step(WAYPOINT_DWELL_SECONDS, ARM_WAYPOINTS["PRE_GRASP"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.GRASP

    # GRASP -> CLOSE_GRIPPER (with WAYPOINT_DWELL_SECONDS dwell)
    fsm.step(WAYPOINT_DWELL_SECONDS, ARM_WAYPOINTS["GRASP"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.CLOSE_GRIPPER
    assert fsm.commanded_gripper == GRIPPER_CLOSED_METERS

    # Object held at 0.015 m per finger -> LIFT
    _settle_gripper(fsm, 0.015)
    assert fsm.state == ArmFSMState.LIFT

    # LIFT -> DROP_IN_BIN
    fsm.step(0.1, ARM_WAYPOINTS["LIFT"], (0.015, 0.015))
    assert fsm.state == ArmFSMState.DROP_IN_BIN

    # DROP_IN_BIN -> RELEASE_GRIPPER
    fsm.step(0.1, ARM_WAYPOINTS["DROP_IN_BIN"], (0.015, 0.015))
    assert fsm.state == ArmFSMState.RELEASE_GRIPPER
    assert fsm.commanded_gripper == GRIPPER_OPEN_METERS

    # RELEASE_GRIPPER -> RETURN_HOME -> IDLE (SUCCESS)
    _settle_gripper(fsm, GRIPPER_OPEN_METERS)
    assert fsm.state == ArmFSMState.RETURN_HOME

    fsm.step(0.1, ARM_WAYPOINTS["HOME"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.IDLE
    assert fsm.status == PickStatus.SUCCESS


def test_empty_grasp_aborts_to_home_with_failed_status():
    fsm = ArmPickFSM()
    fsm.trigger_pick()
    fsm.step(WAYPOINT_DWELL_SECONDS, ARM_WAYPOINTS["PRE_GRASP"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    fsm.step(WAYPOINT_DWELL_SECONDS, ARM_WAYPOINTS["GRASP"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.CLOSE_GRIPPER

    # Fingers close all the way to 0.0 m -> missed object
    _settle_gripper(fsm, 0.0)
    assert fsm.state == ArmFSMState.RETURN_HOME
    assert fsm.commanded_arm == ARM_WAYPOINTS["HOME"]
    assert fsm.commanded_gripper == GRIPPER_OPEN_METERS

    fsm.step(0.1, ARM_WAYPOINTS["HOME"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.IDLE
    assert fsm.status == PickStatus.FAILED


def test_abort_preempts_and_blocks_retrigger_until_cleared():
    fsm = ArmPickFSM()
    fsm.trigger_pick()
    fsm.step(WAYPOINT_DWELL_SECONDS, ARM_WAYPOINTS["PRE_GRASP"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.GRASP

    fsm.set_abort(True)
    assert fsm.state == ArmFSMState.RETURN_HOME
    assert fsm.commanded_arm == ARM_WAYPOINTS["HOME"]
    assert fsm.commanded_gripper == GRIPPER_OPEN_METERS

    fsm.step(0.1, ARM_WAYPOINTS["HOME"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.IDLE
    assert fsm.status == PickStatus.FAILED

    # Cannot trigger while abort is active
    assert fsm.trigger_pick() is False
    fsm.set_abort(False)
    assert fsm.trigger_pick() is True


def test_state_and_overall_timeouts_stow_to_home():
    fsm = ArmPickFSM()
    fsm.trigger_pick()
    # Stall in PRE_GRASP past STATE_TIMEOUT_SECONDS
    fsm.step(STATE_TIMEOUT_SECONDS + 0.1, ARM_WAYPOINTS["HOME"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.RETURN_HOME

    fsm.step(0.1, ARM_WAYPOINTS["HOME"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.IDLE
    assert fsm.status == PickStatus.FAILED

    # Overall timeout check
    fsm.trigger_pick()
    fsm.step(OVERALL_PICK_TIMEOUT_SECONDS + 0.1, ARM_WAYPOINTS["HOME"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.RETURN_HOME


def test_height_aware_grasp_selection():
    fsm = ArmPickFSM()
    # Tall item (e.g. 0.08 m can at z = -0.054 m) -> GRASP_TALL
    assert fsm.trigger_pick(target_z_meters=-0.054) is True
    fsm.step(WAYPOINT_DWELL_SECONDS, ARM_WAYPOINTS["PRE_GRASP"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.state == ArmFSMState.GRASP
    assert fsm.commanded_arm == ARM_WAYPOINTS["GRASP_TALL"]

    # Abort and verify low item (e.g. 0.05 m biscuit box at z = -0.075 m) -> GRASP
    fsm.set_abort(True)
    fsm.step(0.1, ARM_WAYPOINTS["HOME"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    fsm.set_abort(False)
    assert fsm.trigger_pick(target_z_meters=-0.075) is True
    fsm.step(WAYPOINT_DWELL_SECONDS, ARM_WAYPOINTS["PRE_GRASP"], (GRIPPER_OPEN_METERS, GRIPPER_OPEN_METERS))
    assert fsm.commanded_arm == ARM_WAYPOINTS["GRASP"]
