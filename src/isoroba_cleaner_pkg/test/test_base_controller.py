"""
test_base_controller.py — Unit tests for M3.1 Holonomic Base PID logic.
Can be executed with pytest without requiring ROS 2 middleware or Webots.
"""

import math
from isoroba_cleaner_pkg.base_controller_node import (
    ANGULAR_Z_GAINS,
    LINEAR_X_GAINS,
    LINEAR_Y_GAINS,
    MAX_ANGULAR_Z_RADPS,
    MAX_LINEAR_X_MPS,
    MAX_LINEAR_Y_MPS,
    PIDAxis,
    TARGET_STANDOFF_X_METERS,
    clamp_value,
    compute_approach_errors,
)


def test_clamp_value():
    assert clamp_value(5.0, -1.0, 1.0) == 1.0
    assert clamp_value(-3.0, -1.0, 1.0) == -1.0
    assert clamp_value(0.25, -1.0, 1.0) == 0.25


def test_pid_axis_clamps_output_and_integral_anti_windup():
    axis = PIDAxis(gains=(2.0, 1.0, 0.0), output_limit=0.5, integral_limit=0.3)
    for _ in range(20):
        output = axis.compute(error=10.0, delta_time_seconds=0.05)
    assert math.isclose(axis.integral_error, 0.3, rel_tol=1e-6)
    assert math.isclose(output, 0.5, rel_tol=1e-6)
    assert axis.compute(error=1.0, delta_time_seconds=0.0) == 0.0
    assert axis.compute(error=float("nan"), delta_time_seconds=0.05) == 0.0


def test_approach_drives_forward_and_strafes_left_for_front_left_target():
    linear_x_pid = PIDAxis(LINEAR_X_GAINS, MAX_LINEAR_X_MPS)
    linear_y_pid = PIDAxis(LINEAR_Y_GAINS, MAX_LINEAR_Y_MPS)
    angular_z_pid = PIDAxis(ANGULAR_Z_GAINS, MAX_ANGULAR_Z_RADPS)

    error_x, error_y, error_theta = compute_approach_errors(1.20, 0.30)
    assert linear_x_pid.compute(error_x, 0.05) > 0.0
    assert linear_y_pid.compute(error_y, 0.05) > 0.0
    assert angular_z_pid.compute(error_theta, 0.05) > 0.0


def test_approach_backs_up_when_closer_than_standoff():
    linear_x_pid = PIDAxis(LINEAR_X_GAINS, MAX_LINEAR_X_MPS)
    linear_y_pid = PIDAxis(LINEAR_Y_GAINS, MAX_LINEAR_Y_MPS)
    angular_z_pid = PIDAxis(ANGULAR_Z_GAINS, MAX_ANGULAR_Z_RADPS)

    error_x, error_y, error_theta = compute_approach_errors(
        TARGET_STANDOFF_X_METERS - 0.20, 0.0
    )
    assert linear_x_pid.compute(error_x, 0.05) < 0.0
    assert math.isclose(linear_y_pid.compute(error_y, 0.05), 0.0, abs_tol=1e-9)
    assert math.isclose(angular_z_pid.compute(error_theta, 0.05), 0.0, abs_tol=1e-9)
