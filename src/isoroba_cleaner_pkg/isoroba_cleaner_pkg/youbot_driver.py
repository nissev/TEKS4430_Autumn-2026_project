"""
youbot_driver.py — Webots ROS 2 Python Driver Plugin for KUKA YouBot.

Bridges:
  1. Mecanum 4-wheel base (/cmd_vel -> motors, encoders -> /wheel_odom_raw).
  2. Camera Recognition (/camera/recognized_trash).
  3. 5-DOF Arm & parallel gripper (/arm/joint_1..5/cmd, /arm/gripper/cmd, /joint_states).
"""

import math
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import Twist, Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

# Robot geometry & limits
WHEEL_RADIUS = 0.05
# Kinematic lever arm = half wheelbase (0.228 m) + half trackwidth (0.158 m)
KINEMATIC_LEVER_ARM = 0.228 + 0.158
MAX_WHEEL_SPEED = 14.0  # rad/s

ARM_LIMITS = [
    (-2.9496, 2.9496),      # arm1 (base pan)
    (-1.13446, 1.5708),     # arm2 (shoulder)
    (-2.63545, 2.54818),    # arm3 (elbow)
    (-1.78024, 1.78024),    # arm4 (wrist pitch)
    (-2.92343, 2.92343),    # arm5 (wrist roll)
]
GRIPPER_LIMIT = (0.0, 0.025)

ARM_NAMES = [f"arm{i}" for i in range(1, 6)]
HOME_ARM_POSITIONS = (0.0, 0.80, -1.40, 1.00, 0.0)
GRIPPER_NAMES = ["finger::left", "finger::right"]
WHEEL_NAMES = [f"wheel{i}" for i in range(1, 5)]


def get_motor_and_sensor(robot, device_name: str, timestep_ms: int, is_velocity: bool = False):
    """Utility to fetch a motor and enable its built-in position sensor."""
    motor = robot.getDevice(device_name)
    if motor and is_velocity:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    sensor = motor.getPositionSensor() if motor else None
    if sensor:
        sensor.enable(timestep_ms)
    return motor, sensor


def quaternion_from_axis_angle(axis_x: float, axis_y: float, axis_z: float, angle: float):
    """Converts Webots axis-angle [axis_x, axis_y, axis_z, angle] to quaternion (x, y, z, w)."""
    norm = math.hypot(axis_x, axis_y, axis_z)
    if norm > 1e-6:
        sin_half = math.sin(angle / 2.0) / norm
        return axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, math.cos(angle / 2.0)
    return 0.0, 0.0, 0.0, 1.0


class YouBotDriver:

    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        timestep_ms = int(self.robot.getBasicTimeStep())
        self.timestep_seconds = timestep_ms / 1000.0

        # Actuators & Position Sensors
        self.wheels, self.wheel_sensors = zip(
            *(get_motor_and_sensor(self.robot, device_name, timestep_ms, True) for device_name in WHEEL_NAMES)
        )
        self.arm, self.arm_sensors = zip(
            *(get_motor_and_sensor(self.robot, device_name, timestep_ms) for device_name in ARM_NAMES)
        )
        self.grippers, self.grip_sensors = zip(
            *(get_motor_and_sensor(self.robot, device_name, timestep_ms) for device_name in GRIPPER_NAMES)
        )

        for motor, home_angle in zip(self.arm, HOME_ARM_POSITIONS):
            motor.setPosition(home_angle)
        for motor in self.grippers:
            motor.setPosition(GRIPPER_LIMIT[1])

        # Camera with Webots Recognition
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(timestep_ms)
            self.camera.recognitionEnable(timestep_ms)

        # InertialUnit for drift-free corridor yaw
        self.inertial_unit = self.robot.getDevice("inertial_unit")
        if self.inertial_unit:
            self.inertial_unit.enable(timestep_ms)

        # ROS 2 Node & Interfaces
        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass
        self.node = rclpy.create_node("youbot_driver")

        self.node.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.odometry_pub = self.node.create_publisher(Odometry, "/wheel_odom_raw", 10)
        self.trash_pub = self.node.create_publisher(PoseArray, "/camera/recognized_trash", 10)
        self.trash_models_pub = self.node.create_publisher(String, "/camera/recognized_trash_models", 10)
        self.joint_state_pub = self.node.create_publisher(JointState, "/joint_states", 10)

        for i in range(5):
            self.node.create_subscription(
                Float64, f"/arm/joint_{i+1}/cmd", lambda msg, joint_idx=i: self._arm_callback(msg, joint_idx), 10
            )
        self.node.create_subscription(Float64, "/arm/gripper/cmd", self._gripper_callback, 10)

        # Driver State
        self.target_velocity = [0.0, 0.0, 0.0]
        self.previous_wheel_positions = None
        self.odometry_pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.ready = True

    def _cmd_vel_callback(self, msg: Twist):
        self.target_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

    def _arm_callback(self, msg: Float64, joint_idx: int):
        """
        Sets arm position and checks that joint limits are not exceeded.
        """
        lower_limit, upper_limit = ARM_LIMITS[joint_idx]
        self.arm[joint_idx].setPosition(max(lower_limit, min(upper_limit, msg.data)))

    def _gripper_callback(self, msg: Float64):
        """
        Sets gripper position and checks that gripper limits are not exceeded
        """
        position = max(GRIPPER_LIMIT[0], min(GRIPPER_LIMIT[1], msg.data))
        for motor in self.grippers:
            motor.setPosition(position)

    def step(self):
        if not self.ready:
            return

        simulation_time = self.robot.getTime()
        stamp = Time(nanoseconds=int(round(simulation_time * 1e9))).to_msg()
        rclpy.spin_once(self.node, timeout_sec=0)

        # ── 1. Mecanum Inverse Kinematics ─────────────────────────
        velocity_x, velocity_y, angular_velocity_z = self.target_velocity
        wheel_speeds = [
            (velocity_x + velocity_y + KINEMATIC_LEVER_ARM * angular_velocity_z) / WHEEL_RADIUS,
            (velocity_x - velocity_y - KINEMATIC_LEVER_ARM * angular_velocity_z) / WHEEL_RADIUS,
            (velocity_x - velocity_y + KINEMATIC_LEVER_ARM * angular_velocity_z) / WHEEL_RADIUS,
            (velocity_x + velocity_y - KINEMATIC_LEVER_ARM * angular_velocity_z) / WHEEL_RADIUS,
        ]
        scaling_factor = max(1.0, max(abs(speed) for speed in wheel_speeds) / MAX_WHEEL_SPEED)
        for motor, speed in zip(self.wheels, wheel_speeds):
            motor.setVelocity(speed / scaling_factor)

        # ── 2. Mecanum Odometry Integration ───────────────────────
        current_wheel_positions = [sensor.getValue() if sensor else 0.0 for sensor in self.wheel_sensors]
        # Need previous scan.
        if self.previous_wheel_positions:
            # Gather changes in wheel angles
            delta_wheel_angles = [
                current - previous
                for current, previous in zip(current_wheel_positions, self.previous_wheel_positions)
            ]
            # Calculate x and y change
            delta_x = (WHEEL_RADIUS / 4.0) * sum(delta_wheel_angles)
            delta_y = (WHEEL_RADIUS / 4.0) * (
                delta_wheel_angles[0] - delta_wheel_angles[1] - delta_wheel_angles[2] + delta_wheel_angles[3]
            )
            # Calculate the yaw change (CCW positive)
            delta_theta = (WHEEL_RADIUS / (4.0 * KINEMATIC_LEVER_ARM)) * (
                delta_wheel_angles[0] - delta_wheel_angles[1] + delta_wheel_angles[2] - delta_wheel_angles[3]
            )
            # Fuse InertialUnit yaw when available for drift-free heading & position integration
            if self.inertial_unit:
                current_yaw = float(self.inertial_unit.getRollPitchYaw()[2])
                mid_yaw = current_yaw
                self.odometry_pose[2] = current_yaw
            else:
                mid_yaw = self.odometry_pose[2] + delta_theta / 2.0
                self.odometry_pose[2] += delta_theta
            # Map where robot is to pose
            self.odometry_pose[0] += delta_x * math.cos(mid_yaw) - delta_y * math.sin(mid_yaw)
            self.odometry_pose[1] += delta_x * math.sin(mid_yaw) + delta_y * math.cos(mid_yaw)

            # Create odometry msg
            odom_msg = Odometry()
            odom_msg.header.stamp, odom_msg.header.frame_id, odom_msg.child_frame_id = stamp, "odom", "base_link"
            odom_msg.pose.pose.position.x = self.odometry_pose[0]
            odom_msg.pose.pose.position.y = self.odometry_pose[1]
            odom_msg.pose.pose.orientation.z = math.sin(self.odometry_pose[2] / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(self.odometry_pose[2] / 2.0)
            odom_msg.twist.twist.linear.x = delta_x / self.timestep_seconds if self.timestep_seconds > 0 else 0.0
            odom_msg.twist.twist.linear.y = delta_y / self.timestep_seconds if self.timestep_seconds > 0 else 0.0
            odom_msg.twist.twist.angular.z = delta_theta / self.timestep_seconds if self.timestep_seconds > 0 else 0.0
            self.odometry_pub.publish(odom_msg)
        self.previous_wheel_positions = current_wheel_positions

        # ── 3. Camera Recognition ─────────────────────────────────
        # Pure hardware bridge: forwards all raw camera-recognized objects and
        # their raw model names to perception_node without filtering.
        if self.camera:
            detected_objects = self.camera.getRecognitionObjects() or []
            poses_msg = PoseArray()
            poses_msg.header.stamp, poses_msg.header.frame_id = stamp, "camera_link"
            model_names = []
            for detected_object in detected_objects:
                model_names.append((detected_object.getModel() or "").strip())
                litter_pose = Pose()
                litter_pose.position.x, litter_pose.position.y, litter_pose.position.z = detected_object.getPosition()
                axis_angle_orientation = detected_object.getOrientation()
                if axis_angle_orientation and len(axis_angle_orientation) == 4:
                    (
                        litter_pose.orientation.x,
                        litter_pose.orientation.y,
                        litter_pose.orientation.z,
                        litter_pose.orientation.w,
                    ) = quaternion_from_axis_angle(
                        axis_angle_orientation[0],
                        axis_angle_orientation[1],
                        axis_angle_orientation[2],
                        axis_angle_orientation[3],
                    )
                else:
                    litter_pose.orientation.w = 1.0
                poses_msg.poses.append(litter_pose)
            models_msg = String()
            models_msg.data = ",".join(model_names)
            self.trash_models_pub.publish(models_msg)
            self.trash_pub.publish(poses_msg)

        # ── 4. Joint State Telemetry ──────────────────────────────
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = stamp
        joint_state_msg.name = ARM_NAMES + GRIPPER_NAMES
        joint_state_msg.position = [
            sensor.getValue() if sensor else 0.0 for sensor in self.arm_sensors + self.grip_sensors
        ]
        self.joint_state_pub.publish(joint_state_msg)
