#!/usr/bin/env python3
"""
cleaner.launch.py — Webots ROS 2 Driver Launch for Iso-Roba Clean-Up Robot.

Prerequisites:
  - Webots R2025a is open with worlds/isoroba_arena.wbt running (or started).
  - ROS 2 Jazzy workspace is built and sourced.

Usage:
  ros2 launch isoroba_cleaner_pkg cleaner.launch.py
"""

import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    pkg = get_package_share_directory("isoroba_cleaner_pkg")
    robot_urdf = os.path.join(pkg, "resource", "youbot.urdf")

    youbot_driver = WebotsController(
        robot_name="youbot",
        parameters=[
            {"robot_description": robot_urdf},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            youbot_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=youbot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
