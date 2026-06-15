"""
cartesian_spray.launch.py
=========================
Runs the Cartesian path executor against the active MoveIt / Gazebo session.

Usage:
  ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py
  ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py \
      poses_file:=/absolute/path/to/my_poses.yaml \
      velocity_scaling:=0.3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gz_spray_painting_plugin_demo")
    default_poses = os.path.join(pkg_share, "config", "cartesian_poses.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "poses_file",
            default_value=default_poses,
            description="Absolute path to the Cartesian poses YAML file.",
        ),
        DeclareLaunchArgument(
            "planning_group",
            default_value="ur_manipulator",
            description="MoveIt planning group name.",
        ),
        DeclareLaunchArgument(
            "eef_step",
            default_value="0.005",
            description="Cartesian interpolation step in metres.",
        ),
        DeclareLaunchArgument(
            "velocity_scaling",
            default_value="0.1",
            description="Fraction of maximum joint velocity (0–1).",
        ),
        DeclareLaunchArgument(
            "spray_enabled",
            default_value="true",
            description="Publish spray trigger around execution.",
        ),
        Node(
            package="gz_spray_painting_plugin_demo",
            executable="cartesian_path_executor.py",
            name="cartesian_path_executor",
            output="screen",
            parameters=[{
                "use_sim_time":     True,
                "poses_file":       LaunchConfiguration("poses_file"),
                "planning_group":   LaunchConfiguration("planning_group"),
                "eef_step":         LaunchConfiguration("eef_step"),
                "velocity_scaling": LaunchConfiguration("velocity_scaling"),
                "spray_enabled":    LaunchConfiguration("spray_enabled"),
            }],
        ),
    ])
