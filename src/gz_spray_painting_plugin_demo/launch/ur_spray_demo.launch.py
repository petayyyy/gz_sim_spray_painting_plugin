"""
ur_spray_demo.launch.py
=======================
Single entry-point for the UR5e spray painting demo:

  * Gazebo Harmonic          — loads factory_demo.sdf world
  * ros_gz_bridge            — /clock (GZ→ROS) + /spray_paint/trigger (ROS→GZ)
  * robot_state_publisher    — publishes TF from the spray URDF
  * joint_state_publisher    — provides correct initial pose until JSB starts
  * gz service create        — injects the robot at T+8 s
  * controller spawners      — joint_state_broadcaster (T+20 s) +
                               joint_trajectory_controller (T+25 s)
  * MoveIt 2                 — move_group + RViz

Usage:
  ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py
  ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py headless:=true ur_type:=ur5e
"""

import os
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type  = LaunchConfiguration("ur_type")
    headless = LaunchConfiguration("headless")

    # Assets live in gz_sim_spray_painting_plugin (world, URDF, bridge config, RViz config).
    spray_pkg_share  = get_package_share_directory("gz_sim_spray_painting_plugin")
    spray_pkg_prefix = get_package_prefix("gz_sim_spray_painting_plugin")

    description_file = os.path.join(spray_pkg_share, "urdf", "ur_spray_gz.urdf.xacro")
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("gz_spray_painting_plugin_demo"), "config", "ur_sim_controllers.yaml"]
    ).perform(context)

    # ── Build robot description URDF ─────────────────────────────────────────
    ur_type_str = ur_type.perform(context)
    xacro_cmd = [
        "xacro", description_file,
        f"ur_type:={ur_type_str}",
        f"simulation_controllers:={controllers_file}",
        "safety_limits:=true",
        "name:=ur",
        "tf_prefix:=",
    ]
    xacro_env = os.environ.copy()
    demo_pkg_prefix = "/ws/install/gz_spray_painting_plugin_demo"
    if os.path.isdir(demo_pkg_prefix):
        existing = xacro_env.get("AMENT_PREFIX_PATH", "")
        xacro_env["AMENT_PREFIX_PATH"] = f"{demo_pkg_prefix}:{existing}" if existing else demo_pkg_prefix
    urdf_str = subprocess.check_output(xacro_cmd, env=xacro_env, stderr=subprocess.PIPE).decode()
    robot_description = {"robot_description": ParameterValue(urdf_str, value_type=str)}

    # ── Gazebo environment ────────────────────────────────────────────────────
    set_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            os.path.join(spray_pkg_prefix, "lib", "gz_sim_spray_painting_plugin"),
            ":/ws/install/gz_ros2_control/lib:",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.dirname(spray_pkg_share) + ":",
            "/ws/install/gz_spray_painting_plugin_demo/share:",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    ur_sim_pkg_share = get_package_share_directory("gz_spray_painting_plugin_demo")
    world_path = os.path.join(ur_sim_pkg_share, "worlds", "factory_demo.sdf")

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r", "-v", "4"],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", world_path, "-r", "-v", "4"],
        output="screen",
        condition=IfCondition(headless),
    )

    # ── ros_gz_bridge ─────────────────────────────────────────────────────────
    bridge_config = os.path.join(spray_pkg_share, "config", "ros_gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # ── joint_state_publisher (initial pose until JSB starts at T+20 s) ──────
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            robot_description,
            {
                "rate": 10,
                "zeros.shoulder_lift_joint": -1.5708,
                "zeros.elbow_joint":          1.5708,
                "zeros.wrist_1_joint":        -1.5708,
                "zeros.wrist_2_joint":        -1.5708,
            },
        ],
    )

    # ── Spawn robot at T+8 s ──────────────────────────────────────────────────
    urdf_tmp = "/tmp/ur_spray_generated.urdf"
    with open(urdf_tmp, "w") as f:
        f.write(urdf_str)

    spawn_robot = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=[
                "gz", "service",
                "-s", "/world/factory/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "30000",
                "--req",
                (
                    f'sdf_filename: "{urdf_tmp}" '
                    f'name: "ur" '
                    f'allow_renaming: false '
                    f'pose: {{ position: {{x: 0.50 y: -5.90 z: 0.80}} '
                    f'orientation: {{x: 0 y: 0 z: 0.7071 w: 0.7071}} }}'
                ),
            ],
            output="screen",
        )],
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    joint_state_broadcaster_spawner = TimerAction(
        period=20.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )],
    )
    joint_trajectory_controller_spawner = TimerAction(
        period=25.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "-c", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )],
    )

    # ── MoveIt 2 ─────────────────────────────────────────────────────────────
    moveit_actions = []
    try:
        from ur_moveit_config.launch_common import load_yaml  # noqa: PLC0415
        import re as _re

        moveit_pkg_share = get_package_share_directory("ur_moveit_config")

        srdf_xacro = os.path.join(moveit_pkg_share, "srdf", "ur.srdf.xacro")
        srdf_str = subprocess.check_output(
            ["xacro", srdf_xacro, "name:=ur", "prefix:="],
            env=xacro_env, stderr=subprocess.PIPE,
        ).decode()
        srdf_str = _re.sub(
            r'(<chain\b[^>]*\btip_link=")[^"]+(")',
            r'\1spray_gun_nozzle_link\2',
            srdf_str,
        )
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(srdf_str, value_type=str)
        }

        kinematics_yaml = load_yaml("gz_spray_painting_plugin_demo", "config/kinematics.yaml")
        robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

        joint_limits_yaml  = load_yaml("ur_moveit_config", "config/joint_limits.yaml")
        ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
        controllers_yaml   = load_yaml("ur_moveit_config", "config/controllers.yaml")

        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

        ompl_pipeline = {
            "move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": (
                    "default_planner_request_adapters/AddTimeOptimalParameterization "
                    "default_planner_request_adapters/FixWorkspaceBounds "
                    "default_planner_request_adapters/FixStartStateBounds "
                    "default_planner_request_adapters/FixStartStateCollision "
                    "default_planner_request_adapters/FixStartStatePathConstraints"
                ),
                "start_state_max_bounds_error": 0.1,
            }
        }
        ompl_pipeline["move_group"].update(ompl_planning_yaml)

        moveit_controllers = {
            "moveit_simple_controller_manager": controllers_yaml,
            "moveit_controller_manager":
                "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }
        trajectory_execution = {
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
            "trajectory_execution.execution_duration_monitoring": False,
        }
        planning_scene_monitor = {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }
        warehouse_cfg = {
            "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
            "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
        }
        initial_positions = {
            "initial_positions": {
                "shoulder_pan_joint":  0.0,
                "shoulder_lift_joint": -1.5708,
                "elbow_joint":          1.5708,
                "wrist_1_joint":       -1.5708,
                "wrist_2_joint":       -1.5708,
                "wrist_3_joint":        0.0,
            }
        }

        common_params = [
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"robot_description_planning": joint_limits_yaml},
            ompl_pipeline,
            {"use_sim_time": True},
            warehouse_cfg,
            initial_positions,
        ]

        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=common_params + [
                {"publish_robot_description_semantic": True},
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor,
            ],
        )

        rviz_cfg = os.path.join(spray_pkg_share, "config", "moveit.rviz")
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", rviz_cfg],
            parameters=common_params,
        )

        moveit_actions = [move_group_node, rviz_node]

    except Exception as exc:
        import traceback
        print(f"\n[ur_spray_demo] MoveIt failed to load — skipping.\n"
              f"  Error: {exc}\n{traceback.format_exc()}")

    return [
        set_plugin_path,
        set_resource_path,
        gazebo,
        gazebo_headless,
        ros_gz_bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        *moveit_actions,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of UR robot.",
            choices=[
                "ur3", "ur5", "ur10",
                "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e",
                "ur8long", "ur15", "ur18", "ur20", "ur30",
            ],
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server only (no GUI).",
        ),
        SetEnvironmentVariable(name="GZ_VERSION", value="harmonic"),
        OpaqueFunction(function=launch_setup),
    ])
