#!/usr/bin/env python3
"""
独立 Nav2 导航启动文件 - 简洁可靠版
仅启动导航相关节点（controller/planner/bt_navigator 等）
定位节点（map_server/amcl）已在 soldier.launch.py 中启动
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot1_nav_dir = get_package_share_directory("robot1_nav")
    nav2_params = os.path.join(robot1_nav_dir, "config", "nav2_params.yaml")
    bt_xml = "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"

    ns = "/robot_1"
    remaps = [("tf", "tf"), ("tf_static", "tf_static")]

    # ── Controller Server ──
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=ns,
        output="screen",
        parameters=[nav2_params],
        remappings=remaps + [
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # ── Planner Server ──
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=ns,
        output="screen",
        parameters=[nav2_params],
        remappings=remaps,
    )

    # ── Smoother Server ──
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        namespace=ns,
        output="screen",
        parameters=[nav2_params],
        remappings=remaps,
    )

    # ── BT Navigator ──
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=ns,
        output="screen",
        parameters=[nav2_params, {
            "bt_xml_filename": bt_xml,
        }],
        remappings=remaps + [
            ("goal_pose", "goal_pose"),
            ("path", "path"),
        ],
    )

    # ── Behavior Server（提供 spin/back_up/wait 等恢复行为）───
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=ns,
        output="screen",
        parameters=[nav2_params],
        remappings=remaps,
    )

    # ── Waypoint Follower ──
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        namespace=ns,
        output="screen",
        parameters=[nav2_params],
        remappings=remaps + [
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # ── Velocity Smoother ──
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        namespace=ns,
        output="screen",
        parameters=[nav2_params],
        remappings=remaps + [
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    # ── Lifecycle Manager（只管导航节点，不管定位）───
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
                "bond_timeout": 30.0,
            "node_names": [
                "controller_server",
                "planner_server",
                "smoother_server",
                "bt_navigator",
                "behavior_server",
                "waypoint_follower",
                "velocity_smoother",
            ],
        }],
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        smoother_server,
        bt_navigator,
        behavior_server,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
