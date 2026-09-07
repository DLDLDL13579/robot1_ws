#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot1_soldier_dir = get_package_share_directory("robot1_soldier")
    soldier_launch = os.path.join(robot1_soldier_dir, "launch", "soldier.launch.py")

    # 启动 Nav2 的脚本
    nav2_script = "/home/xumeng/robot1_ws/src/robot1_soldier/launch/start_nav2.sh"

    return LaunchDescription([
        # 1. 启动 soldier.launch.py（底盘 + 定位 + map_relay）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(soldier_launch)
        ),

        # 2. 延迟 8 秒启动 Nav2
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=["bash", nav2_script],
                    output="screen",
                    name="nav2_bringup"
                )
            ]
        ),
    ])
