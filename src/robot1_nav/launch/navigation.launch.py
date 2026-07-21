import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('robot_bringup')
    my_nav_dir = get_package_share_directory('robot1_nav')

    # Map and nav2 params
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(my_nav_dir, 'maps', 'lab_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(my_nav_dir, 'config', 'nav2_params.yaml'))

    # === 1. 包含底盘、雷达与 EKF 的底层启动包 ===
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
    )

    # === 2. Nav2 navigation stack (AMCL + costmaps + planner + controller) ===
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': 'robot1',
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': 'false', 'use_namespace': 'true',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        bringup_launch,
        nav2_launch,
    ])