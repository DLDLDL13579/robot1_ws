import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav_dir = get_package_share_directory('robot1_nav')

    map_yaml_file = LaunchConfiguration('map', default=os.path.join(my_nav_dir, 'maps', 'lab_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(my_nav_dir, 'config', 'nav2_params.yaml'))

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': 'robot1',
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([nav2_launch])
