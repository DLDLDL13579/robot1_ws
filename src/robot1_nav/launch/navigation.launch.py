import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_rd = '/home/sunrise/robot1_ws/install/robot_driver'
if _rd not in os.environ.get('AMENT_PREFIX_PATH', ''):
    os.environ['AMENT_PREFIX_PATH'] = _rd + ':' + os.environ.get('AMENT_PREFIX_PATH', '')

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    my_nav_dir = get_package_share_directory('robot1_nav')

    map_yaml_file = LaunchConfiguration('map', default=os.path.join(my_nav_dir, 'maps', 'lab_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(my_nav_dir, 'config', 'nav2_params.yaml'))

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
    )

    # Custom localization: map_server with inline yaml_filename override
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_nav_dir, 'launch', 'localization.launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    # Custom navigation: controller_server with inline critics override
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_nav_dir, 'launch', 'navigation_custom.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    return LaunchDescription([
        bringup_launch,
        localization_launch,
        navigation_launch,
    ])
