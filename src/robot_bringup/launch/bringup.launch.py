import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    ekf_config_path = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    robot_namespace = LaunchConfiguration('robot_namespace', default='')
    port_name = LaunchConfiguration('port_name', default='/dev/ttyACM0')

    # 1. 传感器融合 EKF (in robot1 namespace)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom_filtered')],
    )

    # 2. 履带底盘驱动 (passes robot_namespace down to the driver itself)
    chassis_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_driver'),
                         'launch', 'robot_driver_launch.py')),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'port_name': port_name,
            'baud_rate': '115200',
        }.items(),
    )

    # 3. YDLidar (now passes namespace so it publishes /robot1/scan
    #    and TF uses robot1/laser_frame)
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'),
                         'launch', 'ydlidar_launch.py')),
        launch_arguments={'robot_namespace': robot_namespace}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='',
                              description='Top-level robot namespace'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0',
                              description='Serial port of chassis'),
        chassis_driver_launch,
        lidar_driver_launch,
        ekf_node,
    ])