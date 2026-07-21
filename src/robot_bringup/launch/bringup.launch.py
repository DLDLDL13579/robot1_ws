import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    ekf_config_path = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    # 1. 传感器融合 EKF 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='robot1',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom_filtered')]
    )

    # 2. 履带底盘驱动节点 (保持不变)
    chassis_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_driver'), 'launch', 'robot_driver_launch.py')
        ),
        launch_arguments={'robot_namespace': 'robot1', 'port_name': '/dev/ttyACM0'}.items()
    )

    # 3. 新增：YDLidar 激光雷达驱动节点
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
        )
    )

    # 在 Return 中把雷达（lidar_driver_launch）加入最终的启动列表
    return LaunchDescription([
        chassis_driver_launch, 
        lidar_driver_launch,
        ekf_node
    ])