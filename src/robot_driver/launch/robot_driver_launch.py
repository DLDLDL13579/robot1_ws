from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('robot_namespace', default_value=''),
        DeclareLaunchArgument('wheel_track', default_value='0.13'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0225'),
        DeclareLaunchArgument('slip_factor', default_value='1.0'),
        DeclareLaunchArgument('linear_scale', default_value='1.0'),
        DeclareLaunchArgument('angular_scale', default_value='1.0'),
        Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver_node',
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'wheel_track': LaunchConfiguration('wheel_track'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'slip_factor': LaunchConfiguration('slip_factor'),
                'linear_scale': LaunchConfiguration('linear_scale'),
                'angular_scale': LaunchConfiguration('angular_scale'),
            }],
            output='screen',
        ),
    ])
