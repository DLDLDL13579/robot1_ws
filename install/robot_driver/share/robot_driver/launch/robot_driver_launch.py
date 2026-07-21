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

        Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver_node',
            # 不设置 ROS2 namespace，由节点内部通过参数 robot_namespace 控制 topic 前缀
            # 避免双层嵌套（/robot1/robot1/odom）
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'wheel_track': LaunchConfiguration('wheel_track'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'slip_factor': LaunchConfiguration('slip_factor'),
            }],
            output='screen',
        ),
    ])