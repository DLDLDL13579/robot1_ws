from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value=(
                "/dev/serial/by-id/"
                "usb-1a86_USB_Single_Serial_5B21242534-if00"
            ),
        ),
        Node(
            package="gimbal_control_cpp",
            executable="gimbal_controller",
            name="gimbal_controller",
            namespace="robot_1/gimbal",
            output="screen",
            parameters=[{
                "port_name": port_name,
                "baud_rate": 115200,
                "yaw_level_deg": -20.0,
                "pitch_level_deg": -175.0,
                "move_to_level_on_start": True,
            }],
        ),
    ])