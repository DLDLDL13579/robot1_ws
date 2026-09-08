#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.')

    # Multi-robot strategy:
    # We do NOT put the driver in a namespace, because ydlidar.yaml uses the
    # bare node name `ydlidar_ros2_driver_node` as its top-level key, and
    # the namespace would change the key to `robot1.ydlidar_ros2_driver_node`
    # so the YAML parameters (port, baudrate, ...) would silently fall back
    # to the defaults baked into the driver source.
    #
    # Instead, we keep the driver global and REMAP its 'scan' topic to
    # '/<robot_namespace>/scan'. The static-TF frame names also get prefixed.
    robot_namespace = LaunchConfiguration('robot_namespace', default='')

    scan_topic = PythonExpression([
        "'/", robot_namespace, "/scan' if '", robot_namespace, "' else 'scan'",
    ])
    frame_id_value = PythonExpression([
        "'", robot_namespace, "/laser_frame' if '", robot_namespace, "' else 'laser_frame'",
    ])
    base_link_frame = PythonExpression([
        "'", robot_namespace, "/base_link' if '", robot_namespace, "' else 'base_link'",
    ])

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file, {'frame_id': frame_id_value}],
        remappings=[('scan', scan_topic)],
        namespace="/",
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.02',
            '--roll', '0.0',
            '--pitch', '3.14159',  # 2026-09-08 翻面+180度偏航补偿(Rz(pi)*Rx(pi)=Ry(pi)): 修正激光0度朝向指向车尾导致车向反转
            '--yaw', '0.0',
            '--frame-id', base_link_frame,
            '--child-frame-id', frame_id_value
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='',
                              description='Top-level robot namespace (e.g. robot1)'),
        params_declare,
        driver_node,
        tf2_node,
    ])