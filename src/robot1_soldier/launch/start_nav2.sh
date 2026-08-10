#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot1_ws/install/setup.bash
ros2 launch robot1_nav nav2_bringup.launch.py
