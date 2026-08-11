import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav_dir = get_package_share_directory("robot1_nav")
    nav2_params = os.path.join(nav_dir, "config", "nav2_params.yaml")
    ns = "/robot_1"
    r = [("tf","/tf"), ("tf_static","/tf_static")]

    controller = Node(package="nav2_controller", executable="controller_server",
        name="controller_server", namespace=ns, parameters=[nav2_params], remappings=r)

    planner = Node(package="nav2_planner", executable="planner_server",
        name="planner_server", namespace=ns, parameters=[nav2_params], remappings=r)

    behavior = Node(package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", namespace=ns, parameters=[nav2_params], remappings=r)

    bt_xml = "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    bt = Node(package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", namespace=ns,
        parameters=[nav2_params, {"bt_xml_filename": bt_xml}],
        remappings=r + [("cmd_vel","/robot_1/cmd_vel")])

    mgr = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", namespace=ns,
        parameters=[{"use_sim_time": False, "autostart": True, "bond_timeout": 30.0,
            "node_names": ["controller_server","planner_server","behavior_server","bt_navigator"]}])

    return LaunchDescription([controller, planner, behavior, bt, mgr])
