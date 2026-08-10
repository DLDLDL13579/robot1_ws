import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_rd = "/home/sunrise/robot1_ws/install/robot_driver"
if _rd not in os.environ.get("AMENT_PREFIX_PATH", ""):
    os.environ["AMENT_PREFIX_PATH"] = _rd + ":" + os.environ.get("AMENT_PREFIX_PATH", "")

RELAY_SCRIPT = "/home/sunrise/robot1_ws/src/robot1_nav/scripts/map_relay.py"

def generate_launch_description():
    ns = "robot_1"
    driver_dir = get_package_share_directory("robot_driver")
    ydlidar_dir = get_package_share_directory("ydlidar_ros2_driver")
    my_dir = get_package_share_directory("robot1_nav")
    sld_dir = get_package_share_directory("robot1_soldier")

    map_file = LaunchConfiguration("map", default=os.path.join(my_dir, "maps", "lab_map.yaml"))
    soldier_params = os.path.join(sld_dir, "config", "soldier_params.yaml")
    soldier_ekf = os.path.join(sld_dir, "config", "soldier_ekf.yaml")
    remaps = [("tf", "/tf"), ("tf_static", "/tf_static")]

    chassis = Node(package="robot_driver", executable="robot_driver_node", name="robot_driver_node",
                   output="screen",
                   parameters=[{"robot_namespace": ns, "port_name": "/dev/ttyACM0", "baud_rate": 115200, "linear_scale": 1.58, "angular_scale": 0.62}])

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ydlidar_dir, "launch", "ydlidar_launch.py")),
        launch_arguments={"robot_namespace": ns}.items())

    ekf = Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
               namespace=ns, output="screen", parameters=[soldier_ekf],
               remappings=[("odometry/filtered", "odom_filtered")])

    map_server = Node(package="nav2_map_server", executable="map_server", name="map_server",
                      namespace=ns, output="screen",
                      parameters=[soldier_params, {"yaml_filename": map_file}],
                      remappings=remaps)

    amcl = Node(package="nav2_amcl", executable="amcl", name="amcl",
                namespace=ns, output="screen",
                parameters=[soldier_params], remappings=remaps)

    lcm = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
               name="lifecycle_manager_localization", namespace=ns, output="screen",
               parameters=[{"use_sim_time": False}, {"autostart": True},
                           {"node_names": ["map_server", "amcl"]}])

    pos_pub = Node(package="robot1_soldier", executable="position_publisher", name="position_publisher",
                   namespace=ns, output="screen",
                   parameters=[{"robot_id": ns}])

    # ★ Relay: subscribe robot_0's /map → publish to /robot_1/map
    relay_node = ExecuteProcess(
        cmd=["bash", "-c",
             "source /opt/ros/humble/setup.bash && "
             "source /home/sunrise/robot1_ws/install/setup.bash && "
             "python3 " + RELAY_SCRIPT],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(my_dir, "maps", "lab_map.yaml")),
        chassis, lidar, ekf, map_server, amcl, lcm, pos_pub,
        TimerAction(period=2.0, actions=[relay_node]),
    ])
