import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}
    configured_params = ParameterFile(
        RewrittenYaml(source_file=params_file, root_key=namespace,
                      param_rewrites=param_substitutions, convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # 定义我们要强制覆盖的 AMCL 核心参数
    amcl_override_params = {
        'base_frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'set_initial_pose': True,
        'initial_pose.x': 0.0,
        'initial_pose.y': 0.0,
        'initial_pose.yaw': 0.0
    }

    # Non-composable
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params, {'yaml_filename': map_yaml_file}], # ✅ 修正为 configured_params
                 arguments=['--ros-args', '--log-level', log_level], remappings=remappings),
            
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 # ✅ 修正为 configured_params，并强行注入覆盖参数字典！
                 parameters=[configured_params, amcl_override_params], 
                 remappings=remappings),
            
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 arguments=['--ros-args', '--log-level', log_level],
                 parameters=[{'use_sim_time': use_sim_time},
                             {'autostart': autostart}, {'node_names': lifecycle_nodes}]),
        ])

    # Composable: container loads ENTIRE file, costmap sections visible at root
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(package='nav2_map_server', plugin='nav2_map_server::MapServer',
                name='map_server', 
                parameters=[configured_params, {'yaml_filename': map_yaml_file}], # ✅ 修正为 configured_params
                remappings=remappings),
            
            ComposableNode(package='nav2_amcl', plugin='nav2_amcl::AmclNode',
                name='amcl', 
                # ✅ 修正为 configured_params，并强行注入覆盖参数字典！
                parameters=[configured_params, amcl_override_params], 
                remappings=remappings),
            
            ComposableNode(package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart}, {'node_names': lifecycle_nodes}]),
        ])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('map'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('params_file'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_composition', default_value='False'))
    ld.add_action(DeclareLaunchArgument('container_name', default_value='nav2_container'))
    ld.add_action(DeclareLaunchArgument('use_respawn', default_value='False'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info'))
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    return ld