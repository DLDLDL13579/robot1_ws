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
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server','smoother_server','planner_server',
                       'behavior_server','bt_navigator','waypoint_follower','velocity_smoother']
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = ParameterFile(
        RewrittenYaml(source_file=params_file, root_key=namespace,
                      param_rewrites=param_substitutions, convert_types=True), allow_substs=True)

    critics = {
        'FollowPath.critics': ['RotateToGoal','Oscillation','BaseObstacle','GoalAlign','PathAlign','PathDist','GoalDist'],
        'FollowPath.RotateToGoal.scale': 32.0, 'FollowPath.BaseObstacle.scale': 0.02,
        'FollowPath.GoalAlign.scale': 24.0, 'FollowPath.PathDist.scale': 32.0, 'FollowPath.GoalDist.scale': 24.0,
    }

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Non-composable (use_composition=false)
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(package='nav2_controller', executable='controller_server', name='controller_server',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params, critics], remappings=remappings),
            Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
                 output='screen', parameters=[configured_params], remappings=remappings),
            Node(package='nav2_planner', executable='planner_server', name='planner_server',
                 output='screen', parameters=[configured_params], remappings=remappings),
            Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
                 output='screen', parameters=[configured_params], remappings=remappings),
            Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
                 output='screen', parameters=[configured_params], remappings=remappings),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
                 output='screen', parameters=[configured_params], remappings=remappings),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
                 output='screen', parameters=[configured_params], remappings=remappings),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart},
                             {'node_names': lifecycle_nodes}]),
        ])

    # Composable: ALL root-level params visible (costmaps, etc.)
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(package='nav2_controller', plugin='nav2_controller::ControllerServer',
                name='controller_server', parameters=[configured_params, critics], remappings=remappings),
            ComposableNode(package='nav2_smoother', plugin='nav2_smoother::SmootherServer',
                name='smoother_server', parameters=[configured_params], remappings=remappings),
            ComposableNode(package='nav2_planner', plugin='nav2_planner::PlannerServer',
                name='planner_server', parameters=[configured_params], remappings=remappings),
            ComposableNode(package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
                name='behavior_server', parameters=[configured_params], remappings=remappings),
            ComposableNode(package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator', parameters=[configured_params], remappings=remappings),
            ComposableNode(package='nav2_waypoint_follower', plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower', parameters=[configured_params], remappings=remappings),
            ComposableNode(package='nav2_velocity_smoother', plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother', parameters=[configured_params], remappings=remappings),
            ComposableNode(package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for n,d in [('namespace',''),('use_sim_time','false'),('params_file',''),
                ('autostart','true'),('use_composition', 'False'),
                ('container_name','nav2_container'),('use_respawn','False'),('log_level','info')]:
        ld.add_action(DeclareLaunchArgument(n, default_value=d))
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    return ld
