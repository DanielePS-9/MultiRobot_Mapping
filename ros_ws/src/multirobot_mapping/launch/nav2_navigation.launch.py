import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    namespace_arg    = DeclareLaunchArgument('namespace',    default_value='robot1')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    params_file_arg  = DeclareLaunchArgument('params_file')
    autostart_arg    = DeclareLaunchArgument('autostart',    default_value='true')

    def launch_nav2(context):
        namespace    = context.launch_configurations['namespace']
        use_sim_time = context.launch_configurations['use_sim_time']
        params_file  = context.launch_configurations['params_file']
        autostart    = context.launch_configurations['autostart']

        # RewrittenYaml: trasforma "controller_server:" in "/robot1/controller_server:"
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file,
                root_key=namespace,
                param_rewrites={'use_sim_time': use_sim_time,
                                'autostart': autostart},
                convert_types=True,
            ),
            allow_substs=True,
        )

        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        )

        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        )

        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        )

        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        )

        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings + [
                ('goal_pose', '/goal_pose'),
            ],
        )

        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        )

        velocity_smoother = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        )

        managed_nodes = [
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'velocity_smoother',
            'bt_navigator',
            'waypoint_follower',
        ]

        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time == 'true',
                'autostart': autostart == 'true',
                'node_names': managed_nodes,
                'bond_timeout': 4.0,
            }],
        )

        return [
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager,
        ]

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        OpaqueFunction(function=launch_nav2),
    ])
