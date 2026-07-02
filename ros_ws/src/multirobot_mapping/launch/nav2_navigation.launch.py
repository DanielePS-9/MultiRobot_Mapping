# This launch file is used to start our customnavigation stack for a TurtleBot3 robot 
# in a multi-robot mapping scenario. It includes the necessary nodes and configurations 
# for navigation, such as the controller server, smoother server, planner server, behavior server, 
# BT navigator, waypoint follower, velocity smoother, and lifecycle manager.

import os
import tempfile

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

        # Determine if use_sim_time is set to 'true' and convert it to a boolean
        use_sim_time_bool = context.launch_configurations['use_sim_time'] == 'true'
        sim_time_param = {'use_sim_time': use_sim_time_bool}

        # Read the parameters file
        with open(params_file, 'r') as f:
            template_text = f.read()
        
        # Replace the placeholder ${ROBOT_NAME} with the actual namespace of the robot.
        robot_custom_text = template_text.replace('${ROBOT_NAME}', namespace)
        
        # Create a temporary YAML file to hold the modified parameters for the specific robot.
        tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.yaml', mode='w')
        tmp_file.write(robot_custom_text)
        tmp_file.close() 
        
        # Create a ParameterFile object that rewrites the parameters from the temporary YAML file,
        # applying the necessary substitutions for use_sim_time and autostart.
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=tmp_file.name, # Use the temporary file with the modified parameters
                root_key=namespace,        # Use the robot's namespace as the root key for the parameters
                param_rewrites={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart
                },
                convert_types=True,
            ),
            allow_substs=True,
        )

        # Define the remappings for the navigation nodes to ensure that every robot has 
        # its own set of transformations trees and topics.
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

        # Define the individual navigation nodes with their respective configurations, parameters, and remappings.

        # Controller server node responsible for controlling the robot's movement based on the planned path.
        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,sim_time_param],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        )

        # Smoother server node responsible for smoothing the robot's velocity commands to ensure smooth motion.
        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,sim_time_param],
            remappings=remappings,
        )

        # Planner server node responsible for generating the robot's path based on the current map and goal.
        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,sim_time_param],
            remappings=remappings,
        )

        # Behavior server node responsible for managing the robot's behaviors.
        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,sim_time_param],
            remappings=remappings,
        )

        # BT navigator node responsible for managing the robot's behavior tree.
        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,sim_time_param],
            remappings=remappings,
        )

        # Waypoint follower node responsible for following a series of waypoints defined in the navigation plan.
        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,sim_time_param],
            remappings=remappings,
        )

        # Velocity smoother node responsible for smoothing the robot's velocity commands to ensure smooth motion.
        velocity_smoother = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,sim_time_param],
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

        # Lifecycle manager node responsible for managing the lifecycle of the navigation nodes, ensuring they are properly started and stopped.
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time == 'true',
                'autostart': autostart == 'true', # Automatically start the lifecycle manager and its managed nodes
                'node_names': managed_nodes, # List of nodes to be managed by the lifecycle manager
                'bond_timeout': 4.0, # Timeout for checking the activity of the managed nodes, ensuring they are responsive and alive
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