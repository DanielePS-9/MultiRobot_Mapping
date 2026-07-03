# This launch file is used to spawn three TurtleBot3 robots in a Gazebo simulation,
# relay their namespaced TF topics to the global ones, and launch RViz for visualization. 
# It also includes a map merger node and a swarm exploration node.

import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
 
def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('multirobot_mapping'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rviz_config_path = os.path.join(get_package_share_directory('multirobot_mapping'), 'rviz', 'three_robots_view.rviz')
 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
 
    x_pose_r1 = LaunchConfiguration('x_pose_r1', default='-4.0')
    y_pose_r1 = LaunchConfiguration('y_pose_r1', default='0.0')
    ns_r1 = LaunchConfiguration('ns_r1', default='robot1')

    x_pose_r2 = LaunchConfiguration('x_pose_r2', default='-4.0')
    y_pose_r2 = LaunchConfiguration('y_pose_r2', default='-1.0')
    ns_r2 = LaunchConfiguration('ns_r2', default='robot2')

    x_pose_r3 = LaunchConfiguration('x_pose_r3', default='-4.0')
    y_pose_r3 = LaunchConfiguration('y_pose_r3', default='-2.0')
    ns_r3 = LaunchConfiguration('ns_r3', default='robot3')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'stanza.world'
    )
 
    # Launch Gazebo with the specified world
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )
 
    # Set the GZ_SIM_RESOURCE_PATH environment variable to include the TurtleBot3 models
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))
   
    # Bridge the /clock topic from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
 


                                # ROBOT1
    
    # Start the spawn_tb3.launch.py for robot1
    robot1_spawn_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_tb3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose_r1,
                'y_pose': y_pose_r1,
                'namespace': ns_r1
            }.items()
        )
    
    # Relay the /robot1/tf topic to the global /tf topic
    robot1_tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='robot1_tf_relay',
        arguments=['/robot1/tf', '/tf'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Relay the /robot1/tf_static topic to the global /tf_static topic
    robot1_tf_static_relay = Node(
        package='topic_tools',
        executable='relay',
        name='robot1_tf_static_relay',
        arguments=['/robot1/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
   
    # Publish a static transform from the world frame to robot1's map frame
    robot1_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot1_odom',
        arguments=[x_pose_r1, y_pose_r1, '0.01', '0.0', '0.0', '0.0', 'world', 'robot1/map'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf_static', '/robot1/tf_static')]
    )



                                # ROBOT2

    # Start the spawn_tb3.launch.py for robot2
    robot2_spawn_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_tb3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose_r2,
                'y_pose': y_pose_r2,
                'namespace': ns_r2 
            }.items()
        )
    
    # Relay the /robot2/tf topic to the global /tf topic
    robot2_tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='robot2_tf_relay',
        arguments=['/robot2/tf', '/tf'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Relay the /robot2/tf_static topic to the global /tf_static topic
    robot2_tf_static_relay = Node(
        package='topic_tools',
        executable='relay',
        name='robot2_tf_static_relay',
        arguments=['/robot2/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Publish a static transform from the world frame to robot2's map frame
    robot2_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot2_odom',
        arguments=[x_pose_r2, y_pose_r2, '0.01', '0.0', '0.0', '0.0', 'world', 'robot2/map'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf_static', '/robot2/tf_static')]
    )



                                # ROBOT3

    # Start the spawn_tb3.launch.py for robot3
    robot3_spawn_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_tb3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose_r3,
                'y_pose': y_pose_r3,
                'namespace': ns_r3
            }.items()
        )
    
    # Relay the /robot3/tf topic to the global /tf topic
    robot3_tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='robot3_tf_relay',
        arguments=['/robot3/tf', '/tf'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Relay the /robot3/tf_static topic to the global /tf_static topic
    robot3_tf_static_relay = Node(
        package='topic_tools',
        executable='relay',
        name='robot3_tf_static_relay',
        arguments=['/robot3/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
   
    # Publish a static transform from the world frame to robot3's map frame
    robot3_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot3_odom',
        arguments=[x_pose_r3, y_pose_r3, '0.01', '0.0', '0.0', '0.0', 'world', 'robot3/map'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf_static', '/robot3/tf_static')]
    )


    # Launch RViz with the specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Launch the map merger node with the specified parameters
    map_merge_node = Node(
        package='multirobot_mapping',
        executable='map_merger',  
        name='custom_map_merger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'r1_x': x_pose_r1,
            'r1_y': y_pose_r1,
            'r2_x': x_pose_r2,
            'r2_y': y_pose_r2,
            'r3_x': x_pose_r3,
            'r3_y': y_pose_r3,
        }]
    )

    # Launch the swarm exploration node with the specified parameters
    swarm_explorer_node = Node(
        package='multirobot_mapping', 
        executable='swarm_ex', 
        name='swarm_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
 
    ld = LaunchDescription()
 
    # Declare the launch options
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_cmd)
    ld.add_action(clock_bridge)
   
    # Delay the spawning of all robots to ensure Gazebo is fully initialized
    robots_spawn_delay = TimerAction(
        period=5.0,
        actions=[
            robot1_spawn_cmd, robot1_tf_relay, robot1_tf_static_relay, robot1_tf_publisher,
            robot2_spawn_cmd, robot2_tf_relay, robot2_tf_static_relay, robot2_tf_publisher,
            robot3_spawn_cmd, robot3_tf_relay, robot3_tf_static_relay, robot3_tf_publisher,
        ]
    )
    ld.add_action(robots_spawn_delay)
   
    # Delay the launch of the map merger and RViz nodes to ensure all robots are spawned
    tools_delay = TimerAction(
        period=10.0,
        actions=[
            map_merge_node,
            rviz_node
        ]
    )
    ld.add_action(tools_delay)

    # Delay the launch of the swarm exploration node to ensure all robots are spawned and their TF topics are being relayed
    swm_delay = TimerAction(
        period=35.0,
        actions=[
            swarm_explorer_node
        ]
    )
   
    ld.add_action(swm_delay)

    return ld