import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
 
from launch_ros.actions import Node
 
 
def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('multirobot_mapping'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
 
    x_pose_r1 = LaunchConfiguration('x_pose_r1', default='2.0')
    y_pose_r1 = LaunchConfiguration('y_pose_r1', default='0.0')
    ns_r1 = LaunchConfiguration('ns_r1', default='robot1')
 
    x_pose_r2 = LaunchConfiguration('x_pose_r2', default='1.0')
    y_pose_r2 = LaunchConfiguration('y_pose_r2', default='0.5')
    ns_r2 = LaunchConfiguration('ns_r2', default='robot2')
 
    x_pose_r3 = LaunchConfiguration('x_pose_r3', default='-2.0')
    y_pose_r3 = LaunchConfiguration('y_pose_r3', default='-0.5')
    ns_r3 = LaunchConfiguration('ns_r3', default='robot3')
 
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'stanza.world'
    )
 
    slam_r1 = os.path.join(
        get_package_share_directory('multirobot_mapping'),
        'params',
        'slam_toolbox_robot1.yaml'
    )
 
    slam_r2 = os.path.join(
        get_package_share_directory('multirobot_mapping'),
        'params',
        'slam_toolbox_robot2.yaml'
    )
   
    slam_r3 = os.path.join(
        get_package_share_directory('multirobot_mapping'),
        'params',
        'slam_toolbox_robot3.yaml'
    )
 
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )
 
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))
   
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
 
                                # ROBOT1
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
   
    robot1_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot1_odom',
        arguments=[x_pose_r1, y_pose_r1, '0.01', '0.0', '0.0', '0.0', 'world', 'robot1/map']
    )
 
   
 
                                # ROBOT2    
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
   
    robot2_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot2_odom',
        arguments=[x_pose_r2, y_pose_r2, '0.01', '0.0', '0.0', '0.0', 'world', 'robot2/map']
    )
 
   
                                # ROBOT3    
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
   
    robot3_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot3_odom',
        arguments=[x_pose_r3, y_pose_r3, '0.01', '0.0', '0.0', '0.0', 'world', 'robot3/map']
    )
 
 
    ld = LaunchDescription()
 
    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_cmd)
    ld.add_action(clock_bridge)
    ld.add_action(robot1_spawn_cmd)
    ld.add_action(robot1_tf_publisher)
    ld.add_action(robot2_spawn_cmd)
    ld.add_action(robot2_tf_publisher)
    ld.add_action(robot3_spawn_cmd)
    ld.add_action(robot3_tf_publisher)
    return ld