import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('multirobot_mapping'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    x_pose_r1 = LaunchConfiguration('x_pose_r1', default='2.0')
    y_pose_r1 = LaunchConfiguration('y_pose_r1', default='0.0')
    ns_r1 = LaunchConfiguration('ns_r1', default='robot1')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'stanza.world'
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
    
    rtabmap_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('multirobot_mapping'), 'launch', 'rtabmap_rbt.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace': ns_r1
            }.items()
        )
    
    robot1_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot1_map',
        arguments=[x_pose_r1, y_pose_r1, '0.01', '0.0', '0.0', '0.0', 'world', 'robot1/map']
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_cmd)
    ld.add_action(robot1_spawn_cmd)
    ld.add_action(rtabmap_slam)
    ld.add_action(robot1_tf_publisher)
    return ld