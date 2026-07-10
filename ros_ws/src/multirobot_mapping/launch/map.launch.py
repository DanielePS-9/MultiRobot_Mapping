# This launch file is used to launch two map servers and RViz for visualizing the maps.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
 
def generate_launch_description():
   
    rviz_config_path = os.path.join(get_package_share_directory('multirobot_mapping'), 'rviz', 'map_view.rviz')
 
 
    # First map
    map_server_base = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_base',
        output='screen',
        parameters=[{'yaml_filename': 'stochastic_swarm_map.yaml'}],
    )
 
    lifecycle_base = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server_base'],
                output='screen'
            )
        ]
    )
 
 
    # Second Map
    map_server_2 = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_2',
        output='screen',
        parameters=[{'yaml_filename': 'deterministic_swarm_map.yaml'}],
        remappings=[('map', '/map_2')]
    )
 
    lifecycle_2 = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server_2'],
                output='screen'
            )
        ]
    )
 
 
    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )
 
    return LaunchDescription([
        map_server_base,
        lifecycle_base,
        map_server_2,
        lifecycle_2,
        rviz_node
    ])