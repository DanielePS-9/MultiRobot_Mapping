import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_config_path = os.path.join(get_package_share_directory('multirobot_mapping'), 'rviz', 'map_view.rviz')
    # ==========================================
    # 1. MAPPA BASE (es. Greedy Stocastico)
    # ==========================================
    map_server_base = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_base',
        output='screen',
        parameters=[{'yaml_filename': 'mappa_sciame_completata.yaml'}],
        # Pubblica di default su /map
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

    # ==========================================
    # 2. MAPPA CONFRONTO (es. Market-Based)
    # ==========================================
    map_server_mk = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_mk',
        output='screen',
        parameters=[{'yaml_filename': 'mappa_sciame_completata_mk.yaml'}],
        # Rinominiamo il topic di uscita per non sovrascrivere la mappa base
        remappings=[('map', '/map_mk')]
    )

    lifecycle_mk = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server_mk'],
                output='screen'
            )
        ]
    )

    # ==========================================
    # 3. RVIZ 2
    # ==========================================
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
        map_server_mk,
        lifecycle_mk,
        rviz_node
    ])