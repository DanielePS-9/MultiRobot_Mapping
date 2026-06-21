import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='robot1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                
                # Sostituite le f-string con PathJoinSubstitution per risolvere l'oggetto LaunchConfiguration
                'frame_id': PathJoinSubstitution([namespace, 'base_footprint']),
                'odom_frame_id': PathJoinSubstitution([namespace, 'odom']),
                'map_frame_id': PathJoinSubstitution([namespace, 'map']),
                
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan': False,
                'subscribe_rgbd': False,
                
                # Abilitiamo la sottoscrizione diretta alla PointCloud2
                'subscribe_point_cloud': True,
                
                # Ottimizzazioni per la simulazione (Evita lag eccessivo)
                'Rtabmap/DetectionRate': '1.0', # Aggiorna la mappa 1 volta al secondo (riduce CPU)
                'RGBD/ProximityBySpace': 'true',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'Mem/IncrementalMemory': 'true', # true = SLAM (mappa e localizza), false = solo localizzazione
            }],
            remappings=[
                ('point_cloud', 'stereo_camera/pointCloud'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ],
            arguments=['-d'] # Cancella il database precedente ad ogni avvio (utile nei test)
        )
    ])