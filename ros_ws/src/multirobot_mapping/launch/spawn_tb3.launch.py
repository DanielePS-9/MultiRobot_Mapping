import os
import sys
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
 
# 1. Trova il percorso assoluto della cartella in cui si trova QUESTO file di launch
current_dir = os.path.dirname(os.path.abspath(__file__))
 
# 2. Aggiungi questa cartella alla lista dei percorsi di Python
sys.path.append(current_dir)
from utils import load_sdf_with_namespace, create_namespaced_bridge_yaml
 
def generate_launch_description():
    # Get the sdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
 
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
   
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
 
    sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
 
    bridge_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder+'_bridge.yaml'
    )
 
    # Launch configuration variables specific to simulation
    x_pose    = LaunchConfiguration('x_pose', default='0.0')
    y_pose    = LaunchConfiguration('y_pose', default='0.0')
    namespace = LaunchConfiguration('namespace', default='')
 
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial X position')
 
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial Y position')
 
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Specify namespace of the robot')
 
 
 
    def spawn_rbt(context):
        namespace = context.launch_configurations['namespace']
        x_pose    = context.launch_configurations['x_pose']
        y_pose    = context.launch_configurations['y_pose']
 
        use_sim_time_val = context.launch_configurations.get('use_sim_time', 'true')
 
        ns_sdf = load_sdf_with_namespace(sdf_path, namespace)
        ns_yaml = create_namespaced_bridge_yaml(bridge_path,namespace)
 
        slam_params_file = os.path.join(
            get_package_share_directory('multirobot_mapping'),
            'params',
            'slam_toolbox_' + f'{namespace}' + '.yaml'
        )
 
        gazebo_ros_spawner = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', namespace,
                '-string', ns_sdf,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01'
            ],
            output='screen',
        )
 
        robot_state_publisher_slam = GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('multirobot_mapping'), 'launch', 'publisher_tb3.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'frame_prefix': namespace
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
                    ),
                launch_arguments={
                    'use_sim_time': use_sim_time_val,
                    'slam_params_file': slam_params_file,
                }.items(),
            ),
        ])
       
        gazebo_ros_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={ns_yaml}',
            ],
            output='screen',
        )
 
        return [gazebo_ros_spawner, gazebo_ros_bridge, robot_state_publisher_slam]
 
    ld = LaunchDescription()
 
    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_namespace_cmd)
 
    # Add any conditioned actions
    ld.add_action(OpaqueFunction(function = spawn_rbt))
 
    return ld