import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'MAP.world'
    world_path = os.path.join(get_package_share_directory('ros_tb3'), 'worlds', world_file_name)
    turtlebot3_model = os.getenv('TURTLEBOT3_MODEL', 'waffle_pi')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Full path to world file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch'), '/turtlebot3_empty_world.launch.py']),
            launch_arguments={'world': world_path, 'use_sim_time': use_sim_time}.items(),
        ),
        
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen'),
    ])
