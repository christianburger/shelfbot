from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('shelfbot')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    
    world_file_path = os.path.expanduser('~/.world/shelfbot_world.world')
    
    default_rviz_config_path = os.path.expanduser('~/.rviz2/shelfbot.rviz')

    rviz_config_file = LaunchConfiguration('rviz_config', default=default_rviz_config_path)

    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    controller_config = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'shelfbot'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description},
                        controller_config],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['four_wheel_drive_controller'],
            output='screen',
        ),
    ])
