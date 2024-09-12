from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('shelfbot')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    default_rviz_config_path = os.path.expanduser('~/.rviz2/shelfbot.rviz')  # Expand ~ here

    rviz_config_file = LaunchConfiguration('rviz_config', default=default_rviz_config_path)

    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])