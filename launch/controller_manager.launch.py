from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('shelfbot')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    
    robot_description = Command(['xacro ', urdf_file_path])
    
    controller_config_path = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')

    return LaunchDescription([
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
            parameters=[{'robot_description': robot_description}, controller_config_path],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['four_wheel_drive_controller'],
            output='screen',
        ),
    ])
