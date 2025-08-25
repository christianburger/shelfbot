import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    # Include the real robot launch file to start drivers
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(shelfbot_share_dir, 'launch', 'real_robot_microros.launch.py')
        )
    )

    mission_control_node = Node(
        package='shelfbot',
        executable='mission_control_node',
        name='mission_control_node',
        output='screen',
    )

    return LaunchDescription([
        real_robot_launch,
        mission_control_node,
    ])
