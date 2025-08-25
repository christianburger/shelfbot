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

    # Apriltag Detector Node
    apriltag_detector_node = Node(
        package='shelfbot',
        executable='apriltag_detector_node',
        name='apriltag_detector',
        output='screen',
        remappings=[
            # Remap the image topic to your camera's topic.
            # You may need to change '/camera/image_raw' if your camera publishes elsewhere.
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        parameters=[{
            # The tag family your tags belong to
            'family': 'tag36h11',
            # The size of the tag's black square in meters
            'size': 0.16,
        }]
    )

    return LaunchDescription([
        real_robot_launch,
        apriltag_detector_node
    ])
