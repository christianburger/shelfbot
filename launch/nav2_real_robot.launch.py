import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- 1. Launch the Real Robot Drivers (which includes the ESP32-CAM via Micro-ROS) ---
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(shelfbot_share_dir, 'launch', 'real_robot_microros.launch.py')
        )
    )

    # --- 2. Decompress the Image Stream ---
    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/image_raw/compressed'),
            ('out', '/camera/image_raw')
        ],
        output='screen',
        parameters=[{
            'qos_overrides./in/compressed.subscriber.reliability': 'best_effort',
            'qos_overrides./out.publisher.reliability': 'best_effort',
        }]
    )

    # --- 3. Publish Correct Camera Info ---
    camera_info_node = Node(
        package='shelfbot',
        executable='camera_publisher',
        name='camera_info_publisher',
        parameters=[{
            'camera_info_url': 'package://shelfbot/config/camera_info.yaml',
            'camera_name': 'camera',
            'frame_id': 'camera_link',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        real_robot_launch,
        republish_node,
        camera_info_node,
    ])
