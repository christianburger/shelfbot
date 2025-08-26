import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    # --- 1. Launch the Gazebo Simulation ---
    # This includes the robot drivers and controllers
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(shelfbot_share_dir, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={'gui': 'true'}.items() # Start GUI by default
    )

    # --- 2. Launch the Perception System ---
    apriltag_detector_node = Node(
        package='shelfbot',
        executable='apriltag_detector_node',
        name='apriltag_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            # Remap to the Gazebo camera topic
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
    )

    # --- 3. Launch the Mission Control System ---
    mission_control_node = Node(
        package='shelfbot',
        executable='mission_control_node',
        name='mission_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_sim_launch,
        apriltag_detector_node,
        mission_control_node
    ])
