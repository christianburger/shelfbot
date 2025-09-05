import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    # --- 1. Launch the Real Robot Drivers ---
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(shelfbot_share_dir, 'config', 'four_wheel_drive_controller.yaml')],
        output='screen',
    )
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
            'use_sim_time': False
        }],
        output='screen'
    )

    # --- 4. Launch Robot Localization (EKF) ---
    # This node was commented out in the original, so it remains commented.
    # ekf_config = os.path.join(shelfbot_share_dir, 'config', 'ekf.yaml')
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config, {'use_sim_time': False}]
    # )

    # --- 5. Launch RTAB-Map Node ---
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'frame_id': 'base_footprint',
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'subscribe_rgb': True,
            'approx_sync': True,
            'use_sim_time': False,
            'Reg/Strategy': '1',
            'Vis/MinInliers': '15',
            'RGBD/Enabled': 'false',
            'Grid/FromDepth': 'false',
            'tf_delay': 0.8,  # Adjusted based on average delay of 0.3541s from log analysis
            'odom_queue_size': 2000,
            'Vis/MaxRate': 2,
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--', '--delete_db_on_start'],
        output='screen'
    )

    return LaunchDescription([
        ros2_control_node,
        real_robot_launch,
        republish_node,
        camera_info_node,
        rtabmap_node,
    ])
