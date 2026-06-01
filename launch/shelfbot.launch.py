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
            'frame_id': 'camera_link_optical_frame',
            'use_sim_time': False
        }],
        output='screen'
    )

    # --- 4. Launch RTAB-Map for Visual Odometry and Mapping ---
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'frame_id': 'base_footprint',
            'subscribe_rgb': True,
            'subscribe_odom': True,
            'approx_sync': True,
            'use_sim_time': False,
            'Reg/Strategy': '1', # 1=Vis, 2=ICP, 3=both
            'Vis/MinInliers': '15',
            'Vis/MaxRate': 2,
            'odom_queue_size': 2000,
            'tf_delay': 0.8,
            'qos_image': 1, # Use default QoS for images
            'qos_camera_info': 1,
            'qos_odom': 1,
            'Vis/Force3DoF': 'true',
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--', '--delete_db_on_start'],
        output='screen'
    )

    # --- 5. AprilTag Detection Node ---
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('detections', 'tag_detections')
        ],
        parameters=[{
            'family': 'tag36h11', # Example AprilTag family
        }],
    )

    # --- 6. Custom Behavior Tree Node ---
    # This node will subscribe to the tag detections and publish commands
    # You will need to create this Python or C++ node yourself
    bt_node = Node(
        package='shelfbot', # Example package name
        executable='robot_bt_controller', # The name of your BT executable
        name='bt_controller',
        output='screen',
    )

    return LaunchDescription([
        ros2_control_node,
        real_robot_launch,
        republish_node,
        camera_info_node,
        rtabmap_node,
        apriltag_node,
        bt_node,
    ])