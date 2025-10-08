import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml')
    )

    # 1. Real robot drivers (micro-ROS bridge, wheel odometry)
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(shelfbot_share_dir, 'launch', 'real_robot_microros.launch.py')
        )
    )

    # 2. Camera publisher
    camera_publisher_node = Node(
        package='shelfbot',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_info_url': os.path.join(shelfbot_share_dir, 'config', 'camera_info.yaml'),
            'camera_name': 'esp32_cam',
            'frame_id': 'camera_link',
            'image_width': 800,
            'image_height': 600,
            'focal_length': 600.0,
            'sync_slop': 0.1
        }]
    )

    # 3. ORB-SLAM3 node (optional, monitoring only, no TF)
    orbslam3_root = '/home/chris/ORB_SLAM3'
    voc_file_path = os.path.join(orbslam3_root, 'Vocabulary', 'ORBvoc.txt')

    shelfbot_orb3_node = Node(
        package='shelfbot',
        executable='shelfbot_slam_orb3_node',
        name='shelfbot_slam_orb3_node',
        output='screen',
        env={
            'LD_LIBRARY_PATH': f'/home/chris/ORB_SLAM3/lib:{os.environ.get("LD_LIBRARY_PATH", "")}',
            'ROS_LOG_DIR': '/home/chris/ros2_logs'
        },
        parameters=[{
            'voc_file': voc_file_path,
            'settings_file': os.path.join(shelfbot_share_dir, 'config', 'orb_slam3_monocular.yaml'),
            'camera_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'camera_frame': 'camera_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'publish_tf': False,   # 🚫 do not publish map->odom
            'publish_odom': False, # 🚫 do not publish /slam_odom
            'tracking_lost_timeout': 5.0
        }]
    )

    # 4. AprilTag detector (optional)
    apriltag_detector_node = Node(
        package='shelfbot',
        executable='apriltag_detector_node',
        name='apriltag_detector',
        parameters=[{
            'tag_size': 0.16,
            'pose_error_threshold': 50.0,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    # 5. Nav2 core nodes
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[params_file]),
    ]

    # 6. Lifecycle manager (no map_server here)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    delayed_lifecycle_manager = TimerAction(period=3.0, actions=[lifecycle_manager_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'),
            description='Full path to the ROS 2 parameters file to use for all launched nodes'
        ),
        real_robot_launch,
        camera_publisher_node,
        shelfbot_orb3_node,
        apriltag_detector_node,
        delayed_lifecycle_manager,
    ] + nav2_nodes)

