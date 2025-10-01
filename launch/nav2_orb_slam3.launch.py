import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    params_file = LaunchConfiguration( 'params_file', default=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'))

    # 1. Define ORB-SLAM3 paths
    orbslam3_root = '/home/chris/ORB_SLAM3'
    voc_file_path = os.path.join(orbslam3_root, 'Vocabulary', 'ORBvoc.txt')

    # Check if vocabulary file exists
    if not os.path.exists(voc_file_path):
        print(f"ERROR: ORB-SLAM3 vocabulary file not found at: {voc_file_path}")
        print("Please update the orbslam3_root variable in the launch file")

    # 2. Real robot drivers (includes micro-ROS bridge)
    real_robot_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource( os.path.join(shelfbot_share_dir, 'launch', 'real_robot_microros.launch.py')))

    # 3. Synchronized camera publisher (single source of image and camera_info)
    camera_publisher_node = Node(
        package='shelfbot',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_info_url': os.path.join(shelfbot_share_dir, 'config', 'package://shelfbot/config/camera_info.yaml'),
            'camera_name': 'esp32_cam',
            'frame_id': 'camera_link',
            'image_width': 800,
            'image_height': 600,
            'focal_length': 600.0,   # unified intrinsics: match the working AprilTag setup
            'sync_slop': 0.1
        }]
    )

    # 4. Map server with empty map (map->odom published by ORB-SLAM3, not here)
    map_yaml_path = os.path.join(shelfbot_share_dir, 'config', 'empty_map.yaml')
    map_pgm_path = os.path.join(shelfbot_share_dir, 'config', 'empty_map.pgm')
    print(f"Checking map YAML path: {map_yaml_path}, exists: {os.path.exists(map_yaml_path)}")
    print(f"Checking map PGM path: {map_pgm_path}, exists: {os.path.exists(map_pgm_path)}")
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_yaml_path,
            'topic_name': '/map',
            'frame_id': 'map',
            # CRITICAL: Don't let map_server publish transforms - let ORB-SLAM3 handle map->odom
            'publish_tf': False  # This prevents map_server from publishing map->odom
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 5. ORB-SLAM3 node (authoritative source for map->odom)
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
            'publish_tf': True,  # This will publish map->odom
            'publish_odom': False,  # Don't duplicate odometry
            'tracking_lost_timeout': 5.0
        }]
    )

    # 6. AprilTag detector node
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.16',
        description='Size of AprilTags in meters'
    )

    apriltag_detector_node = Node(
        package='shelfbot',
        executable='apriltag_detector_node',
        name='apriltag_detector',
        parameters=[{
            'tag_size': LaunchConfiguration('tag_size'),
            'pose_error_threshold': 50.0,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    # 7. Nav2 core nodes (planner, controller, etc.)
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[params_file]),
    ]

    # 8. Nav2 lifecycle manager (delayed start)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{ 
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',  # Provides the map topic
                'planner_server',
                'controller_server', 
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    delayed_lifecycle_manager = TimerAction(period=3.0, actions=[lifecycle_manager_node])  # Longer delay

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'),
            description='Full path to the ROS 2 parameters file to use for all launched nodes'
        ),
        tag_size_arg,
        real_robot_launch,
        camera_publisher_node,
        map_server_node,
        shelfbot_orb3_node,  # Start SLAM before lifecycle manager
        delayed_lifecycle_manager,
        apriltag_detector_node,
    ] + nav2_nodes)

