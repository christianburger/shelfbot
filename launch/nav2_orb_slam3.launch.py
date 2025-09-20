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

    # Define ORB-SLAM3 paths
    orbslam3_root = '/home/chris/ORB_SLAM3'
    voc_file_path = os.path.join(orbslam3_root, 'Vocabulary', 'ORBvoc.txt')

    # Check if vocabulary file exists
    if not os.path.exists(voc_file_path):
        print(f"ERROR: ORB-SLAM3 vocabulary file not found at: {voc_file_path}")
        print("Please update the orbslam3_root variable in the launch file")

    # 1. Real robot drivers
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

    # 2. Synchronized Camera Publisher
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
            'focal_length': 800.0,
            'sync_slop': 0.1
        }]
    )

    # 2.5. Map Server with Empty Map - MODIFIED TO NOT PUBLISH map->odom
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

    # 3. Static TF Publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_camera',
        arguments=['0.0', '0.0', '0.02', '-1.5708', '0.0', '0.0', 'base_link', 'camera_link'],  # Fixed transform
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 4. ORB-SLAM3 Node - This will be the AUTHORITATIVE source for map->odom
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

    # 5. Nav2 nodes - MODIFIED to work with visual SLAM
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[params_file]),
    ]

    # 6. Nav2 Lifecycle Manager - MODIFIED node list
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
        ros2_control_node,
        real_robot_launch,
        camera_publisher_node,
        static_tf_node,
        map_server_node,
        shelfbot_orb3_node,  # Start SLAM before lifecycle manager
        delayed_lifecycle_manager,
    ] + nav2_nodes)
