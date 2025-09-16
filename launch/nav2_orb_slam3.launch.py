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
    orbslam3_root = '/home/chris/shelfbot_workspace/src/ORB_SLAM3'
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

    # 2. Decompress image stream (compressed -> raw)
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

    # 3. Camera info publisher
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

    # 4. Static TF base_footprint -> camera_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_cam',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'camera_link'],
        output='screen'
    )

    # 5. Shelfbot integrated ORB-SLAM3 node
    shelfbot_orb3_node = Node(
        package='shelfbot',
        executable='shelfbot_slam_orb3_node',
        name='shelfbot_slam_orb3',
        output='screen',
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('camera/camera_info', '/camera/camera_info')
        ],
        parameters=[{
            'use_sim_time': False,
            'voc_file': voc_file_path,
            'settings_file': os.path.join(shelfbot_share_dir, 'config', 'orb_slam3_monocular.yaml'),
            'camera_frame': 'camera_link',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'publish_pose_topic': True,
            'pose_topic': '/slam_orb3/pose'
        }]
    )

    # 6. Nav2 nodes + lifecycle
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[params_file]),
    ]

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': [
                        'planner_server',
                        'controller_server',
                        'smoother_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower'
                    ]}]
    )
    delayed_lifecycle_manager = TimerAction(period=2.0, actions=[lifecycle_manager_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'),
            description='Full path to the ROS 2 parameters file to use for all launched nodes'
        ),
        ros2_control_node,
        real_robot_launch,
        republish_node,
        camera_info_node,
        static_tf_node,
        shelfbot_orb3_node,
        delayed_lifecycle_manager,
    ] + nav2_nodes)

