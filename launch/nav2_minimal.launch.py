import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package share directory
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    # Nav2 parameters file (passed through, not altered here)
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(shelfbot_share_dir, 'config', 'nav2_odom_params.yaml')
    )

    # Robot bringup: do not modify this other launcher
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(shelfbot_share_dir, 'launch', 'real_robot_microros.launch.py')
        )
    )

    # Nav2 lifecycle nodes (processes start; will be transitioned later)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ]
    )

    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        output='screen',
        parameters=[params_file]
    )

    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='global_costmap',
        name='global_costmap',
        output='screen',
        parameters=[params_file]
    )

    # Lifecycle manager (will be started only when readiness checker succeeds)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'local_costmap/local_costmap',
                'global_costmap/global_costmap'
            ]
        }]
    )

    # Readiness checker process:
    # - Checks TF odom -> base_footprint (as per your odometry)
    # - Checks minimal topics you have at startup (from your list)
    # - Does NOT check lifecycle services
    checker_code = r"""
import sys
import time
import rclpy
from rclpy.node import Node as RclpyNode
from tf2_ros import Buffer, TransformListener

def main():
    rclpy.init()
    node = RclpyNode('nav2_readiness_checker')

    buffer = Buffer()
    listener = TransformListener(buffer, node)

    # TF backbone published by your odometry:
    required_tf = [('odom', 'base_footprint')]

    # Minimal topics present at startup (align with your provided list)
    required_topics = [
        '/tf',
        '/tf_static',
        '/robot_description',
        '/joint_states',
        '/esp32_cam/camera_info',
        '/esp32_cam/image_raw/compressed',
        '/shelfbot_firmware/distance_sensors',
        '/shelfbot_firmware/heartbeat',
        '/shelfbot_firmware/led',
        '/shelfbot_firmware/led_state',
        '/shelfbot_firmware/motor_command',
        '/shelfbot_firmware/motor_positions',
        '/shelfbot_firmware/set_speed',
        '/rosout',
        '/parameter_events',
        '/robot_description',
    ]

    timeout_total_sec = 120.0
    check_interval_sec = 0.25
    start = time.time()

    def tf_ready():
        for parent, child in required_tf:
            try:
                buffer.lookup_transform(parent, child, rclpy.time.Time())
            except Exception:
                return False
        return True

    def topics_ready():
        names_and_types = node.get_topic_names_and_types()
        names = {n for n, _ in names_and_types}
        for topic in required_topics:
            if topic not in names:
                return False
        return True

    while rclpy.ok():
        tf_ok = tf_ready()
        top_ok = topics_ready()

        if tf_ok and top_ok:
            node.get_logger().info('Readiness satisfied: TF odom->base_footprint and minimal topics present')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

        if time.time() - start > timeout_total_sec:
            if not tf_ok:
                node.get_logger().error('Readiness gate: missing TF(s): ' + ', '.join([f'{p}->{c}' for p, c in required_tf]))
            if not top_ok:
                node.get_logger().error('Readiness gate: required minimal topics missing.')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        rclpy.spin_once(node, timeout_sec=check_interval_sec)

if __name__ == '__main__':
    main()
"""
    readiness_checker = ExecuteProcess(
        cmd=['python3', '-u', '-c', checker_code],
        name='nav2_readiness_checker',
        output='screen'
    )

    # Start lifecycle manager only when readiness checker exits successfully
    start_lifecycle_manager = RegisterEventHandler(
        OnProcessExit(
            target_action=readiness_checker,
            on_exit=[lifecycle_manager_node]
        )
    )

    # Perception nodes (unchanged, as per your file)
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

    # ORB-SLAM3 node (unchanged, using your paths and parameters)
    orbslam3_root = '/home/chris/ORB_SLAM3'
    voc_file_path = os.path.join(orbslam3_root, 'Vocabulary', 'ORBvoc.txt')

    orbslam3_node = Node(
        package='shelfbot',
        executable='shelfbot_slam_orb3_node',
        name='shelfbot_slam_orb3_node',
        output='screen',
        env={
            'LD_LIBRARY_PATH': f'{orbslam3_root}/lib:{os.environ.get("LD_LIBRARY_PATH", "")}',
            'ROS_LOG_DIR': '/home/chris/ros2_logs'
        },
        parameters=[{
            'voc_file': voc_file_path,
            'settings_file': os.path.join(shelfbot_share_dir, 'config', 'orb_slam3_monocular.yaml'),
            'camera_topic': '/camera/image_raw',        # keep as in your original; adjust if needed
            'camera_info_topic': '/camera/camera_info', # keep as in your original; adjust if needed
            'camera_frame': 'camera_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'publish_tf': False,
            'publish_odom': False,
            'tracking_lost_timeout': 5.0
        }]
    )

    apriltag_node = Node(
        package='shelfbot',
        executable='apriltag_detector_node',
        name='apriltag_detector',
        parameters=[{'tag_size': 0.16, 'pose_error_threshold': 50.0}],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    return LaunchDescription([
        # Parameter file argument (explicit)
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(shelfbot_share_dir, 'config', 'nav2_odom_params.yaml'),
            description='Full path to Nav2 parameters file'
        ),

        # Robot bringup first (as-is, not modified)
        real_robot_launch,

        # Nav2 lifecycle nodes (wait unconfigured)
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        local_costmap_node,
        global_costmap_node,

        # Readiness gate runs concurrently; lifecycle manager starts on its success
        readiness_checker,
        start_lifecycle_manager,

        # Perception nodes
        camera_publisher_node,
        orbslam3_node,
        apriltag_node,
    ])
