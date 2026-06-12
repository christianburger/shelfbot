import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────────
    pkg_share          = get_package_share_directory('shelfbot')
    nav2_bringup_dir   = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir   = get_package_share_directory('slam_toolbox')

    xacro_file         = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    controller_config  = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')
    nav2_params        = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params        = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config        = os.path.join(pkg_share, 'config', 'nav2_troubleshoot.rviz')
    camera_info_url    = 'file://' + os.path.join(pkg_share, 'config', 'esp32_cam_calibration.yaml')

    # ── Robot description ──────────────────────────────────────────────────────
    doc = xacro.process_file(xacro_file, mappings={'communication_type': 'microros'})
    robot_description = {'robot_description': doc.toxml()}

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 1  (t=0 s) – hardware + camera pipeline
    # ══════════════════════════════════════════════════════════════════════════

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # FIX: controller_server (Nav2) publishes velocity commands to /cmd_vel_nav.
    # velocity_smoother subscribes to /cmd_vel_nav and publishes to /cmd_vel.
    # This is the correct chain for Humble without collision_monitor:
    #   Nav2 controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → hardware
    #
    # The four_wheel_drive_controller subscribes to /cmd_vel directly
    # via the remapping below, which is unchanged.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config, {'use_sim_time': False}],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('/four_wheel_drive_controller/cmd_vel', '/cmd_vel'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # NOTE: static_tf_lidar has been REMOVED.
    #
    # The URDF (shelfbot.urdf.xacro) already defines the lidar link as
    # 'laser_link' and attaches it to base_link via joint_laser at the correct
    # physical position. robot_state_publisher broadcasts this transform on
    # /tf_static automatically.
    #
    # The previous static_tf_lidar node published base_link → lidar_frame at
    # z=0.2m, which is a DIFFERENT frame name from the URDF's laser_link.
    # This created two conflicting lidar frames in the TF tree:
    #   - base_link → laser_link   (from URDF / robot_state_publisher)
    #   - base_link → lidar_frame  (from static_tf_lidar — orphaned, wrong z)
    #
    # Nav2 costmaps and slam_toolbox both look up the frame_id stamped on
    # /scan messages. lidar_relay_node now stamps scans with 'laser_link'
    # (matching the URDF), so the lookup succeeds and costmaps populate.

    lidar_relay = Node(
        package='shelfbot',
        executable='lidar_relay_node',
        name='lidar_relay_node',
        output='screen',
        parameters=[{
            'frame_id': 'laser_link',
            'publish_hz': 10.0,                    # increased from 5.0
            'qos_reliability': 'best_effort',     # <-- critical fix
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        respawn=True,
        respawn_delay=2.0,
    )

    # Camera is used for AprilTag detection only — not SLAM.
    camera_publisher = Node(
        package='shelfbot',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_info_url': camera_info_url,
            'camera_name':     'esp32_cam',
            'frame_id':        'camera_link_optical_frame',
            'image_width':     800,
            'image_height':    600,
            'focal_length':    800.0,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        respawn=True,
        respawn_delay=3.0,
    )

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 2  (t=3 s) – ros2_control spawners
    # Wheel odometry (odom → base_footprint TF) starts publishing here.
    # slam_toolbox must not start before this TF is live.
    # ══════════════════════════════════════════════════════════════════════════

    joint_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['four_wheel_drive_controller', '--controller-manager', '/controller_manager'],
    )

    delay_controllers = TimerAction(
        period=3.0,
        actions=[joint_broadcaster_spawner, drive_controller_spawner],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 3  (t=6 s) – slam_toolbox
    #
    # Requires before starting:
    #   /scan                   — lidar_relay_node (tier 1, immediate)
    #   odom → base_footprint TF — wheel odometry (tier 2, t=3s)
    #   base_link → laser_link TF — robot_state_publisher (tier 1, immediate)
    #
    # Publishes:
    #   /map             — 2D occupancy grid
    #   map → odom TF    — required by Nav2 global costmap (tier 5)
    #
    # The 3s gap between tier 2 and tier 3 gives the controllers time to
    # activate and begin publishing odom. Without this gap, slam_toolbox drops
    # the first several scans because the TF queue is full (seen in launch log).
    # ══════════════════════════════════════════════════════════════════════════

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time':     'false',
        }.items(),
    )

    delay_slam = TimerAction(period=6.0, actions=[slam_toolbox])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 4  (t=8 s) – AprilTag detector
    # Camera is for tag detection only. Independent of SLAM.
    # ══════════════════════════════════════════════════════════════════════════

    apriltag_detector = Node(
        package='shelfbot',
        executable='apriltag_detector_node',
        name='apriltag_detector_node',
        output='screen',
        parameters=[{
            'tag_size':             0.16,
            'pose_error_threshold': 100.0,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        respawn=True,
        respawn_delay=3.0,
    )

    delay_perception = TimerAction(period=8.0, actions=[apriltag_detector])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 5  (t=12 s) – Nav2
    #
    # Requires before starting:
    #   /map             — from slam_toolbox (tier 3, t=6s)
    #   map → odom TF    — from slam_toolbox (tier 3, t=6s)
    #   /scan            — for local costmap (tier 1, immediate)
    #
    # FIX: collision_monitor is NOT included in navigation_launch.py in Humble.
    # Its absence was causing lifecycle_manager to stall indefinitely.
    # Removed from lifecycle_manager node_names in nav2_params.yaml.
    #
    # cmd_vel chain:
    #   controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → hardware
    # ══════════════════════════════════════════════════════════════════════════

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file':  nav2_params,
            'use_sim_time': 'false',
            'autostart':    'true',
        }.items(),
    )

    delay_nav2 = TimerAction(period=12.0, actions=[nav2])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 6  (t=20 s) – RViz
    # ══════════════════════════════════════════════════════════════════════════

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}, robot_description],
        output='screen',
    )

    delay_rviz = TimerAction(period=20.0, actions=[rviz_node])

    return LaunchDescription([
        # Tier 1 – hardware + camera (immediate)
        robot_state_pub,
        control_node,
        lidar_relay,
        camera_publisher,
        # Tier 2 – controllers (t=3 s)
        delay_controllers,
        # Tier 3 – SLAM (t=6 s)
        delay_slam,
        # Tier 4 – AprilTags (t=8 s)
        delay_perception,
        # Tier 5 – Nav2 (t=12 s)
        delay_nav2,
        # Tier 6 – RViz (t=20 s)
        delay_rviz,
    ])
