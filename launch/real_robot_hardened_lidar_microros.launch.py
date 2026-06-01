import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────────
    pkg_share        = get_package_share_directory('shelfbot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    xacro_file        = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')
    nav2_params       = os.path.join(pkg_share, 'config', 'nav2_camera_params.yaml')
    rviz_config       = os.path.join(pkg_share, 'config', 'nav2_troubleshoot.rviz')

    # ── Lidar mount (tweak to match your physical setup) ───────────────────────
    lidar_x, lidar_y, lidar_z         = '0.0', '0.0', '0.2'
    lidar_roll, lidar_pitch, lidar_yaw = '0.0', '0.0', '0.0'

    # ── Camera mount (tweak to match your physical setup) ──────────────────────
    camera_x, camera_y, camera_z         = '0.1', '0.0', '0.15'
    camera_roll, camera_pitch, camera_yaw = '0.0', '0.0', '0.0'

    # ── Robot description ──────────────────────────────────────────────────────
    doc = xacro.process_file(xacro_file, mappings={'communication_type': 'microros'})
    robot_description = {'robot_description': doc.toxml()}

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 1  (t=0 s) – hardware layer
    # Must be up before anything else tries to read /tf or /odom.
    # ══════════════════════════════════════════════════════════════════════════

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config, {'use_sim_time': False}],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
            # Nav2 sends goals to /cmd_vel; the controller listens on ~/cmd_vel.
            # This remap makes them meet in the middle.
            ('/four_wheel_drive_controller/cmd_vel', '/cmd_vel'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Static TF: base_link → lidar_frame
    # The costmap obstacle_layer needs this to project /scan into the odom frame.
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            lidar_x, lidar_y, lidar_z,
            lidar_roll, lidar_pitch, lidar_yaw,
            'base_link', 'lidar_frame',
        ],
        parameters=[{'use_sim_time': False}],
    )

    # Static TF: base_link → camera_frame
    # Required so RViz and Nav2 can project the image into the robot frame.
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            camera_x, camera_y, camera_z,
            camera_roll, camera_pitch, camera_yaw,
            'base_link', 'camera_frame',
        ],
        parameters=[{'use_sim_time': False}],
    )

    # Lidar relay: /shelfbot_firmware/laser_scan → /scan (360° accumulator)
    lidar_relay = Node(
        package='shelfbot',
        executable='lidar_relay_node',
        name='lidar_relay_node',
        output='screen',
        parameters=[{
            'frame_id':   'lidar_frame',
            'publish_hz': 5.0,
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        respawn=True,
        respawn_delay=2.0,
    )

    # Camera republisher: /camera/compressed (CompressedImage) → /camera/raw (Image)
    # RViz's Image display requires a raw sensor_msgs/Image topic; it cannot
    # decode CompressedImage natively. image_transport's republish node does
    # the JPEG decompression so RViz gets a plain RGB frame.
    camera_republisher = Node(
        package='image_transport',
        executable='republish',
        name='camera_republisher',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/compressed'),
            ('out',           '/camera/raw'),
        ],
        parameters=[{'use_sim_time': False}],
        respawn=True,
        respawn_delay=2.0,
    )

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 2  (t=3 s) – ros2_control spawners
    # controller_manager needs a few seconds to finish loading the hardware
    # interface plugin before the spawners can handshake with it.
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
    # TIER 3  (t=8 s) – Nav2
    # Needs /odom and the full TF chain (map→odom→base_footprint→…) to be
    # stable before the costmaps can activate.
    #
    # nav2_camera_params.yaml config:
    #   • No map_server, no static_layer, no AMCL
    #   • Both costmaps use obstacle_layer fed by /scan
    #   • global_frame: odom  (robot navigates in odom space)
    # ══════════════════════════════════════════════════════════════════════════

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file':  nav2_params,
            'use_sim_time': 'False',
            'autostart':    'True',
        }.items(),
    )

    delay_nav2 = TimerAction(period=8.0, actions=[nav2])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 4  (t=15 s) – RViz
    # Last so it connects to already-active topics and shows real data
    # immediately rather than showing everything grey/unknown.
    #
    # In RViz add:
    #   • Image display  → topic: /camera/raw
    #   • LaserScan      → topic: /scan
    # ══════════════════════════════════════════════════════════════════════════

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}, robot_description],
        output='screen',
    )

    delay_rviz = TimerAction(period=15.0, actions=[rviz_node])

    # ══════════════════════════════════════════════════════════════════════════
    # All actions must be listed here — anything omitted simply does not launch.
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        # Tier 1 – hardware (immediate)
        robot_state_pub,
        control_node,
        static_tf_lidar,
        static_tf_camera,
        lidar_relay,
        camera_republisher,
        # Tier 2 – controllers (t=3 s)
        delay_controllers,
        # Tier 3 – Nav2 (t=8 s)
        delay_nav2,
        # Tier 4 – RViz (t=15 s)
        delay_rviz,
    ])
