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

    camera_info_url   = 'file://' + os.path.join(pkg_share, 'config', 'esp32_cam_calibration.yaml')

    # ── Lidar mount ────────────────────────────────────────────────────────────
    lidar_x, lidar_y, lidar_z         = '0.0', '0.0', '0.2'
    lidar_roll, lidar_pitch, lidar_yaw = '0.0', '0.0', '0.0'

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

    lidar_relay = Node(
        package='shelfbot',
        executable='lidar_relay_node',
        name='lidar_relay_node',
        output='screen',
        parameters=[{'frame_id': 'lidar_frame', 'publish_hz': 5.0}],
        arguments=['--ros-args', '--log-level', 'warn'],
        respawn=True,
        respawn_delay=2.0,
    )

    # ── Camera pipeline ────────────────────────────────────────────────────────
    # Subscribes /camera/compressed  (RELIABLE, VOLATILE)  from ESP32 micro-ROS
    # Publishes  /camera/image_raw   (RELIABLE, VOLATILE)
    #            /camera/camera_info (RELIABLE, VOLATILE)
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
    # TIER 3  (t=6 s) – perception nodes
    # Needs camera_publisher to be publishing before these subscribe.
    #
    # apriltag_detector:
    #   Subscribes  /camera/image_raw   (RELIABLE, VOLATILE)
    #               /camera/camera_info (RELIABLE, VOLATILE)
    #   Publishes   /tag_poses          (RELIABLE, VOLATILE)
    #               /apriltag_markers   (RELIABLE, VOLATILE)
    #               TF: camera_link_optical_frame → tag_N
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

    delay_perception = TimerAction(
        period=6.0,
        actions=[apriltag_detector],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 4  (t=10 s) – Nav2
    # Needs /odom and the full TF chain stable before costmaps activate.
    # nav2_camera_params.yaml must set:
    #   global_frame: odom    (navigates in odom space until AMCL provides map)
    #   obstacle_layer source: /scan
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

    delay_nav2 = TimerAction(period=10.0, actions=[nav2])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 5  (t=18 s) – RViz
    # ══════════════════════════════════════════════════════════════════════════

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}, robot_description],
        output='screen',
    )

    delay_rviz = TimerAction(period=18.0, actions=[rviz_node])

    return LaunchDescription([
        # Tier 1 – hardware + camera pipeline (immediate)
        robot_state_pub,
        control_node,
        static_tf_lidar,
        lidar_relay,
        camera_publisher,
        # Tier 2 – controllers (t=3 s)
        delay_controllers,
        # Tier 3 – perception (t=6 s)
        delay_perception,
        # Tier 4 – Nav2 (t=10 s)
        delay_nav2,
        # Tier 5 – RViz (t=18 s)
        delay_rviz,
    ])
