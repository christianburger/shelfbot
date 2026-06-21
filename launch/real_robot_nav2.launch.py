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
    ekf_config         = os.path.join(pkg_share, 'config', 'ekf.yaml')

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

    lidar_relay = Node(
        package='shelfbot',
        executable='lidar_relay_node',
        name='lidar_relay_node',
        output='screen',
        parameters=[{
            'frame_id': 'laser_link',
            'publish_hz': 10.0,
            'qos_reliability': 'best_effort',
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        respawn=True,
        respawn_delay=2.0,
    )

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
    # TIER 3  (t=5 s) – EKF
    #
    # PREREQUISITE: four_wheel_drive_odometry.cpp must publish to /wheel_odom_raw
    # (not /odom) and must have publish_tf set to false so only the EKF owns
    # the odom→base_footprint TF.  See notes in ekf.yaml.
    #
    # The EKF remaps its filtered output to /odom so all downstream consumers
    # (Nav2, slam_toolbox) see a single authoritative /odom topic.
    #
    # CRITICAL: only ONE node must broadcast odom→base_footprint at a time.
    # Having both the raw odometry C++ code and the EKF publish this TF
    # simultaneously causes the "extrapolation into the future" errors in
    # the controller_server because the TF buffer receives conflicting entries
    # from two sources with slightly different timestamps.
    # ══════════════════════════════════════════════════════════════════════════

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            # EKF publishes filtered odometry here; remap so Nav2 sees /odom
            ('odometry/filtered', '/odom'),
        ],
    )

    delay_ekf = TimerAction(period=5.0, actions=[ekf_node])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 4  (t=7 s) – slam_toolbox
    #
    # Delayed after EKF so slam_toolbox's first scan lookup finds a valid
    # odom→base_footprint TF in the EKF's buffer, not the raw odometry's.
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

    delay_slam = TimerAction(period=7.0, actions=[slam_toolbox])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 5  (t=9 s) – AprilTag detector
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

    delay_perception = TimerAction(period=9.0, actions=[apriltag_detector])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 6  (t=14 s) – Nav2
    #
    # Delayed 2 s longer than before so slam_toolbox has had 7 s to build at
    # least one complete map→odom TF entry before the global costmap's
    # message filter starts processing laser_link scans.  Previously scans
    # arrived with firmware timestamps that were already 1-2 s old by the
    # time the global costmap tried to look up the TF, exceeding the 1.0 s
    # transform_tolerance and causing the message filter to drop them.
    #
    # This extra headroom — combined with the transform_tolerance increase in
    # nav2_params.yaml — eliminates the "timestamp earlier than all available"
    # drops from the global_costmap.
    # ══════════════════════════════════════════════════════════════════════════

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file':  nav2_params,
            'use_sim_time': 'false',
            'autostart':    'true',
            'remappings':   '[("controller_server/cmd_vel", "/cmd_vel_nav")]',
        }.items(),
    )

    delay_nav2 = TimerAction(period=14.0, actions=[nav2])

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 7  (t=22 s) – RViz
    # ══════════════════════════════════════════════════════════════════════════

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}, robot_description],
        output='screen',
    )

    delay_rviz = TimerAction(period=22.0, actions=[rviz_node])

    # ── LaunchDescription with all actions ────────────────────────────────────
    return LaunchDescription([
        robot_state_pub,
        control_node,
        lidar_relay,
        camera_publisher,
        delay_ekf,
        delay_controllers,
        delay_slam,
        delay_perception,
        delay_nav2,
        delay_rviz,
    ])