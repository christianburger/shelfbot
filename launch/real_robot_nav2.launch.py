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
    # TIER 3  (t=6 s) – slam_toolbox
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
    # FIX: Explicitly remap controller_server's cmd_vel output to /cmd_vel_nav.
    # Without this, controller_server publishes to the default topic and
    # velocity_smoother never receives commands.
    # ══════════════════════════════════════════════════════════════════════════

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file':  nav2_params,
            'use_sim_time': 'false',
            'autostart':    'true',
            # ─── CRITICAL: remap controller_server's cmd_vel to /cmd_vel_nav ───
            'remappings':   '[("controller_server/cmd_vel", "/cmd_vel_nav")]',
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
        robot_state_pub,
        control_node,
        lidar_relay,
        camera_publisher,
        delay_controllers,
        delay_slam,
        delay_perception,
        delay_nav2,
        delay_rviz,
    ])