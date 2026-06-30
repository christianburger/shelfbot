import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────────
    pkg_share          = get_package_share_directory('shelfbot')
    nav2_bringup_dir   = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir   = get_package_share_directory('slam_toolbox')

    xacro_file          = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    controller_config   = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')
    nav2_params         = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params         = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config         = os.path.join(pkg_share, 'config', 'nav2_troubleshoot.rviz')
    camera_info_url     = 'file://' + os.path.join(pkg_share, 'config', 'esp32_cam_calibration.yaml')
    default_bt_xml_path = os.path.join(pkg_share, 'config', 'exploration_tree.xml')

    # ── Robot description ──────────────────────────────────────────────────────
    doc = xacro.process_file(xacro_file, mappings={'communication_type': 'microros'})
    robot_description = {'robot_description': doc.toxml()}

    # ── Mission launch arguments (exploration BT) ───────────────────────────────
    # These control the new exploration_bt_node mission tier added below. The
    # rest of the launch file (hardware, camera, SLAM, Nav2, RViz) is unchanged.
    run_exploration_mission_arg = DeclareLaunchArgument(
        'run_exploration_mission', default_value='true',
        description='Bring up the frontier_discovery / frontier_queue / tag_registry / '
                     'exploration_bt mission tier on top of base Nav2 bring-up.')

    target_tag_ids_arg = DeclareLaunchArgument(
        'target_tag_ids', default_value='[1, 2, 3]',
        description='YAML list of AprilTag IDs that terminate exploration when all are found.')

    bt_xml_path_arg = DeclareLaunchArgument(
        'bt_xml_path', default_value=default_bt_xml_path,
        description='Path to the exploration BehaviorTree XML.')

    enable_groot_monitoring_arg = DeclareLaunchArgument(
        'enable_groot_monitoring', default_value='false',
        description='Publish BT state over ZMQ for live Groot2 monitoring.')

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
            'reverse_scan_order': True,
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
    # TIER 4  (t=7 s) – slam_toolbox
    #
    # Delayed until the hardware odometry publisher has produced a valid
    # odom→base_footprint TF for slam_toolbox's first scan lookup.
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

    # ══════════════════════════════════════════════════════════════════════════
    # TIER 8  (t=26 s) – Exploration mission (custom BehaviorTree.CPP layer)
    #
    # Brought up 4 s after RViz / 12 s after Nav2's own TimerAction fires, by
    # which point bt_navigator's lifecycle has had time to activate and
    # /navigate_to_pose is serving goals. These four nodes only talk to each
    # other and to Nav2 over topics/services — they hold no references to any
    # node above — so this tier can be disabled wholesale via
    # `run_exploration_mission:=false` without touching Tiers 1-7.
    #
    # frontier_discovery_node  /map            → /frontiers            (service: get_frontiers)
    # frontier_queue_node      /frontiers      → dedup + status queue  (services: get_next /
    #                                                                    update_status / get_summary)
    # tag_registry_node        /tag_detections → found-tag tracking    (service: check_found)
    # exploration_bt_node      ticks the custom BT (config/exploration_tree.xml), driving
    #                           /navigate_to_pose and /cmd_vel until all target_tag_ids are
    #                           found or the frontier queue is exhausted.
    # ══════════════════════════════════════════════════════════════════════════

    frontier_discovery = Node(
        package='shelfbot',
        executable='frontier_discovery_node',
        name='frontier_discovery',
        output='screen',
        parameters=[{
            'min_frontier_size': 5,
            'publish_hz': 2.0,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
    )

    frontier_queue = Node(
        package='shelfbot',
        executable='frontier_queue_node',
        name='frontier_queue',
        output='screen',
        parameters=[{
            'merge_radius':    0.40,
            'max_attempts':    3,
            'requeue_delay_s': 30.0,
            'publish_hz':      2.0,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
    )

    tag_registry = Node(
        package='shelfbot',
        executable='tag_registry_node',
        name='tag_registry',
        output='screen',
        parameters=[{
            'map_frame':       'map',
            'camera_frame':    'camera_link_optical_frame',
            'pose_avg_alpha':  0.10,
            'publish_hz':      2.0,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
    )

    exploration_bt = Node(
        package='shelfbot',
        executable='exploration_bt_node',
        name='exploration_bt',
        output='screen',
        parameters=[{
            'target_tag_ids':          LaunchConfiguration('target_tag_ids'),
            'spin_angular_vel':        0.6,
            'spin_duration_s':         10.5,
            'tick_period_ms':          200,
            'bt_xml_path':             LaunchConfiguration('bt_xml_path'),
            'enable_groot_monitoring': LaunchConfiguration('enable_groot_monitoring'),
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
    )

    delay_mission = TimerAction(
        period=26.0,
        actions=[frontier_discovery, frontier_queue, tag_registry, exploration_bt],
    )

    # ── LaunchDescription with all actions ────────────────────────────────────
    return LaunchDescription([
        run_exploration_mission_arg,
        target_tag_ids_arg,
        bt_xml_path_arg,
        enable_groot_monitoring_arg,

        robot_state_pub,
        control_node,
        lidar_relay,
        camera_publisher,
        delay_controllers,
        delay_slam,
        delay_perception,
        delay_nav2,
        delay_rviz,
        delay_mission,
    ])
