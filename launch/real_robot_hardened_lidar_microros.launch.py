import os
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    # ── Paths ─────────────────────────────────────────────────────────────────
    pkg_share = get_package_share_directory('shelfbot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_troubleshoot.rviz')

    # ── Lidar mount position (adjust to your actual setup) ────────────────────
    lidar_x = '0.0'
    lidar_y = '0.0'
    lidar_z = '0.2'
    lidar_roll = '0.0'
    lidar_pitch = '0.0'
    lidar_yaw = '0.0'

    # ── Robot description ─────────────────────────────────────────────────────
    doc = xacro.process_file(xacro_file, mappings={'communication_type': 'microros'})
    robot_description = {'robot_description': doc.toxml()}

    # ── Core nodes ────────────────────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ros2_control
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

    # Controller spawners
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

    # Static transform base_link → lidar_frame
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            lidar_x, lidar_y, lidar_z,
            lidar_roll, lidar_pitch, lidar_yaw,
            'base_link', 'lidar_frame'
        ],
        parameters=[{'use_sim_time': False}],
    )

    # Lidar relay node – respawned if it crashes
    lidar_relay = Node(
        package='shelfbot',
        executable='lidar_relay_node',
        name='lidar_relay_node',
        output='screen',
        parameters=[{
            'frame_id': 'lidar_frame',        # must match firmware frame
            'publish_hz': 5.0,
            # optionally override input topic:
            # 'input_topic': '/shelfbot_firmware/laser_scan',
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        # Restart automatically if the node dies (e.g., due to a momentary network issue)
        respawn=True,
        respawn_delay=2.0,
    )

    # RViz (delayed so everything is up first)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}, robot_description],
        output='screen',
    )
    delay_rviz = TimerAction(period=5.0, actions=[rviz_node])

    # ── Launch sequence ───────────────────────────────────────────────────────
    return LaunchDescription([
        robot_state_pub,
        control_node,
        joint_broadcaster_spawner,
        drive_controller_spawner,
        static_tf_lidar,
        lidar_relay,
        delay_rviz,
    ])