import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    pkg_share = get_package_share_directory('shelfbot')

    # ── URDF from xacro ──────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'communication_type': 'microros'})
    robot_description_content = doc.toxml()
    robot_description = {'robot_description': robot_description_content}

    controller_config = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')

    # ── Robot state publisher ────────────────────────────────────────────────
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ── ros2_control node ────────────────────────────────────────────────────
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

    # ── Controller spawners ──────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['four_wheel_drive_controller', '--controller-manager', '/controller_manager'],
    )

    # ── Lidar relay node ─────────────────────────────────────────────────────
    # Subscribes to /shelfbot_firmware/lidar_scan (Float32MultiArray, 30 floats)
    # Publishes    /scan (sensor_msgs/LaserScan) at 5 Hz for Nav2 / RViz2 / rtabmap
    lidar_relay_node = Node(
        package='shelfbot',
        executable='lidar_relay_node',
        name='lidar_relay_node',
        output='screen',
        parameters=[{
            'frame_id':       'laser_link',
            'publish_hz':     5.0,
            'min_confidence': 10,
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ── RViz (delayed so controllers are up first) ───────────────────────────
    rviz_config_file = os.path.join(pkg_share, 'config', 'nav2_troubleshoot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': False},
            robot_description,
        ],
        output='screen',
    )

    delay_rviz = TimerAction(period=5.0, actions=[rviz_node])

    # ── Launch description ───────────────────────────────────────────────────
    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        lidar_relay_node,
        delay_rviz,
    ])
