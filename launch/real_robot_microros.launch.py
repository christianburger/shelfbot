import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # --- The Robust URDF Generation Method ---

    # 1. Find the xacro file
    pkg_share = get_package_share_directory('shelfbot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')

    # 2. Process the xacro file in-memory, passing the correct arguments.
    doc = xacro.process_file(xacro_file, mappings={'communication_type': 'microros'})
    robot_description_content = doc.toxml()
    robot_description = {"robot_description": robot_description_content}

    # --- End of Robust URDF Generation Method ---

    controller_config = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {"use_sim_time": False}],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/four_wheel_drive_controller/cmd_vel", "/cmd_vel"),  # ADDED: Map Nav2's cmd_vel to controller
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["four_wheel_drive_controller", "--controller-manager", "/controller_manager"],
    )


    camera_publisher_node = Node(
        package="shelfbot",
        executable="camera_publisher",
        name="camera_publisher",
        output="screen",
        parameters=[{
            "camera_info_url": "package://shelfbot/config/camera_info.yaml",
            "camera_name": "esp32_cam",
            "frame_id": "camera_link_optical_frame",
            "compressed_image_topic": "/camera/compressed",
            "image_topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "image_width": 800,
            "image_height": 600,
            "use_sim_time": False,
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    rviz_config_file = os.path.join(pkg_share, 'config', 'nav2_troubleshoot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': False},
            robot_description
        ],
        output='screen',
    )

    delay_rviz_after_joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    nodes = [
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        camera_publisher_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]
    return LaunchDescription(nodes)
