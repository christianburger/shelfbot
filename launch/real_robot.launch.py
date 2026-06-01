from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='shelfbot').find('shelfbot')
    urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', 'shelfbot.urdf.xacro'])
    
    robot_description_content = Command(
      [
        FindExecutable(name="xacro"),
        " ",
        urdf_file_path,
        " base_url:=http://shelfbot.local/"
      ]
    )

    robot_description = {"robot_description": robot_description_content}

    controller_config = PathJoinSubstitution([pkg_share, 'config', 'real.robot.params.yaml'])

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {"use_sim_time": False}],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/four_wheel_drive_controller/tf_odometry", "/tf"),
        ],
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

    rviz_config_file = PathJoinSubstitution([FindPackageShare('shelfbot'), 'config', 'shelfbot.rviz'])

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
