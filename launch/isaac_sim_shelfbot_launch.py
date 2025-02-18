from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Isaac Sim) clock if true",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_share = FindPackageShare(package='shelfbot').find('shelfbot')
    urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', 'shelfbot.urdf.xacro'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'isaac.sim.params.yaml'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare('shelfbot'), 'config', 'shelfbot.rviz'])

    # Fixed robot description - explicitly set mock communication for Isaac sim
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file_path,
            " sim_mode:=isaac",
            " communication_type:=mock"  # Hardcoded mock for Isaac sim
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Rest of the nodes remain the same
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {"use_sim_time": use_sim_time}],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/four_wheel_drive_controller/tf_odometry", "/tf"),
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["four_wheel_drive_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time},
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
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)