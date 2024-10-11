from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
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
    
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file_path,
            " sim_mode:=isaac"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_config = PathJoinSubstitution([pkg_share, 'config', 'isaac.sim.params.yaml'])

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {"use_sim_time": use_sim_time}],
        output="both",
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

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare('shelfbot'), 'config', 'shelfbot.rviz'])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    nodes = [
        robot_state_pub_node,
        joint_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delayed_rviz,
    ]
    return LaunchDescription(declared_arguments + nodes)