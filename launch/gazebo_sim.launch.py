import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo GUI if true",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    pkg_share = get_package_share_directory('shelfbot')

    xacro_file = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')
    doc = xacro.process_file(xacro_file, mappings={'sim_mode': 'gazebo', 'controller_config_file': controller_config})
    robot_description_content = doc.toxml()
    robot_description = {"robot_description": robot_description_content}
    world_file_path = os.path.join(pkg_share, 'worlds', 'empty.world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Path to the world file'
    )

    # --- Corrected Gazebo Launch ---
    # Launch the Gazebo server with the correct plugins
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Launch the Gazebo client
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )
    # --- End of Correction ---

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'shelfbot'],
        output='screen'
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
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

    # Delay controllers after robot spawn
    delay_spawners_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        robot_controller_spawner,
                    ]
                )
            ]
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description
        ],
        output='screen',
    )

    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz_node],
    )

    nodes = [
        world_arg,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity_node,
        robot_state_pub_node,
        delay_spawners_after_spawn,
        delayed_rviz,
    ]
    return LaunchDescription(declared_arguments + nodes)
