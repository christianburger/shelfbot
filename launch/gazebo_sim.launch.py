from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare(package='shelfbot').find('shelfbot')
    urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', 'shelfbot.urdf.xacro'])
    
    world_file_path = '~/.world/shelfbot_world.world'
    
    robot_description_content = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', urdf_file_path
    ]), value_type=str)

    robot_description = {'robot_description': robot_description_content}

    controller_config = PathJoinSubstitution([pkg_share, 'config', 'four_wheel_drive_controller.yaml'])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'shelfbot'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config],
            output='screen',
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[robot_description, controller_config],
                    output='screen',
                ),
                on_exit=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                        output='screen',
                    ),
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['four_wheel_drive_controller', '--controller-manager', '/controller_manager'],
                        output='screen',
                    ),
                ],
            )
        ),
    ])
