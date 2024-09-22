from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('shelfbot')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'shelfbot.urdf.xacro')
    
    world_file_path = os.path.expanduser('~/.world/shelfbot_world.world')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    controller_config = os.path.join(pkg_share, 'config', 'four_wheel_drive_controller.yaml')

    return LaunchDescription([
        # Launches Gazebo with the specified world file
        # Parameters: world file path, Gazebo ROS plugins
        # Bash equivalent: gazebo --verbose ~/.world/shelfbot_world.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawns the robot entity in Gazebo
        # Parameters: robot description topic, entity name
        # Bash equivalent: ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity shelfbot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'shelfbot'],
            output='screen'
        ),

        # Publishes the robot state
        # Parameters: robot description
        # Bash equivalent: ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /path/to/shelfbot.urdf.xacro)"
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Launches the ros2_control node
        # Parameters: robot description, controller configuration
        # Bash equivalent: ros2 run controller_manager ros2_control_node --ros-args -p robot_description:="$(xacro /path/to/shelfbot.urdf.xacro)" --params-file /path/to/four_wheel_drive_controller.yaml
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description},
                        controller_config],
            output='screen',
        ),

        # Spawns the four wheel drive controller
        # Parameters: controller name
        # Bash equivalent: ros2 run controller_manager spawner four_wheel_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['four_wheel_drive_controller'],
            output='screen',
        ),
    ])
