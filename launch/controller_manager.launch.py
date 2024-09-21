from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to your URDF or Xacro file
    urdf_file_path = os.path.join(get_package_share_directory('shelfbot'), 'urdf', 'shelfbot.urdf.xacro')

    # Check if the URDF/Xacro file exists
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF/Xacro file not found: {urdf_file_path}")

    # Read the URDF/Xacro file content
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Wrap the URDF content in ParameterValue to ensure it is treated as a string
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Path to the YAML configuration file for the four-wheel drive controller
    controller_config_path = os.path.join(get_package_share_directory('shelfbot'), 'config', 'four_wheel_drive_controller.yaml')

    return LaunchDescription([
        # Controller Manager Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controller_config_path],
        ),

        # Use the spawner to spawn the four_wheel_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['four_wheel_drive_controller'],  # Spawning the specific controller
            output='screen',
        ),
    ])
