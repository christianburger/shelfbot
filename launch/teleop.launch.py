# launch/teleop.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'enable_button': 4,
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 0.5,
            'scale_angular.yaw': 1.0,
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        joy,
        teleop,
    ])