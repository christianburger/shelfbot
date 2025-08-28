import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- Launch RTAB-Map Node Directly (RGB-only) ---
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_stereo': False,
            'subscribe_rgbd': False,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
            'qos_image': 1,
            'qos_camera_info': 1,
            'qos_odom': 1,
            # Visual SLAM parameters - all RTAB-Map parameters must be strings
            'Reg/Strategy': '1',      # Visual registration
            'Vis/MinInliers': '15',   # Minimum visual inliers
            'RGBD/Enabled': 'false',  # Disable RGBD mode (string, not bool)
            'Grid/FromDepth': 'false',  # Don't create grid from depth (string, not bool)
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--', '--delete_db_on_start'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        rtabmap_node
    ])
