
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # RTAB-Map node
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': False, # We are using depth camera, not LIDAR cloud

            # Input topics
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',

            # RTAB-Map configuration
            'Reg/Strategy': '1',  # 1=Visual, 0=Laser
            'Grid/FromDepth': 'true',
            'Grid/MaxObstacleHeight': '1.0',
            'Grid/RangeMax': '5.0',

            # Publish the obstacle cloud for Nav2
            'cloud_output': '/rtabmap/obstacles',
        }],
        remappings=[
            ('odom', '/odom') # Use the odom from ros2_control
        ],
        arguments=['-d'] # delete_db_on_start
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        rtabmap_node
    ])
