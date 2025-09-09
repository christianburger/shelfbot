from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.16',
            description='Size of AprilTags in meters'
        ),
        
        # Camera Info Publisher
        Node(
            package='shelfbot',
            executable='camera_publisher',
            name='camera_info_publisher',
            parameters=[{
                'image_width': 800,
                'image_height': 600,
                'focal_length': 600.0,
                'frame_id': 'camera_link'
            }],
            output='screen'
        ),
        
        # AprilTag Detector Node - THIS WAS MISSING!
        Node(
            package='shelfbot',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            parameters=[{
                'tag_size': LaunchConfiguration('tag_size'),
                'pose_error_threshold': 50.0,
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')  # Fixed remapping
            ],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/apriltag_config.rviz'],
            output='screen'
        )
    ])