
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'))
    bt_xml_file = LaunchConfiguration('bt_xml_file', default=os.path.join(shelfbot_share_dir, 'config', 'mission.xml'))

    # --- 1. Launch the Gazebo Simulation ---
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(shelfbot_share_dir, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={'gui': 'true'}.items()
    )

    # --- 2. Launch the VSLAM System (RTAB-Map) ---
    rtabmap_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'odom_topic': '/odom',
            'qos': '2',
            'rtabmap_args': '-d',
            'approx_sync': 'true' 
        }.items()
    )

    # --- 3. Launch the Nav2 Stack ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'map': '',
            'bt_xml_filename': bt_xml_file
        }.items(),
    )

    # --- 4. Launch the Mission Starter Node ---
    mission_starter_node = Node(
        package='shelfbot',
        executable='mission_starter.py',
        name='mission_starter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'),
            description='Full path to the Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=os.path.join(shelfbot_share_dir, 'config', 'mission.xml'),
            description='Full path to the custom behavior tree XML file'
        ),

        gazebo_sim_launch,
        rtabmap_launch_include,
        nav2_bringup_launch,
        mission_starter_node
    ])
