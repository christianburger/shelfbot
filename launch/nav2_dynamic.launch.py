
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    # --- Launch Arguments ---
    params_file = LaunchConfiguration('params_file', default=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'))
    bt_xml_file = LaunchConfiguration('bt_xml_file', default=os.path.join(shelfbot_share_dir, 'config', 'mission.xml'))

    # --- Robot State Publisher ---
    robot_description_content = Command(['xacro ', os.path.join(shelfbot_share_dir, 'urdf', 'shelfbot.urdf.xacro')])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    # --- 1. Launch the VSLAM System (RTAB-Map) ---
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_base_link',
        arguments=['0', '0', '0.085', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )
    static_tf_base_link_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_camera_link',
        arguments=['0', '0', '0.02', '0', '0', '-1.57079632679', 'base_link', 'camera_link'],
        output='screen'
    )
    rtabmap_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'use_sim_time': False,
            'frame_id': 'base_footprint',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'odom_topic': '/odom',
            'qos': '2',
            'rtabmap_args': '-d',
            'approx_sync': 'true',
            'tf_delay': 0.5,
        }.items()
    )

    # --- 3. Launch the Nav2 Stack ---
    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'smoother_server',
                                        'behavior_server',
                                        'bt_navigator',
                                        'waypoint_follower']}])
    ]

    # --- 4. Launch the Mission Starter Node ---
    mission_starter_node = Node(
        package='shelfbot',
        executable='mission_starter.py',
        name='mission_starter',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
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

        robot_state_publisher_node,
        static_tf_base_footprint_to_base_link,
        static_tf_base_link_to_camera_link,
        rtabmap_launch_include,
        mission_starter_node
    ] + nav2_nodes)
