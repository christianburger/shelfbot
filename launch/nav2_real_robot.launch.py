import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    shelfbot_share_dir = get_package_share_directory('shelfbot')

    # --- Launch Arguments ---
    params_file = LaunchConfiguration('params_file', default=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'))

    # --- 1. Launch the Real Robot Drivers ---
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(shelfbot_share_dir, 'config', 'four_wheel_drive_controller.yaml')],
        output='screen',
    )
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(shelfbot_share_dir, 'launch', 'real_robot_microros.launch.py')
        )
    )

    # --- 4. Launch Robot Localization (EKF) ---
    # ekf_config = os.path.join(shelfbot_share_dir, 'config', 'ekf.yaml')
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config, {'use_sim_time': False}]
    # )

    # --- 5. Launch RTAB-Map Node (mono-RGB only, publish SLAM pose + sparse map) ---
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
          'frame_id': 'base_footprint',
          'map_frame_id': 'odom',            # VO-only: no map frame
          'publish_tf': False,               # don't publish map->odom
          'use_sim_time': False,
          'subscribe_rgb': True,
          'subscribe_depth': False,
          'subscribe_rgbd': False,
          'RGBD/Enabled': 'false',
          'RGBD/LoopClosureDetection': 'false',  # disable loop closures
          'tf_delay': 0.0,
          'odom_sensor_sync': True,
          'tf_tolerance': 0.8,
          'Rtabmap/PublishLastSignature': 'true',
          'Rtabmap/PublishMapData': 'true',
          'approx_sync': True,
          'topic_queue_size': 500,
          'sync_queue_size': 500,
        }],
        remappings=[
            ('rgb/image',        '/camera/image_raw'),
            ('rgb/camera_info',  '/camera/camera_info'),
            ('odom',             '/odom'),
        ],
        arguments=[
            '--ros-args', '--log-level', 'info',
            '--', '--delete_db_on_start'
        ],
        output='screen'
    )

    # --- 6. Launch Nav2 Nodes ---
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
    ]

    lifecycle_manager_node = Node(
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
    
    delayed_lifecycle_manager = TimerAction(period=2.0, actions=[lifecycle_manager_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(shelfbot_share_dir, 'config', 'nav2_camera_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),

        ros2_control_node,
        real_robot_launch,
        # ekf_node,
        rtabmap_node,
        delayed_lifecycle_manager,
    ] + nav2_nodes)
