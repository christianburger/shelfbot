def generate_launch_description():
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    rtabmap_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
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

    return LaunchDescription([
        rtabmap_launch_include
    ])
