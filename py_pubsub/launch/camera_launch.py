# ==============================================================================
# camera_launch.py
# Launches the USB camera hardware driver to broadcast raw video to the ROS 2 network
# Author : Author : Marcos Ferrando España
# ==============================================================================

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('py_pubsub'),
        'config',
        'logitech_streamcam_1080p.yaml'
    )

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            parameters=[{
                'image_size': [1920, 1080],
                'video_device':'/dev/video2',
                'camera_frame_id': 'camera_link_optical',
                'camera_info_url': f'file://{config_dir}'
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=[
                '0.0', '0.0', '0.50',       # Distance according to wheels in (x,y,z)
                '0.0', '-1.5708', '0.0',    # -90 degrees because the camera is now facing upstairs
                'base_link',
                'camera_link_optical'
            ]
        )
    ])