# ==============================================================================
# camera_launch.py
# Launches the USB camera hardware driver to broadcast raw video to the ROS 2 network
# Author : Author : Marcos Ferrando España
# ==============================================================================

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            parameters=[{
                'image_size': [640, 480],
                'video_device':'/dev/video1',
                'camera_frame_id': 'camera_link_optical'
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=[
                '0.0', '0.0', '0.40',
                '0.0', '0.0', '0.0',
                'base_link',
                'camera_link_optical'
            ]
        )
    ])