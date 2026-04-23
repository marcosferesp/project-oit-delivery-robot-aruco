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
                'video_device': '/dev/video2'
            }]
        )
    ])