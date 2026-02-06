import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('alfa_robot_vision'),
        'config',
        'vision_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='alfa_robot_vision',
            executable='vision_server',
            name='vision_server',
            output='screen',
            parameters=[config],
            remappings=[
                ('/head_camera/rgb/image_raw', '/camera/camera/color/image_raw'),
                ('/head_camera/depth/image_rect_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('/head_camera/camera_info', '/camera/camera/color/camera_info')
            ]
        )
    ])
