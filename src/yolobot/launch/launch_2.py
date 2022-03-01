import os
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    camera_source = Node(
        namespace='yolobot_2',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera'
        parameters=[
                {"align_depth": True},
                {"pointcloud": True},
                {"clip_distance": 1.5},
                ]
    )

    return LaunchDescription([
        camera_source,
    ])
