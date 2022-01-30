import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('yolobot_detection'),
        'config',
        'params.yaml'
        )

    node = Node(
        namespace='yolobot',
        package='yolobot_detection',
        name='detectron',
        executable='yolobot_detection',
        parameters=[config]
    )
    ld.add_action(node)
    return ld
