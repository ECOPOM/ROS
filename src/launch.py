from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    camera = DeclareLaunchArgument('camera', default_value='realsense')
    if camera == 'realsense': print("REAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAL")

    # realsense2_camera  = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('realsense2_camera'),
    #             'launch/rs_launch.py')
    #     ),
    #     launch_arguments={'enable_pointcloud': 'true'}.items(),
    # )

    realsense = False
    if realsense:
        camera_source = Node(
                namespace='yolobot',
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='camera'
            )
    else:
        camera_source = Node(
                namespace='yolobot',
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='camera',
                remappings=[
                    ("image_raw", "color/image_raw")
                ]
            )

    triger = Node(
            namespace='yolobot',
            package='yolobot_triger',
            executable='triger',
            name='triger',
            parameters=[
                      {"background_r": 0},
                      {"background_g": 0},
                      {"background_b": 0}
                       ]
        )

    detectron = Node(
            namespace='yolobot',
            package='yolobot_detection',
            executable='detect',
            name='detectron'
        )

    trigered = Node(
            namespace='trigered',
            package='yolobot_triger',
            executable='trigered',
            name='trigered',
        )

    rosbridge = Node(
            namespace='trigered',
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge'
        )

    return LaunchDescription([
        camera,
        triger,
        trigered,
        camera_source,
        detectron,
        rosbridge,
    ])
