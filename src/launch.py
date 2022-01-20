from  launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    camera = DeclareLaunchArgument('camera', default_value='realsense')

    rosbridge = Node(
            namespace='yolobot',
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge'
        )

    if camera == 'realsense': print("REAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAL")

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
                name='camera'
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
    triger_sub = Node(
            namespace='yolobot',
            package='yolobot_triger',
            executable='triger_sub',
            name='triger_sub',
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


    return LaunchDescription([
        camera,
        triger,
        triger_sub,
        camera_source,
        detectron,
        rosbridge,
    ])
