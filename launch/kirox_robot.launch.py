from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='攝影機的裝置路徑，例如 /dev/video0, /dev/video1'
    )
    return LaunchDescription([
        video_device_arg,

        Node(package='kirox_robot', executable='robotbrain', name='robotbrain'),
        Node(package='kirox_robot', executable='robotface', name='robotface'),  # 先不啟
        Node(package='kirox_robot', executable='roboteyes', name='roboteyes'),
        # Node(package='kirox_robot', executable='robotbody', name='robotbody'),  # 先不啟
        Node(package='kirox_robot',executable='robotears',name='robotears'),
        Node(package='kirox_robot', executable='wsconnection', name='wsconnection'),
        Node(
            package='kirox_robot',
            executable='camera',
            name='camera',
            parameters=[{'device': LaunchConfiguration('video_device')}]
        ),
    ])
