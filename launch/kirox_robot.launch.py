from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='攝影機的裝置路徑，例如 /dev/video0, /dev/video1'
    )

    return LaunchDescription([
        # 讓所有子行程預設走 Jetson 本機 X11 顯示
        SetEnvironmentVariable(name='DISPLAY', value=':0'),

        video_device_arg,

        Node(package='kirox_robot', executable='robotbrain', name='robotbrain'),
        Node(package='kirox_robot', executable='roboteyes', name='roboteyes'),
        Node(package='kirox_robot', executable='robotbody', name='robotbody'),
        Node(package='kirox_robot', executable='robotears', name='robotears'),
        Node(package='kirox_robot', executable='wsconnection', name='wsconnection'),

        Node(
            package='kirox_robot',
            executable='camera',
            name='camera',
            parameters=[{'device': LaunchConfiguration('video_device')}]
        ),
        Node(
            package='kirox_robot',
            executable='robotface',
            name='robotface',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'}
        ),
    ])