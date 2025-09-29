from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='kirox_robot', executable='robotbrain', name='robotbrain'),
        Node(package='kirox_robot', executable='robotface', name='robotface'),
        Node(package='kirox_robot', executable='roboteyes', name='roboteyes'),
        Node(package='kirox_robot', executable='robotears', name='robotears'),
        Node(package='kirox_robot', executable='wsconnection', name='wsconnection'),
        Node(package='kirox_robot', executable='camera', name='camera'),

    ])
