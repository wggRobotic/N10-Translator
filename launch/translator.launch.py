import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='translator',
            executable='translator',
            name='translator',
            parameters=[os.path.join(os.path.dirname(__file__), '../parameter/translator.yaml')]
        )
    ])
