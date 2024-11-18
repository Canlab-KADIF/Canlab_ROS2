from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canlab_listen',
            executable='canlab_listen_node',
            name='canlab_listen_node',
            output='screen',
        ),
    ])

