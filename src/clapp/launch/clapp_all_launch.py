from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(package='clapp', executable='clapp_node', name='clapp_node', output='screen'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('clpe_ros'), '/launch', '/clpe_ros.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ros2_socketcan'), '/launch', '/can.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('rslidar_sdk'), '/launch', '/start.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('canlab_listen'), '/launch', '/canlab_listener.py'])
        ),
    ])

