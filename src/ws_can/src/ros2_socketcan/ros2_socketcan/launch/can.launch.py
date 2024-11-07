from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    socket_can_bridge_launch = get_package_share_directory('ros2_socketcan') + '/launch/socket_can_bridge.launch.xml'

    return LaunchDescription([
        GroupAction([
            PushRosNamespace('can0'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(socket_can_bridge_launch),
                launch_arguments={'interface': 'can0'}.items(),
            )
        ]),
        
        GroupAction([
            PushRosNamespace('can1'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(socket_can_bridge_launch),
                launch_arguments={'interface': 'can1'}.items(),
            )
        ]),
        
        # PCAN USB verify
        #GroupAction([
        #    PushRosNamespace('can4'),
        #    IncludeLaunchDescription(
        #        AnyLaunchDescriptionSource(socket_can_bridge_launch),
        #        launch_arguments={'interface': 'can4'}.items(),
        #    )
        #]),
        
        #GroupAction([
        #    PushRosNamespace('can5'),
        #    IncludeLaunchDescription(
        #        AnyLaunchDescriptionSource(socket_can_bridge_launch),
        #        launch_arguments={'interface': 'can5'}.items(),
        #    )
        #]),
        
        #GroupAction([
        #    PushRosNamespace('can7'),
        #    IncludeLaunchDescription(
        #        AnyLaunchDescriptionSource(socket_can_bridge_launch),
        #        launch_arguments={'interface': 'can7'}.items(),
        #    )
        #]),
        
        #GroupAction([
        #    PushRosNamespace('can8'),
        #    IncludeLaunchDescription(
        #        AnyLaunchDescriptionSource(socket_can_bridge_launch),
        #        launch_arguments={'interface': 'can8'}.items(),
        #    )
        #]),
        
        #GroupAction([
        #    PushRosNamespace('can9'),
        #    IncludeLaunchDescription(
        #        AnyLaunchDescriptionSource(socket_can_bridge_launch),
        #        launch_arguments={'interface': 'can9'}.items(),
        #    )
        #]),
    ])

