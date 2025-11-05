# Copyright 2022 Can-lab Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


## Configurable parameters ##
'''Available QoS values are the following strings:
SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT,
SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100),
EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durability).
'''
parameters = {
  "cam_0_enable": True,
  "cam_0_frame_id": "base_cam_0",
  "cam_0_image_qos": "SENSOR_DATA",
  "cam_0_info_qos": "SYSTEM_DEFAULT",
  "cam_0_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

  "cam_1_enable": True,
  "cam_1_frame_id": "base_cam_1",
  "cam_1_image_qos": "SENSOR_DATA",
  "cam_1_info_qos": "SYSTEM_DEFAULT",
  "cam_1_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

  "cam_2_enable": True,
  "cam_2_frame_id": "base_cam_2",
  "cam_2_image_qos": "SENSOR_DATA",
  "cam_2_info_qos": "SYSTEM_DEFAULT",
  "cam_2_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  
  "cam_3_enable": True,
  "cam_3_frame_id": "base_cam_3",
  "cam_3_image_qos": "SENSOR_DATA",
  "cam_3_info_qos": "SYSTEM_DEFAULT",
  "cam_3_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  
  "cam_4_enable": True,
  "cam_4_frame_id": "base_cam_4",
  "cam_4_image_qos": "SENSOR_DATA",
  "cam_4_info_qos": "SYSTEM_DEFAULT",
  "cam_4_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  
  "cam_5_enable": True,
  "cam_5_frame_id": "base_cam_5",
  "cam_5_image_qos": "SENSOR_DATA",
  "cam_5_info_qos": "SYSTEM_DEFAULT",
  "cam_5_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  
  "cam_6_enable": True,
  "cam_6_frame_id": "base_cam_6",
  "cam_6_image_qos": "SENSOR_DATA",
  "cam_6_info_qos": "SYSTEM_DEFAULT",
  "cam_6_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  
  "cam_7_enable": True,
  "cam_7_frame_id": "base_cam_7",
  "cam_7_image_qos": "SENSOR_DATA",
  "cam_7_info_qos": "SYSTEM_DEFAULT",
  "cam_7_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    password_arg = DeclareLaunchArgument(
        "password", default_value=TextSubstitution(text="'1'")
    )
    
    slave_arg = DeclareLaunchArgument(
        "slave", default_value=TextSubstitution(text="y")
    )

    encoding_arg = DeclareLaunchArgument(
        "encoding", default_value=TextSubstitution(text="jpeg")
    )
    
    timestamp_arg = DeclareLaunchArgument(
        "timestamp", default_value=TextSubstitution(text="local")
    )

    canlab_node = Node(
            package='clpe_ros',
            namespace='clpe',
            executable='clpe_ros',
            name='clpe_ros',
            parameters=[parameters, {
                "password": LaunchConfiguration('password'),
                "slave": LaunchConfiguration('slave'),
                "encoding": LaunchConfiguration('encoding'),
                "timestamp": LaunchConfiguration('timestamp'),
            }]
        )

    return LaunchDescription([
        password_arg,
        slave_arg,
        encoding_arg,
        timestamp_arg,
        canlab_node,
    ])
