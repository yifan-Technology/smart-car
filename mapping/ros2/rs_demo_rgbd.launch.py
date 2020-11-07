# Copyright (c) 2019 Intel Corporation
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

# /* Author: Gary Liu */
import os
import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rgbd_base_frame_id = LaunchConfiguration('base_frame_id', default='d400_link')
    t265_base_frame_id = LaunchConfiguration('base_frame_id', default='t265_link')

    rviz_config_dir = os.path.join(get_package_share_directory('realsense_examples'), 'config', 'demo_rgbd.rviz')

    rgbd_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace='',
        remappings=[( 'camera/color/camera_info', 'rgb/camera_info')],
        output='screen',
        parameters=[{'base_frame_id': rgbd_base_frame_id,
                     'enable_pointcloud': False}]
        )

    tf_node = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.03', '0', '0', '0', t265_base_frame_id, rgbd_base_frame_id]
            )

    rviz_node = Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': 'false'}]
            )    

    return launch.LaunchDescription([rgbd_node, tf_node])
