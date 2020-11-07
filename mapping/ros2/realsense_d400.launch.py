# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 run realsense_node realsense_node
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory('realsense_examples'), 'config', 'demo_rgbd.rviz')

    parameters=[{'rviz':True, 'visual_odometry':False,'rgbd_sync':True,
          'frame_id':'t265_link',
          'subscribe_depth':True,
          'approx_sync':True}]

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          #('rgb/camera_info', '/d400/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Nodes to launch
        #Node(
        #    package='rtabmap_ros', node_executable='rgbd_odometry', output='screen',
        #    parameters=parameters,
        #    remappings=remappings),
            
        Node(
            package='rtabmap_ros', node_executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time':False}]
            )    
    ])
