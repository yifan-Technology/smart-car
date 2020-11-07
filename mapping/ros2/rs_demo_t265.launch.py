import os
import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    t265_base_frame_id = LaunchConfiguration('base_frame_id', default='t265_link')

    rviz_config_dir = os.path.join(get_package_share_directory('realsense_examples'), 'config', 'demo_t265.rviz')

    t265_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/t265",
	remappings=[('camera/odom/sample','/odom')],
        output='screen',
        parameters=[{'base_frame_id': t265_base_frame_id}]
        )
    return launch.LaunchDescription([t265_node])
