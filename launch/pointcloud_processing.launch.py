import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_config = os.path.join(get_package_share_directory('pointcloud_processing'), 'config', 'tf_lidar.yaml')
    return LaunchDescription([
        Node
        (
            package='pointcloud_processing',
            node_executable='pointcloud_processing_node',
            node_name='pointcloud_processing_node',
            remappings=[('/points_in','/camera/points'),
                        ('/laser', '/scan')],
            parameters=[param_config]           
        )
    ])