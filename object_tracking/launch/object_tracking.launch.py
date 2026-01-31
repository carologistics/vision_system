
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('object_tracking'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_tracking',
            executable='object_tracking_node',
            name='object_tracking_server',
            parameters=[config],
            output='screen'
        )
    ])