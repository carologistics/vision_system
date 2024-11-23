from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('picam_client'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='picam_client',
            executable='picam_client_node',
            name='picam_client',
            parameters=[config],
            output='screen',
        )
    ])