from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    default_params_file = PathJoinSubstitution([
        FindPackageShare('object_tracking'),
        'config',
        'params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the object_tracking parameter file'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/frame_rgb',
            description='RGB image topic for segmentation'
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/camera/frame_pc',
            description='Organized camera point cloud topic'
        ),
        Node(
            package='object_tracking',
            executable='object_tracking_py',
            name='object_tracking_server',
            parameters=[
                params_file,
                {
                    'image_topic': image_topic,
                    'pointcloud_topic': pointcloud_topic,
                },
            ],
            output='screen'
        )
    ])
