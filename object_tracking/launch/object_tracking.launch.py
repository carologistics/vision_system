from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    image_topic = LaunchConfiguration('image_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    segmentation_height_fraction = LaunchConfiguration('segmentation_height_fraction')
    debug = LaunchConfiguration('debug')
    capture = LaunchConfiguration('capture')
    default_params_file = PathJoinSubstitution([
        FindPackageShare('object_tracking'),
        'config',
        'params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for the object tracking node'
        ),
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
        DeclareLaunchArgument(
            'segmentation_height_fraction',
            default_value='0.4',
            description='Keep only this top fraction of the image for segmentation results'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Publish segmented debug image for RViz'
        ),
        DeclareLaunchArgument(
            'capture',
            default_value='false',
            description='Save source and segmented images to disk'
        ),
        Node(
            package='object_tracking',
            executable='object_tracking_py',
            name='object_tracking_server',
            namespace=namespace,
            parameters=[
                params_file,
                {
                    'image_topic': image_topic,
                    'pointcloud_topic': pointcloud_topic,
                    'segmentation_height_fraction': ParameterValue(
                        segmentation_height_fraction,
                        value_type=float,
                    ),
                    'debug': ParameterValue(debug, value_type=bool),
                    'capture': ParameterValue(capture, value_type=bool),
                },
            ],
            output='screen'
        )
    ])
