import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    package_dir = os.path.dirname(launch_dir)
    config_dir = os.path.join(package_dir, 'config')
    default_map = os.path.join(config_dir, 'correctionMap_left.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to correction map'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/camera/infra1/image_rect_raw',
        description='Input image topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/corrected/image_raw',
        description='Output image topic'
    )

    remapper_node = Node(
        package='jir_image_remapper',
        executable='jir_image_remapper',
        name='image_remapper',
        output='screen',
        parameters=[{
            'left_map': LaunchConfiguration('map')
        }],
        remappings=[
            ('in_left', LaunchConfiguration('input_topic')),
            ('out_left_rect/image_raw', LaunchConfiguration('output_topic')),
        ]
    )

    return LaunchDescription([
        map_arg,
        input_topic_arg,
        output_topic_arg,
        remapper_node,
    ])
